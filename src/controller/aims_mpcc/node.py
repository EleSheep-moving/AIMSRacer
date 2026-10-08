"""ROS adapter; every nonzero command is supervised independently of solving."""
from dataclasses import asdict
from collections import deque
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from .frames import apply_alignment, planar_alignment
from .io import load_config, yaw_from_quaternion
from .path import ReferencePath
from .runtime import Supervisor, State, Command, angle_difference, clip
from .worker import AsyncSolver
from .history import AppliedHistory
from .localization import LocalizationHealth
from .speed_planner import SpeedPlanner


class MPCCNode(Node):
    def __init__(self):
        super().__init__('aims_mpcc')
        self.declare_parameter('path_directory','')
        self.declare_parameter('vehicle_config','')
        self.declare_parameter('simulation',False)
        self.declare_parameter('odom_topic','/odometry/filtered')
        self.declare_parameter('log_directory','')
        self.declare_parameter('backend','ipopt',ParameterDescriptor(read_only=True))
        self.declare_parameter('artifact_directory','',ParameterDescriptor(read_only=True))
        self.backend=self.get_parameter('backend').value
        if self.backend not in ('ipopt','acados','qp'):
            raise ValueError('backend must be ipopt, acados or qp')
        self.declare_parameter('horizon',10,ParameterDescriptor(
            description='Number of 0.1 s prediction intervals; prepare matching native cache before launch',
            read_only=True))
        self.horizon=self.get_parameter('horizon').value
        if isinstance(self.horizon,bool) or not isinstance(self.horizon,int) or self.horizon<1:
            raise ValueError('positive integer horizon required')
        self.declare_parameter('solve_frequency',5.,ParameterDescriptor(read_only=True))
        self.declare_parameter('plan_ttl',self.horizon*.8*.1,ParameterDescriptor(
            description='Seconds from ORIGINAL EKF measurement, not receipt/takeover; default horizon * 0.8 * 0.1 s',
            read_only=True))
        self.declare_parameter('solver_timeout',.25,ParameterDescriptor(
            description='Seconds from request submission to parent reply; late results are skipped without killing the worker',
            read_only=True))
        self.declare_parameter('handover_delay',.02,ParameterDescriptor(
            description='Forecast lead in seconds, independent of the solve period',read_only=True))
        frequency=self.get_parameter('solve_frequency').value
        self.plan_ttl=self.get_parameter('plan_ttl').value
        if not math.isfinite(frequency) or not 0<frequency<=50:
            raise ValueError('solve_frequency must be positive and at most 50 Hz')
        self.solve_period=1./frequency
        self.solver_timeout=self.get_parameter('solver_timeout').value
        self.handover_delay=self.get_parameter('handover_delay').value
        if not math.isfinite(self.handover_delay) or self.handover_delay<=0:
            raise ValueError('Positive finite handover delay required')
        if not math.isfinite(self.solver_timeout) or self.solver_timeout<=0:
            raise ValueError('solver_timeout result budget must be positive and finite')
        if (not math.isfinite(self.plan_ttl)
                or self.plan_ttl<=self.solve_period+max(self.solver_timeout,self.handover_delay)+.02):
            raise ValueError('plan_ttl must cover solve period, result budget, handover and control tick')
        if self.plan_ttl>self.handover_delay+self.horizon*.1:
            raise ValueError('plan_ttl exceeds the available prediction horizon; increase horizon or reduce timing budgets')
        self.config=load_config(self.get_parameter('vehicle_config').value)
        self.config.validate(require_verified=True,allow_synthetic=self.get_parameter('simulation').value)
        self.path=ReferencePath.load(self.get_parameter('path_directory').value)
        self.path.validate_config(self.config,require_recording=not self.get_parameter('simulation').value)
        self.speed_planner=SpeedPlanner(self.path,self.config)
        self.localization_health=LocalizationHealth()
        self.map_sha256=None
        self.map_buffer=None;self.map_alignment=None
        if self.path.frame_id=='map':
            self.map_buffer=Buffer(node=self)
            # Bound queued TF under load; the default depth of 100 can make
            # this consumer process old corrections while broadcasts are fresh.
            self.map_listener=TransformListener(self.map_buffer,self,qos=QoSProfile(depth=10))
            latched_id=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String,'/localization/map_sha256',self.map_identity,latched_id)
            self.create_subscription(DiagnosticArray,'/localization/status',self.localization_status,10)
        self.supervisor=Supervisor(self.config,self.path.length,self.plan_ttl,self.handover_delay,
                                   solve_period=self.solve_period)
        directory=self.get_parameter('log_directory').value
        self.worker=AsyncSolver(self.get_parameter('path_directory').value,self.config,self.horizon,
                                deadline=self.solver_timeout,log_directory=directory,backend=self.backend,
                                artifact_directory=self.get_parameter('artifact_directory').value or None)
        self.last_solve=-math.inf
        self.next_solve=-math.inf
        self.last_forwarded=0.; self.last_forwarded_time=-math.inf
        self.last_proposed=Command(0.,0.)
        self.recent_proposals=deque(maxlen=16)
        self.steering_estimate=0.;self.last_steering_update=time.monotonic()
        self.history=AppliedHistory(self.config.steering_tau)
        self.source_previous=dict(acceleration=0.,steering=0.,steering_rate=0.)
        self.solve_times=[];self.deadline_misses=0
        self.iteration_limit_skips=0
        self.last_solver_status=None;self.last_solver_iterations=None
        self.last_solver_diagnostics=None;self.last_solve_sequence=None
        self.request_timing=None
        self.map_tf_age=None
        self.map_correction_change=None
        self.last_time_rejection=None
        self.log=None
        if directory:
            Path(directory).mkdir(parents=True,exist_ok=True)
            self.log=(Path(directory)/f'controller-{time.time_ns()}.jsonl').open('x')
        self.command_pub=self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.status_pub=self.create_publisher(DiagnosticArray,'/mpcc/status',10)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(PathMsg,'/mpcc/reference',latched)
        self.prediction_pub=self.create_publisher(PathMsg,'/mpcc/prediction',10)
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.odometry,qos_profile_sensor_data)
        self.create_subscription(Bool,'/control/autonomy_speed_enabled',self.mode_status,10)
        self.create_subscription(AckermannDriveStamped,'/ackermann_cmd',self.forwarded,10)
        self.create_service(SetBool,'/mpcc/enable',self.enable)
        self.timer=self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_path(self.path_pub,[self.path.at(s) for s in self.path.s[:-1]])

    def publish_path(self,publisher,points,frame_id=None):
        msg=PathMsg();msg.header.frame_id=frame_id or self.path.frame_id;msg.header.stamp=self.get_clock().now().to_msg()
        for point in points:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x=float(point['x']);pose.pose.position.y=float(point['y'])
            pose.pose.orientation.z=math.sin(point['yaw']/2);pose.pose.orientation.w=math.cos(point['yaw']/2)
            msg.poses.append(pose)
        publisher.publish(msg)

    def mode_status(self,msg):
        self.supervisor.set_mode(msg.data,time.monotonic())

    def map_identity(self,msg):
        self.map_sha256=msg.data
        if self.path.frame_id=='map' and self.supervisor.active and self.map_sha256!=self.path.metadata['map_sha256']:
            self.supervisor.fault('Map identity changed or does not match reference')

    def localization_status(self,msg):
        now=time.monotonic()
        statuses=[status for status in msg.status if status.name=='aims_racer_system/localization']
        values={entry.key:entry.value for entry in statuses[0].values} if len(statuses)==1 else {}
        changed=self.localization_health.observe(values,self.get_clock().now().nanoseconds,now)
        if changed:
            self.map_alignment=None
            if self.supervisor.active:
                self.supervisor.fault('Localization epoch changed; re-enable required')
        if self.supervisor.active and not self.localization_health.usable(self.get_clock().now().nanoseconds,now):
            self.supervisor.fault('Trusted localization unavailable or expired')

    def require_localization(self,now):
        if self.path.frame_id=='map' and not self.localization_health.usable(self.get_clock().now().nanoseconds,now):
            raise ValueError('Fresh trusted localization required')

    def map_matches_reference(self):
        # Check the reference's coordinate identity, not localization health.
        return self.map_sha256==self.path.metadata['map_sha256']

    def forwarded(self,msg):
        if msg.drive.jerk==0. and math.isfinite(msg.drive.steering_angle):
            self.last_forwarded=clip(msg.drive.steering_angle,-self.config.steer_limit,self.config.steer_limit)
            self.last_forwarded_time=time.monotonic()
            s=self.supervisor
            # Publication on /drive is only a proposal. The selector's output
            # is the sole source of applied-command history in either RC mode.
            # The selector can forward the preceding publication after a new
            # control tick. Match its REAL output to recent issued commands,
            # rather than dropping the derivatives whenever it is one tick late.
            matched=next((entry for entry in reversed(self.recent_proposals)
                          if s.mode and 0<=self.last_forwarded_time-entry[0]<=.1 and
                          abs(msg.drive.steering_angle-entry[1].steering)<1e-6 and
                          abs(msg.drive.speed-entry[1].speed)<1e-6),None)
            self.history.record(self.last_forwarded_time,self.last_forwarded,msg.drive.speed,
                                matched[2] if matched else 0.,
                                matched[3] if matched else 0.)
            if s.active and not s.mode:
                # Keep proposal smoothing anchored to actual manual commands,
                # so switching authority cannot reuse a fictitious execution.
                s.last_command=Command(msg.drive.speed,self.last_forwarded)
                s.last_acceleration=0.
                s.last_steering_rate=0.

    def odometry(self,msg):
        now=time.monotonic()
        try:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            ros_now=self.get_clock().now().nanoseconds*1e-9
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
                raise ValueError('Expected odom/base_link state frames')
            if not 0 <= ros_now-stamp <= .1:
                self.last_time_rejection=dict(signal='odometry',age_s=ros_now-stamp,
                                              now=ros_now,stamp=stamp)
                raise ValueError(f'Odometry timestamp stale or in future (age {ros_now-stamp:.6f} s)')
            yaw=yaw_from_quaternion(msg.pose.pose.orientation)
            x,y=msg.pose.pose.position.x,msg.pose.pose.position.y
            x-=self.config.rear_offset*math.cos(yaw)
            y-=self.config.rear_offset*math.sin(yaw)
            if self.path.frame_id=='map':
                if not self.localization_health.usable(self.get_clock().now().nanoseconds,now):
                    if self.supervisor.active: raise ValueError('Trusted localization unavailable or expired')
                    return
                if not self.map_matches_reference():
                    if self.supervisor.active: raise ValueError('Map identity does not match reference')
                    return
                try:
                    transform=self.map_buffer.lookup_transform('map','odom',Time())
                except TransformException as exc:
                    if self.supervisor.active: raise ValueError('Map transform unavailable') from exc
                    return
                transform_stamp=(transform.header.stamp.sec+
                                 transform.header.stamp.nanosec*1e-9)
                lookup_now=self.get_clock().now().nanoseconds*1e-9
                self.map_tf_age=lookup_now-transform_stamp
                alignment=planar_alignment(transform)
                reference_x,reference_y,reference_yaw=apply_alignment(x,y,yaw,alignment)
                if self.supervisor.active and self.map_alignment is not None:
                    old_x,old_y,old_yaw=apply_alignment(x,y,yaw,self.map_alignment)
                    self.map_correction_change=dict(
                        position=math.hypot(reference_x-old_x,reference_y-old_y),
                        yaw=abs(angle_difference(reference_yaw,old_yaw)))
                self.map_alignment=alignment
            else:
                reference_x,reference_y,reference_yaw=x,y,yaw
            source_time=now-(ros_now-stamp)
            self.steering_estimate,self.source_previous=self.history.at(source_time)
            speed=msg.twist.twist.linear.x
            progress,error=self.path.project([reference_x,reference_y])
            ref=self.path.at(progress)
            state=State(x,y,yaw,speed,self.steering_estimate,stamp)
            reference_state=State(reference_x,reference_y,reference_yaw,speed,
                                  self.steering_estimate,stamp)
            self.supervisor.observe(state,now-(ros_now-stamp),progress,error,
                                    angle_difference(reference_yaw,ref['yaw']))
            if (self.config.enforce_corridor and self.supervisor.active
                    and not self.footprint_inside(reference_state,ref)):
                self.supervisor.fault('Measured footprint outside configured corridor')
        except (ValueError,TypeError,OverflowError) as exc:
            self.supervisor.fault(str(exc))

    def footprint_inside(self,state,ref):
        c,s=math.cos(state.yaw),math.sin(state.yaw)
        nx,ny=-math.sin(ref['yaw']),math.cos(ref['yaw'])
        for along in self.config.longitudinal_offsets():
            for sy in (-1,1):
                across=sy*self.config.half_width
                lateral=nx*(state.x+along*c-across*s-ref['x'])+ny*(state.y+along*s+across*c-ref['y'])
                if not -self.path.right_width <= lateral <= self.path.left_width: return False
        return True

    def enable(self,request,response):
        try:
            if request.data:
                self.require_localization(time.monotonic())
                if not self.worker.ready: raise ValueError('Solver not ready; restart node after worker failure')
                if self.path.frame_id=='map' and not self.map_matches_reference():
                    raise ValueError('Reference map identity required')
                if self.count_publishers('/drive')!=1:
                    raise ValueError('MPCC must be the sole /drive publisher; stop Nav2')
                self.applied_ready(time.monotonic())
                state=self.supervisor.state
                if state is None: raise ValueError('Fresh state required before enabling')
                x,y,yaw=state.x,state.y,state.yaw
                if self.path.frame_id=='map':
                    if self.map_alignment is None: raise ValueError('Map alignment unavailable')
                    x,y,yaw=apply_alignment(x,y,yaw,self.map_alignment)
                reference_state=State(x,y,yaw,state.speed,state.steering,state.timestamp)
                if (self.config.enforce_corridor and
                        not self.footprint_inside(reference_state,self.path.at(self.supervisor.wrapped_progress))):
                    raise ValueError('Starting footprint outside configured corridor')
                self.supervisor.start(time.monotonic())
                self.last_solve=-math.inf
                self.next_solve=-math.inf
            else:
                self.supervisor.stop()
            response.success=True;response.message=self.supervisor.status
        except ValueError as exc:
            response.success=False;response.message=str(exc)
        return response

    def applied_ready(self,now):
        if not self.history.records or not 0<=now-self.history.records[-1][0]<=.1:
            raise ValueError('Fresh applied speed-mode command history required')
        self.history.command_at(self.supervisor.state_received)

    def prepare_request(self,now):
        """Bridge the measurement to a fixed future takeover, before solving."""
        s=self.supervisor
        self.applied_ready(now)
        actual=self.history.command_at(now)
        forecast=copy.copy(s)
        forecast.pending_plan=None
        forecast.last_command=Command(actual['speed'],actual['steering'])
        forecast.last_acceleration=actual['acceleration']
        forecast.last_steering_rate=actual['steering_rate']
        forecast.last_tick=now
        last_forecast=[now]
        def future_command(stamp,state):
            stamp=max(stamp,last_forecast[0])
            if not s.mode:
                # Future operator input is unknown; hold the last REAL target
                # and validate the resulting prediction at takeover.
                return dict(actual,acceleration=0.,steering_rate=0.)
            forecast.state=state
            forecast.state_received=forecast.mode_received=stamp
            forecast.progress+=max(0.,state.speed)*(stamp-last_forecast[0])
            last_forecast[0]=stamp
            command=forecast.command(stamp,enforce_plan_age=False)
            if forecast.status=='FAULT':
                raise ValueError('Old plan cannot bridge scheduled handover: '+forecast.reason)
            actuator=forecast.actuator_command(command)
            return dict(speed=actuator.speed,steering=actuator.steering,
                        acceleration=forecast.last_acceleration if actuator.speed==command.speed else 0.,
                        steering_rate=forecast.last_steering_rate)
        takeover=now+self.handover_delay
        predicted,previous=self.history.predict(s.state,s.state_received,takeover,self.config,
                                                known_until=now,future_command=future_command)
        reference_x,reference_y=predicted.x,predicted.y
        if self.path.frame_id=='map':
            if self.map_alignment is None:
                raise ValueError('Map alignment unavailable')
            reference_x,reference_y,_=apply_alignment(predicted.x,predicted.y,predicted.yaw,
                                                     self.map_alignment)
        progress,_=self.path.project([reference_x,reference_y])
        forecast.state=predicted
        forecast.progress=s.progress+(progress-s.wrapped_progress+self.path.length/2)%self.path.length-self.path.length/2
        state=asdict(predicted);state.pop('timestamp')
        elapsed=self.solve_period if not math.isfinite(self.last_solve) else now-self.last_solve
        request=dict(state=state,previous={key:previous[key] for key in ('acceleration','steering','steering_rate')},
                     speed_refs=forecast.refs(self.horizon,.1),generation=s.generation,
                     source_stamp=s.state_received,stamp=takeover,submitted_at=now,elapsed=elapsed,
                     handover_command=dict(previous))
        if self.path.frame_id=='map':request['map_alignment']=self.map_alignment
        if hasattr(self,'speed_planner'):
            caps=self.speed_planner.refs(forecast.progress,self.horizon,.1)
            request['speed_refs']=[min(target,cap) for target,cap in zip(request['speed_refs'],caps)]
        return request

    def tick(self):
        now=time.monotonic();s=self.supervisor
        if self.path.frame_id=='map' and s.active:
            try:
                self.require_localization(now)
            except ValueError as exc:
                s.fault(str(exc))
        if self.path.frame_id=='map' and s.active and not self.map_matches_reference():
            s.fault('Map identity does not match reference')
        reply=self.worker.poll(now)
        if reply:
            if reply['kind']=='error':
                s.fault(reply['error']);self.get_logger().error(reply['error'])
            elif reply['kind']=='restarting':
                s.solver_failure(now,reply['reason'])
                self.get_logger().warning(reply['reason'])
            elif reply['kind']=='skipped':
                self.deadline_misses+=1
                s.solver_failure(now,'Solver request deadline exceeded')
            elif reply['kind']=='result':
                self.solve_times.append(reply['solve_time_s'])
                self.last_solver_status=reply.get('status')
                self.last_solver_iterations=reply.get('iterations')
                self.last_solver_diagnostics=reply.get('diagnostics')
                self.last_solve_sequence=reply.get('solve_sequence')
                self.request_timing=dict(source_to_submit=reply['submitted_at']-reply['source_stamp'],
                    request_to_reply=now-reply['submitted_at'],
                    worker_queue=reply['worker_started_at']-reply['submitted_at'],
                    result_delivery=now-reply['worker_finished_at'],
                    reply_before_handover=reply['stamp']-now)
                iteration_limited=(not reply.get('success') and
                                   reply.get('status')=='Maximum_Iterations_Exceeded')
                if iteration_limited:
                    # Keep the last accepted plan and its original expiry.
                    # The failed trajectory never becomes an executed plan.
                    self.iteration_limit_skips+=1
                    s.solver_failure(now,'Solver iteration budget exhausted')
                if reply.get('discarded'):
                    if not reply.get('skip_notified'): self.deadline_misses+=1
                    if not reply.get('skip_notified') and not iteration_limited:
                        s.solver_failure(now,'Late solver result discarded')
                elif not iteration_limited and s.accept(reply,now):
                    self.publish_path(self.prediction_pub,
                                      [dict(x=r[0],y=r[1],yaw=r[2]) for r in reply['states']],
                                      frame_id='odom')
        if (s.active and s.pending_plan is not None
                and now>=s.pending_plan['stamp']):
            try:
                self.applied_ready(now)
                if not s.fresh(now):raise ValueError('Fresh state required at plan handover')
                actual,_=self.history.predict(s.state,s.state_received,now,self.config)
                pending=s.pending_plan
                predicted=pending['states'][0]
                expected=State(predicted[0],predicted[1],predicted[2],predicted[3],predicted[5],s.state.timestamp)
                expected,_=self.history.predict(expected,pending['stamp'],now,self.config)
                xy=[actual.x,actual.y]
                if self.path.frame_id=='map':
                    mx,my,_=apply_alignment(actual.x,actual.y,actual.yaw,self.map_alignment)
                    xy=[mx,my]
                progress,_=self.path.project(xy)
                progress=predicted[4]+(progress-predicted[4]+self.path.length/2)%self.path.length-self.path.length/2
                s.activate(now,actual,self.history.command_at(now),expected_at_activation=expected,
                           path=self.path,progress=progress,map_alignment=self.map_alignment)
            except ValueError as exc:
                s.fault(str(exc))
        if s.active and self.count_publishers('/drive')>1:
            s.fault('Another /drive publisher appeared')
        command=s.command(now)
        actuator=s.actuator_command(command)
        self.last_proposed=actuator
        self.recent_proposals.append((now,actuator,
            s.last_acceleration if s.active and actuator.speed==command.speed else 0.,
            s.last_steering_rate if s.active else 0.))
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.drive.speed=actuator.speed;msg.drive.steering_angle=actuator.steering
        msg.drive.acceleration=0.;msg.drive.jerk=0.
        self.command_pub.publish(msg)
        if (s.active and s.fresh(now) and now>=self.next_solve-1e-6
                and self.worker.pending is None and s.pending_plan is None):
            try:
                request=self.prepare_request(now)
                if self.worker.submit(request):
                    self.last_solve=now
                    self.next_solve=(self.next_solve+self.solve_period
                                     if math.isfinite(self.next_solve) else now+self.solve_period)
                    # Keep the update clock anchored despite timer jitter; skip
                    # missed slots instead of accumulating delay or bursting.
                    while self.next_solve<=now:self.next_solve+=self.solve_period
            except ValueError as exc:
                s.fault(str(exc))
        values=dict(status=s.status,reason=s.reason,worker_ready=self.worker.ready,horizon=self.horizon,
                    backend=self.backend,solver_restart_count=self.worker.restart_count,
                    consecutive_solver_failures=s.consecutive_failures,
                    recovery_good_candidates=s.recovery_good_candidates,
                    corridor_enforced=self.config.enforce_corridor,
                    autonomy_selected=s.mode,progress=s.progress,
                    start_progress=s.start_progress,lap_progress=s.progress-s.start_progress,
                    lap_remaining=s.lap_goal-s.progress,
                    cross_track=s.cross_track,state_age=None if s.state is None else now-s.state_received,
                    plan_age=None if s.plan is None else now-s.plan.get('source_stamp',s.plan['stamp']),
                    plan_phase=None if s.plan is None else now-s.plan['stamp'],
                    handover_lateness=None if s.plan is None else s.plan['stamp']-s.plan.get('scheduled_stamp',s.plan['stamp']),
                    solve_frequency=1./self.solve_period,plan_ttl=self.plan_ttl,
                    solver_timeout=self.solver_timeout,
                    handover_delay=self.handover_delay,request_timing=self.request_timing,
                    map_tf_age=self.map_tf_age,last_time_rejection=self.last_time_rejection,
                    map_correction_change=self.map_correction_change,
                    pending_handover_in=None if s.pending_plan is None else s.pending_plan['stamp']-now,
                    handover_error=s.handover_error,rejected_plans=s.rejected_plans,
                    handover_reprojection=None if s.plan is None else s.plan.get('handover_reprojection'),
                    handover_limits={**s.HANDOVER_STATE_LIMITS,**s.HANDOVER_COMMAND_LIMITS},
                    solver_max_iterations=self.config.solver_max_iterations,
                    solver_status=self.last_solver_status,solver_iterations=self.last_solver_iterations,
                    solve_sequence=self.last_solve_sequence,solver_diagnostics=self.last_solver_diagnostics,
                    iteration_limit_skips=self.iteration_limit_skips,
                    late_result_skips=self.deadline_misses,solver_busy=self.worker.pending is not None,
                    speed_command=actuator.speed,model_speed_command=command.speed,
                    minimum_drive_speed=self.config.minimum_drive_speed,
                    steering_command=actuator.steering,
                    speed=None if s.state is None else s.state.speed,
                    solve_time=None if not self.solve_times else self.solve_times[-1],deadline_misses=self.deadline_misses)
        diag=DiagnosticArray();diag.header.stamp=msg.header.stamp
        item=DiagnosticStatus();item.name='aims_mpcc';item.level=b'\x02' if s.status=='FAULT' else b'\x00'
        item.message=s.reason or s.status
        item.values=[KeyValue(key=k,value=json.dumps(v,allow_nan=False)) for k,v in values.items()]
        diag.status=[item];self.status_pub.publish(diag)
        if self.log:
            self.log.write(json.dumps(dict(monotonic=now,**values),allow_nan=False)+'\n');self.log.flush()

    def destroy_node(self):
        self.worker.close()
        if self.log:self.log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=MPCCNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
