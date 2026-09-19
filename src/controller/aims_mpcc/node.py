"""ROS adapter; every nonzero command is supervised independently of solving."""
from dataclasses import asdict
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from .io import load_config, yaw_from_quaternion
from .path import ReferencePath
from .runtime import Supervisor, State, angle_difference, clip
from .worker import AsyncSolver
from .history import AppliedHistory


class MPCCNode(Node):
    def __init__(self):
        super().__init__('aims_mpcc')
        self.declare_parameter('path_directory','')
        self.declare_parameter('vehicle_config','')
        self.declare_parameter('output_mode','shadow')
        self.declare_parameter('simulation',False)
        self.declare_parameter('odom_topic','/odometry/filtered')
        self.declare_parameter('log_directory','')
        self.mode=self.get_parameter('output_mode').value
        if self.mode not in ('shadow','drive'): raise ValueError('output_mode must be shadow or drive')
        self.config=load_config(self.get_parameter('vehicle_config').value)
        self.config.validate(require_verified=True,allow_synthetic=(
            self.mode=='shadow' or self.get_parameter('simulation').value))
        self.path=ReferencePath.load(self.get_parameter('path_directory').value)
        self.path.validate_config(self.config,require_recording=self.mode=='drive')
        recorded=self.path.metadata.get('vehicle_geometry')
        if recorded and any(abs(recorded[k]-getattr(self.config,k))>1e-8
                            for k in ('rear_offset','half_length','half_width','wheelbase')):
            raise ValueError('Prepared path uses a different vehicle geometry')
        self.supervisor=Supervisor(self.config,self.path.length)
        self.worker=AsyncSolver(self.get_parameter('path_directory').value,self.config)
        self.last_solve=-math.inf
        self.last_forwarded=0.; self.last_forwarded_time=-math.inf
        self.steering_estimate=0.;self.last_steering_update=time.monotonic()
        self.history=AppliedHistory(self.config.steering_tau)
        self.source_previous=dict(acceleration=0.,steering=0.,steering_rate=0.)
        self.solve_times=[];self.deadline_misses=0
        self.log=None
        directory=self.get_parameter('log_directory').value
        if directory:
            Path(directory).mkdir(parents=True,exist_ok=True)
            self.log=(Path(directory)/f'controller-{time.time_ns()}.jsonl').open('x')
        command_topic='/drive' if self.mode=='drive' else '/mpcc/drive_preview'
        self.command_pub=self.create_publisher(AckermannDriveStamped,command_topic,10)
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

    def publish_path(self,publisher,points):
        msg=PathMsg();msg.header.frame_id=self.path.frame_id;msg.header.stamp=self.get_clock().now().to_msg()
        for point in points:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x=float(point['x']);pose.pose.position.y=float(point['y'])
            pose.pose.orientation.z=math.sin(point['yaw']/2);pose.pose.orientation.w=math.cos(point['yaw']/2)
            msg.poses.append(pose)
        publisher.publish(msg)

    def mode_status(self,msg):
        self.supervisor.set_mode(msg.data,time.monotonic())

    def forwarded(self,msg):
        if msg.drive.jerk==0. and math.isfinite(msg.drive.steering_angle):
            self.last_forwarded=clip(msg.drive.steering_angle,-self.config.steer_limit,self.config.steer_limit)
            self.last_forwarded_time=time.monotonic()
            if self.mode!='shadow' or not self.supervisor.active:
                s=self.supervisor
                matching=(abs(msg.drive.steering_angle-s.last_command.steering)<1e-6 and
                          abs(msg.drive.speed-s.last_command.speed)<1e-6)
                self.history.record(self.last_forwarded_time,self.last_forwarded,msg.drive.speed,
                                    s.last_acceleration if matching else 0.,
                                    s.last_steering_rate if matching else 0.)

    def odometry(self,msg):
        now=time.monotonic()
        try:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            ros_now=self.get_clock().now().nanoseconds*1e-9
            if msg.header.frame_id!=self.path.frame_id or msg.child_frame_id!='base_link':
                raise ValueError('Expected odom/base_link state frames')
            if not 0 <= ros_now-stamp <= .1:
                raise ValueError('Odometry timestamp stale or in future')
            yaw=yaw_from_quaternion(msg.pose.pose.orientation)
            x=msg.pose.pose.position.x-self.config.rear_offset*math.cos(yaw)
            y=msg.pose.pose.position.y-self.config.rear_offset*math.sin(yaw)
            source_time=now-(ros_now-stamp)
            self.steering_estimate,self.source_previous=self.history.at(source_time)
            speed=msg.twist.twist.linear.x
            progress,error=self.path.project([x,y])
            ref=self.path.at(progress)
            state=State(x,y,yaw,speed,self.steering_estimate,stamp)
            self.supervisor.observe(state,now-(ros_now-stamp),progress,error,angle_difference(yaw,ref['yaw']))
            if self.supervisor.active and not self.footprint_inside(state,ref):
                self.supervisor.fault('Measured footprint outside configured corridor')
        except (ValueError,TypeError,OverflowError) as exc:
            self.supervisor.fault(str(exc))

    def footprint_inside(self,state,ref):
        c,s=math.cos(state.yaw),math.sin(state.yaw)
        nx,ny=-math.sin(ref['yaw']),math.cos(ref['yaw'])
        for sx in (-1,1):
            for sy in (-1,1):
                along=self.config.rear_offset+sx*self.config.half_length
                across=sy*self.config.half_width
                lateral=nx*(state.x+along*c-across*s-ref['x'])+ny*(state.y+along*s+across*c-ref['y'])
                if not -self.path.right_width <= lateral <= self.path.left_width: return False
        return True

    def enable(self,request,response):
        try:
            if request.data:
                if not self.worker.ready: raise ValueError('Solver not ready; restart node after worker failure')
                if self.mode=='drive' and self.count_publishers('/drive')!=1:
                    raise ValueError('MPCC must be the sole /drive publisher; stop Nav2')
                self.supervisor.start(time.monotonic())
                self.last_solve=-math.inf
            else:
                self.supervisor.stop()
            response.success=True;response.message=self.supervisor.status
        except ValueError as exc:
            response.success=False;response.message=str(exc)
        return response

    def tick(self):
        now=time.monotonic();s=self.supervisor
        reply=self.worker.poll(now)
        if reply:
            if reply['kind']=='error':
                if 'deadline' in reply['error'].lower(): self.deadline_misses+=1
                s.fault(reply['error']);self.get_logger().error(reply['error'])
            elif reply['kind']=='result':
                self.solve_times.append(reply['solve_time_s'])
                if s.accept(reply,now):
                    self.publish_path(self.prediction_pub,[dict(x=r[0],y=r[1],yaw=r[2]) for r in reply['states']])
        if s.active and self.mode=='drive' and self.count_publishers('/drive')>1:
            s.fault('Another /drive publisher appeared')
        command=s.command(now)
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.drive.speed=command.speed;msg.drive.steering_angle=command.steering
        msg.drive.acceleration=0.;msg.drive.jerk=0.
        self.command_pub.publish(msg)
        if self.mode=='shadow':
            # Preview actuator estimate follows hypothetical commands only while preview is active.
            if s.active:
                self.last_forwarded=command.steering
                self.history.record(now,command.steering,command.speed,s.last_acceleration,s.last_steering_rate)
        if s.active and s.fresh(now) and now-self.last_solve>=.095 and self.worker.pending is None:
            state=asdict(s.state);state.pop('timestamp')
            elapsed=.1 if not math.isfinite(self.last_solve) else now-self.last_solve
            request=dict(state=state,previous=dict(self.source_previous),speed_refs=s.refs(15,.1),generation=s.generation,
                         stamp=s.state_received,submitted_at=now,elapsed=elapsed)
            if self.worker.submit(request): self.last_solve=now
        values=dict(status=s.status,reason=s.reason,worker_ready=self.worker.ready,progress=s.progress,
                    cross_track=s.cross_track,state_age=None if s.state is None else now-s.state_received,
                    plan_age=None if s.plan is None else now-s.plan['stamp'],
                    speed_command=command.speed,steering_command=command.steering,
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
