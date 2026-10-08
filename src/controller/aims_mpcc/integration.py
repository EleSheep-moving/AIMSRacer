"""Hardware-free ROS acceptance using actual RC selection and VESC conversion.

The plant independently integrates a bicycle with lagged speed/steering, consumes
ERPM/servo outputs, and supplies synthetic EKF odometry. It is not a tire model.
"""
import argparse
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from crsf_receiver_msg.msg import CRSFChannels16
from .io import load_config
from .path import prepare_recording,ReferencePath
from .node import MPCCNode


# Include every actuator output, even unused converter current/duty modes.
DRIVING_TOPICS=('/drive','/ackermann_cmd','/commands/motor/speed',
                '/commands/servo/position','/commands/motor/current',
                '/commands/motor/duty_cycle')
SHADOW_ENTITIES=DRIVING_TOPICS+('/rc/channels','/control/autonomy_speed_enabled',
    '/odometry/filtered','/calib/ackermann_cmd','/sensors/core','/mpcc/status',
    '/mpcc/reference','/mpcc/prediction','/mpcc/enable',
    '/localization/map_sha256','/localization/status','/tf','/tf_static')


def topic_remap_arguments(prefix):
    """Remap absolute source names for both Python nodes and ROS subprocesses."""
    if not prefix:return []
    if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+',prefix):
        raise ValueError('topic-prefix must be an absolute ROS namespace without a trailing slash')
    return [argument for topic in SHADOW_ENTITIES
            for argument in ('-r',f'{topic}:={prefix}{topic}')]


def child_commands(vesc_config,remaps):
    return [
        ('rc',['ros2','run','ackermann_mux','joystick_control_v2','--ros-args',
               '-p','channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10',*remaps]),
        ('converter',['ros2','run','vesc_ackermann','ackermann_to_vesc_node',
                      '--ros-args','--params-file',vesc_config,*remaps])]


class ShadowMPCCNode(MPCCNode):
    def count_publishers(self,topic_name):
        # Humble graph counts do not apply remaps. Keep the existing authority
        # checks querying the same resolved topic as the command publisher.
        return super().count_publishers(self.resolve_topic_name(topic_name))


def shadow_graph_report(node,prefix):
    """Query original names directly; graph counts intentionally ignore remaps."""
    original={topic:node.count_publishers(topic) for topic in DRIVING_TOPICS}
    shadow={prefix+topic:node.count_publishers(prefix+topic) for topic in DRIVING_TOPICS}
    return dict(unprefixed_driving_publisher_counts=original,
                unprefixed_driving_publishers_zero=not any(original.values()),
                shadow_driving_publisher_counts=shadow,
                shadow_route_ready=all(count==1 for count in shadow.values()))


class Plant(Node):
    def __init__(self,path,config,speed_tau=.2,steer_tau=.15,odom_delay=0.):
        super().__init__('synthetic_vehicle')
        self.path,self.config=path,config
        initial=path.at(0.)
        self.x,self.y,self.yaw=initial['x'],initial['y'],initial['yaw']
        self.speed=self.steering=0.
        self.speed_target=self.steer_target=0.
        self.speed_tau,self.steer_tau=speed_tau,steer_tau
        self.last=time.monotonic();self.last_odom=0.
        self.publish_odom=True;self.publish_rc=True;self.autonomy=True;self.clock_offset=0.
        self.samples=[];self.commands=[]
        self.true_angle=math.atan2(self.y,self.x);self.true_progress=0.
        self.odom_delay=odom_delay;self.delayed=deque()
        self.odom_pub=self.create_publisher(Odometry,'/odometry/filtered',10)
        self.rc_pub=self.create_publisher(CRSFChannels16,'/rc/channels',qos_profile_sensor_data)
        self.create_subscription(Float64,'/commands/motor/speed',self.erpm,10)
        self.create_subscription(Float64,'/commands/servo/position',self.servo,10)
        self.timer=self.create_timer(.005,self.step)

    def erpm(self,msg):
        self.speed_target=msg.data/4650.
        self.commands.append((time.monotonic(),self.speed_target,self.steer_target))

    def servo(self,msg):
        self.steer_target=(msg.data-.506)/(-.5137)

    def step(self):
        now=time.monotonic();elapsed=min(.1,now-self.last);self.last=now
        # Independent small-step midpoint integrator; never call solver dynamics.
        count=max(1,math.ceil(elapsed/.002))
        dt=elapsed/count
        for _ in range(count):
            self.speed+=(self.speed_target-self.speed)*(1-math.exp(-dt/self.speed_tau))
            self.steering+=(self.steer_target-self.steering)*(1-math.exp(-dt/self.steer_tau))
            yaw_rate=self.speed*math.tan(self.steering)/self.config.wheelbase
            self.x+=self.speed*math.cos(self.yaw+.5*yaw_rate*dt)*dt
            self.y+=self.speed*math.sin(self.yaw+.5*yaw_rate*dt)*dt
            self.yaw+=yaw_rate*dt
            angle=math.atan2(self.y,self.x)
            self.true_progress+=2*((angle-self.true_angle+math.pi)%(2*math.pi)-math.pi)
            self.true_angle=angle
        if now-self.last_odom < .019:return
        self.last_odom=now
        if self.publish_rc:
            rc=CRSFChannels16()
            for i in range(1,17):setattr(rc,f'ch{i}',992)
            rc.ch5=1810;rc.ch6=172;rc.ch7=1810 if self.autonomy else 172;rc.ch8=172;rc.ch10=172
            self.rc_pub.publish(rc)
        if self.publish_odom:
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link'
            stamp=self.get_clock().now().nanoseconds+int(self.clock_offset*1e9)
            odom.header.stamp.sec=stamp//10**9;odom.header.stamp.nanosec=stamp%10**9
            odom.pose.pose.position.x=self.x+self.config.rear_offset*math.cos(self.yaw)
            odom.pose.pose.position.y=self.y+self.config.rear_offset*math.sin(self.yaw)
            odom.pose.pose.orientation.z=math.sin(self.yaw/2);odom.pose.pose.orientation.w=math.cos(self.yaw/2)
            odom.twist.twist.linear.x=self.speed
            odom.twist.twist.linear.y=self.config.rear_offset*self.speed*math.tan(self.steering)/self.config.wheelbase
            odom.twist.twist.angular.z=self.speed*math.tan(self.steering)/self.config.wheelbase
            self.delayed.append((now,odom))
            while self.delayed and now-self.delayed[0][0]>=self.odom_delay:
                self.odom_pub.publish(self.delayed.popleft()[1])


def fixture(directory,config,direction):
    recording=directory/'manual-lap.csv'
    with recording.open('w',newline='') as handle:
        writer=csv.writer(handle);writer.writerow(['timestamp','x','y','yaw','speed','frame_id','child_frame_id'])
        for i in range(501):
            angle=direction*2*math.pi*i/500
            yaw=angle+direction*math.pi/2
            writer.writerow([i*.05,2*math.cos(angle)+config.rear_offset*math.cos(yaw),
                             2*math.sin(angle)+config.rear_offset*math.sin(yaw),yaw,.5,'odom','base_link'])
    return prepare_recording(recording,directory/'reference',config,.9,.9)


def check_nominal_status(status,reason):
    if status in ('READY','FAULT'):
        raise RuntimeError('Nominal run stopped: '+reason)


def speed_attainment(samples,reference_length,requested_cruise,initial_position):
    """Use the radius-2 fixture's true progress, including all recovery samples."""
    samples=np.asarray(samples,float)
    if not math.isfinite(requested_cruise) or requested_cruise<=0:
        raise ValueError('Positive finite requested cruise speed required')
    initial_angle=math.atan2(initial_position[1],initial_position[0])
    angles=np.unwrap(np.arctan2(samples[:,2],samples[:,1]))
    progress=2*np.abs(angles-initial_angle)
    central=samples[(progress>=.1*reference_length-1e-9)&
                    (progress<=.7*reference_length+1e-9),3]
    report=dict(requested_cruise_speed_mps=requested_cruise,
                central_progress_fraction=[.1,.7],central_sample_count=len(central),
                minimum_central_p50_ratio=.8,speed_tracking_pass=False)
    for name,value in [('mean',np.mean(central) if len(central) else None),
                       ('p50',np.percentile(central,50) if len(central) else None),
                       ('p95',np.percentile(central,95) if len(central) else None)]:
        report[f'central_speed_{name}_mps']=None if value is None else float(value)
        report[f'central_speed_{name}_ratio']=None if value is None else float(value/requested_cruise)
    report['speed_tracking_pass']=bool(len(central) and
                                      report['central_speed_p50_ratio']>=.8)
    return report


def check_scenario_acceptance(result,scenario,max_speed,steer_limit,speed_target,last_command_speed):
    """Apply the original physical limits and the current authority contract."""
    assert result['maximum_command_speed_mps']<=max_speed+1e-8
    assert result['maximum_command_steering_rad']<=steer_limit+1e-8
    assert result['minimum_footprint_margin_m']>=0
    status=result['status_at_end']
    if scenario=='nominal':
        assert status=='COMPLETE'
        assert result['cross_track_rms_m']<=.10 and result['cross_track_max_m']<=.25
        assert result['finish_error_m']<=.2 and result['final_speed_mps']<.05
        length=result['reference_length_m']
        assert length-.2 <= result['lap_progress_m'] <= length+.2
    elif scenario=='disable':
        assert 'fault_detection_s' not in result,'Normal stopping entered FAULT'
        assert status=='READY'
        assert abs(speed_target)<1e-9 and abs(result['final_speed_mps'])<.05
    elif scenario in ('solver_stall','solver_crash'):
        assert 'fault_detection_s' not in result,'Single worker failure entered FAULT'
        assert result['worker_restarts']==1 and result['worker_ready']
        if status=='RUNNING':
            minimum=2 if result['recovery_observed'] else 1
            assert result['post_restart_activations']>=minimum
            assert result['final_speed_mps']>=.05
        else:
            assert status=='READY'
            assert result['reason_at_end']=='Recovery stopped; re-enable required'
            assert result['stopped_latch_duration_s']>=.5
            assert abs(result['final_speed_mps'])<.05
            assert abs(speed_target)<1e-9 and last_command_speed==0.
    else:
        assert status=='FAULT','Injected fault was not latched'
        assert result['fault_detection_s']<.4
        assert abs(speed_target)<1e-9 and last_command_speed==0.


def annular_footprint_margins(x,y,yaw,longitudinal_offsets,half_width):
    """Original four-corner synthetic-circle metric for either body geometry."""
    front,rear=longitudinal_offsets
    c,sn=math.cos(yaw),math.sin(yaw)
    margins=[]
    for along in (rear,front):
        for sy in (-1,1):
            across=sy*half_width
            corner_x=x+along*c-across*sn
            corner_y=y+along*sn+across*c
            margins.append(.9-abs(math.hypot(corner_x,corner_y)-2.))
    return margins


def run(args):
    prefix=getattr(args,'topic_prefix','')
    remaps=topic_remap_arguments(prefix)
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    config_file=Path(args.vehicle_config).resolve();config=load_config(config_file)
    footprint_offsets=config.longitudinal_offsets()
    if args.reuse_prepared_reference:
        path=ReferencePath.load(out/'reference')
        path.validate_config(config,require_recording=True)
    else:
        path=fixture(out,config,-1 if args.clockwise else 1)
    if args.prepare_only:
        from .prepare_solver import main as prepare_solver
        prepare_solver([str(out/'reference'),'--vehicle-config',str(config_file),
                        '--backend',args.backend,'--horizon',str(args.horizon)])
        return
    rclpy.init(args=['--ros-args','-p',f'path_directory:={out / "reference"}',
                     '-p',f'vehicle_config:={config_file}',
                     '-p',f'horizon:={args.horizon}',
                     '-p',f'backend:={args.backend}','-p',f'solve_frequency:={args.solve_frequency}',
                     '-p','simulation:=true','-p',f'log_directory:={out}',*remaps])
    children=[];logs=[];controller=plant=None
    result=dict(status='FAIL',scenario=args.scenario,physics='independent lagged kinematic bicycle',
                hardware_validated=False)
    if prefix:result.update(topic_prefix=prefix,shadow_graph_checks=0,
                           authority_graph_adapter='ROS resolved topic names; original publisher-count thresholds')
    started=None;injected=None;fault_observed=None;errors=[];margins=[];last_metrics=0.
    recovery_observed=False;post_restart_activations=0;last_activation=None
    stopped_latch_since=None
    try:
        for name,cmd in child_commands(args.vesc_config,remaps):
            log=(out/f'{name}.log').open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        controller=(ShadowMPCCNode if prefix else MPCCNode)()
        plant=Plant(path,config,args.speed_tau,args.steer_tau,args.odom_delay)
        executor=SingleThreadedExecutor();executor.add_node(controller);executor.add_node(plant)
        client=plant.create_client(SetBool,'/mpcc/enable')
        enable_future=None
        graph_ready=not prefix;last_graph_check=0.
        deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.005)
            now=time.monotonic();s=controller.supervisor
            if any(p.poll() is not None for p in children):raise RuntimeError('RC/converter process exited')
            if prefix and now-last_graph_check>=.1:
                result.update(shadow_graph_report(plant,prefix))
                result['shadow_graph_checks']+=1;last_graph_check=now
                if not result['unprefixed_driving_publishers_zero']:
                    raise RuntimeError('Unprefixed driving publisher detected: '+
                                       repr(result['unprefixed_driving_publisher_counts']))
                graph_ready=result['shadow_route_ready']
                if started is not None and not graph_ready:
                    raise RuntimeError('Shadow driving publisher graph changed during run')
            if started is None and enable_future is None and graph_ready and controller.worker.ready and s.fresh(now) and client.service_is_ready():
                if prefix:
                    # Audit immediately before the enable request as well as
                    # periodically; never enable using a cached graph sample.
                    result.update(shadow_graph_report(plant,prefix));result['shadow_graph_checks']+=1
                    if not result['unprefixed_driving_publishers_zero']:
                        raise RuntimeError('Unprefixed driving publisher detected: '+
                                           repr(result['unprefixed_driving_publisher_counts']))
                    if not result['shadow_route_ready']:continue
                request=SetBool.Request();request.data=True;enable_future=client.call_async(request)
            if enable_future is not None and enable_future.done() and started is None:
                response=enable_future.result()
                if not response.success:raise RuntimeError('Enable rejected: '+response.message)
                started=now
            if started is None:
                if s.status=='FAULT':raise RuntimeError(s.reason)
                continue
            if args.scenario=='nominal':check_nominal_status(s.status,s.reason)
            if injected is None and now-started>3 and args.scenario!='nominal':
                injected=now
                if args.scenario=='odom_drop':plant.publish_odom=False
                elif args.scenario=='manual':plant.autonomy=False
                elif args.scenario=='rc_loss':plant.publish_rc=False
                elif args.scenario=='clock_reset':plant.clock_offset=-5.
                elif args.scenario=='solver_stall':os.kill(controller.worker.process.pid,signal.SIGSTOP)
                elif args.scenario=='solver_crash':controller.worker.process.kill()
                elif args.scenario=='disable':
                    request=SetBool.Request();request.data=False;client.call_async(request)
            if now-last_metrics>=.019:
                last_metrics=now
                progress,error=path.project([plant.x,plant.y]);errors.append(error)
                # Exact annular corridor at the original four corners,
                # independent of the optimizer's tangent approximation.
                margins.extend(annular_footprint_margins(plant.x,plant.y,plant.yaw,
                                                       footprint_offsets,config.half_width))
                plant.samples.append((now-started,plant.x,plant.y,plant.speed,error,plant.speed_target,plant.steer_target))
            if s.status=='FAULT':
                if args.scenario in ('nominal','solver_stall','solver_crash'):
                    raise RuntimeError(s.reason)
                if injected is None:raise RuntimeError('Fault before injection: '+s.reason)
                if fault_observed is None:fault_observed=now
                # Allow converter messages to reach the plant, then verify latch.
                if now-fault_observed>.35:
                    assert abs(plant.speed_target)<1e-9,'Fault did not reach zero-speed converter output'
                    assert s.last_command.speed==0,'Fault resumed nonzero control'
                    break
            if args.scenario=='disable' and injected and s.status=='READY' and plant.speed<.05:
                break
            if args.scenario in ('solver_stall','solver_crash') and injected:
                recovery_observed=recovery_observed or s.status=='RECOVERING'
                if (controller.worker.restart_count and s.plan is not None and
                        s.plan['source_stamp']>=injected and s.last_usable_update!=last_activation):
                    post_restart_activations+=1
                    last_activation=s.last_usable_update
                if s.status=='READY' and s.reason=='Recovery stopped; re-enable required':
                    if stopped_latch_since is None:stopped_latch_since=now
                    if controller.worker.ready and now-stopped_latch_since>=.5:
                        break
                else:
                    stopped_latch_since=None
                minimum=2 if recovery_observed else 1
                if (controller.worker.ready and s.status=='RUNNING' and plant.speed>=.05 and
                        post_restart_activations>=minimum):
                    break
            if s.status=='COMPLETE':break
        else:raise RuntimeError('Acceptance run timed out')
        if prefix:
            result.update(shadow_graph_report(plant,prefix));result['shadow_graph_checks']+=1
            assert result['unprefixed_driving_publishers_zero'],'Unprefixed driving publisher at completion'
            assert result['shadow_route_ready'],'Shadow driving publisher graph changed at completion'
        samples=np.asarray(plant.samples);commands=np.asarray(plant.commands)
        result.update(elapsed_s=time.monotonic()-started,status_at_end=controller.supervisor.status,
                      reason_at_end=controller.supervisor.reason,
                      cross_track_rms_m=float(np.sqrt(np.mean(np.square(errors)))),
                      cross_track_max_m=float(np.max(np.abs(errors))),minimum_footprint_margin_m=float(min(margins)),
                      finish_error_m=float(np.hypot(plant.x-path.at(0)['x'],plant.y-path.at(0)['y'])),
                      final_speed_mps=plant.speed,maximum_command_speed_mps=float(np.max(commands[:,1])),
                      lap_progress_m=abs(plant.true_progress),reference_length_m=path.length,
                      maximum_command_steering_rad=float(np.max(np.abs(commands[:,2]))),
                      solve_count=len(controller.solve_times),deadline_misses=controller.deadline_misses,
                      solve_p50_s=float(np.percentile(controller.solve_times,50)),
                      solve_p95_s=float(np.percentile(controller.solve_times,95)),
                      solve_max_s=float(max(controller.solve_times)))
        if fault_observed is not None:
            result['fault_detection_s']=fault_observed-injected
        if args.scenario in ('solver_stall','solver_crash'):
            result.update(worker_restarts=controller.worker.restart_count,worker_ready=controller.worker.ready,
                          recovery_observed=recovery_observed,post_restart_activations=post_restart_activations,
                          stopped_latch_duration_s=0. if stopped_latch_since is None else now-stopped_latch_since)
        check_scenario_acceptance(result,args.scenario,config.max_speed,config.steer_limit,
                                  plant.speed_target,controller.supervisor.last_command.speed)
        if args.scenario=='nominal':
            initial=path.at(0.)
            result.update(speed_attainment(samples,path.length,config.cruise_speed,
                                           (initial['x'],initial['y'])),geometry_pass=True)
            result['overall_qualification_pass']=result['speed_tracking_pass']
            result['qualification_status']='PASS' if result['overall_qualification_pass'] else 'FAIL'
        np.savetxt(out/'trajectory.csv',samples,delimiter=',',header='time,x,y,speed,cross_track,speed_command,steering_command',comments='')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,3,figsize=(12,3.5))
        axes[0].plot(path.points[:,0],path.points[:,1],'--',label='reference')
        axes[0].plot(samples[:,1],samples[:,2],label='vehicle')
        axes[0].set_aspect('equal');axes[0].set(xlabel='x [m]',ylabel='y [m]');axes[0].legend()
        axes[1].plot(samples[:,0],samples[:,3],label='measured')
        axes[1].plot(samples[:,0],samples[:,5],label='command')
        axes[1].set(xlabel='time [s]',ylabel='speed [m/s]');axes[1].legend()
        axes[2].plot(samples[:,0],samples[:,4]);axes[2].set(xlabel='time [s]',ylabel='cross-track error [m]')
        fig.tight_layout();fig.savefig(out/'tracking.png',dpi=150);plt.close(fig)
        result['status']='PASS'
    except BaseException as exc:
        result['status']='FAIL'
        result['error']=repr(exc)
        raise
    finally:
        if plant and plant.samples:
            np.savetxt(out/'trajectory.csv',np.asarray(plant.samples),delimiter=',',header='time,x,y,speed,cross_track,speed_command,steering_command',comments='')
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        if controller:controller.destroy_node()
        if plant:plant.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in children:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--topic-prefix',default='',
                        help='Hardware-free shadow namespace, e.g. /mpcc_shadow; empty preserves existing topics')
    parser.add_argument('--vehicle-config',default='/ws/src/controller/config/synthetic.yaml')
    parser.add_argument('--scenario',choices=['nominal','odom_drop','manual','rc_loss','clock_reset','solver_stall','solver_crash','disable'],default='nominal')
    parser.add_argument('--clockwise',action='store_true')
    parser.add_argument('--speed-tau',type=float,default=.2)
    parser.add_argument('--steer-tau',type=float,default=.15)
    parser.add_argument('--odom-delay',type=float,default=0.)
    parser.add_argument('--timeout',type=float,default=240.)
    parser.add_argument('--backend',choices=['ipopt','acados','qp'],default='ipopt')
    parser.add_argument('--horizon',type=int,choices=[10,15,20],default=10)
    parser.add_argument('--solve-frequency',type=float,default=5.)
    parser.add_argument('--vesc-config',default='/ws/test_config/vesc.yaml')
    parser.add_argument('--prepare-only',action='store_true',help='Generate fixture and prepare solver offline, without ROS nodes')
    parser.add_argument('--reuse-prepared-reference',action='store_true',help='Load the reference created by the prepare-only step')
    from rclpy.utilities import remove_ros_args
    run(parser.parse_args(remove_ros_args()[1:]))


if __name__=='__main__':main()
