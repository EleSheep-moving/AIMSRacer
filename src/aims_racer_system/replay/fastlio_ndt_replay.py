#!/usr/bin/env python3
"""Replay raw sensors through new FAST-LIO/EKF/NDT, without command publishers."""
import argparse
import fcntl
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage



def ns(stamp):
    return stamp.sec*1000000000+stamp.nanosec


def quantiles(values):
    return dict(count=len(values),p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)),max=float(max(values))) if values else dict(count=0)


def children(pid):
    relation={}
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            try:
                parent=int((entry/'stat').read_text().rsplit(')',1)[1].split()[1])
                relation.setdefault(parent,[]).append(int(entry.name))
            except (OSError,ValueError,IndexError):
                pass
    found=[];todo=[pid]
    while todo:
        found.extend(relation.get(todo[-1],[]));todo.extend(relation.get(todo.pop(),[]))
    return found


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--map',type=Path);parser.add_argument('--seed',type=Path)
    parser.add_argument('--ndt-pause',type=float,default=0.)
    parser.add_argument('--pause-offset',type=float,default=20.)
    parser.add_argument('--lifecycle-deactivate',action='store_true')
    parser.add_argument('--use-initializer-cli',action='store_true',help='exercise the installed map/base_link initialization CLI')
    parser.add_argument('--max-seconds',type=float)
    parser.add_argument('--discovery-delay',type=float,default=5.,help='DDS discovery time before sensor publication, seconds')
    args=parser.parse_args()
    if not np.isfinite(args.discovery_delay) or args.discovery_delay<0:
        parser.error('--discovery-delay must be finite and nonnegative')
    if bool(args.map)!=bool(args.seed):
        parser.error('--map and --seed are required together; neither means local-only replay')
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    replay_lock=open('/tmp/aimsracer-replay-domain-'+domain+'.lock','w')
    try:
        fcntl.flock(replay_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another replay owns this ROS domain; wait for its complete shutdown')
    installed_hashes={}
    source_root=Path(__file__).resolve().parents[1]
    for relative in ('scripts/activate_ndt.py','scripts/relocalize_known_map.py','scripts/localization_monitor.py',
                     'scripts/localization_policy.py','scripts/localization_map_io.py',
                     'params/ndt_fastlio.yaml','params/localization_monitor.yaml'):
        source=source_root/relative
        installed=(Path(get_package_prefix('aims_racer_system'))/'lib/aims_racer_system'/Path(relative).name
                   if relative.startswith('scripts/') else Path(get_package_share_directory('aims_racer_system'))/relative)
        installed_hashes[relative]=hashlib.sha256(installed.read_bytes()).hexdigest()
        if source.exists() and hashlib.sha256(source.read_bytes()).hexdigest()!=installed_hashes[relative]:
            parser.error('installed artifact differs from source; rebuild overlay: '+relative)
    args.output.mkdir(parents=True,exist_ok=False)
    database=list(args.bag.glob('*.db3'))
    if len(database)!=1:
        parser.error('one SQLite bag database required')
    with sqlite3.connect(f'file:{database[0]}?mode=ro',uri=True) as connection:
        start,end=connection.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
        expected_raw_counts=dict(connection.execute('SELECT t.name,count(*) FROM messages m JOIN topics t ON t.id=m.topic_id GROUP BY t.name'))
    seed=json.loads(args.seed.read_text()) if args.seed else None
    logs=[];processes=[];paused=None;pause_start=None;injection=[]
    rclpy.init();node=Node('fastlio_ndt_replay_audit',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    lifecycle=node.create_client(ChangeState,'/lidar_localization/change_state')
    deactivate_future=None;initializer_process=None
    seed_sent=None;source_start=None;events=[];health=[];timings=[];odom_ages=[];cloud_ages=[];counts=Counter()
    eventfile=(args.output/'events.jsonl').open('w')
    def row(kind,message,values):
        entry=dict(kind=kind,source_ns=ns(message.header.stamp),ros_now_ns=node.get_clock().now().nanoseconds,
                   received=time.monotonic(),values=values)
        eventfile.write(json.dumps(entry)+'\n');return entry
    def alignment(message):
        for status in message.status:
            if status.name=='lidar_localization/anchor':
                events.append(row('anchor',message,{v.key:v.value for v in status.values}))
            elif status.name=='lidar_localization/timing':
                timings.append(row('native_timing',message,{v.key:v.value for v in status.values}))
    def health_update(message):
        for status in message.status:
            if status.name=='aims_racer_system/localization':
                health.append(row('health',message,{v.key:v.value for v in status.values}))
    def odometry(message):
        nonlocal seed_sent,source_start,initializer_process
        if source_start is None:source_start=ns(message.header.stamp)
        counts['ekf']+=1
        if counts['ekf']%20==0:
            p,q=message.pose.pose.position,message.pose.pose.orientation
            row('ekf_pose',message,dict(position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w]))
        odom_ages.append((node.get_clock().now().nanoseconds-ns(message.header.stamp))*1e-6)
        if seed and seed_sent is None and ns(message.header.stamp)-start>=1000000000 and pub.get_subscription_count()>0:
            initial=PoseWithCovarianceStamped();initial.header=message.header;initial.header.frame_id='map'
            p,q=initial.pose.pose.position,initial.pose.pose.orientation
            p.x,p.y,p.z=seed['position'];q.x,q.y,q.z,q.w=seed['orientation']
            initial.pose.covariance[0]=initial.pose.covariance[7]=.25;initial.pose.covariance[35]=.04
            if args.use_initializer_cli:
                from scipy.spatial.transform import Rotation
                angles=Rotation.from_quat(seed['orientation']).as_euler('xyz')
                command=['ros2','run','aims_racer_system','relocalize_known_map.py',str(args.map),'--pose-frame','base_link']
                for key,value in zip(('x','y','z','roll','pitch','yaw'),list(seed['position'])+list(angles)):
                    command.extend(['--'+key,str(value)])
                initializer_process=spawn(command+['--timeout','20','--ros-args','-p','use_sim_time:=true'],'initializer-cli')
            else:
                pub.publish(initial)
            seed_sent=ns(message.header.stamp)
            row('initialization',initial,dict(seed=seed))
    def cloud(message):
        counts['body_cloud']+=1
        cloud_ages.append((node.get_clock().now().nanoseconds-ns(message.header.stamp))*1e-6)
    def tf(message):
        for transform in message.transforms:
            edge=(transform.header.frame_id,transform.child_frame_id)
            if edge in [('map','odom'),('odom','base_link')]:
                counts['tf_'+edge[0]+'_'+edge[1]]+=1
    subscriptions=[node.create_subscription(Odometry,'/odometry/filtered',odometry,1000),
        node.create_subscription(PointCloud2,'/fastlio2/body_cloud',cloud,qos_profile_sensor_data),
        node.create_subscription(DiagnosticArray,'/localization/anchor_status',alignment,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(DiagnosticArray,'/localization/status',health_update,100),
        node.create_subscription(TFMessage,'/tf',tf,100)]
    executor=SingleThreadedExecutor();executor.add_node(node)
    def spawn(command,name):
        log=(args.output/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    try:
        audit_path=args.output/'tf-authorities.jsonl'
        audit=spawn(['ros2','run','aims_racer_system','tf_authority_audit',str(audit_path),'--ros-args','-p','use_sim_time:=true'],'tf-audit')
        stack_command=['ros2','launch','aims_racer_system','fastlio_ndt_replay.launch.py',
            'known_map:='+str(bool(args.map)).lower(),
            'timing_trace_path:='+str((args.output/'fastlio-trace.csv').resolve())]
        if args.map:
            stack_command.append('map_file:='+str(args.map))
        stack=spawn(stack_command,'stack')
        until=time.monotonic()+8.
        while time.monotonic()<until:
            if stack.poll() is not None:
                raise RuntimeError('replay launch exited; inspect stack.log')
            executor.spin_once(timeout_sec=.05)
        playback=spawn(['ros2','bag','play',str(args.bag),'--delay',str(args.discovery_delay),'--clock','200','--rate','1.0','--topics',
            '/livox/lidar','/livox/imu','/rear_axle/wheel_odom'],'bag')
        deadline=time.monotonic()+(args.max_seconds if args.max_seconds is not None else (end-start)*1e-9+args.discovery_delay+25.)
        while time.monotonic()<deadline and playback.poll() is None:
            if stack.poll() is not None:
                raise RuntimeError('replay launch exited; inspect stack.log')
            executor.spin_once(timeout_sec=.01)
            elapsed=(node.get_clock().now().nanoseconds-start)*1e-9
            if args.lifecycle_deactivate and not injection and elapsed>=args.pause_offset:
                request=ChangeState.Request();request.transition.id=Transition.TRANSITION_DEACTIVATE
                deactivate_future=lifecycle.call_async(request)
                injection.append(dict(kind='lifecycle_deactivate',ros_offset=elapsed,received=time.monotonic()))
            if args.ndt_pause>0 and not injection and elapsed>=args.pause_offset:
                for pid in children(stack.pid):
                    try:command=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')[0]
                    except OSError:continue
                    if command.endswith(b'/lidar_localization_node'):
                        os.kill(pid,signal.SIGSTOP);paused=pid;pause_start=time.monotonic()
                        injection.append(dict(kind='ndt_pause',ros_offset=elapsed,received=pause_start,pid=pid));break
            if paused is not None and time.monotonic()-pause_start>=args.ndt_pause:
                os.kill(paused,signal.SIGCONT);paused=None
                injection.append(dict(kind='ndt_resume',ros_offset=elapsed,received=time.monotonic()))
        until=time.monotonic()+1.
        while time.monotonic()<until:executor.spin_once(timeout_sec=.02)
        if audit.poll() is not None:
            raise RuntimeError('TF authority auditor exited; inspect tf-audit.log')
        authorities={}
        for line in audit_path.read_text().splitlines():
            item=json.loads(line)
            if item.get('event')=='tf':
                authorities.setdefault(item['frame_id']+'/'+item['child_frame_id'],set()).add(item['publisher_gid'])
        accepted=[e for e in events if e['values'].get('anchor_committed')=='true']
        result=dict(installed_artifact_sha256=installed_hashes,bag=str(args.bag),map=str(args.map),map_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest() if args.map else None,
            expected_raw_counts=expected_raw_counts,full_bag_playback=playback.poll()==0,
            intentionally_partial=args.max_seconds is not None,discovery_delay_sec=args.discovery_delay,
            seed=seed,seed_sent_ns=seed_sent,counts=dict(counts),tf_authorities={edge:len(gids) for edge,gids in authorities.items()},
            accepted=len(accepted),rejection_reasons=dict(Counter(e['values'].get('reason') for e in events if e['values'].get('anchor_committed')!='true')),
            ndt_processing_ms=quantiles([float(e['values']['scan_processing_time_sec'])*1000 for e in timings]),
            attempt_alignment_ms=quantiles([float(e['values']['alignment_time_sec'])*1000 for e in events if 'alignment_time_sec' in e['values']]),
            alignment_ms=quantiles([float(e['values']['alignment_time_sec'])*1000 for e in accepted]),
            trusted_source_age_ms=quantiles([(e['ros_now_ns']-e['source_ns'])*1e-6 for e in accepted]),
            ekf_age_ms=quantiles(odom_ages),body_cloud_age_ms=quantiles(cloud_ages),
            independent_quality_samples=len({e['values']['quality_stamp_ns'] for e in health if 'inlier_fraction' in e['values']}),
            health_states=dict(Counter(e['values']['state'] for e in health)),injection=injection,
            initializer_cli_exit=initializer_process.poll() if initializer_process else None,
            lifecycle_deactivated=bool(deactivate_future and deactivate_future.done() and deactivate_future.result().success),
            first_accepted_ns=accepted[0]['source_ns'] if accepted else None,last_accepted_ns=accepted[-1]['source_ns'] if accepted else None)
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        if counts['ekf']==0 or counts['body_cloud']==0 or authorities.get('odom/base_link') is None:
            raise RuntimeError('local graph produced no complete local state')
        if any(len(gids)!=1 for gids in authorities.values()):
            raise RuntimeError('dynamic localization TF edge has multiple publishers')
        if args.use_initializer_cli and (initializer_process is None or initializer_process.poll()!=0):
            raise RuntimeError('initialization CLI did not complete successfully')
        if args.map and (not accepted or authorities.get('map/odom') is None):
            raise RuntimeError('no trusted global localization established')
    finally:
        if paused is not None:os.kill(paused,signal.SIGCONT)
        for process in reversed(processes):
            owned=children(process.pid)
            for pid in owned:
                try:os.kill(pid,signal.SIGINT)
                except ProcessLookupError:pass
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                for pid in owned:
                    try:os.kill(pid,signal.SIGTERM)
                    except ProcessLookupError:pass
                process.terminate();process.wait(timeout=3.)
        eventfile.close()
        for log in logs:log.close()
        executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
