#!/usr/bin/env python3
"""Isolated raw-bag replay. Run inside the pinned Humble replay environment.

Only raw Livox, IMU and recorded wheel velocity enter the live graph. Historical
map/LIO poses supply one initial seed, and are retained only for offline comparison.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

from ndt_replay_metrics import summarize, input_freshness


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def vector(p):
    return [p.x, p.y, p.z]


def quaternion(q):
    return [q.x, q.y, q.z, q.w]


def bag_information(bag):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import numpy as np
    from scipy.spatial.transform import Rotation
    database = sorted(bag.glob('*.db3'))
    if len(database) != 1:
        raise ValueError('This harness expects a single SQLite database; split bags need a rosbag2 reader.')
    with sqlite3.connect(f'file:{database[0]}?mode=ro', uri=True) as con:
        topics = {name:(ident,typ) for ident,name,typ in con.execute('SELECT id,name,type FROM topics')}
        start,end = con.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
        target = start+int(1e9)
        def near(name):
            ident,typ = topics[name]
            row = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY abs(timestamp-?) LIMIT 1', (ident,target)).fetchone()
            return row[0], deserialize_message(row[1],get_message(typ))
        seed = None
        seed_error = 'no_recorded_rear_lio'
        if '/rear_axle/lio_odom' in topics:
            odom_t,odom = near('/rear_axle/lio_odom')
            tf_topic = '/localizer/raw_tf' if '/localizer/raw_tf' in topics else '/tf'
            ident,typ = topics[tf_topic]
            selected = None
            for ts,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY abs(timestamp-?)', (ident,target-int(1e9),target+int(1e9),target)):
                msg = deserialize_message(data,get_message(typ))
                selected = next((t for t in msg.transforms if t.header.frame_id=='map' and t.child_frame_id=='odom'),None)
                if selected:
                    tf_t = ts
                    break
            if selected is None:
                seed_error = 'no_recorded_map_odom_near_seed'
                selected = None
            if odom.header.frame_id!='odom' or odom.child_frame_id!='base_link':
                raise ValueError('Historical rear-axle seed frame contract mismatch.')
            if selected is not None:
                map_r = Rotation.from_quat(quaternion(selected.transform.rotation))
                odom_r = Rotation.from_quat(quaternion(odom.pose.pose.orientation))
                position = np.array(vector(selected.transform.translation))+map_r.apply(vector(odom.pose.pose.position))
                seed = dict(position=position.tolist(), orientation=(map_r*odom_r).as_quat().tolist(),
                            target_source_sec=target*1e-9, provenance=dict(odom_topic='/rear_axle/lio_odom',
                            tf_topic=tf_topic, odom_record_sec=odom_t*1e-9, tf_record_sec=tf_t*1e-9,
                            odom_stamp_sec=seconds(odom.header.stamp), tf_stamp_sec=seconds(selected.header.stamp)))
        counts = {name:con.execute('SELECT count(*) FROM messages WHERE topic_id=?',(ident,)).fetchone()[0] for name,(ident,typ) in topics.items() if name in ('/livox/lidar','/livox/imu','/rear_axle/wheel_odom')}
    return dict(start_sec=start*1e-9,end_sec=end*1e-9,duration_sec=(end-start)*1e-9,counts=counts,seed=seed,seed_error=seed_error if seed is None else None)


def descendants(pid):
    # Read only the owned launch process tree; never signal unrelated ROS nodes.
    children = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            data = (entry/'stat').read_text().rsplit(')',1)[1].split()
            children.setdefault(int(data[1]),[]).append(int(entry.name))
        except (OSError,ValueError,IndexError):
            continue
    found, queue = [], [pid]
    while queue:
        current=queue.pop()
        for child in children.get(current,[]):
            found.append(child); queue.append(child)
    return found


def ndt_pid(launch_pid):
    for pid in descendants(launch_pid):
        try:
            command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode()
            if '/lib/lidar_localization_ros2/lidar_localization_node ' in command:
                return pid
        except OSError:
            pass
    raise RuntimeError('Owned native NDT process not found for fault injection.')


def terminate(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--local-only',action='store_true')
    parser.add_argument('--initial-pose',type=Path,help='Explicit fixed map/base seed JSON; provenance preserved, never ongoing input.')
    parser.add_argument('--zero-wheel',action='store_true',help='Stationary D only: explicit synthetic zero wheel, never a measured-wheel claim.')
    parser.add_argument('--fault',choices=['none','lidar_drop','ndt_pause'],default='none')
    parser.add_argument('--fault-duration',type=float,default=.5)
    parser.add_argument('--fault-offset',type=float,default=20.)
    args=parser.parse_args()
    if args.fault_duration<=0 or args.fault_offset<0:
        parser.error('Fault duration must be positive and offset non-negative.')
    info=bag_information(args.bag)
    if args.initial_pose:
        info['seed']=json.loads(args.initial_pose.read_text())
        info['seed']['target_source_sec']=info['start_sec']+1.
        if len(info['seed']['position'])!=3 or len(info['seed']['orientation'])!=4:
            parser.error('Initial pose requires position[3] and orientation[4].')
        if not all(math.isfinite(v) for v in info['seed']['position']+info['seed']['orientation']):
            parser.error('Initial pose must be finite.')
        if abs(sum(v*v for v in info['seed']['orientation'])-1.)>.02:
            parser.error('Initial pose quaternion must be unit length.')
    if not args.local_only and info['seed'] is None:
        parser.error('Known-map replay needs a historical initial seed; D must use --local-only.')
    if '/rear_axle/wheel_odom' not in info['counts'] and not args.zero_wheel:
        parser.error('Bag has no measured wheel; only stationary testing allows explicit --zero-wheel.')
    if args.zero_wheel and '/rear_axle/wheel_odom' in info['counts']:
        parser.error('Synthetic zero wheel cannot replace recorded motion measurements.')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output directory must be new or empty to preserve evidence.')
    args.output.mkdir(parents=True,exist_ok=True)
    source_root=Path(__file__).resolve().parents[1]
    source_files={str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in source_root.rglob('*') if p.is_file() and p.suffix in ('.py','.hpp','.cpp','.yaml','.patch','.sh','.xml') and '__pycache__' not in p.parts and 'docker' not in p.parts and 'test' not in p.parts}
    dependency_identity={}
    for name in ('lidar_localization_ros2','ndt_omp_ros2','livox_driver_source'):
        location=Path('/deps')/name
        if (location/'.git').exists():
            head=subprocess.check_output(['git','-c','safe.directory='+str(location),'-C',str(location),'rev-parse','HEAD'],text=True).strip()
            diff=subprocess.check_output(['git','-c','safe.directory='+str(location),'-C',str(location),'diff','HEAD','--binary'])
            dependency_identity[name]=dict(commit=head,diff_sha256=hashlib.sha256(diff).hexdigest(),diff_bytes=len(diff))
    try:
        source_git_head=subprocess.check_output(['git','-c','safe.directory=/repo','-C',str(source_root),'rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL).strip()
    except subprocess.CalledProcessError:
        source_git_head=os.environ.get('SOURCE_GIT_HEAD','worktree_git_metadata_not_mounted; source_files_sha256_identifies_candidate')
    provenance=dict(source_git_head=source_git_head,source_files_sha256=source_files,dependency_identity=dependency_identity,arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                    bag=info,map_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),
                    environment={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS')})
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,PointCloud2
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool

    output=(args.output/'events.jsonl').open('w')
    rows=[]
    seed_sent=None
    fault_marker=None
    rclpy.init()
    node=Node('ndt_replay_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    last_input={}
    last_anchor=[None]
    counts=dict(lidar=0,imu=0,wheel=0,deskew=0)
    initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    wheel_pub=node.create_publisher(Odometry,'/rear_axle/wheel_odom',10) if args.zero_wheel else None
    relay_pub=node.create_publisher(CustomMsg,'/livox/lidar',qos_profile_sensor_data) if args.fault=='lidar_drop' else None
    def record(stream,stamp=None,**fields):
        now=node.get_clock().now().nanoseconds*1e-9
        if now<=0:
            return
        row=dict(stream=stream,now=now,mono=time.monotonic(),**fields)
        if stamp is not None:
            row['stamp']=stamp
        rows.append(row)
        output.write(json.dumps(row,allow_nan=False)+'\n')
    def raw(kind,msg):
        counts[kind]+=1
        if kind=='imu':
            return  # raw source count; freshness comes from validated transformed gyro below.
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link' and math.isfinite(msg.twist.twist.linear.x) and math.isfinite(msg.twist.covariance[0]) and msg.twist.covariance[0]>=0.:
            last_input[kind]=seconds(msg.header.stamp)
    def gyro(msg):
        if msg.header.frame_id=='base_link' and all(math.isfinite(v) for v in (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)) and math.isfinite(msg.angular_velocity_covariance[8]) and msg.angular_velocity_covariance[8]>0.:
            last_input['imu']=seconds(msg.header.stamp)
    def lidar(msg):
        nonlocal fault_marker
        counts['lidar']+=1
        if relay_pub:
            now=node.get_clock().now().nanoseconds*1e-9
            begin=info['start_sec']+args.fault_offset
            if begin<=now<begin+args.fault_duration:
                if fault_marker is None:
                    fault_marker=dict(kind='lidar_drop',start_source_sec=begin,end_source_sec=begin+args.fault_duration,duration_sec=args.fault_duration,dropped=0)
                fault_marker['dropped']+=1
                return
            relay_pub.publish(msg)
    def ekf(msg):
        nonlocal seed_sent
        now=node.get_clock().now().nanoseconds*1e-9
        age=input_freshness(now,last_input)
        p=msg.pose.pose
        record('ekf',seconds(msg.header.stamp),input_age=age,position=vector(p.position),orientation=quaternion(p.orientation),yaw=yaw(p.orientation),vx=msg.twist.twist.linear.x,wz=msg.twist.twist.angular.z)
        seed=info['seed']
        if not args.local_only and seed_sent is None and now>=seed['target_source_sec'] and initial_pub.get_subscription_count()>0:
            initial=PoseWithCovarianceStamped()
            initial.header.stamp=node.get_clock().now().to_msg();initial.header.frame_id='map'
            initial.pose.pose.position.x,initial.pose.pose.position.y,initial.pose.pose.position.z=seed['position']
            initial.pose.pose.orientation.x,initial.pose.pose.orientation.y,initial.pose.pose.orientation.z,initial.pose.pose.orientation.w=seed['orientation']
            initial.pose.covariance[0]=initial.pose.covariance[7]=.25
            initial.pose.covariance[35]=(.1)**2
            initial_pub.publish(initial)
            seed_sent=now
            record('initialization',now,pose=seed)
    def finite(value):
        try:
            number=float(value)
            return number if math.isfinite(number) else None
        except (ValueError,TypeError):
            return None
    def diag(stream,msg):
        for status in msg.status:
            values={kv.key:kv.value for kv in status.values}
            level=status.level[0] if isinstance(status.level,bytes) else status.level
            extra={}
            if stream=='ndt':
                extra=dict(accepted=level==0 and status.message=='ok',
                           fitness=finite(values.get('fitness_score')),
                           processing_ms=(finite(values.get('alignment_time_sec')) or 0.)*1000)
            record(stream,seconds(msg.header.stamp),values=values,level=level,message=status.message,**extra)
            if stream=='health':
                anchor=finite(values.get('last_trustworthy_scan_sec'))
                if anchor is not None and anchor!=last_anchor[0]:
                    last_anchor[0]=anchor
                    record('anchor',anchor,accepted=True)
    def tf(msg):
        for trans in msg.transforms:
            if (trans.header.frame_id,trans.child_frame_id)==('map','odom'):
                record('map_tf',seconds(trans.header.stamp),position=vector(trans.transform.translation),orientation=quaternion(trans.transform.rotation),yaw=yaw(trans.transform.rotation))
    def cloud(msg):
        counts['deskew']+=1
        record('cloud',seconds(msg.header.stamp),points=msg.width*msg.height)
    subscriptions=[node.create_subscription(CustomMsg,'/ndt_replay/raw_lidar' if relay_pub else '/livox/lidar',lidar,qos_profile_sensor_data,raw=True),
        node.create_subscription(Imu,'/livox/imu',lambda m:raw('imu',m),qos_profile_sensor_data),
        node.create_subscription(Imu,'/rear_axle/imu',gyro,qos_profile_sensor_data),
        node.create_subscription(Odometry,'/rear_axle/wheel_odom',lambda m:raw('wheel',m),qos_profile_sensor_data),
        node.create_subscription(Odometry,'/odometry/filtered',ekf,100),
        node.create_subscription(DiagnosticArray,'/localization/ndt_status',lambda m:diag('ndt',m),100),
        node.create_subscription(DiagnosticArray,'/localization/status',lambda m:diag('health',m),100),
        node.create_subscription(DiagnosticArray,'/localization/deskew_status',lambda m:diag('deskew_status',m),100),
        node.create_subscription(TFMessage,'/tf',tf,100),
        node.create_subscription(PointCloud2,'/localization/deskewed_cloud',cloud,qos_profile_sensor_data),
        node.create_subscription(Bool,'/localization/map_valid',lambda m:record('map_valid',value=m.data),10)]
    def zero_wheel():
        now=node.get_clock().now()
        if now.nanoseconds<info['start_sec']*1e9:
            return
        msg=Odometry();msg.header.stamp=now.to_msg();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
        msg.twist.covariance[0]=.04
        wheel_pub.publish(msg)
    zero_timer=node.create_timer(.02,zero_wheel) if wheel_pub else None
    stack=None;player=None;stopped_pid=None;stop_until=None
    stack_log=(args.output/'stack.log').open('w'); player_log=(args.output/'player.log').open('w')
    try:
        stack=subprocess.Popen(['ros2','launch','aims_racer_system','wheel_imu_ndt_localization.launch.py',f'map_file:={args.map}','use_sim_time:=true',f'enable_ndt:={str(not args.local_only).lower()}'],stdout=stack_log,stderr=subprocess.STDOUT,start_new_session=True)
        # Lifecycle setup loads the prior map before live input starts.
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if stack.poll() is not None:
                raise RuntimeError('Localization launch exited during startup.')
        command=['ros2','bag','play',str(args.bag),'--topics','/livox/lidar','/livox/imu']
        if not args.zero_wheel:command+=['/rear_axle/wheel_odom']
        command+=['--clock','1000','--rate','1.0','--delay','2.0']
        if relay_pub:command+=['--remap','/livox/lidar:=/ndt_replay/raw_lidar']
        player=subprocess.Popen(command,stdout=player_log,stderr=subprocess.STDOUT,start_new_session=True)
        timeout=time.monotonic()+info['duration_sec']+45.
        while player.poll() is None:
            rclpy.spin_once(node,timeout_sec=.002)
            now=node.get_clock().now().nanoseconds*1e-9
            if args.fault=='ndt_pause' and fault_marker is None and now>=info['start_sec']+args.fault_offset:
                stopped_pid=ndt_pid(stack.pid)
                os.kill(stopped_pid,signal.SIGSTOP)
                stop_until=time.monotonic()+args.fault_duration
                fault_marker=dict(kind='ndt_pause',start_source_sec=now,end_source_sec=now+args.fault_duration,duration_sec=args.fault_duration)
                record('fault_begin',now,kind=args.fault)
            if stopped_pid is not None and time.monotonic()>=stop_until:
                os.kill(stopped_pid,signal.SIGCONT);stopped_pid=None
                fault_marker['end_source_sec']=now
                record('fault_end',now,kind=args.fault)
            if stack.poll() is not None:raise RuntimeError('Localization launch exited during playback.')
            if time.monotonic()>timeout:raise TimeoutError('Playback exceeded bag duration +45sec.')
        if player.returncode:raise RuntimeError(f'Bag player exit {player.returncode}')
        drain=time.monotonic()+1.
        while time.monotonic()<drain:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        if stopped_pid is not None:
            try:os.kill(stopped_pid,signal.SIGCONT)
            except ProcessLookupError:pass
        terminate(player);terminate(stack)
        output.close();stack_log.close();player_log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    result=summarize(rows,ndt_enabled=not args.local_only,initialization_time=seed_sent,injection=fault_marker)
    result.update(input_counts=counts,initialization_time=seed_sent,synthetic_zero_wheel=args.zero_wheel)
    if args.fault!='none' and fault_marker is None:
        result['gates']['fault_injected']=False;result['timing_pass']=False
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (args.output/'ekf.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['source_sec','receipt_source_sec','x','y','z','yaw','vx','wz','input_age_sec'])
        for row in rows:
            if row['stream']=='ekf':writer.writerow([row['stamp'],row['now'],*row['position'],row['yaw'],row['vx'],row['wz'],row['input_age']])
    print(json.dumps(dict(output=str(args.output),timing_pass=result['timing_pass'],gates=result['gates'],input_counts=counts)))


if __name__=='__main__':
    main()
