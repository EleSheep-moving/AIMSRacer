"""Read-only sqlite3 ROS2 extraction, preserving split-bag receipt order and CDR hashes."""
import csv
from contextlib import ExitStack
import hashlib
import heapq
import json
from pathlib import Path
import shutil
import sqlite3

from .analyze import sha

POSE_TYPES = {'geometry_msgs/msg/TransformStamped','geometry_msgs/msg/PoseStamped',
              'geometry_msgs/msg/PoseWithCovarianceStamped','nav_msgs/msg/Odometry'}
KINDS = dict(velocity='geometry_msgs/msg/TwistWithCovarianceStamped',
    acceleration='geometry_msgs/msg/AccelWithCovarianceStamped',yaw_rate='sensor_msgs/msg/Imu',
    odometry='nav_msgs/msg/Odometry',status='diagnostic_msgs/msg/DiagnosticArray',
    command='vesc_msgs/msg/VescCommand',telemetry='vesc_msgs/msg/VescStateStamped',gyro='sensor_msgs/msg/Imu')


def source_ns(message):
    return int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)


def topic_specifications(config, external_vicon=False):
    result = {}
    for item in config['streams']:
        if item['kind'] not in KINDS: raise ValueError('unsupported stream kind')
        if item['topic'] in result: raise ValueError('duplicate stream topic')
        result[item['topic']] = dict(item,message_type=KINDS[item['kind']])
    for item in config.get('fastlio',{}).get('streams',[]):
        topic=item['topic']
        if topic in result:
            if result[topic]['message_type']!='nav_msgs/msg/Odometry':raise ValueError('FastLIO topic type conflict')
            result[topic]['fastlio_stream']=item['name']
        else:result[topic]=dict(name=item['name'],topic=topic,kind='odometry',reference='rear_axle',
            message_type='nav_msgs/msg/Odometry',raw_only=True,fastlio_stream=item['name'])
    if 'actuator' in config:
        for topic,kind,name in [(config['actuator'].get('command_topic','/vesc/command'),'command','commands'),
                                (config['actuator'].get('telemetry_topic','/sensors/core'),'telemetry','telemetry'),
                                ('/livox/imu_bias_corrected','gyro','raw_gyro')]:
            if topic in result:raise ValueError('auxiliary topic conflict: '+topic)
            result[topic]=dict(name=name,topic=topic,kind=kind,message_type=KINDS[kind])
    if not external_vicon:
        vicon = config['vicon']
        if not vicon.get('topic') or vicon.get('message_type') not in POSE_TYPES:
            raise ValueError('configure Vicon topic and supported pose message_type, or supply external Vicon CSV')
        if vicon['topic'] in result: raise ValueError('Vicon topic conflicts with estimate stream')
        result[vicon['topic']] = dict(name='vicon',topic=vicon['topic'],kind='pose',message_type=vicon['message_type'])
    return result


def extract(bag, config, output, vicon_csv=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag,output = Path(bag),Path(output)
    if output.exists(): raise ValueError('output directory must not exist')
    paths = [bag] if bag.is_file() and bag.suffix=='.db3' else sorted(bag.glob('*.db3'))
    if not paths: raise ValueError('expected sqlite3 .db3 file or bag directory; record with --storage sqlite3')
    specs = topic_specifications(config,vicon_csv is not None)
    before = {str(p):dict(bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns) for p in paths}
    connections, iterators, discovered = [], [], {}

    def records(db, topics, index):
        if not topics: return
        query='SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?'*len(topics))+') ORDER BY timestamp,id'
        for row_id,tid,receipt,cdr in db.execute(query,list(topics)):
            yield int(receipt),index,int(row_id),topics[tid],cdr

    try:
        for index,path in enumerate(paths):
            db=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True);connections.append(db)
            topics={}
            for tid,name,kind,serialization in db.execute('SELECT id,name,type,serialization_format FROM topics'):
                if name not in specs: continue
                if kind != specs[name]['message_type'] or serialization != 'cdr':
                    raise ValueError('topic type/serialization mismatch: '+name)
                discovered[name]=kind;topics[tid]=name
            iterators.append(records(db,topics,index))
        output.mkdir(parents=True)
        counts={item['name']:0 for item in specs.values()};digest=hashlib.sha256()
        with ExitStack() as stack:
            pose_file=stack.enter_context((output/'poses.csv').open('w'))
            estimate_file=stack.enter_context((output/'estimates.csv').open('w'))
            status_file=stack.enter_context((output/'status.jsonl').open('w'))
            extras={}
            for name,fields in {
                'odometry.csv':['source_ns','receive_ns','stream','frame_id','child_frame_id','px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'],
                'commands.csv':['source_ns','receive_ns','mode','steering_angle','target_speed_mps','target_current','target_duty','target_brake_current'],
                'telemetry.csv':['source_ns','receive_ns','erpm','current_motor','avg_iq'],
                'gyro.csv':['source_ns','receive_ns','wx','wy','wz']}.items():
                writer=csv.writer(stack.enter_context((output/name).open('w')));writer.writerow(fields);extras[name]=writer
            pose=csv.writer(pose_file);pose.writerow(['source_ns','receive_ns','px','py','pz','qx','qy','qz','qw','valid'])
            estimate=csv.writer(estimate_file);estimate.writerow(['source_ns','receive_ns','stream','reference','quantity','value','frame_id'])
            types={name:get_message(spec['message_type']) for name,spec in specs.items()}
            for receipt,index,row_id,name,cdr in heapq.merge(*iterators,key=lambda row:row[:3]):
                item=specs[name];message=deserialize_message(cdr,types[name]);stamp=source_ns(message)
                counts[item['name']]+=1
                digest.update(f'{index}:{row_id}:{receipt}:{name}\n'.encode());digest.update(cdr)
                if item['kind']=='command':
                    extras['commands.csv'].writerow([stamp,receipt,message.control_mode,message.steering_angle,message.target_speed_mps,message.target_current,message.target_duty,message.target_brake_current]);continue
                if item['kind']=='telemetry':
                    extras['telemetry.csv'].writerow([stamp,receipt,message.state.speed,message.state.current_motor,message.state.avg_iq]);continue
                if item['kind']=='gyro':
                    w=message.angular_velocity;extras['gyro.csv'].writerow([stamp,receipt,w.x,w.y,w.z]);continue
                if item['kind']=='odometry':
                    p,q=message.pose.pose.position,message.pose.pose.orientation;v,w=message.twist.twist.linear,message.twist.twist.angular
                    extras['odometry.csv'].writerow([stamp,receipt,item.get('fastlio_stream',item['name']),message.header.frame_id,message.child_frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z])
                    if item.get('raw_only'):continue
                if item['kind']=='pose':
                    if hasattr(message,'transform'):p,q=message.transform.translation,message.transform.rotation
                    else:
                        value=message.pose.pose if hasattr(message.pose,'pose') else message.pose;p,q=value.position,value.orientation
                    # Standard pose types do not carry occlusion flags. This only
                    # establishes numerical availability; tracking semantics remain metadata.
                    import math
                    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                    valid=all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:])-1)<=.002001
                    pose.writerow([stamp,receipt,*values,int(valid)])
                elif item['kind']=='status':
                    for status in message.status:
                        status_file.write(json.dumps(dict(source_ns=stamp,receive_ns=receipt,stream=item['name'],
                            name=status.name,fields={pair.key:pair.value for pair in status.values}))+'\n')
                else:
                    frame=message.child_frame_id if item['kind']=='odometry' else message.header.frame_id
                    if item['kind'] in ('velocity','odometry'):
                        twist=message.twist.twist;values=dict(vx=twist.linear.x,vy=twist.linear.y)
                        if item['kind']=='odometry':values['yaw_rate']=twist.angular.z
                    elif item['kind']=='acceleration':values=dict(ax=message.accel.accel.linear.x,ay=message.accel.accel.linear.y)
                    else:
                        if message.angular_velocity_covariance[0]<0:continue
                        values=dict(yaw_rate=message.angular_velocity.z)
                    for quantity,value in values.items():
                        estimate.writerow([stamp,receipt,item['name'],item['reference'],quantity,value,frame])
        if vicon_csv is not None:
            shutil.copy2(vicon_csv,output/'poses.csv')
            from .analyze import load_poses
            counts['vicon_external']=len(load_poses(output/'poses.csv')['source_ns'])
        for p in paths:
            if before[str(p)] != dict(bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns):
                raise RuntimeError('source bag changed during read: '+str(p))
        metadata=dict(source=str(bag.resolve()),synthetic=bool(config.get('synthetic',False)),bag_files_sha256={str(p):sha(p) for p in paths},
            file_metadata=before,counts=counts,missing_topics=sorted(set(specs)-set(discovered)),
            selected_cdr_sha256=digest.hexdigest(),hash_protocol='receipt,file-index,row-id merge; index:id:receipt:topic newline then CDR',
            selected_topics=specs,config=config,extractor_sha256=sha(Path(__file__)),
            data_sha256={name:sha(output/name) for name in ['poses.csv','estimates.csv','status.jsonl','odometry.csv','commands.csv','telemetry.csv','gyro.csv']},
            external_vicon_csv_sha256=sha(vicon_csv) if vicon_csv else None,
            tracking_note='pose numerical validity only; bridge occlusion semantics must be declared or external CSV validity supplied')
        (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        return metadata
    finally:
        for db in connections:db.close()
