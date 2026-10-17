import importlib.util
import json
from pathlib import Path
import sqlite3
import sys

import pytest

from test_reference import configuration


def extractor():
    path = Path(__file__).with_name('bag.py')
    assert path.exists(), 'read-only multi-file bag extractor is missing'
    sys.path.insert(0,str(path.parent))
    spec=importlib.util.spec_from_file_location('vicon_bag',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def fixture_bag(folder, pose_type='geometry_msgs/msg/TransformStamped', samples=2):
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    folder.mkdir()
    topics=[('/mocap/car',pose_type),('/own/velocity','geometry_msgs/msg/TwistWithCovarianceStamped'),
            ('/own/acceleration','geometry_msgs/msg/AccelWithCovarianceStamped'),('/own/yaw','sensor_msgs/msg/Imu'),
            ('/odometry/filtered','nav_msgs/msg/Odometry'),('/own/status','diagnostic_msgs/msg/DiagnosticArray'),
            ('/own/cg/velocity','geometry_msgs/msg/TwistWithCovarianceStamped'),
            ('/own/cg/acceleration','geometry_msgs/msg/AccelWithCovarianceStamped'),('/own/cg/yaw','sensor_msgs/msg/Imu')]
    origin=1_800_000_000_000_000_000
    for file_index, indices in enumerate([range(0,samples//2),range(samples//2,samples)]):
        db=sqlite3.connect(folder/f'bag_{file_index}.db3')
        db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        db.executemany('INSERT INTO topics VALUES(?,?,?,?)',[(i+1,n,t,'cdr') for i,(n,t) in enumerate(topics)])
        for i in indices:
            source=origin+i*10_000_000
            for tid,(name,kind) in enumerate(topics,1):
                message=get_message(kind)();message.header.stamp.sec,message.header.stamp.nanosec=divmod(source,10**9)
                message.header.frame_id='vicon_world' if tid==1 else 'base_link'
                if '/cg/' in name:message.header.frame_id='center_of_mass'
                if tid==1:
                    if hasattr(message,'transform'):
                        message.transform.translation.x=.5*i*.01;message.transform.rotation.w=1.
                    else:
                        pose=message.pose.pose if hasattr(message.pose,'pose') else message.pose
                        pose.position.x=.5*i*.01;pose.orientation.w=1.
                elif kind=='geometry_msgs/msg/TwistWithCovarianceStamped':message.twist.twist.linear.x=.6
                elif kind=='geometry_msgs/msg/AccelWithCovarianceStamped':message.accel.accel.linear.x=.0
                elif kind=='sensor_msgs/msg/Imu':message.angular_velocity.z=0.
                elif tid==5:
                    message.header.frame_id='odom';message.child_frame_id='base_link'
                    message.pose.pose.orientation.w=1.;message.twist.twist.linear.x=.5
                else:
                    from diagnostic_msgs.msg import DiagnosticStatus,KeyValue
                    status=DiagnosticStatus();kv=KeyValue();kv.key='vx_valid';kv.value='true';status.values=[kv];message.status=[status]
                db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(tid,source+4_000_000,serialize_message(message)))
        db.commit();db.close()
    return origin


def bag_config(pose_type='geometry_msgs/msg/TransformStamped'):
    c=configuration();c['vicon'].update(topic='/mocap/car',message_type=pose_type)
    c['synthetic']=True
    c['streams']=[dict(name='own_rear_velocity',topic='/own/velocity',kind='velocity',reference='rear_axle'),
                  dict(name='own_rear_acceleration',topic='/own/acceleration',kind='acceleration',reference='rear_axle'),
                  dict(name='own_rear_yaw',topic='/own/yaw',kind='yaw_rate',reference='rear_axle'),
                  dict(name='robot_localization',topic='/odometry/filtered',kind='odometry',reference='rear_axle'),
                  dict(name='own_rear_status',topic='/own/status',kind='status',reference='rear_axle')]
    c['streams'] += [dict(name='own_cg_'+kind,topic='/own/cg/'+topic,kind=kind,reference='center_of_mass')
                     for kind,topic in [('velocity','velocity'),('acceleration','acceleration'),('yaw_rate','yaw')]]
    return c


@pytest.mark.parametrize('pose_type',['geometry_msgs/msg/TransformStamped','geometry_msgs/msg/PoseStamped',
    'geometry_msgs/msg/PoseWithCovarianceStamped','nav_msgs/msg/Odometry'])
def test_real_ros_cdr_split_sqlite_extraction(tmp_path,pose_type):
    m=extractor();folder=tmp_path/'bag';fixture_bag(folder,pose_type)
    before={p.name:p.read_bytes() for p in folder.iterdir()}
    out=tmp_path/'extracted';result=m.extract(folder,bag_config(pose_type),out)
    assert result['counts']['vicon']==2
    assert result['counts']['own_rear_velocity']==2
    assert len((out/'status.jsonl').read_text().splitlines())==2
    assert all(p.read_bytes()==before[p.name] for p in folder.iterdir())
    assert (out/'poses.csv').read_text().splitlines()[1].startswith('1800000000000000000,')
    assert 'robot_localization,rear_axle,vx,0.5,base_link' in (out/'estimates.csv').read_text()


def test_wrong_configured_pose_type_is_not_silently_decoded(tmp_path):
    m=extractor();folder=tmp_path/'bag';fixture_bag(folder)
    with pytest.raises(ValueError,match='type'):m.extract(folder,bag_config('geometry_msgs/msg/PoseStamped'),tmp_path/'out')
