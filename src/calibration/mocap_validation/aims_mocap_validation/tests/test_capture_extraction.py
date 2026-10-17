"""Actual CDR fields, source clock and native timing capture contracts."""
from pathlib import Path
import csv
import importlib
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def module(name):
    assert (ROOT/'aims_mocap_validation'/f'{name}.py').exists(),f'new canonical module {name} is missing'
    return importlib.import_module('aims_mocap_validation.'+name)


def test_extended_bag_preserves_raw_pose_command_feedback_and_gyro(tmp_path):
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from vesc_msgs.msg import VescCommand,VescStateStamped
    core=module('bag');template=module('configuration').load(None)
    template['vicon'].update(topic='/mocap/car',message_type='geometry_msgs/msg/PoseStamped')
    db=sqlite3.connect(tmp_path/'input.db3')
    db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT);CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    pose=PoseStamped();pose.pose.orientation.w=1.
    odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='livox_frame';odom.pose.pose.orientation.w=1.;odom.pose.pose.position.x=1.2;odom.twist.twist.linear.y=.3
    command=VescCommand();command.control_mode=command.MODE_CURRENT;command.target_current=12.;command.steering_angle=.1
    state=VescStateStamped();state.state.speed=345.;state.state.avg_iq=11.
    gyro=Imu();gyro.angular_velocity.z=.8
    messages=[('/mocap/car','geometry_msgs/msg/PoseStamped',pose),('/fastlio2/lio_odom','nav_msgs/msg/Odometry',odom),('/vesc/command','vesc_msgs/msg/VescCommand',command),('/sensors/core','vesc_msgs/msg/VescStateStamped',state),('/livox/imu_bias_corrected','sensor_msgs/msg/Imu',gyro)]
    origin=1_800_000_000_000_000_000
    for i,(topic,kind,m) in enumerate(messages,1):
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(origin+i,10**9)
        db.execute('INSERT INTO topics VALUES(?,?,?,?)',(i,topic,kind,'cdr'))
        db.execute('INSERT INTO messages VALUES(?,?,?,?)',(i,i,origin+10_000_000+i,serialize_message(m)))
    db.commit();db.close()
    result=core.extract(tmp_path/'input.db3',template,tmp_path/'data')
    def rows(name):
        with (tmp_path/'data'/name).open() as stream:return list(csv.DictReader(stream))
    assert rows('odometry.csv')[0]['source_ns']==str(origin+2)
    assert float(rows('odometry.csv')[0]['py'])==0.
    assert float(rows('commands.csv')[0]['target_current'])==12.
    assert float(rows('telemetry.csv')[0]['erpm'])==345.
    assert float(rows('gyro.csv')[0]['wz'])==pytest.approx(.8)
    assert 'commands.csv' in result['data_sha256']


def test_native_timing_capture_source_dds_clock_and_clean_stop(tmp_path):
    import hashlib
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64
    binary=Path(os.environ.get('MOCAP_CAPTURE_EXECUTABLE','/missing/mocap_capture'))
    assert binary.is_file(),'native timing collector is missing'
    rclpy.init();node=rclpy.create_node('mocap_timing_test_publisher')
    imu=node.create_publisher(Imu,'/synthetic/imu',10)
    unstamp=node.create_publisher(Float64,'/synthetic/unstamped',10)
    raw_digests=[]
    raw_subscription=node.create_subscription(Imu,'/synthetic/imu',lambda data:raw_digests.append(hashlib.sha256(data).hexdigest()),10,raw=True)
    output=tmp_path/'timing.csv';log=(tmp_path/'node.log').open('w')
    process=subprocess.Popen([str(binary),'--ros-args','-p',f'output_file:={output}','-p',"topics:=['/synthetic/imu','/synthetic/unstamped']",'-p',"types:=['sensor_msgs/msg/Imu','std_msgs/msg/Float64']",'-p','max_gaps_s:=[0.04,0.04]'],stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+10
        while not (imu.get_subscription_count()>=2 and unstamp.get_subscription_count()):
            assert process.poll() is None
            assert time.monotonic()<deadline
            rclpy.spin_once(node,timeout_sec=.02)
        message=Imu();stamp=node.get_clock().now().nanoseconds-2_000_000
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,10**9)
        message.angular_velocity.z=.3
        for _ in range(20):
            imu.publish(message);unstamp.publish(Float64(data=.5))
            rclpy.spin_once(node,timeout_sec=.02)
            time.sleep(.02)
        until=time.monotonic()+.2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        imu.publish(message) # source duplicate must remain visible, not overwritten
        time.sleep(.1)
        assert not any(topic in {'/vesc/command','/drive'} for topic,_ in node.get_publisher_names_and_types_by_node('mocap_capture','/'))
        process.send_signal(signal.SIGINT);assert process.wait(timeout=8)==0
        with output.open() as stream:rows=list(csv.DictReader(stream))
        own=[r for r in rows if r['topic']=='/synthetic/imu'];assert len(own)>=2
        assert all(int(r['source_ns'])==stamp for r in own)
        assert all(int(r['receive_system_ns'])>=stamp for r in own)
        assert all(int(r['receive_steady_ns'])>0 for r in own)
        assert raw_digests and all(r['cdr_sha256'] in raw_digests for r in own)
        assert all(not r['source_ns'] for r in rows if r['topic']=='/synthetic/unstamped')
        health=json.loads(output.with_suffix('.health.json').read_text())
        assert health['topics']['/synthetic/imu']['source_duplicates']==len(own)-1
        assert health['queue_drops']==0
    finally:
        if process.poll() is None:process.terminate();process.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()


def test_final_flush_failure_returns_nonzero(tmp_path):
    """A tiny file-size quota forces the buffered CSV flush to fail on shutdown."""
    import resource
    binary=Path(os.environ['MOCAP_CAPTURE_EXECUTABLE'])
    def limit():
        signal.signal(signal.SIGXFSZ,signal.SIG_IGN)
        resource.setrlimit(resource.RLIMIT_FSIZE,(1,1))
    output=tmp_path/'limited.csv'
    process=subprocess.Popen([str(binary),'--ros-args','-p',f'output_file:={output}','-p',"topics:=['/no_data']",'-p',"types:=['sensor_msgs/msg/Imu']"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,preexec_fn=limit)
    try:
        deadline=time.monotonic()+8
        while not output.exists():
            assert process.poll() is None
            assert time.monotonic()<deadline
            time.sleep(.02)
        time.sleep(.2)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=8)!=0
    finally:
        if process.poll() is None:process.kill();process.wait()


def test_legacy_record_api_forwards_native(monkeypatch,tmp_path):
    legacy=module('capture');session=module('session');calls=[]
    monkeypatch.setattr(session,'record',lambda *args:calls.append(args) or {'ok':True})
    assert legacy.record({'site':'unknown'},tmp_path,1)=={'ok':True}
    assert calls==[({'site':'unknown'},tmp_path,1)]


def test_record_ctrl_c_finalizes_real_bag_and_native_timing(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from aims_mocap_validation import configuration
    from aims_mocap_validation.bag import extract
    config=configuration.load(None)
    config['vrpn']['start_bridge']=False
    config['vicon'].update(topic='/mocap/car_test',message_type='geometry_msgs/msg/PoseStamped')
    config['streams']=[];config['fastlio']['streams']=[]
    selected=tmp_path/'config.json';selected.write_text(json.dumps(config))
    output=tmp_path/'capture';log=(tmp_path/'record.log').open('w')
    process=subprocess.Popen([sys.executable,'-m','aims_mocap_validation.cli','record','--config',str(selected),'--output',str(output)],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('mocap_record_fixture')
    pub=node.create_publisher(PoseStamped,'/mocap/car_test',10)
    def tick():
        msg=PoseStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.pose.orientation.w=1.;pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=.01)
    try:
        deadline=time.monotonic()+20
        while not list((output/'bag').glob('*.db3')):
            tick()
            assert process.poll() is None,(tmp_path/'record.log').read_text()
            assert time.monotonic()<deadline
        end=time.monotonic()+1.
        while time.monotonic()<end:tick()
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=15)==0,(tmp_path/'record.log').read_text()
        manifest=json.loads((output/'capture.json').read_text())
        assert manifest['collector_returncode']==0
        assert manifest['collector_health']['messages']>0
        assert (output/'bag/metadata.yaml').is_file()
        metadata=extract(output/'bag',config,tmp_path/'extracted')
        assert metadata['counts']['vicon']>0
    finally:
        if process.poll() is None:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('fault',['bag_finalization','truncated_health'])
def test_record_failure_retains_final_manifest(tmp_path,monkeypatch,fault):
    from aims_mocap_validation import session,configuration
    config=configuration.load(None);config['vrpn']['start_bridge']=False
    config['vicon'].update(topic='/mocap/absent',message_type='geometry_msgs/msg/PoseStamped')
    config['streams']=[];config['fastlio']['streams']=[]
    monkeypatch.setattr(session,'inspect_live',lambda *args:dict(type_mismatches={},graph={},nodes=[]))
    original_popen=subprocess.Popen
    def popen(args,**kwargs):
        if args[:3]==['ros2','bag','record']:
            code=7 if fault=='bag_finalization' else 0
            return original_popen([sys.executable,'-c',f'import signal,time,sys; signal.signal(signal.SIGINT,lambda *args:sys.exit({code})); time.sleep(30)'],**kwargs)
        return original_popen(args,**kwargs)
    monkeypatch.setattr(session.subprocess,'Popen',popen)
    output=tmp_path/'record'
    original_stop=session.stop
    def stop(process):
        original_stop(process)
        if fault=='truncated_health' and process and str(process.args[0]).endswith('mocap_capture'):
            (output/'timing.health.json').write_text('{')
    monkeypatch.setattr(session,'stop',stop)
    with pytest.raises(RuntimeError):session.record(config,output,.3)
    manifest=json.loads((output/'capture.json').read_text())
    assert manifest['end_wall_ns']>=manifest['start_wall_ns']
    assert manifest['error']
    if fault=='bag_finalization':assert manifest['returncode']==7
    else:assert manifest['collector_health_error']
