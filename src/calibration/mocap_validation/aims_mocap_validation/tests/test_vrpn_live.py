"""Real loopback VRPN server -> pinned bridge -> ROS pose, synthetic only."""
from pathlib import Path
import os
import subprocess
import time
import json
import signal
import pytest


def test_vrpn_keeps_server_timestamp_and_coordinates(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    binary=Path(os.environ.get('MOCAP_VRPN_FIXTURE','/missing/vrpn_fixture_server'))
    assert binary.is_file(),'VRPN server fixture must be built'
    rclpy.init();node=rclpy.create_node('vrpn_loopback_probe');poses=[]
    sub=node.create_subscription(PoseStamped,'/vrpn_mocap/synthetic_car/pose',poses.append,qos_profile_sensor_data)
    port=43883
    config=tmp_path/'site.json';config.write_text(json.dumps(dict(vrpn=dict(server='127.0.0.1',port=port,use_vrpn_timestamps=True,update_freq=200.))))
    with (tmp_path/'server.log').open('w') as serverlog,(tmp_path/'bridge.log').open('w') as bridgelog:
        server=subprocess.Popen([str(binary),str(port)],stdout=serverlog,stderr=subprocess.STDOUT)
        bridge=subprocess.Popen(['ros2','launch','aims_mocap_validation','bridge.launch.py','config:='+str(config)],stdout=bridgelog,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+12
            while len(poses)<5:
                assert server.poll() is None and bridge.poll() is None
                assert time.monotonic()<deadline,(tmp_path/'bridge.log').read_text()
                rclpy.spin_once(node,timeout_sec=.05)
            message=poses[-1]
            assert message.pose.position.x==pytest.approx(1.25)
            source=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            assert 1.5<(node.get_clock().now().nanoseconds-source)*1e-9<3.
        finally:
            os.killpg(bridge.pid,signal.SIGINT);bridge.wait(timeout=8)
            server.terminate();server.wait(timeout=5);node.destroy_node();rclpy.shutdown()
