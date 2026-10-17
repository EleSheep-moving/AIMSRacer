"""Actual ROS DDS -> native rosbag recorder -> SQLite -> extractor; synthetic only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from test_bag import bag_config, extractor


def test_real_ros_inspect_record_stop_and_extract(tmp_path):
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    sys.path.insert(0,str(Path(__file__).parent))
    from bag import topic_specifications
    config=bag_config();config_path=tmp_path/'config.json';config_path.write_text(json.dumps(config))
    # Dedicated localhost-only domain; no vehicle publishers or command interfaces.
    os.environ['ROS_DOMAIN_ID']='209';os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=rclpy.create_node('synthetic_vicon_capture_fixture')
    publishers={topic:node.create_publisher(get_message(spec['message_type']),topic,20)
                for topic,spec in topic_specifications(config).items()}
    classes={topic:get_message(spec['message_type']) for topic,spec in topic_specifications(config).items()}
    output=tmp_path/'session';log=(tmp_path/'cli.log').open('w')
    process=subprocess.Popen([sys.executable,str(Path(__file__).with_name('run.py')),'record',
        '--config',str(config_path),'--output',str(output),'--seconds','1.5'],stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+20
        while process.poll() is None and time.monotonic()<deadline:
            stamp=node.get_clock().now().nanoseconds
            for topic,publisher in publishers.items():
                message=classes[topic]();message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,10**9)
                message.header.frame_id='center_of_mass' if '/cg/' in topic else 'base_link'
                if topic=='/mocap/car':message.header.frame_id='vicon_world';message.transform.rotation.w=1.
                if hasattr(message,'child_frame_id'):message.child_frame_id='base_link'
                publisher.publish(message)
            rclpy.spin_once(node,timeout_sec=.01)
        assert process.poll()==0,(tmp_path/'cli.log').read_text()
        preflight=json.loads((output/'preflight.json').read_text())
        assert preflight['samples']['/mocap/car']['messages']>0
        assert (output/'bag/metadata.yaml').exists()
        result=extractor().extract(output/'bag',config,tmp_path/'extracted')
        assert result['counts']['vicon']>20 and result['counts']['own_rear_velocity']>20
        assert json.loads((output/'capture.json').read_text())['returncode'] in (0,-2)
    finally:
        if process.poll() is None:process.terminate();process.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
