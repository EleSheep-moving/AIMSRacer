"""Synthetic ROS integration; localhost isolated domain required, no hardware nodes."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from vesc_msgs.msg import VescStateStamped

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('mapping', [False, True], ids=['ekf-driving','adapter-mapping'])
def test_synthetic_turn_pipeline(mapping, tmp_path):
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or int(os.environ.get('ROS_DOMAIN_ID','0')) == 0:
        pytest.skip('Set ROS_LOCALHOST_ONLY=1 and an unused nonzero ROS_DOMAIN_ID')
    commands = [
        ['ros2','launch','aims_racer_system','rear_axle_frames.launch.py',f'publish_odom_tf:={str(mapping).lower()}'],
        ['ros2','run','vesc_ackermann','vesc_to_odom_node','--ros-args','--params-file',str(PACKAGE/'params/vesc.yaml'),'-r','odom:=/rear_axle/wheel_odom'],
    ]
    if not mapping:
        commands.append(['ros2','run','robot_localization','ekf_node','--ros-args','-r','__node:=ekf_filter_node','--params-file',str(PACKAGE/'params/ekf_rear.yaml')])
    processes=[]; logs=[]
    rclpy.init(); node=rclpy.create_node('rear_axle_synthetic_test')
    observations={name:[] for name in ('lio','imu','wheel','ekf','tf','static')}
    subscriptions=[]
    for name, topic, kind in [('lio','/rear_axle/lio_odom',Odometry),('imu','/rear_axle/imu',Imu),('wheel','/rear_axle/wheel_odom',Odometry),('ekf','/odometry/filtered',Odometry),('tf','/tf',TFMessage)]:
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,key=name:observations[key].append(msg),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',lambda msg:observations['static'].append(msg),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    imu_pub=node.create_publisher(Imu,'/livox/imu',qos_profile_sensor_data)
    lio_pub=node.create_publisher(Odometry,'/fastlio2/lio_odom',10)
    wheel_pub=node.create_publisher(VescStateStamped,'/sensors/core',10)
    servo_pub=node.create_publisher(Float64,'/sensors/servo_position_command',10)
    expected={}
    try:
        for i,cmd in enumerate(commands):
            handle=(tmp_path/f'node-{i}.log').open('w'); logs.append(handle)
            processes.append(subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True))
        ready_deadline=time.monotonic()+15
        while imu_pub.get_subscription_count()<2 or wheel_pub.get_subscription_count()<1 or (not mapping and node.count_subscribers('/rear_axle/lio_odom')<2):
            assert all(p.poll() is None for p in processes), 'ROS child exited; inspect logs'
            assert time.monotonic()<ready_deadline, 'ROS discovery timed out'
            rclpy.spin_once(node,timeout_sec=.02)
        started=time.monotonic(); last_lio=0.; next_tick=started
        v,w,lever=.5,.4,.3
        while time.monotonic()-started<5.:
            now=time.monotonic()
            if now<next_tick:
                rclpy.spin_once(node,timeout_sec=min(.001,next_tick-now)); continue
            next_tick=now+.005
            t=now-started; yaw=w*t
            stamp=node.get_clock().now().to_msg()
            imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='livox_frame'
            imu.angular_velocity.z=w
            imu.linear_acceleration.x=-w*w*lever/9.80665
            imu.linear_acceleration.y=w*v/9.80665
            imu.linear_acceleration.z=1.
            imu_pub.publish(imu)
            if now-last_lio>=.02:
                last_lio=now
                rear=[v/w*math.sin(yaw),v/w*(1-math.cos(yaw)),0.]
                odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='odom'; odom.child_frame_id='livox_frame'
                odom.pose.pose.position.x=rear[0]+lever*math.cos(yaw)
                odom.pose.pose.position.y=rear[1]+lever*math.sin(yaw)
                odom.pose.pose.position.z=.03
                odom.pose.pose.orientation.z=math.sin(yaw/2); odom.pose.pose.orientation.w=math.cos(yaw/2)
                odom.twist.twist.linear.x=v; odom.twist.twist.linear.y=w*lever
                expected[(stamp.sec,stamp.nanosec)]=rear
                lio_pub.publish(odom)
                servo=Float64(); servo.data=.506-.5137*.5*math.atan(w*.36/v); servo_pub.publish(servo)
                wheel=VescStateStamped(); wheel.header.stamp=stamp; wheel.state.speed=v*4650.; wheel_pub.publish(wheel)
            for _ in range(3): rclpy.spin_once(node,timeout_sec=0.)
        assert all(p.poll() is None for p in processes)
        assert len(observations['lio'])>20 and len(observations['wheel'])>20
        for msg in observations['lio'][-20:]:
            assert (msg.header.frame_id,msg.child_frame_id)==('odom','base_link')
            p=msg.pose.pose.position; vel=msg.twist.twist.linear
            np.testing.assert_allclose([p.x,p.y,p.z],expected[(msg.header.stamp.sec,msg.header.stamp.nanosec)],atol=1e-8)
            np.testing.assert_allclose([vel.x,vel.y,vel.z],[v,0.,0.],atol=1e-8)
        valid_imu=[m for m in observations['imu'] if m.linear_acceleration_covariance[0]>=0]
        assert len(valid_imu)>20, 'Compensated IMU never became available'
        for msg in valid_imu[-20:]:
            assert msg.header.frame_id=='base_link'
            a=msg.linear_acceleration
            np.testing.assert_allclose([a.x,a.y,a.z],[0.,w*v,9.80665],atol=1e-6)
        wheel=observations['wheel'][-1]
        assert (wheel.header.frame_id,wheel.child_frame_id)==('odom','base_link')
        assert abs(wheel.twist.twist.linear.x-v)<1e-9
        static={(t.header.frame_id,t.child_frame_id) for m in observations['static'] for t in m.transforms}
        assert static=={('base_link','livox_frame'),('base_link','base_footprint')}
        dynamic={(t.header.frame_id,t.child_frame_id) for m in observations['tf'] for t in m.transforms}
        assert dynamic=={('odom','base_link')}
        publishers=node.get_publishers_info_by_topic('/tf')
        assert len(publishers)==1
        assert publishers[0].node_name==('lio_to_rear_axle' if mapping else 'ekf_filter_node')
        if not mapping:
            assert len(observations['ekf'])>20
            state=observations['ekf'][-1]
            assert (state.header.frame_id,state.child_frame_id)==('odom','base_link')
            assert abs(state.twist.twist.linear.x-v)<.08
            assert abs(state.twist.twist.angular.z-w)<.08
        else:
            assert not observations['ekf']
    finally:
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=3)
        for handle in logs: handle.close()
        node.destroy_node(); rclpy.shutdown()
