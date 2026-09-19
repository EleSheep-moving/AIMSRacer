"""Synthetic raw Livox + LIO + wheel speed -> installed adapter/helper -> real ROS EKF."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import pytest
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('scenario', ['turn', 'fast_turn', 'tilted_stationary', 'accelerating', 'yaw_ramp'])
def test_installed_pipeline_with_wheel_speed(tmp_path, scenario):
    share = Path(get_package_share_directory('aims_racer_system'))
    commands = [
        ['ros2', 'launch', 'aims_racer_system', 'rear_axle_frames.launch.py'],
        ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '-r', '__node:=ekf_filter_node',
         '--params-file', str(share/'params/ekf_rear.yaml')],
    ]
    logs = [open(tmp_path/f'process-{i}.log', 'w') for i in range(2)]
    processes = [subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                 for cmd, log in zip(commands, logs)]
    rclpy.init(); node = rclpy.create_node('raw_lio_fixture')
    filtered, transformed, imu_out, transforms = [], [], [], []
    node.create_subscription(Odometry, '/odometry/filtered', filtered.append, 20)
    node.create_subscription(Odometry, '/fastlio2/base_odom', transformed.append, 20)
    node.create_subscription(Imu, '/livox/imu_ekf', imu_out.append, 20)
    node.create_subscription(TFMessage, '/tf', transforms.append, 100)
    ip = node.create_publisher(Imu, '/livox/imu', 20)
    op = node.create_publisher(Odometry, '/fastlio2/lio_odom', 20)
    wp = node.create_publisher(Odometry, '/odom', 10)
    try:
        end = time.monotonic()+2.
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
        start = time.monotonic(); last_truth = None; next_odom = 0.; next_wheel = 0.
        while time.monotonic()-start < 4.:
            elapsed = time.monotonic()-start
            if scenario in ('turn', 'fast_turn'):
                speed=5. if scenario == 'fast_turn' else 1.
                yaw_rate=speed/5.; yaw=yaw_rate*elapsed; alpha=0.; pitch=0.
                x,y=5.*math.sin(yaw),5.*(1.-math.cos(yaw)); ax=0.; ay=speed*yaw_rate
            elif scenario == 'tilted_stationary':
                yaw=yaw_rate=alpha=x=y=speed=ax=ay=0.; pitch=.25
            elif scenario == 'accelerating':
                yaw=yaw_rate=alpha=pitch=y=ay=0.; x=.25*elapsed**2; speed=.5*elapsed; ax=.5
            else:
                yaw=.25*elapsed**2; yaw_rate=.5*elapsed; alpha=.5
                pitch=x=y=speed=ax=ay=0.
            rotation=Rotation.from_euler('xyz',[0.,pitch,yaw])
            lever=np.array([.311,.02329,-.01412])
            omega=np.array([0.,0.,yaw_rate]); angular_acc=np.array([0.,0.,alpha])
            gravity=rotation.inv().apply([0.,0.,9.80665])
            rear_force=np.array([ax,ay,0.])+gravity
            sensor_force=rear_force+np.cross(angular_acc,lever)+np.cross(omega,np.cross(omega,lever))
            sensor_position=np.array([x,y,0.])+rotation.apply(lever)
            sensor_velocity=np.array([speed,0.,0.])+np.cross(omega,lever)
            stamp = node.get_clock().now().to_msg()
            imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'laser'
            imu.angular_velocity.z = yaw_rate
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=(sensor_force/9.80665).tolist()
            ip.publish(imu)
            if elapsed >= next_wheel:
                wheel = Odometry(); wheel.header.stamp = stamp
                wheel.header.frame_id = 'odom'; wheel.child_frame_id = 'base_link'
                wheel.pose.pose.orientation.w = 1.
                wheel.twist.twist.linear.x = speed; wheel.twist.covariance[0] = .04
                wp.publish(wheel); next_wheel = elapsed+.02  # VESC fixture 50 Hz
            rclpy.spin_once(node, timeout_sec=.002)
            if elapsed >= next_odom:
                msg = Odometry(); msg.header.stamp = stamp
                msg.header.frame_id = 'odom'; msg.child_frame_id = 'livox_imu'
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=sensor_position.tolist()
                q=rotation.as_quat().tolist()
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=q
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=sensor_velocity.tolist()
                op.publish(msg); next_odom = elapsed+.1  # LIO 10 Hz
            last_truth = (x, y, yaw)
            rclpy.spin_once(node, timeout_sec=.005)
        assert all(p.poll() is None for p in processes), 'Launch or EKF exited unexpectedly'
        assert len(filtered)>30 and len(transformed)>30 and len(imu_out)>30
        assert filtered[-1].child_frame_id == 'base_link'
        assert filtered[-1].twist.twist.angular.z == pytest.approx(yaw_rate, abs=.08)
        assert filtered[-1].twist.twist.linear.x == pytest.approx(speed, abs=.15)
        assert math.hypot(filtered[-1].pose.pose.position.x-last_truth[0],
                          filtered[-1].pose.pose.position.y-last_truth[1]) < .15
        valid=[m for m in imu_out if m.linear_acceleration_covariance[0]>0]
        assert len(valid)>30
        assert valid[-1].header.frame_id == 'base_link'
        assert valid[-1].orientation_covariance[0]>0
        assert valid[-1].angular_velocity.z == pytest.approx(yaw_rate, abs=.03)
        assert [valid[-1].linear_acceleration.x,valid[-1].linear_acceleration.y,valid[-1].linear_acceleration.z] == pytest.approx(rear_force, abs=.03)
        # Gyro-only fusion must not integrate gravity or physical centripetal ay.
        if scenario == 'fast_turn':
            assert abs(filtered[-1].twist.twist.linear.y)<.15
        # A pitched stationary car must not accelerate.
        if scenario == 'tilted_stationary':
            assert abs(filtered[-1].twist.twist.linear.x)<.03
            assert abs(filtered[-1].twist.twist.linear.y)<.03
        edges = {(t.header.frame_id,t.child_frame_id) for batch in transforms for t in batch.transforms}
        assert edges == {('odom','base_link')}
        assert len(node.get_publishers_info_by_topic('/tf')) == 1
    finally:
        node.destroy_node(); rclpy.shutdown()
        for p in processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in logs: log.close()
