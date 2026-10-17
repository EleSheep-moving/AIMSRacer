"""Source-time conversion with late LIO and continuously arriving IMU; no hardware."""
import os
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def test_late_lio_uses_source_time_gyro_while_imu_remains_live(tmp_path):
    rclpy.init()
    node = rclpy.create_node('rear_timestamp_test')
    seen_imu, seen_lio, seen_tf = [], [], []
    subscriptions = [
        node.create_subscription(TFMessage, '/tf', seen_tf.append, 10),
        node.create_subscription(Imu, '/rear_axle/imu', seen_imu.append, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/rear_axle/lio_odom', seen_lio.append, qos_profile_sensor_data),
    ]
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    lio_pub = node.create_publisher(Odometry, '/fastlio2/lio_odom', 10)
    processes = []
    handles = []
    try:
        for exe in ['imu_to_rear_axle', 'lio_to_rear_axle']:
            handle = (tmp_path / (exe + '.log')).open('w'); handles.append(handle)
            processes.append(subprocess.Popen(
                ['ros2', 'run', 'aims_racer_system', exe, '--ros-args', '-p',
                 'livox_translation:=[0.3,0.0,0.03]',
                 *(['-p', 'publish_tf:=true'] if exe == 'lio_to_rear_axle' else [])],
                stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                assert all(p.poll() is None for p in processes)
                rclpy.spin_once(node, timeout_sec=.01)
        end = time.monotonic() + 12.
        while imu_pub.get_subscription_count() < 2 or lio_pub.get_subscription_count() < 1:
            assert time.monotonic() < end, 'ROS discovery timed out'
            spin(.05)
        spin(.3)
        source_ns = 10000000001
        def imu(offset_ns):
            msg = Imu(); msg.header.frame_id = 'livox_frame'
            ns = source_ns + offset_ns
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
            msg.angular_velocity.z = .4 + offset_ns * 1e-9
            msg.linear_acceleration.z = 1.
            imu_pub.publish(msg); spin(.01)
        # No IMU sample exactly at the LIO timestamp. A latest-past selection is wrong.
        for offset in [-13000000, -8000000, -3000000, 2000000, 7000000]:
            imu(offset)
        for offset in range(12000000, 172000000, 5000000):
            imu(offset)
        spin(.1)
        assert seen_imu, 'IMU must publish before any LIO has arrived'
        assert seen_imu[-1].linear_acceleration_covariance[0] >= 0
        assert seen_imu[-1].orientation_covariance[0] == -1
        assert node.count_publishers('/rear_axle/imu_aligned') == 0
        last_imu_ns = seen_imu[-1].header.stamp.sec * 1000000000 + seen_imu[-1].header.stamp.nanosec
        assert last_imu_ns == source_ns + 167000000
        msg = Odometry(); msg.header.frame_id = 'odom'; msg.child_frame_id = 'livox_frame'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns, 1000000000)
        msg.pose.pose.orientation.w = 1.
        msg.pose.pose.position.x = .3; msg.pose.pose.position.z = .03
        msg.twist.twist.linear.x = 1.; msg.twist.twist.linear.y = .4 * .3
        lio_pub.publish(msg); spin(.4)
        assert len(seen_lio) == 1
        out = seen_lio[0]
        assert out.header.stamp == msg.header.stamp
        np.testing.assert_allclose(out.twist.twist.angular.z, .4, atol=1e-12)
        np.testing.assert_allclose(
            [out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z],
            [1., 0., 0.], atol=1e-12)
        assert out.pose.pose.orientation == msg.pose.pose.orientation
        assert seen_tf
        transform = seen_tf[-1].transforms[0]
        assert transform.header == out.header and transform.child_frame_id == 'base_link'
        assert transform.transform.rotation == out.pose.pose.orientation
        np.testing.assert_allclose([transform.transform.translation.x,
            transform.transform.translation.y, transform.transform.translation.z],
            [out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z], atol=1e-12)
        # LIO receipt must not cause historical IMU samples to be republished as current.
        count = len(seen_imu)
        spin(.1); assert len(seen_imu) == count
        imu(172000000); spin(.1)
        assert len(seen_imu) == count + 1
        # Reverse callback order: LIO arrives before the upper IMU knot.
        next_ns = source_ns + 175000000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(next_ns, 1000000000)
        msg.twist.twist.linear.y = .575 * .3
        lio_pub.publish(msg); spin(.1)
        assert len(seen_lio) == 1, 'Must wait rather than extrapolate from the last gyro'
        imu(177000000); spin(.2)
        assert len(seen_lio) == 2
        assert seen_lio[-1].header.stamp == msg.header.stamp
        np.testing.assert_allclose(seen_lio[-1].twist.twist.angular.z, .575, atol=1e-12)
        np.testing.assert_allclose(seen_lio[-1].twist.twist.linear.y, 0., atol=1e-12)
        # A gyro rewind clears pending LIO and the published monotonicity anchor.
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns + 1_000_000_000, 10**9)
        lio_pub.publish(msg); spin(.05)
        assert len(seen_lio) == 2
        imu(-10_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns - 5_000_000, 10**9)
        msg.twist.twist.linear.y = .395 * .3
        lio_pub.publish(msg); spin(.05)
        assert len(seen_lio) == 2
        imu(0); spin(.1)
        assert len(seen_lio) == 3
        np.testing.assert_allclose(seen_lio[-1].twist.twist.angular.z, .395, atol=1e-12)
        imu(1_000_000_000); spin(.1)
        assert len(seen_lio) == 3, 'rewind must clear pre-reset pending LIO'
        # Measured knots separated by a gap cannot cover an intermediate LIO time.
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns + 500_000_000, 10**9)
        lio_pub.publish(msg); spin(.1)
        assert len(seen_lio) == 3
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, 15)
        for p in processes:
            p.wait(timeout=8)
        for handle in handles: handle.close()
        for sub in subscriptions: node.destroy_subscription(sub)
        node.destroy_node(); rclpy.shutdown()
