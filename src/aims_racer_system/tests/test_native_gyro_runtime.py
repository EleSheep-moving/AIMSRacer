"""Native gyro node stays live after calibration; preserved ROS sensor contract."""
import os
import subprocess
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def test_native_calibration_publication_freeze_and_rewind(tmp_path):
    rclpy.init()
    node = rclpy.create_node('native_gyro_contract')
    outputs, statuses = [], []
    subscriptions = [
        node.create_subscription(Imu, '/livox/imu_bias_corrected', outputs.append, 50),
        node.create_subscription(DiagnosticArray, '/imu/gyro_bias/status', statuses.append, 10),
    ]
    ip = node.create_publisher(Imu, '/livox/imu', qos_profile_sensor_data)
    wp = node.create_publisher(Odometry, '/rear_axle/wheel_odom', qos_profile_sensor_data)
    lp = node.create_publisher(Odometry, '/fastlio2/lio_odom', qos_profile_sensor_data)
    bias = np.array([.001, -.002, .02])
    with (tmp_path / 'native-gyro.log').open('w') as log:
        process = subprocess.Popen([os.environ['NATIVE_GYRO_EXECUTABLE'],
            '--ros-args', '-p', 'duration:=0.05', '-p', 'min_samples:=3'],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin(seconds=.04):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    assert process.poll() is None, (tmp_path/'native-gyro.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.005)
            deadline = time.monotonic() + 10.
            while min(ip.get_subscription_count(), wp.get_subscription_count(), lp.get_subscription_count()) < 1:
                assert time.monotonic() < deadline, 'DDS discovery timed out'
                spin()
            spin(.2)

            def observations(ns, speed=0.):
                w = Odometry(); w.header.frame_id = 'odom'; w.child_frame_id = 'base_link'
                w.header.stamp.sec, w.header.stamp.nanosec = divmod(ns, 10**9)
                w.twist.twist.linear.x = speed
                wp.publish(w)
                l = Odometry(); l.header.frame_id = 'odom'; l.child_frame_id = 'livox_frame'
                l.header.stamp = w.header.stamp
                l.pose.pose.orientation.w = 1.
                l.twist.twist.linear.x = speed
                lp.publish(l)
                spin()

            def imu(ns, rate=bias, known=False):
                m = Imu(); m.header.frame_id = 'livox_frame'
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 10**9)
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = rate.tolist()
                m.linear_acceleration.z = 1.
                m.orientation_covariance[0] = -1.
                if known:
                    m.angular_velocity_covariance = [.01, .001, 0., .001, .02, 0., 0., 0., .03]
                ip.publish(m); spin()
                return m

            base = 1_800_000_000_000_000_000
            # Future external observations must not permit calibration.
            observations(base + 1_000_000_000)
            for index in range(4):
                imu(base + index * 20_000_000)
            assert not outputs
            # Rewind clears external observation histories.
            imu(base - 1_000_000_000)
            for index in range(4):
                ns = base + index * 20_000_000
                observations(ns)
                imu(ns)
                if index < 3:
                    assert not outputs
            assert len(outputs) == 1
            np.testing.assert_allclose([outputs[-1].angular_velocity.x,
                outputs[-1].angular_velocity.y, outputs[-1].angular_velocity.z], [0.]*3, atol=1e-14)
            assert list(outputs[-1].angular_velocity_covariance) == [0.]*9
            # The calibrated node must keep forwarding motion with a frozen bias.
            ns = base + 80_000_000
            original = imu(ns, bias + [.1, .2, .5], known=True)
            assert len(outputs) == 2
            out = outputs[-1]
            assert out.header == original.header
            assert out.linear_acceleration == original.linear_acceleration
            np.testing.assert_array_equal(out.orientation_covariance, original.orientation_covariance)
            np.testing.assert_allclose([out.angular_velocity.x, out.angular_velocity.y,
                out.angular_velocity.z], [.1, .2, .5], atol=1e-14)
            expected = np.array(original.angular_velocity_covariance).reshape(3,3) + np.eye(3)*1e-6
            np.testing.assert_allclose(np.array(out.angular_velocity_covariance).reshape(3,3), expected, atol=1e-14)
            spin(1.1)
            assert statuses
            assert any(item.key == 'ready' and item.value == 'true'
                       for item in statuses[-1].status[0].values)
            # Same source time resets calibration; no forwarding until a new static window.
            count = len(outputs)
            imu(ns)
            imu(ns + 20_000_000)
            assert len(outputs) == count
        finally:
            process.terminate(); process.wait(timeout=5)
            for sub in subscriptions:
                node.destroy_subscription(sub)
            node.destroy_node(); rclpy.shutdown()
