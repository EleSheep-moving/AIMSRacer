"""Native DDS/source-clock contracts against the retained Python numerical core."""
import os
import subprocess
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

from aims_racer_system.longitudinal_observer import LongitudinalObserver, ObserverConfig


def test_native_callbacks_match_reference_and_reset_on_clock_reversal(tmp_path):
    rclpy.init()
    node = rclpy.create_node('native_acceleration_contract')
    outputs, statuses = [], []
    subscriptions = [
        node.create_subscription(AccelWithCovarianceStamped,
            '/state_estimation/longitudinal_acceleration', outputs.append, 100),
        node.create_subscription(DiagnosticArray,
            '/state_estimation/longitudinal_acceleration/status', statuses.append, 10),
    ]
    clock = node.create_publisher(Clock, '/clock', 10)
    wheel = node.create_publisher(Odometry, '/rear_axle/wheel_odom', qos_profile_sensor_data)
    imu = node.create_publisher(Imu, '/rear_axle/imu', qos_profile_sensor_data)
    reference = LongitudinalObserver(ObserverConfig(use_imu=True, jerk_psd=30.,
        disturbance_psd=.1, speed_variance=.01, force_variance=16.))
    current = 1_800_000_000_000_000_003
    with (tmp_path / 'native.log').open('w') as log:
        process = subprocess.Popen([os.environ['NATIVE_ACCELERATION_EXECUTABLE'],
            '--ros-args', '-p', 'use_sim_time:=true'], stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin(seconds=.04):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    assert process.poll() is None, (tmp_path / 'native.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.005)

            deadline = time.monotonic() + 10.
            while min(clock.get_subscription_count(), wheel.get_subscription_count(),
                      imu.get_subscription_count()) < 1:
                assert time.monotonic() < deadline, 'DDS discovery timed out'
                spin()
            # Discovery for outputs can lag input discovery.
            spin(.2)

            def set_clock(ns):
                nonlocal current
                current = ns
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = divmod(ns, 10**9)
                clock.publish(msg)
                spin()

            def send(kind, ns, value=0., available=True, frame='base_link', admitted=True):
                if kind == 'speed':
                    message = Odometry()
                    message.child_frame_id = frame
                    message.twist.twist.linear.x = value
                    message.pose.pose.orientation.w = float('nan')
                    wheel.publish(_stamp(message, ns))
                else:
                    message = Imu()
                    message.header.frame_id = frame
                    message.linear_acceleration.x = value
                    message.linear_acceleration_covariance[0] = 0. if available else -1.
                    message.orientation.w = float('nan')
                    imu.publish(_stamp(message, ns))
                if admitted:
                    assert reference.receive(kind, ns, value)
                spin()

            def check_latest():
                assert outputs
                output = outputs[-1]
                expected = reference.snapshot(current)
                assert output.header.stamp.sec * 10**9 + output.header.stamp.nanosec == current
                assert output.header.frame_id == 'base_link'
                np.testing.assert_allclose(output.accel.accel.linear.x,
                    expected.acceleration, rtol=2e-11, atol=2e-11)
                np.testing.assert_allclose(output.accel.covariance[0],
                    expected.acceleration_variance, rtol=2e-11, atol=2e-11)
                np.testing.assert_array_equal(np.array(output.accel.covariance).reshape(6, 6).diagonal()[1:], [1e6]*5)

            set_clock(current)
            send('force', current, available=False, admitted=False)
            send('speed', current)
            assert not outputs
            send('force', current, .1)
            check_latest()
            # Accelerating wheel + offset force, each prediction at receipt time.
            for index in range(1, 21):
                set_clock(current + 20_000_000)
                before = len(outputs)
                send('speed', current - 5_000_000, .04 * index)
                assert len(outputs) == before, 'wheel callback must not publish'
                send('force', current - 1_000_000, 2.1)
                check_latest()
            # Late wheel admission replays a suffix; unavailable force is a heartbeat.
            send('speed', current - 15_000_000, .77)
            before = len(outputs)
            set_clock(current + 10_000_000)
            send('force', current, float('nan'), available=False, admitted=False)
            assert len(outputs) == before + 1
            check_latest()
            # Frames, invalid numerics, and future sources cannot become observations.
            before = len(outputs)
            send('speed', current, frame='livox_frame', admitted=False)
            send('force', current, float('nan'), admitted=False)
            send('force', current + 1, admitted=False)
            send('force', -1, admitted=False)
            assert len(outputs) == before
            set_clock(current + 1_100_000_000)
            assert len(outputs) == before, 'status timer must not publish acceleration'
            assert statuses
            fields = {item.key: item.value for item in statuses[-1].status[0].values}
            for key in ('input.wheel_wrong_frame', 'input.imu_nonfinite',
                        'input.future_stamp', 'input.invalid_stamp', 'input.force_unavailable'):
                assert int(fields[key]) >= 1
            assert int(fields['observer.delayed']) >= 1
            assert fields['force_valid'] == 'false'
            # Rewind requires a fresh wheel/force pair; pre-reset state must disappear.
            set_clock(current - 2_000_000_000)
            send('force', current, .1, admitted=False)
            assert len(outputs) == before
            reference = LongitudinalObserver(reference.config)
            assert reference.receive('force', current, .1)
            send('speed', current)
            assert len(outputs) == before
            send('force', current, .1)
            assert len(outputs) == before + 1
            check_latest()
        finally:
            process.terminate()
            process.wait(timeout=5)
            for sub in subscriptions:
                node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()


def _stamp(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
    return message
