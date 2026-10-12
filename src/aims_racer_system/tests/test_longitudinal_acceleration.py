"""Scalar observer runtime contracts; synthetic/DDS evidence, no vehicle claim."""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

MODULE = 'aims_racer_system.longitudinal_acceleration'
if importlib.util.find_spec(MODULE):
    from aims_racer_system.longitudinal_acceleration import LongitudinalAccelerationNode
else:
    LongitudinalAccelerationNode = None

EPOCH = 1_800_000_000_000_000_003


def stamp(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return message


def wheel(ns, speed=0., frame='base_link'):
    msg = stamp(Odometry(), ns)
    msg.child_frame_id = frame
    msg.twist.twist.linear.x = speed
    # Deliberately unrelated pose and non-x fields must not enter the observer.
    msg.pose.pose.orientation.w = math.nan
    msg.twist.twist.linear.y = math.nan
    return msg


def imu(ns, force=.1, frame='base_link', available=True):
    msg = stamp(Imu(), ns)
    msg.header.frame_id = frame
    msg.linear_acceleration.x = force
    msg.linear_acceleration_covariance[0] = 0. if available else -1.
    msg.orientation.w = math.nan
    msg.orientation_covariance[0] = -1.
    msg.angular_velocity.x = math.nan
    return msg


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def runtime():
    assert LongitudinalAccelerationNode is not None, 'longitudinal ROS runtime is missing'
    rclpy.init()
    node = LongitudinalAccelerationNode()
    now = [EPOCH]
    node._now_ns = lambda: now[0]
    output, status = Capture(), Capture()
    node.acceleration_publisher = output
    node.status_publisher = status
    try:
        yield node, now, output, status
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_initialization_requires_admitted_wheel_and_force(runtime):
    node, now, out, status = runtime
    node._on_imu(imu(EPOCH, available=False))
    node._on_wheel(wheel(EPOCH))
    assert not out.messages
    node._on_imu(imu(EPOCH))
    assert len(out.messages) == 1
    assert node.initialized
    assert node.observer.config.use_imu and not node.observer.config.use_gravity
    assert node.observer.config.jerk_psd == 30.
    assert node.observer.config.force_variance == 16.


def test_source_integer_stamp_and_publication_stamp_and_covariance(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    now[0] += 1_000_000
    node._on_imu(imu(EPOCH + 17))
    message = out.messages[-1]
    assert message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec == now[0]
    assert node.observer._stamps == [EPOCH, EPOCH + 17]
    assert message.header.frame_id == 'base_link'
    assert message.accel.accel.linear.x == node.last_estimate.acceleration
    covariance = np.array(message.accel.covariance).reshape(6, 6)
    assert covariance[0, 0] == node.last_estimate.acceleration_variance
    np.testing.assert_array_equal(np.diag(covariance)[1:], [1e6] * 5)
    assert np.count_nonzero(covariance - np.diag(np.diag(covariance))) == 0
    assert message.accel.accel.linear.y == message.accel.accel.linear.z == 0.
    assert message.accel.accel.angular.x == message.accel.accel.angular.y == message.accel.accel.angular.z == 0.


def test_stationary_force_offset_does_not_become_motion(runtime):
    node, now, out, status = runtime
    for index in range(801):
        now[0] = EPOCH + index * 5_000_000
        if index % 4 == 0:
            node._on_wheel(wheel(now[0]))
        node._on_imu(imu(now[0], .1))
    assert abs(node.last_estimate.acceleration) < .015
    assert abs(node.last_estimate.velocity) < .005
    assert abs(node.last_estimate.disturbance - .1) < .015


def test_scalar_constant_acceleration(runtime):
    node, now, out, status = runtime
    for index in range(801):
        now[0] = EPOCH + index * 5_000_000
        if index % 4 == 0:
            node._on_wheel(wheel(now[0], 2. * index * .005))
        node._on_imu(imu(now[0], 2.1))
    assert node.last_estimate.acceleration == pytest.approx(2., abs=.03)
    assert node.last_estimate.velocity == pytest.approx(8., abs=.01)


def test_unavailable_force_after_init_skips_fusion_and_keeps_snapshot(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    node._on_imu(imu(EPOCH))
    admitted = node.observer.counters['admitted']
    now[0] += 10_000_000
    node._on_imu(imu(now[0], math.nan, available=False))
    assert len(out.messages) == 2
    assert node.observer.counters['admitted'] == admitted
    assert not node.force_valid
    node._publish_status()
    diag = status.messages[-1].status[0]
    assert diag.level == DiagnosticStatus.WARN
    values = {item.key: item.value for item in diag.values}
    assert values['input.force_unavailable'] == '1'
    assert values['force_valid'] == 'false'
    assert float(values['timing.imu.p95_ms']) >= 0
    assert values['imu_age_s'] == '0.0'
    assert values['force_age_s'] == '0.01'


def test_bad_frame_nonfinite_future_and_invalid_source_are_visible(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH, frame='livox_frame'))
    node._on_wheel(wheel(EPOCH, math.nan))
    node._on_wheel(wheel(EPOCH + 1))
    node._on_imu(imu(EPOCH, frame='livox_frame'))
    node._on_imu(imu(EPOCH, math.nan))
    bad = imu(EPOCH); bad.header.stamp.sec = -1
    node._on_imu(bad)
    assert not out.messages
    node._publish_status()
    values = {item.key: item.value for item in status.messages[-1].status[0].values}
    assert values['input.wheel_wrong_frame'] == '1'
    assert values['input.imu_wrong_frame'] == '1'
    assert values['input.wheel_nonfinite'] == values['input.imu_nonfinite'] == '1'
    assert values['input.future_stamp'] == values['input.invalid_stamp'] == '1'
    assert node.observer.history_size == 0


def test_late_wheel_is_replayed_without_using_lio(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    now[0] += 20_000_000
    node._on_imu(imu(now[0]))
    now[0] += 10_000_000
    node._on_wheel(wheel(EPOCH + 10_000_000, .2))
    assert node.observer.counters['delayed'] == 1
    assert len(out.messages) == 1
    node._on_imu(imu(now[0]))
    assert len(out.messages) == 2
    assert out.messages[-1].header.stamp.nanosec == now[0] % 1_000_000_000
    now[0] += 3_000_000_000
    node._on_imu(imu(now[0]))
    before = len(out.messages)
    node._on_wheel(wheel(EPOCH))
    assert node.observer.counters['too_old'] == 1
    assert len(out.messages) == before
    node._publish_status()
    assert status.messages[-1].status[0].level == DiagnosticStatus.WARN


def test_clock_reversal_resets_history_and_requires_new_pair(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    node._on_imu(imu(EPOCH))
    now[0] = EPOCH - 1_000_000_000
    node._on_imu(imu(now[0]))
    assert not node.initialized
    assert node.observer._stamps == [now[0]]
    assert len(out.messages) == 1
    node._on_wheel(wheel(now[0]))
    assert len(out.messages) == 1
    node._on_imu(imu(now[0]))
    assert len(out.messages) == 2
    node._publish_status()
    values = {item.key: item.value for item in status.messages[-1].status[0].values}
    assert values['input.clock_resets'] == '1'
    assert values['wheel_age_s'] == values['imu_age_s'] == '0.0'


def test_dds_independent_process_outputs_without_lio_tf_or_drive(tmp_path):
    assert LongitudinalAccelerationNode is not None, 'longitudinal ROS runtime is missing'
    script = Path(__file__).resolve().parents[1] / 'scripts/longitudinal_acceleration.py'
    rclpy.init()
    node = rclpy.create_node('longitudinal_dds_contract')
    seen, status = [], []
    subscriptions = [
        node.create_subscription(AccelWithCovarianceStamped, '/state_estimation/longitudinal_acceleration', seen.append, 10),
        node.create_subscription(DiagnosticArray, '/state_estimation/longitudinal_acceleration/status', status.append, 10),
    ]
    wp = node.create_publisher(Odometry, '/rear_axle/wheel_odom', qos_profile_sensor_data)
    ip = node.create_publisher(Imu, '/rear_axle/imu', qos_profile_sensor_data)
    with (tmp_path / 'runtime.log').open('w') as log:
        process = subprocess.Popen([sys.executable, str(script)], stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    assert process.poll() is None, (tmp_path / 'runtime.log').read_text()
                    rclpy.spin_once(node, timeout_sec=.01)
            deadline = time.monotonic() + 10.
            while wp.get_subscription_count() < 1 or ip.get_subscription_count() < 1:
                assert time.monotonic() < deadline, 'DDS discovery timeout'
                spin(.05)
            info = node.get_subscriber_names_and_types_by_node('longitudinal_acceleration', '/')
            assert {name for name, _ in info} == {'/rear_axle/wheel_odom', '/rear_axle/imu'}
            pubs = node.get_publisher_names_and_types_by_node('longitudinal_acceleration', '/')
            assert {name for name, _ in pubs} <= {'/rosout', '/parameter_events', '/state_estimation/longitudinal_acceleration', '/state_estimation/longitudinal_acceleration/status'}
            assert not any(node.count_publishers(topic) for topic in ('/tf', '/tf_static', '/drive', '/ackermann_cmd'))
            ns = node.get_clock().now().nanoseconds - 10_000_000
            wp.publish(wheel(ns)); spin(.1)
            assert not seen
            ip.publish(imu(ns)); spin(.15)
            assert seen
            count = len(seen)
            spin(1.1)
            assert len(seen) == count, 'status timer must not fabricate acceleration outputs'
            assert status
            ip.publish(imu(node.get_clock().now().nanoseconds - 1_000_000, available=False))
            spin(.15)
            assert len(seen) == count + 1
        finally:
            process.terminate(); process.wait(timeout=5)
            for sub in subscriptions:
                node.destroy_subscription(sub)
            node.destroy_node(); rclpy.shutdown()


def test_wheel_receipt_updates_history_without_publishing(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    node._on_imu(imu(EPOCH))
    now[0] += 20_000_000
    node._on_wheel(wheel(now[0], .1))
    assert node.observer.counters['admitted'] == 3
    assert len(out.messages) == 1
    node._on_imu(imu(now[0], available=False))
    assert len(out.messages) == 2
    assert out.messages[-1].accel.accel.linear.x > 0.


def test_numerical_measurement_failure_is_visible_and_stream_continues(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH, 1.79e308))
    node._on_imu(imu(EPOCH))
    node._on_wheel(wheel(EPOCH, -1.79e308))
    assert node.observer.counters['numerical_failure'] == 1
    node._on_wheel(wheel(EPOCH, 0.))
    node._on_imu(imu(EPOCH, 0.))
    assert len(out.messages) == 2
    assert math.isfinite(out.messages[-1].accel.accel.linear.x)
    node._publish_status()
    values = {item.key: item.value for item in status.messages[-1].status[0].values}
    assert values['observer.numerical_failure'] == '1'
    assert status.messages[-1].status[0].level == DiagnosticStatus.WARN


def test_timing_history_bound_and_source_age_only_warn(runtime):
    node, now, out, status = runtime
    node._on_wheel(wheel(EPOCH))
    for index in range(520):
        now[0] = EPOCH + index * 5_000_000
        node._on_imu(imu(now[0]))
    assert len(node.timing_ms['imu']) == 512
    assert len(out.messages) == 520
    node._publish_status()
    values = {item.key: item.value for item in status.messages[-1].status[0].values}
    assert float(values['wheel_age_s']) > .2
    assert status.messages[-1].status[0].level == DiagnosticStatus.WARN
    assert values['timing.imu.samples'] == '512'
