"""Independent scalar acceleration estimate from rear-axle wheel speed and force.

Source stamps enter the causal core only in callback receipt order. The output
is d(body_vx)/dt; corrected body specific force is measured as a + d. No pose,
orientation, gravity, LIO, TF, navigation EKF or command path enters this node.
"""
from collections import Counter, deque
from math import isfinite
from time import perf_counter_ns

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .longitudinal_observer import LongitudinalObserver, ObserverConfig


class LongitudinalAccelerationNode(Node):
    """Serialized observer, publishing only in response to input callbacks."""

    def __init__(self):
        super().__init__('longitudinal_acceleration')
        defaults = {
            'wheel_topic': '/rear_axle/wheel_odom', 'imu_topic': '/rear_axle/imu',
            'output_topic': '/state_estimation/longitudinal_acceleration',
            'status_topic': '/state_estimation/longitudinal_acceleration/status',
            'base_frame_id': 'base_link', 'jerk_psd': 30., 'disturbance_psd': .1,
            'speed_variance': .01, 'force_variance': 16., 'history_seconds': 2.,
            'max_history_measurements': 4096,
            'wheel_warning_age_s': .2, 'imu_warning_age_s': .05,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        param = lambda name: self.get_parameter(name).value
        self.base_frame_id = param('base_frame_id')
        self.config = ObserverConfig(use_imu=True, use_gravity=False, **{
            name: param(name) for name in ('jerk_psd', 'disturbance_psd', 'speed_variance',
                                         'force_variance', 'history_seconds',
                                         'max_history_measurements')})
        self.warning_age = {'wheel': param('wheel_warning_age_s'),
                            'imu': param('imu_warning_age_s')}
        if any(not isfinite(value) or value < 0 for value in self.warning_age.values()):
            raise ValueError('source warning ages must be finite and nonnegative')
        self.input_counters = Counter({name: 0 for name in (
            'wheel_callbacks', 'imu_callbacks', 'wheel_wrong_frame', 'imu_wrong_frame',
            'wheel_nonfinite', 'imu_nonfinite', 'force_unavailable', 'invalid_stamp',
            'future_stamp', 'clock_resets', 'snapshot_failure', 'receive_failure')})
        self.timing_ms = {kind: deque(maxlen=512) for kind in ('wheel', 'imu')}
        self._last_clock_ns = None
        self._reset_state()
        # All input and status callbacks share one mutually exclusive group.
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self.acceleration_publisher = self.create_publisher(
            AccelWithCovarianceStamped, param('output_topic'), 10)
        self.status_publisher = self.create_publisher(DiagnosticArray, param('status_topic'), 10)
        self._wheel_subscription = self.create_subscription(
            Odometry, param('wheel_topic'), self._on_wheel, qos_profile_sensor_data,
            callback_group=self._callback_group)
        self._imu_subscription = self.create_subscription(
            Imu, param('imu_topic'), self._on_imu, qos_profile_sensor_data,
            callback_group=self._callback_group)
        self._status_timer = self.create_timer(
            1., self._publish_status, callback_group=self._callback_group)

    @property
    def initialized(self):
        return self._have_wheel and self._have_force

    def _reset_state(self):
        self.observer = LongitudinalObserver(self.config)
        self._have_wheel = self._have_force = False
        self.force_valid = False
        self.source_stamps = {'wheel': None, 'imu': None}
        self.last_force_source_ns = None
        self.last_estimate = None

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _observe_clock(self, now_ns):
        if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
            self._reset_state()
            self.input_counters['clock_resets'] += 1
            self.get_logger().warning('ROS clock reversed; longitudinal observer reset')
        self._last_clock_ns = now_ns

    def _source_ns(self, message, receipt_ns):
        seconds, nanos = message.header.stamp.sec, message.header.stamp.nanosec
        stamp_ns = seconds * 1_000_000_000 + nanos
        if seconds < 0 or not 0 <= nanos < 1_000_000_000 or not 0 <= stamp_ns <= 2**63 - 1:
            self.input_counters['invalid_stamp'] += 1
            return None
        if stamp_ns > receipt_ns:
            self.input_counters['future_stamp'] += 1
            return None
        return stamp_ns

    def _on_wheel(self, message):
        self._on_input('wheel', message)

    def _on_imu(self, message):
        self._on_input('imu', message)

    def _on_input(self, kind, message):
        started = perf_counter_ns()
        try:
            self.input_counters[kind + '_callbacks'] += 1
            receipt_ns = self._now_ns()
            self._observe_clock(receipt_ns)
            if kind == 'imu':
                self.force_valid = False
            frame = message.child_frame_id if kind == 'wheel' else message.header.frame_id
            if frame != self.base_frame_id:
                self.input_counters[kind + '_wrong_frame'] += 1
                return
            source_ns = self._source_ns(message, receipt_ns)
            if source_ns is None:
                return
            if kind == 'imu':
                previous = self.source_stamps['imu']
                self.source_stamps['imu'] = source_ns if previous is None else max(previous, source_ns)
            if kind == 'imu' and message.linear_acceleration_covariance[0] < 0:
                self.input_counters['force_unavailable'] += 1
                # This input is a heartbeat with unavailable force, not a zero measurement.
                self._publish_estimate()
                return
            value = message.twist.twist.linear.x if kind == 'wheel' else message.linear_acceleration.x
            if (not isfinite(value) or
                    (kind == 'imu' and not isfinite(message.linear_acceleration_covariance[0]))):
                self.input_counters[kind + '_nonfinite'] += 1
                return
            measurement_kind = 'speed' if kind == 'wheel' else 'force'
            try:
                admitted = self.observer.receive(measurement_kind, source_ns, value)
            except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError):
                self.input_counters['receive_failure'] += 1
                admitted = False
            if admitted:
                if kind == 'wheel':
                    self._have_wheel = True
                else:
                    self._have_force = self.force_valid = True
                    previous = self.last_force_source_ns
                    self.last_force_source_ns = source_ns if previous is None else max(previous, source_ns)
                if kind == 'wheel':
                    previous = self.source_stamps[kind]
                    self.source_stamps[kind] = source_ns if previous is None else max(previous, source_ns)
            if kind == 'imu':
                self._publish_estimate()
        finally:
            self.timing_ms[kind].append((perf_counter_ns() - started) * 1e-6)

    def _publish_estimate(self):
        if not self.initialized:
            return
        now_ns = self._now_ns()
        self._observe_clock(now_ns)
        if not self.initialized:
            return
        try:
            estimate = self.observer.snapshot(now_ns)
        except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError):
            self.input_counters['snapshot_failure'] += 1
            return
        message = AccelWithCovarianceStamped()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now_ns, 1_000_000_000)
        message.header.frame_id = self.base_frame_id
        message.accel.accel.linear.x = estimate.acceleration
        for index in (7, 14, 21, 28, 35):
            message.accel.covariance[index] = 1e6
        message.accel.covariance[0] = estimate.acceleration_variance
        self.last_estimate = estimate
        self.acceleration_publisher.publish(message)

    def _publish_status(self):
        now_ns = self._now_ns()
        self._observe_clock(now_ns)
        status = DiagnosticStatus()
        status.name = 'longitudinal_acceleration'
        status.hardware_id = 'scalar_observer'
        warnings = []
        if not self.initialized:
            warnings.append('waiting for wheel and valid force')
        if not self.force_valid:
            warnings.append('force unavailable or invalid')
        values = {'initialized': str(self.initialized).lower(),
                  'force_valid': str(self.force_valid).lower(),
                  'history_size': self.observer.history_size}
        for kind, source_ns in self.source_stamps.items():
            age = None if source_ns is None else (now_ns - source_ns) * 1e-9
            values[kind + '_source_stamp_ns'] = 'none' if source_ns is None else source_ns
            values[kind + '_age_s'] = 'none' if age is None else age
            if age is not None and age > self.warning_age[kind]:
                warnings.append(kind + ' source stale')
        force_age = None if self.last_force_source_ns is None else (now_ns - self.last_force_source_ns) * 1e-9
        values['force_source_stamp_ns'] = 'none' if self.last_force_source_ns is None else self.last_force_source_ns
        values['force_age_s'] = 'none' if force_age is None else force_age
        if force_age is not None and force_age > self.warning_age['imu']:
            warnings.append('valid force source stale')
        if self.last_estimate is not None:
            for name in ('velocity', 'acceleration', 'disturbance', 'acceleration_variance'):
                values['estimate.' + name] = getattr(self.last_estimate, name)
            values['estimate.stamp_ns'] = self.last_estimate.stamp_ns
        for name, count in self.observer.counters.items():
            values['observer.' + name] = count
        for name, count in self.input_counters.items():
            values['input.' + name] = count
        if self.observer.counters['numerical_failure'] or self.input_counters['snapshot_failure'] or self.input_counters['receive_failure']:
            warnings.append('numerical failure recorded')
        for kind, samples in self.timing_ms.items():
            for metric, value in (
                ('p50_ms', float(np.percentile(samples, 50)) if samples else 0.),
                ('p95_ms', float(np.percentile(samples, 95)) if samples else 0.),
                ('max_ms', max(samples) if samples else 0.),
                ('samples', len(samples)),
            ):
                values['timing.' + kind + '.' + metric] = value
        status.level = DiagnosticStatus.WARN if warnings else DiagnosticStatus.OK
        status.message = '; '.join(warnings) if warnings else 'observer initialized'
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now_ns, 1_000_000_000)
        message.header.frame_id = self.base_frame_id
        message.status = [status]
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LongitudinalAccelerationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
