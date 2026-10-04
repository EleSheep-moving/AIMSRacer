#!/usr/bin/env python3
"""Diagnostic observer for NDT anchors and source-time scan/map consistency.

This node never publishes TF or changes NDT/EKF estimates. Timer bridge poses
are deliberately not subscribed: only scan-stamped alignment + accepted pose
can advance measurement freshness.
"""
from collections import deque, OrderedDict
from concurrent.futures import ThreadPoolExecutor
import hashlib
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from localization_map_io import map_points
from localization_policy import MeasurementHealth, interpolate_pose, trustworthy_alignment, MapOdomPackets


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_nanoseconds(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def nanoseconds_seconds(stamp):
    return stamp // 1000000000 + (stamp % 1000000000) * 1e-9


def matrix(position, quaternion):
    values = np.array([position.x, position.y, position.z,
                       quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1.) > .01:
        raise ValueError('Expected finite pose with unit quaternion')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def usable_covariance(values, dimension, measured_index):
    covariance = np.asarray(values, dtype=float).reshape(dimension, dimension)
    return (np.isfinite(covariance).all() and covariance[measured_index, measured_index] > 0. and
            np.allclose(covariance, covariance.T, atol=1e-10) and
            np.linalg.eigvalsh(covariance).min() >= -1e-10)


def quality_check(cloud, source_pose, map_to_odom, mount, tree):
    started = time.perf_counter()
    values = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z'])
    xyz = np.column_stack([values[name] for name in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) == 0:
        raise ValueError('No finite scan points')
    xyz = xyz[::max(1, math.ceil(len(xyz) / 2000))]
    transform = map_to_odom @ source_pose @ mount
    world = xyz @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = tree.query(world, workers=1)
    inliers = distances <= .25
    return {'quality_points': len(xyz), 'inlier_fraction': float(inliers.mean()),
            'inlier_rmse_m': float(np.sqrt(np.mean(distances[inliers] ** 2))) if inliers.any() else float('nan'),
            'quality_stamp_sec': stamp_seconds(cloud.header.stamp),
            'quality_time_ms': (time.perf_counter() - started) * 1000.}


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        data = Path(self.declare_parameter('map_file', '').value).read_bytes()
        self.sha = hashlib.sha256(data).hexdigest()
        self.tree = cKDTree(map_points(data))
        translation = self.declare_parameter('livox_translation', [.3, 0., .03]).value
        quaternion = self.declare_parameter('livox_quaternion', [0., 0., 0., 1.]).value
        self.mount = np.eye(4)
        self.mount[:3, 3] = translation
        self.mount[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        self.health = MeasurementHealth()
        self.history = deque()
        self.poses = OrderedDict()
        self.pending = OrderedDict()
        self.pose_latest_stamp = None
        self.map_packets = MapOdomPackets()
        self.inputs = {}
        self.cloud = None
        self.quality = {}
        self.anchor_check = {}
        self.quality_error = 'waiting_for_scan'
        self.future = None
        self.last_quality_stamp = None
        self.last_clock = None
        self.quality_worker = ThreadPoolExecutor(max_workers=1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Accepted scan poses share a topic with 50 Hz bridge timer poses.
        # A depth-one subscriber can lose the scan before its callback runs.
        ndt_input_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sha_pub = self.create_publisher(String, '/localization/map_sha256', latched)
        self.valid_pub = self.create_publisher(Bool, '/localization/map_valid', latched)
        self.status_pub = self.create_publisher(DiagnosticArray, '/localization/status', 10)
        self.sha_pub.publish(String(data=self.sha))
        self.subscriptions_owned = [
            self.create_subscription(Odometry, '/odometry/filtered', self.odometry, 500),
            self.create_subscription(PointCloud2, '/localization/deskewed_cloud', self.scan, qos_profile_sensor_data),
            self.create_subscription(PoseWithCovarianceStamped, '/localization/ndt_pose', self.pose, ndt_input_qos),
            self.create_subscription(DiagnosticArray, '/localization/ndt_status', self.alignment, ndt_input_qos),
            self.create_subscription(TFMessage, '/tf', self.transforms, 100),
            self.create_subscription(Imu, '/rear_axle/imu', self.gyro, qos_profile_sensor_data),
            self.create_subscription(Imu, self.declare_parameter('imu_topic', '/livox/imu').value,
                                     lambda msg: self.input('raw_imu', msg), qos_profile_sensor_data),
            self.create_subscription(Odometry, self.declare_parameter('wheel_topic', '/rear_axle/wheel_odom').value,
                                     self.wheel, qos_profile_sensor_data)]
        self.timer = self.create_timer(.1, self.tick)
        self.quality_timer = self.create_timer(.5, self.launch_quality)

    def input(self, key, msg):
        self.inputs[key] = stamp_seconds(msg.header.stamp)

    def gyro(self, msg):
        omega = msg.angular_velocity
        if (msg.header.frame_id == 'base_link' and
                all(math.isfinite(value) for value in (omega.x, omega.y, omega.z)) and
                usable_covariance(msg.angular_velocity_covariance, 3, 2)):
            self.input('imu', msg)

    def wheel(self, msg):
        if ((msg.header.frame_id, msg.child_frame_id) == ('odom', 'base_link') and
                math.isfinite(msg.twist.twist.linear.x) and
                usable_covariance(msg.twist.covariance, 6, 0)):
            self.input('wheel', msg)

    def reset_epoch(self):
        self.map_packets.clear()
        self.health = MeasurementHealth()
        self.history.clear()
        self.pending.clear()
        self.poses.clear()
        self.pose_latest_stamp = None
        self.inputs.clear()
        self.cloud = None
        self.last_quality_stamp = None
        self.quality = {}
        self.quality_error = 'clock_reset'
        self.anchor_check = {}
        # Running work carries its source stamp and is discarded on completion.
        self.quality_generation = getattr(self, 'quality_generation', 0) + 1

    def odometry(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        stamp = stamp_seconds(msg.header.stamp)
        if self.history and stamp < self.history[-1][0]:
            self.reset_epoch()
        try:
            pose = matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError:
            return
        if not self.history or stamp > self.history[-1][0]:
            self.history.append((stamp, pose))
        while self.history and stamp - self.history[0][0] > 2.:
            self.history.popleft()
        self.input('ekf', msg)

    def scan(self, msg):
        if msg.header.frame_id == 'livox_frame':
            self.cloud = msg
            self.input('deskew', msg)

    def pose(self, msg):
        if msg.header.frame_id != 'map':
            return
        try:
            stamp = stamp_nanoseconds(msg.header.stamp)
            self.poses[stamp] = matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError:
            return
        self.pose_latest_stamp = stamp if self.pose_latest_stamp is None else max(self.pose_latest_stamp, stamp)
        for key in list(self.poses):
            if key < self.pose_latest_stamp - 5000000000:
                self.poses.pop(key)

    def transforms(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) != ('map', 'odom'):
                continue
            try:
                self.map_packets.add(stamp_nanoseconds(transform.header.stamp),
                    matrix(transform.transform.translation, transform.transform.rotation))
            except ValueError:
                continue

    def alignment(self, msg):
        stamp = stamp_nanoseconds(msg.header.stamp)
        for status in msg.status:
            if status.name != 'lidar_localization_ros2/alignment':
                continue
            values = {item.key: item.value for item in status.values}
            # Non-OK and rejection statuses reset the acceptance streak immediately.
            if status.level != DiagnosticStatus.OK or status.message != 'ok':
                self.health.observe(nanoseconds_seconds(stamp), False, status.message)
                return
            self.pending[stamp] = (status, values, time.monotonic())
            while len(self.pending) > 20:
                self.pending.popitem(last=False)

    def map_odom(self, stamp_ns, exact=True):
        transform = self.map_packets.exact(stamp_ns) if exact else self.map_packets.held(stamp_ns)
        if transform is None:
            raise ValueError('waiting_for_exact_map_odom_packet' if exact else 'quality_map_odom_source_not_covered')
        return transform

    def evaluate_pending(self):
        for stamp_ns, (status, values, received) in list(self.pending.items()):
            stamp = nanoseconds_seconds(stamp_ns)
            if self.health.last_scan is not None and stamp <= self.health.last_scan:
                self.pending.pop(stamp_ns)
                continue
            try:
                correction = float(values.get('correction_translation_m', 'nan'))
                yaw = float(values.get('correction_yaw_deg', 'nan'))
                source = interpolate_pose(list(self.history), stamp)
                accepted_pose = self.poses.get(stamp_ns)
                if source is None or accepted_pose is None:
                    raise ValueError('waiting_for_source_pose')
                anchored_pose = self.map_odom(stamp_ns) @ source
                residual = np.linalg.inv(accepted_pose) @ anchored_pose
                translation_error = float(np.linalg.norm(residual[:3, 3]))
                rotation_error = float(Rotation.from_matrix(residual[:3, :3]).magnitude())
                anchor_matches = translation_error < .01 and rotation_error < .01
                self.anchor_check = dict(anchor_check_scan_sec=stamp,
                    anchor_residual_translation_m=translation_error,
                    anchor_residual_rotation_rad=rotation_error)
                # The upstream node publishes diagnostics before accepted pose
                # and TF. A cached frozen edge may already cover this source
                # stamp, so a successful TF lookup can still be an old value.
                # Give its accepted update the same bounded delivery wait used
                # for missing pose/TF; a persistent mismatch remains rejected.
                if (not anchor_matches and trustworthy_alignment(values, True) and
                        time.monotonic() - received <= .5):
                    continue
                trusted = trustworthy_alignment(values, anchor_matches)
                reason = 'ok' if trusted else ('accepted_pose_not_anchored' if not anchor_matches else 'untrusted_alignment_metadata')
                self.health.observe(stamp, trusted, reason, correction, yaw)
                self.pending.pop(stamp_ns)
            except Exception as error:
                if time.monotonic() - received > .5:
                    self.health.observe(stamp, False, str(error))
                    self.pending.pop(stamp_ns)

    def launch_quality(self):
        if self.future is not None:
            return
        if self.cloud is None:
            return
        stamp = stamp_seconds(self.cloud.header.stamp)
        if stamp == self.last_quality_stamp:
            return
        try:
            source = interpolate_pose(list(self.history), stamp)
            if source is None:
                raise ValueError('quality_source_time_not_covered')
            map_odom = self.map_odom(stamp_nanoseconds(self.cloud.header.stamp), exact=False)
            self.future_map_packet_stamp = self.map_packets.held_stamp(stamp_nanoseconds(self.cloud.header.stamp))
            self.last_quality_stamp = stamp
            self.future_generation = getattr(self, 'quality_generation', 0)
            self.future = self.quality_worker.submit(quality_check, self.cloud, source, map_odom, self.mount, self.tree)
        except Exception as error:
            self.quality_error = str(error)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.reset_epoch()
        self.last_clock = now
        self.evaluate_pending()
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if self.future_generation == getattr(self, 'quality_generation', 0):
                    self.quality = result
                    self.quality['quality_map_packet_source_sec'] = nanoseconds_seconds(self.future_map_packet_stamp)
                    self.quality['quality_map_packet_hold_sec'] = result['quality_stamp_sec'] - nanoseconds_seconds(self.future_map_packet_stamp)
                    self.quality_error = ''
            except Exception as error:
                self.quality_error = str(error)
            self.future = None
        ages = {key + '_age_sec': now - self.inputs.get(key, float('-inf')) for key in ('imu', 'wheel', 'ekf', 'deskew')}
        present = all(-.05 <= age < .5 for age in ages.values())
        state = self.health.state(now, present)
        valid = state == 'tracking'
        total_translation, total_yaw = self.health.correction_totals(now)
        measurement = self.health.last_trustworthy
        values = dict(state=state, map_sha256=self.sha, map_valid=str(valid).lower(),
                      accepted_streak=self.health.accepts, last_trustworthy_scan_sec=measurement,
                      measurement_age_sec=now - measurement if measurement is not None else float('inf'),
                      rejection_reason=self.health.reason,
                      correction_window_translation_m=total_translation,
                      correction_window_yaw_deg=total_yaw, quality_error=self.quality_error,
                      raw_imu_age_sec=now - self.inputs.get('raw_imu', float('-inf')),
                      pending_anchor_checks=len(self.pending),
                      **ages, **self.quality, **self.anchor_check)
        status = DiagnosticStatus(name='aims_racer_system/localization', message=state,
                                  level=DiagnosticStatus.OK if valid else (DiagnosticStatus.ERROR if state == 'lost' else DiagnosticStatus.WARN),
                                  values=[KeyValue(key=key, value=str(value)) for key, value in values.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(array)
        self.valid_pub.publish(Bool(data=valid))

    def destroy_node(self):
        self.quality_worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
