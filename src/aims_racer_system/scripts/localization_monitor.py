#!/usr/bin/env python3
"""Observe authoritative NDT commits and independent scan/map diagnostics."""
from collections import deque
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
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from localization_map_io import map_points
from localization_policy import AnchorHealth, interpolate_pose, MapOdomPackets


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_matrix(position, quaternion):
    values = np.asarray(position + quaternion, dtype=float)
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1.) > .01:
        raise ValueError('finite pose and unit quaternion required')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def quality_check(cloud, source_pose, correction, mount, tree):
    started = time.perf_counter()
    points = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z'], skip_nans=True)
    # Humble returns tuples; newer sensor_msgs_py returns a structured ndarray.
    xyz = np.asarray([(float(row[0]), float(row[1]), float(row[2])) for row in points])
    if xyz.size == 0:
        raise ValueError('no finite scan points')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xyz = xyz[::max(1, math.ceil(len(xyz) / 2000))]
    if len(xyz) == 0:
        raise ValueError('no finite scan points')
    transform = correction @ source_pose @ mount
    world = xyz @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = tree.query(world, distance_upper_bound=np.nextafter(.25, np.inf), workers=1)
    inliers = distances <= .25
    return dict(quality_points=len(xyz), inlier_fraction=float(inliers.mean()),
                inlier_rmse_m=float(np.sqrt(np.mean(distances[inliers] ** 2))) if inliers.any() else float('nan'),
                quality_stamp_ns=stamp_ns(cloud.header.stamp), quality_time_ms=(time.perf_counter()-started)*1000.)


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        data = Path(self.declare_parameter('map_file', '').value).expanduser().read_bytes()
        self.sha = hashlib.sha256(data).hexdigest()
        self.tree = cKDTree(map_points(data))
        self.max_age = float(self.declare_parameter('anchor_max_age_sec', 1.0).value)
        self.ekf_max_age = float(self.declare_parameter('ekf_max_age_sec', .1).value)
        self.cloud_max_age = float(self.declare_parameter('cloud_max_age_sec', .5).value)
        self.health = AnchorHealth(self.max_age, int(self.declare_parameter('recovery_commits', 3).value))
        self.health_publish_epoch = ''
        self.health_publish_sequence = 0
        self.mount = None
        self.history = deque()
        self.inputs = {}
        self.cloud = None
        self.map_packets = MapOdomPackets()
        self.last_clock_ns = None
        self.quality_generation = 0
        self.quality = {}
        self.quality_error = 'waiting_for_scan_and_mount'
        self.last_quality_stamp = None
        self.future = None
        self.worker = ThreadPoolExecutor(max_workers=1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sha_pub = self.create_publisher(String, '/localization/map_sha256', latched)
        self.valid_pub = self.create_publisher(Bool, '/localization/map_valid', latched)
        self.status_pub = self.create_publisher(DiagnosticArray, '/localization/status', 10)
        self.sha_pub.publish(String(data=self.sha))
        self.subscriptions_owned = [
            self.create_subscription(Odometry, '/odometry/filtered', self.odometry, 500),
            self.create_subscription(PointCloud2, '/fastlio2/body_cloud', self.scan, qos_profile_sensor_data),
            self.create_subscription(DiagnosticArray, '/localization/anchor_status', self.anchor,
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(TFMessage, '/tf_static', self.static_transforms, static_qos)]
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.1, self.tick, clock=self.watchdog_clock)
        self.quality_timer = self.create_timer(.5, self.launch_quality, clock=self.watchdog_clock)

    def synchronize_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            self.health.clock_invalid = True
            self.health.invalidate('clock_rewind')
            self.history.clear()
            self.inputs.clear()
            self.map_packets.clear()
            self.cloud = None
            self.quality_generation += 1
            self.quality.clear()
        self.last_clock_ns = now
        return now

    def remember_input(self, key, message):
        now = self.synchronize_clock()
        source = stamp_ns(message.header.stamp)
        previous = self.inputs.get(key)
        if source > now or source < 0 or (previous and source < previous[0]):
            self.inputs.pop(key, None)
            self.health.invalidate('invalid_' + key + '_stamp')
            return False
        if previous and source == previous[0]:
            return False  # Repeated source stamps cannot refresh a wall watchdog.
        self.inputs[key] = (source, time.monotonic())
        return True

    def odometry(self, message):
        if (message.header.frame_id, message.child_frame_id) != ('odom', 'base_link'):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        try:
            pose = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        except ValueError:
            return
        if self.remember_input('ekf', message):
            source = stamp_ns(message.header.stamp) * 1e-9
            self.history.append((source, pose))
            while self.history and source - self.history[0][0] > 5.:
                self.history.popleft()

    def scan(self, message):
        if message.header.frame_id == 'livox_frame' and self.remember_input('body_cloud', message):
            self.cloud = message

    def static_transforms(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id, transform.child_frame_id) == ('base_link', 'livox_frame'):
                p, q = transform.transform.translation, transform.transform.rotation
                try:
                    self.mount = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                except ValueError:
                    self.mount = None

    def anchor(self, message):
        now = self.synchronize_clock()
        for status in message.status:
            if status.name != 'lidar_localization/anchor':
                continue
            values = {item.key: item.value for item in status.values}
            previous_epoch = self.health.epoch
            accepted = self.health.observe(values, stamp_ns(message.header.stamp), now, time.monotonic())
            if self.health.epoch != previous_epoch:
                self.map_packets.clear()
                self.quality_generation += 1
                self.quality.clear()
                self.last_quality_stamp = None
            # Missing diagnostic snapshots affect only the independent diagnostic.
            if accepted and values.get('anchor_committed') == 'true':
                try:
                    transform = pose_matrix([float(values['map_odom_' + key]) for key in ('x','y','z')],
                                            [float(values['map_odom_' + key]) for key in ('qx','qy','qz','qw')])
                    self.map_packets.add(self.health.last_anchor_stamp_ns, transform)
                except (KeyError, ValueError):
                    self.quality_error = 'missing_or_invalid_commit_snapshot'
            self.publish_status(now)

    def launch_quality(self):
        if self.future is not None or self.cloud is None:
            return
        source_ns = stamp_ns(self.cloud.header.stamp)
        if source_ns == self.last_quality_stamp:
            return
        try:
            source = interpolate_pose(list(self.history), source_ns * 1e-9)
            correction = self.map_packets.held(source_ns)
            if self.mount is None:
                raise ValueError('quality_mount_unavailable')
            if source is None:
                raise ValueError('quality_source_pose_unavailable')
            if correction is None:
                raise ValueError('quality_correction_unavailable')
            self.last_quality_stamp = source_ns
            self.future_generation = self.quality_generation
            self.future_packet_stamp = self.map_packets.held_stamp(source_ns)
            self.future = self.worker.submit(quality_check, self.cloud, source, correction, self.mount.copy(), self.tree)
        except ValueError as error:
            self.quality_error = str(error)

    def tick(self):
        now = self.synchronize_clock()
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if self.future_generation == self.quality_generation:
                    self.quality = result
                    self.quality['quality_correction_stamp_ns'] = self.future_packet_stamp
                    self.quality_error = ''
            except Exception as error:
                self.quality_error = str(error)
            self.future = None
        self.publish_status(now)

    def publish_status(self, now_ns):
        mono = time.monotonic()
        ages, present = {}, True
        for key, limit in (('ekf', self.ekf_max_age), ('body_cloud', self.cloud_max_age)):
            stamp, received = self.inputs.get(key, (None, None))
            age = (now_ns - stamp) * 1e-9 if stamp is not None else float('inf')
            wall_age = mono - received if received is not None else float('inf')
            ages[key + '_age_sec'] = age
            ages[key + '_receive_age_sec'] = wall_age
            present = present and 0. <= age <= limit and 0. <= wall_age <= limit
        values = self.health.evaluate(now_ns * 1e-9, mono, present, source_now_ns=now_ns)
        valid = values['ready']
        if values['epoch'] != self.health_publish_epoch:
            self.health_publish_epoch = values['epoch']
            self.health_publish_sequence = 0
        self.health_publish_sequence += 1
        values['health_sequence'] = self.health_publish_sequence
        values.update(map_sha256=self.sha, map_valid=valid, quality_error=self.quality_error,
                      **ages, **self.quality)
        status = DiagnosticStatus(name='aims_racer_system/localization', message=values['state'],
            level=DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR,
            values=[KeyValue(key=key, value=str(value).lower() if isinstance(value, bool) else str(value))
                    for key, value in values.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(array)
        self.valid_pub.publish(Bool(data=valid))

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
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
