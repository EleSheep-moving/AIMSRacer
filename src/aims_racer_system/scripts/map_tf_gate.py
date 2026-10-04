#!/usr/bin/env python3
"""Bounded map-correction holding with independent scan/map consistency checks."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import io
import json
import math
from pathlib import Path
import time

import numpy as np
import message_filters
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from interface.srv import IsValid


from localization_map_io import map_points


def scan_match_quality(cloud, odometry, transform, tree, distance_limit, max_points):
    """Check a fixed alignment; no ICP optimization or pose correction occurs here."""
    started = time.perf_counter()
    if (cloud.header.frame_id != 'livox_frame' or cloud.height != 1 or
            (odometry.header.frame_id, odometry.child_frame_id) != ('odom', 'livox_frame')):
        raise ValueError('Expected FAST-LIO body cloud and raw odometry in odom / livox_frame')
    if cloud.header.stamp != odometry.header.stamp:
        raise ValueError('Body cloud and raw LIO odometry must have the same scan-end stamp')
    values = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z'])
    xyz = np.column_stack([values[k] for k in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 100:
        raise ValueError('Insufficient finite scan points')
    xyz = xyz[::max(1, math.ceil(len(xyz) / max_points))]
    pose = odometry.pose.pose
    oq, ot = pose.orientation, pose.position
    pose_values = np.array([ot.x, ot.y, ot.z, oq.x, oq.y, oq.z, oq.w])
    if not np.isfinite(pose_values).all() or abs(np.linalg.norm(pose_values[3:]) - 1.) > .01:
        raise ValueError('Raw LIO pose must be finite with a unit quaternion')
    lio_rotation = Rotation.from_quat(pose_values[3:]).as_matrix()
    xyz = xyz @ lio_rotation.T + pose_values[:3]
    q = transform.transform.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t = transform.transform.translation
    distances, _ = tree.query(xyz @ rotation.T + [t.x, t.y, t.z], workers=1)
    inside = distances <= distance_limit
    return dict(points=len(xyz), inlier_fraction=float(inside.mean()),
                inlier_rmse_m=float(np.sqrt(np.mean(distances[inside] ** 2))) if inside.any() else None,
                check_time_ms=(time.perf_counter() - started) * 1000)


class MapTfGate(Node):
    def __init__(self):
        super().__init__('map_tf_gate')
        self.declare_parameter('map_file', '')
        settings = dict(quality_period=.5, match_distance=.25, max_check_points=2000)
        for key, value in settings.items():
            self.declare_parameter(key, value)
            setattr(self, key, self.get_parameter(key).value)
            if isinstance(getattr(self, key), bool) or not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if not isinstance(self.max_check_points, int) or self.max_check_points < 100:
            raise ValueError('max_check_points must be an integer of at least 100')
        map_file = Path(self.get_parameter('map_file').value)
        data = map_file.read_bytes()
        self.map_sha256 = hashlib.sha256(data).hexdigest()
        self.tree = cKDTree(map_points(data))
        self.valid = False
        self.last_check = -math.inf
        self.last_raw_received = -math.inf
        self.last_cloud_received = -math.inf
        self.transform = None
        self.cloud = None
        self.odometry = None
        self.pending = None
        self.quality = None
        self.quality_stamp = None
        self.quality_future = None
        self.quality_generation = 0
        self.last_quality_submit = -math.inf
        self.last_status_publish = -math.inf
        self.initial_check_error = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='map-consistency')
        self.check_client = self.create_client(IsValid, '/localizer/relocalize_check')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, '/localizer/raw_tf', self.raw_tf, 100)
        # Use the same functional scan/pose inputs as ICP/PGO. Exact source-time
        # pairing avoids applying a newer pose (or the EKF TF) to an older scan.
        self.cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/fastlio2/body_cloud', qos_profile=qos_profile_sensor_data)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, '/fastlio2/lio_odom', qos_profile=qos_profile_sensor_data)
        self.scan_sync = message_filters.TimeSynchronizer([self.cloud_sub, self.odom_sub], 10)
        self.scan_sync.registerCallback(self.matched_scan)
        self.valid_pub = self.create_publisher(Bool, '/localization/map_valid', 10)
        self.status_pub = self.create_publisher(DiagnosticArray, '/localization/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.identity_pub = self.create_publisher(String, '/localization/map_sha256', latched)
        self.identity_pub.publish(String(data=self.map_sha256))
        self.create_timer(.1, self.check_initial_alignment)
        self.create_timer(.02, self.tick)

    @staticmethod
    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def checked(self, future):
        try:
            verified = bool(future.result().valid)
            self.last_check = time.monotonic()
            self.initial_check_error = None
            if not verified:
                self.transform = None
                self.cloud = None
                self.odometry = None
                self.quality = None
                self.quality_stamp = None
                self.quality_generation += 1
            self.valid = verified
        except Exception as exc:
            # A failed service call is not evidence that the prior alignment
            # changed. Retain it; an explicit false response resets it above.
            self.initial_check_error = str(exc)
            self.get_logger().warning(f'Known-map check failed: {exc}')
        self.pending = None

    def check_initial_alignment(self):
        if self.pending is None and self.check_client.service_is_ready():
            request = IsValid.Request()
            request.code = 0  # Never bypass upstream initial-alignment verification.
            self.pending = self.check_client.call_async(request)
            self.pending.add_done_callback(self.checked)

    def raw_tf(self, msg):
        if not self.valid:
            return
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) != ('map', 'odom'):
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            values = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
            if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1.) > .01:
                continue
            self.transform = deepcopy(transform)
            self.last_raw_received = time.monotonic()

    def matched_scan(self, cloud, odometry):
        if (cloud.header.frame_id == 'livox_frame' and
                (odometry.header.frame_id, odometry.child_frame_id) == ('odom', 'livox_frame') and
                cloud.header.stamp == odometry.header.stamp):
            self.cloud = cloud
            self.odometry = odometry
            self.last_cloud_received = time.monotonic()

    def readiness(self, now):
        if not self.valid:
            return False, 'Waiting for verified initial alignment'
        if self.transform is None:
            return False, 'Waiting for upstream map transform'
        return True, 'Holding last map correction' if now - self.last_raw_received > .05 else 'Map correction available'

    def tick(self):
        now = time.monotonic()
        stamp = self.get_clock().now()
        ros_now = stamp.nanoseconds * 1e-9
        if self.quality_future is not None and self.quality_future.done():
            future, generation, source_stamp = self.quality_future, self.submitted_generation, self.submitted_stamp
            self.quality_future = None
            if generation == self.quality_generation:
                try:
                    self.quality = future.result()
                except Exception as exc:
                    self.quality = dict(error=str(exc))
                self.quality_stamp = source_stamp
        # A separate bounded worker checks fresh scans, so map matching cannot
        # suspend the 50 Hz TF timer. At most one consistency query is in flight.
        if (self.quality_future is None and now - self.last_quality_submit >= self.quality_period and
                self.valid and self.transform is not None and self.cloud is not None and
                self.odometry is not None and self.stamp(self.cloud.header) != self.quality_stamp):
            self.submitted_generation = self.quality_generation
            self.submitted_stamp = self.stamp(self.cloud.header)
            self.submitted_transform = deepcopy(self.transform)
            self.quality_future = self.pool.submit(scan_match_quality, self.cloud, self.odometry, self.submitted_transform,
                                                   self.tree, self.match_distance, self.max_check_points)
            self.last_quality_submit = now
        ready, reason = self.readiness(now)
        if ready:
            # Publish the latest upstream correction at the current epoch; the
            # upstream stamp is a cloud epoch, not a correction-expiry clock.
            # Original source ages remain visible separately in diagnostics.
            output = deepcopy(self.transform)
            output.header.stamp = stamp.to_msg()
            self.broadcaster.sendTransform(output)
        if now - self.last_status_publish >= .095:
            self.last_status_publish = now
            self.valid_pub.publish(Bool(data=ready))
            values = dict(map_valid=ready, initial_alignment_verified=self.valid,
                          initial_check_age_s=None if not math.isfinite(self.last_check) else now - self.last_check,
                          raw_tf_receive_age_s=None if not math.isfinite(self.last_raw_received) else now - self.last_raw_received,
                          raw_tf_source_age_s=None if self.transform is None else ros_now - self.stamp(self.transform.header),
                          body_cloud_age_s=None if self.cloud is None else ros_now - self.stamp(self.cloud.header),
                          matched_scan_receive_age_s=None if not math.isfinite(self.last_cloud_received) else now - self.last_cloud_received,
                          quality_source_age_s=None if self.quality_stamp is None else ros_now - self.quality_stamp,
                          initial_check_error=self.initial_check_error,
                          consistency_match_rate=None if self.quality is None else self.quality.get('inlier_fraction'),
                          quality=self.quality, icp_per_update_success='not exposed by upstream',
                          quality_role='diagnostic only; never gates TF or controller',
                          quality_input_topics=['/fastlio2/body_cloud', '/fastlio2/lio_odom'],
                          quality_pairing='identical scan-end timestamps; raw LIO pose, not EKF TF',
                          quality_method='fixed-transform scan-to-saved-map nearest-neighbor consistency')
            item = DiagnosticStatus(name='known_map_localization', message=reason,
                                    level=b'\x00' if ready else b'\x01')
            item.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False)) for k, v in values.items()]
            self.status_pub.publish(DiagnosticArray(header=deepcopy(output.header) if ready else self._header(stamp), status=[item]))

    @staticmethod
    def _header(stamp):
        from std_msgs.msg import Header
        return Header(stamp=stamp.to_msg(), frame_id='map')

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapTfGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
