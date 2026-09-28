#!/usr/bin/env python3
"""Expose localizer map->odom TF only after verified known-map alignment."""

import hashlib
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from interface.srv import IsValid


class MapTfGate(Node):
    def __init__(self):
        super().__init__('map_tf_gate')
        self.declare_parameter('map_file', '')
        map_file = Path(self.get_parameter('map_file').value)
        if not map_file.is_file():
            raise ValueError('map_file must name an existing PGO map.pcd')
        self.map_sha256 = hashlib.sha256(map_file.read_bytes()).hexdigest()
        self.valid = False
        self.last_check = -math.inf
        self.last_tf_relay = -math.inf
        self.pending = None
        self.check_client = self.create_client(IsValid, '/localizer/relocalize_check')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, '/localizer/raw_tf', self.raw_tf, 100)
        self.valid_pub = self.create_publisher(Bool, '/localization/map_valid', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.identity_pub = self.create_publisher(String, '/localization/map_sha256', latched)
        self.identity_pub.publish(String(data=self.map_sha256))
        self.create_timer(.1, self.tick)

    def checked(self, future):
        try:
            self.valid = bool(future.result().valid)
            self.last_check = time.monotonic()
            if not self.valid:
                self.last_tf_relay = -math.inf
        except Exception as exc:
            self.valid = False
            self.last_tf_relay = -math.inf
            self.get_logger().warning(f'Known-map check failed: {exc}')
        self.pending = None

    def tick(self):
        now = time.monotonic()
        if now - self.last_check > .3:
            self.valid = False
            self.last_tf_relay = -math.inf
        if self.pending is None and self.check_client.service_is_ready():
            request = IsValid.Request()
            request.code = 0  # code 1 bypasses the upstream validity check.
            self.pending = self.check_client.call_async(request)
            self.pending.add_done_callback(self.checked)
        ready = (self.valid and now - self.last_check <= .3 and
                 now - self.last_tf_relay <= .3)
        self.valid_pub.publish(Bool(data=ready))

    def raw_tf(self, msg):
        if not self.valid or time.monotonic() - self.last_check > .3:
            return
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        for transform in msg.transforms:
            if transform.header.frame_id != 'map' or transform.child_frame_id != 'odom':
                continue
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if 0 <= ros_now - stamp <= .3:
                self.broadcaster.sendTransform(transform)
                self.last_tf_relay = time.monotonic()


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
