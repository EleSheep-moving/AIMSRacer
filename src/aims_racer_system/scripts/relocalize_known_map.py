#!/usr/bin/env python3
"""Initialize NDT with an explicit map/base_link pose and verify new trusted epoch."""
import argparse
import hashlib
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from localization_policy import initial_pose_quaternion, initialization_complete, unsigned


class MapInitializer(Node):
    def __init__(self, target):
        super().__init__('map_initializer')
        self.map_sha256 = None
        self.status = None
        self.status_received = None
        self.health_epoch = ''
        self.health_sequence = -1
        self.retired_epochs = set()
        self.create_subscription(String, '/localization/map_sha256',
            lambda message: setattr(self, 'map_sha256', message.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/localization/status', self.health, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.lifecycle = self.create_client(GetState, target.rstrip('/') + '/get_state')

    def health(self, message):
        for status in message.status:
            if status.name == 'aims_racer_system/localization':
                values = {item.key: item.value for item in status.values}
                try:
                    epoch = values['epoch']
                    sequence = unsigned(values['health_sequence'])
                    if values['protocol_version'] != '1' or epoch in self.retired_epochs:
                        return
                    if epoch != self.health_epoch:
                        if self.health_epoch:
                            self.retired_epochs.add(self.health_epoch)
                        self.health_epoch, self.health_sequence = epoch, -1
                    if sequence <= self.health_sequence:
                        return
                    self.health_sequence = sequence
                except (KeyError, ValueError):
                    return
                self.status = values
                self.status_received = time.monotonic()


def wait_until(node, predicate, deadline):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if predicate():
            return True
    return False


def wait_until_active(node, deadline):
    while time.monotonic() < deadline:
        future = node.lifecycle.call_async(GetState.Request())
        if not wait_until(node, future.done, deadline):
            return False
        response = future.result()
        if response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_file', help='exact immutable PCD passed to known_map_localization.launch.py')
    parser.add_argument('--pose-frame', choices=['base_link'], required=True,
                        help='xyz/rpy specify base_link in map; angles are radians')
    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        parser.add_argument('--' + name, type=float, default=0.)
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--ndt-node', default='/lidar_localization')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    try:
        quaternion = initial_pose_quaternion(args.pose_frame, [args.x,args.y,args.z], [args.roll,args.pitch,args.yaw])
        if not 0. < args.timeout < float('inf'):
            raise ValueError('timeout must be finite and positive')
        path = Path(args.map_file).expanduser().resolve(strict=True)
        expected = hashlib.sha256(path.read_bytes()).hexdigest()
    except (ValueError, OSError) as error:
        parser.error(str(error))
    rclpy.init()
    node = MapInitializer(args.ndt_node)
    try:
        deadline = time.monotonic() + args.timeout
        if not wait_until(node, lambda: node.map_sha256 is not None and node.status is not None, deadline):
            raise RuntimeError('localization map identity/health unavailable')
        if node.map_sha256 != expected:
            raise RuntimeError('PCD SHA-256 differs from localization monitor map')
        if not wait_until(node, node.lifecycle.service_is_ready, deadline):
            raise RuntimeError('NDT lifecycle service unavailable')
        if not wait_until_active(node, deadline):
            raise RuntimeError('NDT must be lifecycle active before initialization')
        if not wait_until(node, lambda: node.publisher.get_subscription_count() > 0, deadline):
            raise RuntimeError('no /initialpose subscriber')
        previous_epoch = node.status.get('epoch', '')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = args.x,args.y,args.z
        q = pose.pose.pose.orientation
        q.x,q.y,q.z,q.w = quaternion
        for index, variance in ((0,.25),(7,.25),(14,.25),(21,.04),(28,.04),(35,.04)):
            pose.pose.covariance[index] = variance
        node.publisher.publish(pose)
        if not wait_until(node, lambda: node.map_sha256 == expected and initialization_complete(
                node.status, previous_epoch, node.get_clock().now().nanoseconds * 1e-9,
                time.monotonic(), node.status_received, source_now_ns=node.get_clock().now().nanoseconds), deadline):
            raise RuntimeError('new NDT epoch with ready, fresh committed anchor not observed')
        print(f'Known-map NDT initialized: {path} (SHA-256 {expected}, epoch {node.status["epoch"]}, '
              f'anchor {node.status.get("anchor_sequence", "?")})')
        return 0
    except (RuntimeError, OSError) as error:
        print(f'Relocalization failed: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
