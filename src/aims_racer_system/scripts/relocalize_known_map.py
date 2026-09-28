#!/usr/bin/env python3
"""Load the selected PGO map and wait for a real localizer alignment."""

import argparse
import hashlib
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from interface.srv import IsValid, Relocalize


class MapInitializer(Node):
    def __init__(self):
        super().__init__('map_initializer')
        self.map_sha256 = None
        self.map_valid = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/localization/map_sha256',
                                 lambda msg: setattr(self, 'map_sha256', msg.data), qos)
        self.create_subscription(Bool, '/localization/map_valid',
                                 lambda msg: setattr(self, 'map_valid', msg.data), 10)
        self.reloc = self.create_client(Relocalize, '/localizer/relocalize')
        self.check = self.create_client(IsValid, '/localizer/relocalize_check')


def wait_until(node, predicate, deadline):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if predicate():
            return True
    return False


def call(node, client, request, deadline):
    if not wait_until(node, client.service_is_ready, deadline):
        raise RuntimeError(f'service {client.srv_name} unavailable')
    future = client.call_async(request)
    if not wait_until(node, future.done, deadline):
        raise RuntimeError(f'service {client.srv_name} timed out')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_file', help='the exact PGO map.pcd passed to known_map_localization.launch.py')
    parser.add_argument('--x', type=float, default=0.)
    parser.add_argument('--y', type=float, default=0.)
    parser.add_argument('--z', type=float, default=0.)
    parser.add_argument('--yaw', type=float, default=0.)
    parser.add_argument('--pitch', type=float, default=0.)
    parser.add_argument('--roll', type=float, default=0.)
    parser.add_argument('--timeout', type=float, default=60.)
    args = parser.parse_args()
    path = Path(args.map_file).expanduser().resolve()
    if not path.is_file():
        parser.error(f'map file not found: {path}')
    expected = hashlib.sha256(path.read_bytes()).hexdigest()
    rclpy.init()
    node = MapInitializer()
    try:
        deadline = time.monotonic() + args.timeout
        if not wait_until(node, lambda: node.map_sha256 is not None, deadline):
            raise RuntimeError('known-map TF gate did not publish a map identity')
        if node.map_sha256 != expected:
            raise RuntimeError('map file differs from the known-map TF gate')
        request = Relocalize.Request()
        request.pcd_path = str(path)
        for name in ('x', 'y', 'z', 'yaw', 'pitch', 'roll'):
            setattr(request, name, getattr(args, name))
        response = call(node, node.reloc, request, deadline)
        if not response.success:
            raise RuntimeError(f'map load rejected: {response.message}')
        # Upstream service success only confirms map loading; ICP runs later.
        while time.monotonic() < deadline:
            check = IsValid.Request()
            check.code = 0  # Never use code 1: it unconditionally returns true.
            if call(node, node.check, check, deadline).valid:
                if wait_until(node, lambda: node.map_valid, deadline):
                    print(f'Known-map alignment and fresh TF verified: {path} '
                          f'(SHA-256 {expected})')
                    return 0
                break
            rclpy.spin_once(node, timeout_sec=.3)
        raise RuntimeError('known-map ICP or fresh map-to-odom TF unavailable')
    except (RuntimeError, OSError) as exc:
        print(f'Relocalization failed: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
