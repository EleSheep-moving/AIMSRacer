#!/usr/bin/env python3
"""Configure and activate the upstream lifecycle node, with bounded wall timeout."""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


def main():
    rclpy.init()
    node = Node('activate_ndt')
    target = node.declare_parameter('target', '/lidar_localization').value
    client = node.create_client(ChangeState, target + '/change_state')
    deadline = time.monotonic() + 60.
    try:
        while not client.wait_for_service(timeout_sec=.2):
            if time.monotonic() > deadline:
                raise RuntimeError('NDT lifecycle service unavailable')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not future.done() or not future.result().success:
                raise RuntimeError(f'NDT lifecycle transition {transition} failed')
        node.get_logger().info('NDT configured and active')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
