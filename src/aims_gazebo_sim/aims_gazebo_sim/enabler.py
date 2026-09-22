"""Enable MPCC once its ROS service becomes ready for an interactive run."""

import time
import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_srvs.srv import SetBool


def enable_once(timeout_s=30.0):
    rclpy.init()
    node = Node('aims_mpcc_visual_enabler')
    client = node.create_client(SetBool, '/mpcc/enable')
    worker_ready = False

    def status(message):
        nonlocal worker_ready
        for item in message.status:
            if item.name == 'aims_mpcc':
                values = {pair.key: json.loads(pair.value) for pair in item.values}
                worker_ready = bool(values.get('worker_ready'))

    node.create_subscription(DiagnosticArray, '/mpcc/status', status, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not (client.service_is_ready() and worker_ready):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not (client.service_is_ready() and worker_ready):
            raise TimeoutError('MPCC service or solver worker did not become ready')
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('MPCC enable request timed out')
        if not future.result().success:
            raise RuntimeError('MPCC enable request was rejected: ' + future.result().message)
        node.get_logger().info('MPCC enabled for visual closed-loop run')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    enable_once()
