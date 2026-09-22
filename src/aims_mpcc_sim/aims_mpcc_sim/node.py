"""ROS adapter for the numerical vehicle plant."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from crsf_receiver_msg.msg import CRSFChannels16

from .plant import LaggedBicyclePlant, autonomous_channels, odometry_fields


class NumericalPlantNode(Node):
    def __init__(self):
        super().__init__('aims_mpcc_numerical_plant')
        for name, value in (
            ('wheelbase', 0.36), ('speed_to_erpm_gain', 4650.0),
            ('steering_gain', -0.5137), ('steering_offset', 0.506),
            ('speed_tau', 0.2), ('steering_tau', 0.15),
            ('initial_x', 2.0), ('initial_y', 0.0), ('initial_yaw', math.pi / 2),
            ('odom_rate_hz', 50.0),
        ):
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.plant = LaggedBicyclePlant(
            value('wheelbase'), value('speed_to_erpm_gain'), value('steering_gain'),
            value('steering_offset'), value('speed_tau'), value('steering_tau'),
            x=value('initial_x'), y=value('initial_y'), yaw=value('initial_yaw'),
        )
        self.odom_period = 1.0 / value('odom_rate_hz')
        self.last_step = self.last_odom = time.monotonic()
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.rc_pub = self.create_publisher(CRSFChannels16, '/rc/channels', qos_profile_sensor_data)
        self.create_subscription(Float64, '/commands/motor/speed', self._erpm, 10)
        self.create_subscription(Float64, '/commands/servo/position', self._servo, 10)
        self.create_timer(0.005, self._tick)

    def _erpm(self, msg):
        self.plant.receive_erpm(msg.data)

    def _servo(self, msg):
        self.plant.receive_servo(msg.data)

    def _tick(self):
        now = time.monotonic()
        elapsed = min(0.05, now - self.last_step)
        self.last_step = now
        count = max(1, math.ceil(elapsed / 0.002))
        for _ in range(count):
            self.plant.step(elapsed / count)
        if now - self.last_odom < self.odom_period:
            return
        self.last_odom = now
        self._publish_rc()
        self._publish_odometry()

    def _publish_rc(self):
        message = CRSFChannels16()
        for index, channel in enumerate(autonomous_channels(), start=1):
            setattr(message, f'ch{index}', channel)
        self.rc_pub.publish(message)

    def _publish_odometry(self):
        values = odometry_fields(self.plant)
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = values['x']
        message.pose.pose.position.y = values['y']
        message.pose.pose.orientation.z = math.sin(values['yaw'] / 2.0)
        message.pose.pose.orientation.w = math.cos(values['yaw'] / 2.0)
        message.twist.twist.linear.x = values['speed']
        message.twist.twist.angular.z = values['yaw_rate']
        self.odom_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NumericalPlantNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
