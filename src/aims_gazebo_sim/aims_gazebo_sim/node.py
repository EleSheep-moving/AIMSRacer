"""Bridge the existing VESC command interface to a Gazebo Ackermann vehicle."""

import math
import time

import rclpy
from crsf_receiver_msg.msg import CRSFChannels16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from aims_mpcc_sim.plant import autonomous_channels
from .bridge import vesc_to_twist


class VescGazeboBridge(Node):
    """Publish Gazebo velocity commands and relay physical Gazebo odometry.

    The two inputs deliberately match the production VESC output topics. This
    keeps VESC scaling, servo polarity, the RC selector, and MPCC supervision in
    the closed loop while Gazebo supplies vehicle and contact dynamics.
    """

    def __init__(self):
        super().__init__('aims_vesc_gazebo_bridge')
        defaults = (
            ('speed_to_erpm_gain', 4650.0), ('steering_gain', -0.5137),
            ('steering_offset', 0.506), ('wheelbase', 0.36),
            ('steer_limit', 0.4), ('command_timeout_s', 0.10),
            ('command_rate_hz', 100.0), ('rc_rate_hz', 50.0),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.speed_to_erpm_gain = float(get('speed_to_erpm_gain'))
        self.steering_gain = float(get('steering_gain'))
        self.steering_offset = float(get('steering_offset'))
        self.wheelbase = float(get('wheelbase'))
        self.steer_limit = float(get('steer_limit'))
        self.command_timeout_s = float(get('command_timeout_s'))
        self.erpm = 0.0
        self.servo = self.steering_offset
        self.last_command = -math.inf
        self.cmd_pub = self.create_publisher(Twist, '/gazebo/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data)
        self.rc_pub = self.create_publisher(CRSFChannels16, '/rc/channels', qos_profile_sensor_data)
        self.create_subscription(Float64, '/commands/motor/speed', self._erpm, 10)
        self.create_subscription(Float64, '/commands/servo/position', self._servo, 10)
        self.create_subscription(Odometry, '/gazebo/odometry', self._odometry, qos_profile_sensor_data)
        self.create_timer(1.0 / float(get('command_rate_hz')), self._publish_command)
        self.create_timer(1.0 / float(get('rc_rate_hz')), self._publish_rc)

    def _erpm(self, message):
        self.erpm = message.data
        self.last_command = time.monotonic()

    def _servo(self, message):
        self.servo = message.data
        self.last_command = time.monotonic()

    def _publish_command(self):
        message = Twist()
        if time.monotonic() - self.last_command <= self.command_timeout_s:
            speed, yaw_rate = vesc_to_twist(
                self.erpm, self.servo, self.speed_to_erpm_gain,
                self.steering_gain, self.steering_offset, self.wheelbase,
                self.steer_limit,
            )
            message.linear.x = speed
            message.angular.z = yaw_rate
        self.cmd_pub.publish(message)

    def _publish_rc(self):
        message = CRSFChannels16()
        for index, channel in enumerate(autonomous_channels(), start=1):
            setattr(message, f'ch{index}', channel)
        self.rc_pub.publish(message)

    def _odometry(self, message):
        # The SDF model origin is the rear axle used by MPCC.
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        self.odom_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VescGazeboBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
