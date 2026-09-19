#!/usr/bin/env python3
"""Convert raw FAST-LIO IMU odometry; never relabel sensor pose as vehicle pose."""
from collections import deque
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from aims_racer_system.rear_axle_odometry import convert_odometry


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LioToRearAxle(Node):
    def __init__(self):
        super().__init__('lio_to_rear_axle')
        self.translation = self.declare_parameter('imu_translation', [.0, .0, .0]).value
        self.quaternion = self.declare_parameter('imu_quaternion', [.0, .0, .0, 1.]).value
        self.max_age = self.declare_parameter('max_gyro_age', .05).value
        self.pose_variance = self.declare_parameter('pose_variance', [.01]*6).value
        self.twist_variance = self.declare_parameter('twist_variance', [.04]*3+[.01]*3).value
        self.broadcast = self.declare_parameter('publish_tf', False).value
        self.history = deque(maxlen=2000)
        self.last_odom = None
        self.publisher = self.create_publisher(Odometry, '/fastlio2/base_odom', 10)
        self.tf = TransformBroadcaster(self) if self.broadcast else None
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odometry, qos_profile_sensor_data)

    def imu(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        w = msg.angular_velocity
        if not all(math.isfinite(x) for x in [stamp, w.x, w.y, w.z]):
            return
        if self.history and stamp < self.history[-1][0]:
            self.history.clear()
            self.last_odom = None
        self.history.append((stamp, [w.x, w.y, w.z]))

    def odometry(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        # No future gyro samples; do not silently publish uncorrected velocity.
        candidates = [record for record in self.history if 0 <= stamp-record[0] <= self.max_age]
        if not candidates or (self.last_odom is not None and stamp <= self.last_odom):
            self.get_logger().warning('Dropping LIO odometry: missing source-time gyro or nonmonotonic stamp', throttle_duration_sec=2.)
            return
        try:
            out = convert_odometry(msg, candidates[-1][1], self.translation, self.quaternion,
                                   self.pose_variance, self.twist_variance)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.)
            return
        self.last_odom = stamp
        self.publisher.publish(out)
        if self.tf:
            tf = TransformStamped(); tf.header = out.header; tf.child_frame_id = out.child_frame_id
            p = out.pose.pose.position
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = LioToRearAxle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
