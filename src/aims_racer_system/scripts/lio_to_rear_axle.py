#!/usr/bin/env python3
"""Convert unified Livox-origin odometry to the rear-axle reference point."""
from collections import deque
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from aims_racer_system.rear_axle_odometry import convert_odometry, source_time_angular_velocity


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class LioToRearAxle(Node):
    def __init__(self):
        super().__init__('lio_to_rear_axle')
        self.translation = self.declare_parameter('livox_translation', [.0, .0, .0]).value
        self.quaternion = self.declare_parameter('livox_quaternion', [.0, .0, .0, 1.]).value
        self.max_interval_ns = int(self.declare_parameter('max_gyro_interval', .03).value * 1e9)
        if self.max_interval_ns <= 0:
            raise ValueError('max_gyro_interval must be positive')
        self.pose_variance = self.declare_parameter('pose_variance', [.01]*6).value
        self.twist_variance = self.declare_parameter('twist_variance', [.04]*3+[.01]*3).value
        self.broadcast = self.declare_parameter('publish_tf', False).value
        self.history = deque(maxlen=2000)
        self.pending = deque(maxlen=20)
        self.last_odom = None
        self.publisher = self.create_publisher(Odometry, '/rear_axle/lio_odom', 10)
        self.tf = TransformBroadcaster(self) if self.broadcast else None
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odometry, qos_profile_sensor_data)

    def imu(self, msg):
        if msg.header.frame_id != 'livox_frame':
            self.get_logger().warning('Expected raw IMU in livox_frame', throttle_duration_sec=2.)
            return
        stamp = stamp_nanoseconds(msg.header.stamp)
        w = msg.angular_velocity
        if not all(math.isfinite(x) for x in [stamp, w.x, w.y, w.z]):
            return
        if self.history and stamp < self.history[-1][0]:
            self.history.clear()
            self.pending.clear()
            self.last_odom = None
        if self.history and stamp == self.history[-1][0]:
            return
        self.history.append((stamp, [w.x, w.y, w.z]))
        self.publish_pending()

    def odometry(self, msg):
        stamp = stamp_nanoseconds(msg.header.stamp)
        if ((self.last_odom is not None and stamp <= self.last_odom) or
                (self.pending and stamp <= stamp_nanoseconds(self.pending[-1].header.stamp))):
            self.get_logger().warning('Dropping nonmonotonic LIO stamp', throttle_duration_sec=2.)
            return
        if len(self.pending) == self.pending.maxlen:
            self.get_logger().warning('Dropping oldest pending LIO: IMU coverage unavailable', throttle_duration_sec=2.)
        self.pending.append(msg)
        self.publish_pending()

    def publish_pending(self):
        while self.pending and self.history:
            msg = self.pending[0]
            stamp = stamp_nanoseconds(msg.header.stamp)
            if stamp > self.history[-1][0]:
                return  # Wait for a measured IMU knot; never use an old rate as a prediction.
            omega = source_time_angular_velocity(self.history, stamp, self.max_interval_ns)
            self.pending.popleft()
            if omega is None:
                self.get_logger().warning('Dropping LIO: missing source-time IMU coverage', throttle_duration_sec=2.)
                continue
            self.publish_converted(msg, stamp, omega)

    def publish_converted(self, msg, stamp, omega):
        try:
            out = convert_odometry(msg, omega, self.translation, self.quaternion,
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
