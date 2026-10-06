#!/usr/bin/env python3
"""Publish one bias-corrected gyro stream for both rear-axle adapters."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from aims_racer_system.gyro_bias import GyroBiasCalibrator, correct_imu


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class LivoxGyroBias(Node):
    def __init__(self):
        super().__init__('livox_gyro_bias')
        defaults=dict(duration=10.,min_samples=1000,max_observation_age=.25,
            max_imu_gap=.05,max_wheel_speed=.02,max_lio_speed=.025,
            max_position_change=.02,max_orientation_change=.01,
            max_gyro_norm=.1,max_gyro_std=.005,max_accel_std=.02,
            accel_norm_min=.75,accel_norm_max=1.25)
        self.calibrator=GyroBiasCalibrator(**{k:self.declare_parameter(k,v).value for k,v in defaults.items()})
        self.output=self.create_publisher(Imu,'/livox/imu_bias_corrected',50)
        self.status=self.create_publisher(DiagnosticArray,'/imu/gyro_bias/status',10)
        self.create_subscription(Imu,'/livox/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/rear_axle/wheel_odom',self.wheel,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/fastlio2/lio_odom',self.lio,qos_profile_sensor_data)
        self.create_timer(1.,self.diagnostics)
        self.get_logger().info('Gyro calibration requires a stationary car for %.1f s; FAST-LIO keeps raw IMU input.' % self.calibrator.duration)

    def wheel(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
            return
        try:self.calibrator.add_wheel(seconds(msg.header.stamp),msg.twist.twist.linear.x)
        except ValueError as error:self.get_logger().warning(str(error),throttle_duration_sec=2.)

    def lio(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='livox_frame':
            return
        p,q,v=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear
        try:self.calibrator.add_lio(seconds(msg.header.stamp),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[v.x,v.y,v.z])
        except ValueError as error:self.get_logger().warning(str(error),throttle_duration_sec=2.)

    def imu(self,msg):
        if msg.header.frame_id!='livox_frame' or msg.angular_velocity_covariance[0]==-1.:
            return
        w,a=msg.angular_velocity,msg.linear_acceleration
        was_ready=self.calibrator.ready
        corrected=self.calibrator.process(seconds(msg.header.stamp),[w.x,w.y,w.z],[a.x,a.y,a.z])
        if corrected is not None:
            self.output.publish(correct_imu(msg,self.calibrator.bias))
        if self.calibrator.ready and not was_ready:
            self.get_logger().info('Gyro bias calibrated in livox_frame [rad/s]: '+json.dumps(self.calibrator.bias.tolist()))
            self.diagnostics()

    def diagnostics(self):
        c=self.calibrator
        item=DiagnosticStatus();item.name='livox_gyro_bias';item.hardware_id='livox_frame'
        item.level=b'\x00' if c.ready else b'\x01';item.message=c.reason
        fields=dict(ready=c.ready,gyro_bias_radps=c.bias.tolist(),samples=c.sample_count,
            gyro_std_radps=c.gyro_std.tolist(),frame_id='livox_frame',duration_s=c.duration)
        item.values=[KeyValue(key=k,value=json.dumps(v)) for k,v in fields.items()]
        out=DiagnosticArray();out.header.stamp=self.get_clock().now().to_msg();out.status=[item]
        self.status.publish(out)


def main():
    rclpy.init();node=LivoxGyroBias()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
