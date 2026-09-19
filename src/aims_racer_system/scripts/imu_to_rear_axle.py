#!/usr/bin/env python3
"""Publish rear-axle specific force; robot_localization removes gravity once."""
from collections import deque
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from aims_racer_system.rear_axle_imu import AngularHistory, compensate_force


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def covariance(values, fallback):
    c=np.asarray(values,dtype=float).reshape(3,3)
    if not np.isfinite(c).all(): raise ValueError('Nonfinite covariance')
    if c[0,0] == -1: raise ValueError('Sensor marks measurement unavailable')
    c=(c+c.T)/2
    if np.linalg.eigvalsh(c).min() < -1e-9: raise ValueError('Invalid covariance')
    return c if np.any(c) else np.eye(3)*fallback


class ImuToRearAxle(Node):
    def __init__(self):
        super().__init__('imu_to_rear_axle')
        self.lever=np.array(self.declare_parameter('imu_translation',[0.,0.,0.]).value)
        q=self.declare_parameter('imu_quaternion',[0.,0.,0.,1.]).value
        if self.lever.shape!=(3,) or not np.isfinite(self.lever).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
            raise ValueError('Invalid IMU mounting transform')
        self.r_bi=Rotation.from_quat(q)
        self.scale=float(self.declare_parameter('accel_scale',9.80665).value)
        self.max_attitude_age=float(self.declare_parameter('max_attitude_age',.25).value)
        window=float(self.declare_parameter('angular_acceleration_window',.025).value)
        gap=float(self.declare_parameter('max_gyro_gap',.03).value)
        if not np.isfinite([self.scale,self.max_attitude_age]).all() or self.scale<=0 or not 0<self.max_attitude_age<=1.:
            raise ValueError('Invalid scale or attitude age')
        self.history=AngularHistory(window,gap)
        self.attitudes=deque(maxlen=100)
        self.publisher=self.create_publisher(Imu,'/livox/imu_ekf',50)
        self.create_subscription(Imu,'/livox/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/fastlio2/lio_odom',self.odometry,qos_profile_sensor_data)

    def odometry(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='livox_imu': return
        q=msg.pose.pose.orientation
        quat=np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>1e-2: return
        stamp=seconds(msg.header.stamp)
        if self.attitudes and stamp<=self.attitudes[-1][0]: return
        q_wb=(Rotation.from_quat(quat)*self.r_bi.inv()).as_quat()
        c=np.array(msg.pose.covariance).reshape(6,6)[3:,3:]
        try: c=covariance(c.ravel(),.01)
        except ValueError: return
        self.attitudes.append((stamp,q_wb,c))

    def imu(self,msg):
        stamp=seconds(msg.header.stamp)
        raw_w=msg.angular_velocity; raw_f=msg.linear_acceleration
        raw_values=[raw_w.x,raw_w.y,raw_w.z,raw_f.x,raw_f.y,raw_f.z]
        if not np.isfinite(raw_values).all(): return
        try:
            r=self.r_bi.as_matrix()
            w=r@np.array(raw_values[:3])
            force=r@np.array(raw_values[3:])*self.scale
            cw=r@covariance(msg.angular_velocity_covariance,.01)@r.T
            # MID360 driver values/covariance are in g and g^2, respectively.
            cf=r@covariance(msg.linear_acceleration_covariance,.05/self.scale**2)@r.T*self.scale**2
            if self.history.add(stamp,w,cw): self.attitudes.clear()
        except ValueError: return
        out=Imu(); out.header=copy.deepcopy(msg.header); out.header.frame_id='base_link'
        out.orientation.w=1.; out.orientation_covariance[0]=-1.
        out.linear_acceleration_covariance[0]=-1.
        out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z=w.tolist()
        out.angular_velocity_covariance=cw.ravel().tolist()
        derivative=self.history.derivative()
        anchors=[a for a in self.attitudes if 0<=stamp-a[0]<=self.max_attitude_age]
        if derivative is not None and anchors:
            anchor,q,cq=anchors[-1]
            propagated=self.history.orientation(anchor,q,stamp)
            if propagated is not None:
                alpha,ca=derivative
                f,cf_out=compensate_force(force,w,alpha,self.lever,cf,cw,ca)
                # Add a conservative gravity-direction uncertainty contribution.
                attitude_variance=max(np.linalg.eigvalsh(cq).max(),1e-6)+(stamp-anchor)**2*np.linalg.eigvalsh(cw).max()
                cf_out+=np.eye(3)*9.80665**2*attitude_variance
                out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w=propagated.tolist()
                out.orientation_covariance=(np.eye(3)*attitude_variance).ravel().tolist()
                out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z=f.tolist()
                out.linear_acceleration_covariance=cf_out.ravel().tolist()
        self.publisher.publish(out)


def main():
    rclpy.init(); node=ImuToRearAxle()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
