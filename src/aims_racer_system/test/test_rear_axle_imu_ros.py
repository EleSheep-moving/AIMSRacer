import importlib.util
from pathlib import Path
import time
import numpy as np
import pytest
import rclpy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def test_rear_imu_gravity_attitude_warmup_and_stale_attitude():
    path=Path(__file__).parents[1]/'scripts/imu_to_rear_axle.py'
    spec=importlib.util.spec_from_file_location('rear_imu_node',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args','-p','imu_translation:=[0.3,0.0,0.0]'])
    node=module.ImuToRearAxle(); probe=rclpy.create_node('rear_imu_probe')
    received=[]; probe.create_subscription(Imu,'/livox/imu_ekf',received.append,10)
    def drain():
        end=time.monotonic()+.03
        while time.monotonic()<end: rclpy.spin_once(probe,timeout_sec=.005)
    try:
        end=time.monotonic()+.3
        while time.monotonic()<end: rclpy.spin_once(probe,timeout_sec=.01)
        attitude=Rotation.from_euler('xyz',[.2,-.3,0.])
        force=attitude.inv().apply([0.,0.,9.80665])
        imu=Imu(); imu.header.stamp.sec=10; imu.header.frame_id='laser'
        imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=(force/9.80665).tolist()
        node.imu(imu); drain()
        assert received[-1].linear_acceleration_covariance[0] == -1.
        odom=Odometry(); odom.header.stamp.sec=10
        odom.header.frame_id='odom'; odom.child_frame_id='livox_imu'
        q=attitude.as_quat().tolist()
        odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=q
        node.odometry(odom)
        for i in range(1,9):
            imu.header.stamp.nanosec=i*5_000_000; node.imu(imu)
        drain(); out=received[-1]
        assert out.header.frame_id == 'base_link'
        assert out.header.stamp.nanosec == 40_000_000
        assert out.linear_acceleration_covariance[0] > 0
        assert [out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z] == pytest.approx(force)
        assert [out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w] == pytest.approx(q)
        for i in range(9,70):
            imu.header.stamp.nanosec=i*5_000_000; node.imu(imu)
        drain()
        assert received[-1].linear_acceleration_covariance[0] == -1.
        assert received[-1].orientation_covariance[0] == -1.
        # Invalid accelerometer data must not crash or enter the filter.
        count=len(received); imu.linear_acceleration.x=float('nan')
        node.imu(imu); drain(); assert len(received)==count
    finally:
        node.destroy_node(); probe.destroy_node(); rclpy.shutdown()
