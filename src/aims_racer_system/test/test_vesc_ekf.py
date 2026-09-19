"""Actual VESC speed conversion and EKF's vx-only input contract."""
import os
from pathlib import Path
import signal
import subprocess
import time
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped


def finish(process,log):
    if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
    try: process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL);process.wait()
    log.close()


@pytest.mark.parametrize('variance', [.04, .09])
def test_real_vesc_converter_speed_and_variance(tmp_path, variance):
    share=Path(get_package_share_directory('aims_racer_system'))
    log=open(tmp_path/'vesc.log','w')
    p=subprocess.Popen(['ros2','run','vesc_ackermann','vesc_to_odom_node','--ros-args',
                        '--params-file',str(share/'params/vesc.yaml'),
                        '-p',f'vesc_to_odom_node:longitudinal_velocity_variance:={variance}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('vesc_fixture');received=[]
    node.create_subscription(Odometry,'/odom',received.append,10)
    pub=node.create_publisher(VescStateStamped,'/sensors/core',10)
    servo=node.create_publisher(Float64,'/sensors/servo_position_command',10)
    try:
        end=time.monotonic()+3.
        while time.monotonic()<end:
            servo.publish(Float64(data=.506))
            m=VescStateStamped();m.header.stamp=node.get_clock().now().to_msg();m.state.speed=4650.*1.5
            pub.publish(m);rclpy.spin_once(node,timeout_sec=.01)
        assert p.poll() is None and len(received)>5
        assert received[-1].child_frame_id=='base_link'
        assert received[-1].twist.twist.linear.x==pytest.approx(1.5)
        assert received[-1].twist.covariance[0]==pytest.approx(variance)
    finally:
        node.destroy_node();rclpy.shutdown();finish(p,log)


def test_ekf_uses_only_vesc_longitudinal_velocity(tmp_path):
    share=Path(get_package_share_directory('aims_racer_system'))
    log=open(tmp_path/'ekf.log','w')
    p=subprocess.Popen(['ros2','run','robot_localization','ekf_node','--ros-args','-r','__node:=ekf_filter_node',
                        '--params-file',str(share/'params/ekf_rear.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('wheel_ekf_fixture');received=[]
    node.create_subscription(Odometry,'/odometry/filtered',received.append,10)
    pub=node.create_publisher(Odometry,'/odom',10)
    imu_pub=node.create_publisher(Imu,'/livox/imu_ekf',10)
    try:
        end=time.monotonic()+3.
        while time.monotonic()<end:
            m=Odometry();m.header.stamp=node.get_clock().now().to_msg()
            m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.position.x=1000.;m.pose.pose.position.y=1000.
            m.pose.pose.orientation.z=1.;m.pose.pose.orientation.w=0.
            m.twist.twist.linear.x=1.5;m.twist.twist.linear.y=20.;m.twist.twist.angular.z=5.
            m.twist.covariance[0]=.04;m.twist.covariance[7]=.001;m.twist.covariance[35]=.001
            imu=Imu();imu.header.stamp=m.header.stamp;imu.header.frame_id='base_link'
            imu.orientation.w=1.;imu.linear_acceleration.x=20.;imu.linear_acceleration.y=20.
            imu.linear_acceleration.z=9.80665
            imu.angular_velocity_covariance[8]=.001
            imu.linear_acceleration_covariance[0]=.001;imu.linear_acceleration_covariance[4]=.001
            imu_pub.publish(imu)
            pub.publish(m);rclpy.spin_once(node,timeout_sec=.01)
        assert p.poll() is None and len(received)>5
        out=received[-1]
        assert out.twist.twist.linear.x==pytest.approx(1.5,abs=.05)
        assert abs(out.twist.twist.linear.y)<.01
        assert abs(out.twist.twist.angular.z)<.01
        assert abs(out.pose.pose.position.y)<.1 and out.pose.pose.position.x<10.
    finally:
        node.destroy_node();rclpy.shutdown();finish(p,log)
