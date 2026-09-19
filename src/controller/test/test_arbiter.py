"""Run the actual existing RC node, not a mock of its policy."""
import importlib.util
import os
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from crsf_receiver_msg.msg import CRSFChannels16
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.duration import Duration


def load_node():
    root=Path(os.environ.get('AIMSRACER_SOURCE', '/ws'))
    path=root/'src/ackermann_mux/scripts/joystick_control_v2.py'
    spec=importlib.util.spec_from_file_location('arbiter_under_test',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.JoystickControl


def test_mode_status_and_expired_navigation():
    rclpy.init()
    node=load_node()()
    sink=rclpy.create_node('mode_sink')
    modes=[]
    commands=[]
    sub=sink.create_subscription(Bool,'/control/autonomy_speed_enabled',lambda m:modes.append(m.data),10)
    drive_sub=sink.create_subscription(AckermannDriveStamped,'/ackermann_cmd',lambda m:commands.append(m.drive.speed),10)
    try:
        # Default profile: lock3, ESC4, nav5, calibration7.
        rc=CRSFChannels16()
        for i in range(1,17): setattr(rc,f'ch{i}',992)
        rc.ch3=1810; rc.ch4=172; rc.ch5=1810; rc.ch7=172
        node.joystick_callback(rc)
        # Actual status publisher must exist and report independent of /drive freshness.
        assert hasattr(node,'autonomy_status_publisher')
        node.timer_callback()
        import time
        until=time.monotonic()+.15
        while time.monotonic()<until:
            rclpy.spin_once(sink,timeout_sec=.01)
            node.timer_callback()
        assert True in modes
        rc.ch5=172; node.joystick_callback(rc); node.timer_callback()
        until=time.monotonic()+.1
        while time.monotonic()<until:
            node.timer_callback(); rclpy.spin_once(sink,timeout_sec=.01)
        assert modes[-1] is False
        rc.ch5=1810;node.joystick_callback(rc)
        command=AckermannDriveStamped();command.drive.speed=.5
        node.ackermann_callback(command)
        until=time.monotonic()+.1
        while time.monotonic()<until:
            node.joystick_callback(rc);node.timer_callback();rclpy.spin_once(sink,timeout_sec=.005)
        assert .5 in commands
        node.last_nav_time=node.get_clock().now()-Duration(seconds=.3)
        until=time.monotonic()+.1
        while time.monotonic()<until:
            node.joystick_callback(rc);node.timer_callback();rclpy.spin_once(sink,timeout_sec=.005)
        assert commands[-1]==0.
    finally:
        node.destroy_node(); sink.destroy_node(); rclpy.shutdown()
