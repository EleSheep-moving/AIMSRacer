# Copyright 2026 AIMSRacer. Licensed under the Apache License, Version 2.0.
"""RC/nav -> installed selector -> native driver -> PTY -> wheel odometry."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_vehicle_pipeline(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from ackermann_msgs.msg import AckermannDriveStamped
    from crsf_receiver_msg.msg import CRSFChannels16
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from vesc_msgs.msg import VescCommand, VescStateStamped, VescImuStamped

    # Reuse the VESC submodule's protocol emulator; it creates /dev/pts/* only.
    helper = Path(__file__).resolve().parents[2]/'vesc/vesc_driver/test/test_driver_link.py'
    spec = importlib.util.spec_from_file_location('vesc_pty', helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    emulator = module.VescEmulator()
    assert emulator.port.startswith('/dev/pts/')
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('native_vehicle_pipeline', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    rc_pub = node.create_publisher(CRSFChannels16, '/rc/channels', 1)
    nav_pub = node.create_publisher(AckermannDriveStamped, '/drive', 1)
    calib_pub = node.create_publisher(VescCommand, '/calib/vesc_command', 1)
    samples = {key: [] for key in ('command', 'core', 'imu', 'odom')}
    for key, topic, kind in [('command', '/vesc/command', VescCommand),
            ('core', '/sensors/core', VescStateStamped), ('imu', '/sensors/imu', VescImuStamped),
            ('odom', '/rear_axle/wheel_odom', Odometry)]:
        node.create_subscription(kind, topic,
            lambda msg, key=key: samples[key].append((time.monotonic(), msg)), 10)
    authority = []
    node.create_subscription(Bool, '/control/autonomy_speed_enabled', authority.append, 10)
    rc = CRSFChannels16()
    for i in range(1, 17):
        setattr(rc, f'ch{i}', 992)
    rc.ch5, rc.ch6, rc.ch7, rc.ch8, rc.ch10 = 1810, 172, 1810, 172, 1810
    nav = AckermannDriveStamped()
    nav.drive.speed, nav.drive.steering_angle = 2.0, .2

    def binary(pkg, name):
        return str(Path(get_package_prefix(pkg))/'lib'/pkg/name)

    common = ['--ros-args', '-p', 'speed_to_erpm_gain:=3465.0', '-p',
              'steering_angle_to_servo_gain:=-0.5137', '-p', 'steering_angle_to_servo_offset:=0.506',
              '-p', 'speed_to_erpm_offset:=0.0']
    args = [
        [binary('ackermann_mux', 'joystick_control_v2'), '--ros-args', '-p',
         'channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10'],
        [binary('vesc_driver', 'vesc_driver_node'), *common, '-p', f'port:={emulator.port}',
         '-p', 'speed_min:=-40000.0', '-p', 'speed_max:=40000.0', '-p',
         'current_min:=-80.0', '-p', 'current_max:=80.0', '-p', 'duty_cycle_min:=-1.0',
         '-p', 'duty_cycle_max:=1.0', '-p', 'servo_min:=0.264', '-p', 'servo_max:=0.750',
         '-p', 'current_startup_enabled:=false'],
        [binary('vesc_ackermann', 'vesc_to_odom_node'), *common, '-p', 'wheelbase:=0.36',
         '-p', 'publish_tf:=false', '-r', 'odom:=/rear_axle/wheel_odom'],
    ]
    logs = [(tmp_path/f'node{i}.log').open('w') for i in range(3)]
    processes = [subprocess.Popen(a, stdout=log, stderr=subprocess.STDOUT)
                 for a, log in zip(args, logs)]

    def pump(seconds, send_rc=True, send_nav=False, calib=None):
        end, next_publish = time.monotonic()+seconds, 0.0
        while time.monotonic() < end:
            assert all(p.poll() is None for p in processes), [
                (tmp_path/f'node{i}.log').read_text() for i in range(3)]
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic()+.01
                if send_rc:
                    rc_pub.publish(rc)
                if send_nav:
                    nav.header.stamp = node.get_clock().now().to_msg()
                    nav_pub.publish(nav)
                if calib is not None:
                    calib.header.stamp = node.get_clock().now().to_msg()
                    calib_pub.publish(calib)
            executor.spin_once(timeout_sec=.001)

    def last():
        return samples['command'][-1][1]

    def motors_after(start):
        return [(t, p) for t, p in emulator.packets if t > start and p[0] in (5, 6, 7, 8)]

    def value(p):
        return int.from_bytes(p[1:], 'big', signed=True)

    try:
        end = time.monotonic()+8
        while time.monotonic() < end and not all(samples.values()):
            pump(.05, send_nav=True)
        assert all(samples.values())
        assert authority[-1].data
        start = time.monotonic()
        pump(3, send_nav=True)
        end = time.monotonic()
        measured = {}
        for key, records in samples.items():
            hz = sum(start+.2 <= t <= end-.2 for t, _ in records)/(end-start-.4)
            measured[key] = hz
            assert 90 <= hz <= 110, (key, hz)
        motor = [p for t, p in emulator.packets if start+.2 <= t <= end-.2 and p[0] == 8]
        assert motor and all(value(p) == 6930 for p in motor)
        assert last().target_speed_mps == 2. and last().steering_angle == pytest.approx(.2)
        assert all(m.twist.covariance[0] == pytest.approx(.04) for _, m in samples['odom'])
        for topic in ('/ackermann_cmd', '/commands/motor/speed', '/commands/servo/position'):
            assert node.count_subscribers(topic) == node.count_publishers(topic) == 0

        # Nav feedforward/legacy jerk modes cannot leak into native current commands.
        nav.drive.jerk = 1.
        pump(.1, send_nav=True)
        assert last().control_mode == 0 and last().target_speed_mps == 0.
        nav.drive.jerk = 0.
        pump(.1, send_nav=True)
        pump(.3)  # Source expires while RC keeps navigation authority selected.
        assert last().target_speed_mps == 0. and authority[-1].data
        rc.ch7, rc.ch6, rc.ch3 = 172, 992, 1810
        start = time.monotonic()
        pump(.15, send_nav=True)
        assert last().control_mode == 1 and last().target_current == 20.
        assert not authority[-1].data
        assert any(p[0] == 6 and value(p) == 20000 for _, p in motors_after(start+.03))
        rc.ch6 = 1810
        start = time.monotonic()
        pump(.15)
        assert last().control_mode == 2 and last().target_duty == pytest.approx(.8)
        assert any(p[0] == 5 and abs(value(p)-80000) <= 1 for _, p in motors_after(start+.03))

        rc.ch8 = 1810
        pump(.08)
        assert last().target_speed_mps == last().target_current == last().target_duty == 0.
        calib = VescCommand(control_mode=VescCommand.MODE_CURRENT, target_current=8.)
        calib.header.frame_id = 'base_link'
        pump(.15, calib=calib)
        assert last().target_current == 8.
        rc.ch5 = 172
        pump(.08)
        assert last().control_mode == 0 and last().target_speed_mps == 0.
        rc.ch5 = 1810
        pump(.08)
        assert last().target_current == 0.  # Cached calibration cannot survive unlock.
        pump(.1, calib=calib)
        assert last().target_current == 8.
        pump(.3)
        assert last().control_mode == 0 and last().target_speed_mps == 0.

        # Driver watchdog protects the final link when the central publisher stalls.
        rc.ch8, rc.ch7, rc.ch6 = 172, 1810, 172
        pump(.1, send_nav=True)
        processes[0].send_signal(signal.SIGSTOP)
        stalled = time.monotonic()
        pump(.3)
        assert any(p[0] == 8 and value(p) == 0 for _, p in motors_after(stalled+.15))
        assert all(p[0] == 8 and value(p) == 0 for _, p in motors_after(stalled+.15))
        processes[0].send_signal(signal.SIGCONT)
        pump(.05, send_rc=False)
        assert last().target_speed_mps == 0.
        pump(.15, send_nav=True)
        assert last().target_speed_mps == 2.
        pump(.3, send_rc=False, send_nav=True)
        assert last().target_speed_mps == 0. and not authority[-1].data
        assert not emulator.errors
        (tmp_path/'metrics.json').write_text(json.dumps(measured, indent=2)+'\n')
        print('PTY rates Hz: '+json.dumps(measured, sort_keys=True))
    finally:
        for p in processes:
            p.send_signal(signal.SIGCONT)
            p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=5)
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        emulator.close()
