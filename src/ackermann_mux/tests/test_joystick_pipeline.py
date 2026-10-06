# Copyright 2026 AIMSRacer.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise the installed C++ selector with synthetic inputs in an isolated DDS domain."""
import os
from pathlib import Path
import subprocess
import time

import pytest


@pytest.mark.parametrize('paused_clock', [False, True])
def test_joystick_pipeline(tmp_path, monkeypatch, paused_clock):
    # Never join the vehicle's domain or start a receiver, controller or motor driver.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(202 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from ackermann_msgs.msg import AckermannDriveStamped
    from crsf_receiver_msg.msg import CRSFChannels16
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool

    executable = (Path(get_package_prefix('ackermann_mux')) /
                  'lib/ackermann_mux/joystick_control_v2')
    args = [str(executable), '--ros-args', '-p',
            'channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10',
            '-p', f'use_sim_time:={str(paused_clock).lower()}']
    log_path = tmp_path / 'selector.log'
    context = Context()
    process = node = executor = None
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            rclpy.init(context=context)
            node = rclpy.create_node('selector_test', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            rc_pub = node.create_publisher(CRSFChannels16, '/rc/channels', qos)
            nav_pub = node.create_publisher(AckermannDriveStamped, '/drive', qos)
            calib_pub = node.create_publisher(AckermannDriveStamped, '/calib/ackermann_cmd', qos)
            clock_pub = node.create_publisher(Clock, '/clock', 10)
            commands, statuses = [], []
            node.create_subscription(AckermannDriveStamped, '/ackermann_cmd', commands.append, 10)
            node.create_subscription(Bool, '/control/autonomy_speed_enabled', statuses.append, 10)
            rc = CRSFChannels16()
            for i in range(1, 17):
                setattr(rc, f'ch{i}', 992)
            rc.ch5 = 1810  # unlock
            rc.ch6 = 172   # speed
            rc.ch7 = 172   # manual
            rc.ch8 = 172   # calibration off
            rc.ch10 = 1810

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and (
                    rc_pub.get_subscription_count() == 0 or
                    nav_pub.get_subscription_count() == 0 or
                    calib_pub.get_subscription_count() == 0 or not commands):
                assert process.poll() is None, log_path.read_text()
                executor.spin_once(timeout_sec=.01)
            assert rc_pub.get_subscription_count() > 0, log_path.read_text()
            assert commands, log_path.read_text()

            def pump(seconds, *, send_rc=True, nav=None, calib=None, restamp=True):
                end = time.monotonic() + seconds
                next_publish = 0.0
                while time.monotonic() < end:
                    assert process.poll() is None, log_path.read_text()
                    now = time.monotonic()
                    if now >= next_publish:
                        next_publish = now + .01
                        if paused_clock:
                            clock = Clock()
                            clock.clock.sec = 100
                            clock_pub.publish(clock)
                        if send_rc:
                            rc_pub.publish(rc)
                        for publisher, message in ((nav_pub, nav), (calib_pub, calib)):
                            if message is not None:
                                if restamp:
                                    if paused_clock:
                                        message.header.stamp.sec = 100
                                        message.header.stamp.nanosec = 0
                                    else:
                                        message.header.stamp = node.get_clock().now().to_msg()
                                publisher.publish(message)
                    executor.spin_once(timeout_sec=.002)

            def stopped():
                assert len(commands) >= 5
                for msg in commands[-5:]:
                    assert msg.drive.speed == 0.0
                    assert msg.drive.acceleration == 0.0
                    assert msg.drive.jerk == 0.0
                    assert msg.drive.steering_angle == 0.0

            pump(.08, send_rc=False)
            stopped()  # Before the first RC frame, actively publish a safe command.
            rc.ch3 = 1810
            pump(.15)
            assert commands[-1].drive.speed == pytest.approx(12.0)
            rc.ch6 = 992
            pump(.1)
            assert commands[-1].drive.jerk == 2.0
            assert commands[-1].drive.acceleration == pytest.approx(20.0)
            rc.ch6 = 1810
            pump(.1)
            assert commands[-1].drive.jerk == 3.0
            assert commands[-1].drive.acceleration == pytest.approx(.8)
            rc.ch5 = 172
            pump(.1)
            stopped()

            rc.ch5 = 1810
            rc.ch6 = 172
            rc.ch8 = 1810
            pump(.1)
            stopped()
            calib = AckermannDriveStamped()
            calib.drive.acceleration = 8.0
            calib.drive.jerk = 2.0
            pump(.15, calib=calib)
            assert commands[-1].drive.jerk == 2.0
            assert commands[-1].drive.acceleration == 8.0
            assert not statuses[-1].data
            commands.clear()
            pump(.35)  # RC remains alive, calibration producer disappears.
            stopped()
            assert any(msg.drive.acceleration == 8.0 for msg in commands)
            assert any(msg.drive.acceleration == 0.0 for msg in commands)
            if not paused_clock:
                # Even continuous receipt cannot renew a stale source timestamp.
                pump(.15, calib=calib, restamp=False)
                stopped()
            pump(.1, calib=calib)
            assert commands[-1].drive.acceleration == 8.0
            rc.ch5 = 172
            pump(.1)
            stopped()
            rc.ch5 = 1810
            pump(.1)
            stopped()  # Unlock does not revive a cached calibration command.
            pump(.1, calib=calib)
            assert commands[-1].drive.acceleration == 8.0
            pump(.35, send_rc=False)
            stopped()
            pump(.1)
            stopped()  # RC recovery must receive a new calibration command.

            for jerk, speed, acceleration in ((0.0, 2.0, 0.0), (3.0, 0.0, .2)):
                calib.drive.jerk = jerk
                calib.drive.speed = speed
                calib.drive.acceleration = acceleration
                pump(.1, calib=calib)
                assert commands[-1].drive.jerk == jerk
                assert commands[-1].drive.speed == pytest.approx(speed)
                assert commands[-1].drive.acceleration == pytest.approx(acceleration)
            calib.drive.jerk = 1.0  # Unsupported by downstream.
            pump(.1, calib=calib)
            stopped()
            calib.drive.jerk = 0.0
            calib.drive.speed = float('nan')
            pump(.1, calib=calib)
            stopped()
            calib.drive.speed = 2.0
            calib.header.stamp.sec = calib.header.stamp.nanosec = 0
            pump(.1, calib=calib, restamp=False)
            stopped()

            rc.ch8 = 172
            rc.ch7 = 1810
            pump(.1)
            stopped()
            assert statuses[-1].data  # Navigation producer may now start; no circular enable.
            nav = AckermannDriveStamped()
            nav.drive.speed = 3.0
            nav.drive.steering_angle = .2
            pump(.15, nav=nav)
            assert commands[-1].drive.speed == 3.0
            pump(.35)
            stopped()
            assert statuses[-1].data
            rc.ch6 = 992
            pump(.1, nav=nav)
            stopped()
            assert not statuses[-1].data
            rc.ch6 = 1810
            pump(.1, nav=nav)
            stopped()
            rc.ch6 = 172
            pump(.1, nav=nav)
            assert commands[-1].drive.speed == 3.0
            rc.ch7 = 172
            rc.ch3 = 992
            pump(.1, nav=nav)
            stopped()  # Manual selection overrides a live navigation command.
            assert not statuses[-1].data

            # Output continues around 200 Hz, including with paused simulation time.
            commands.clear()
            pump(.3)
            assert 35 <= len(commands) <= 85
            assert process.poll() is None
            # No warning spam while stopped. Failures log only transitions.
            assert len(log_path.read_text().splitlines()) < 80
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
