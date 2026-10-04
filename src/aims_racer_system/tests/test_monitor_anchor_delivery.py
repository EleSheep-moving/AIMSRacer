"""NDT diagnostic, accepted pose and TF can arrive on different ROS callbacks."""
from collections import OrderedDict, deque
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from localization_monitor import LocalizationMonitor
from localization_policy import MeasurementHealth, MapOdomPackets
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def pending_monitor(age=0.):
    pose = np.eye(4)
    pose[0, 3] = .2
    values = dict(has_converged='true', fitness_score='1.', correction_rotation_deg='0.',
                  correction_translation_m='.2', correction_yaw_deg='0.')
    return SimpleNamespace(health=MeasurementHealth(), history=deque([(1., np.eye(4))]),
        poses={1000000000: pose}, pending=OrderedDict({1000000000: (None, values, time.monotonic()-age)}),
        map_odom=lambda stamp: np.eye(4))


def test_old_frozen_tf_is_retried_until_accepted_tf_arrives():
    monitor = pending_monitor()
    LocalizationMonitor.evaluate_pending(monitor)
    assert monitor.health.last_scan is None, 'Arrival ordering must not create a rejection'
    assert 1000000000 in monitor.pending
    monitor.map_odom = lambda stamp: monitor.poses[stamp]
    LocalizationMonitor.evaluate_pending(monitor)
    assert monitor.health.last_trustworthy == 1.
    assert monitor.health.accepts == 1
    assert not monitor.pending


def test_duplicate_source_tf_packet_confirms_pose_even_after_future_timer():
    monitor = pending_monitor()
    monitor.map_packets = MapOdomPackets()
    monitor.map_odom = lambda stamp: LocalizationMonitor.map_odom(monitor, stamp)
    monitor.map_packets.add(1000000000, np.eye(4))
    monitor.map_packets.add(2000000000, np.eye(4))
    LocalizationMonitor.evaluate_pending(monitor)
    assert monitor.health.last_trustworthy is None
    monitor.map_packets.add(1000000000, monitor.poses[1000000000])
    LocalizationMonitor.evaluate_pending(monitor)
    assert monitor.health.last_trustworthy == 1.
    assert monitor.health.accepts == 1


def test_permanent_anchor_mismatch_still_rejects_after_deadline():
    monitor = pending_monitor(age=.6)
    LocalizationMonitor.evaluate_pending(monitor)
    assert monitor.health.last_trustworthy is None
    assert monitor.health.reason == 'accepted_pose_not_anchored'
    assert monitor.health.accepts == 0
    assert not monitor.pending


def test_invalid_transformed_gyro_does_not_refresh_usable_input():
    monitor = SimpleNamespace(inputs={})
    monitor.input = lambda key, msg: LocalizationMonitor.input(monitor, key, msg)
    msg = Imu()
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = 1
    msg.angular_velocity_covariance = [.01,0.,0.,0.,.01,0.,0.,0.,.01]
    LocalizationMonitor.gyro(monitor, msg)
    assert monitor.inputs['imu'] == 1.
    msg.header.stamp.sec = 2
    msg.header.frame_id = 'livox_frame'
    LocalizationMonitor.gyro(monitor, msg)
    assert monitor.inputs['imu'] == 1.
    msg.header.frame_id = 'base_link'
    msg.angular_velocity.z = float('nan')
    LocalizationMonitor.gyro(monitor, msg)
    assert monitor.inputs['imu'] == 1.


def test_invalid_wheel_speed_and_covariance_do_not_refresh_input():
    monitor = SimpleNamespace(inputs={})
    monitor.input = lambda key, msg: LocalizationMonitor.input(monitor, key, msg)
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp.sec = 1
    msg.twist.covariance[0] = .01
    LocalizationMonitor.wheel(monitor, msg)
    assert monitor.inputs['wheel'] == 1.
    msg.header.stamp.sec = 2
    msg.twist.twist.linear.x = float('inf')
    LocalizationMonitor.wheel(monitor, msg)
    assert monitor.inputs['wheel'] == 1.
    msg.twist.twist.linear.x = 1.
    msg.twist.covariance[0] = -1.
    LocalizationMonitor.wheel(monitor, msg)
    assert monitor.inputs['wheel'] == 1.
