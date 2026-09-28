"""Recompute low-speed, speed-mode response diagnostics from a ROS 2 SQLite bag.

Observations are diagnostic fits, not hardware servo-angle or tire-slip data.
"""

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.optimize import minimize, minimize_scalar
from scipy.signal import lfilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    database = sqlite3.connect(f'file:{Path(path).resolve()}?mode=ro', uri=True)
    topics = {name: (ident, get_message(kind)) for ident, name, kind in
              database.execute('SELECT id, name, type FROM topics')}
    required = ('/ackermann_cmd', '/livox/imu', '/rear_axle/wheel_odom',
                '/rear_axle/lio_odom')
    if not set(required).issubset(topics):
        raise ValueError('bag lacks required speed-mode calibration topics')
    arrays = {}
    for name in required:
        topic_id, message_type = topics[name]
        rows = []
        for receipt_ns, payload in database.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (topic_id,)):
            message = deserialize_message(payload, message_type)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if name == '/ackermann_cmd':
                values = (message.drive.jerk, message.drive.speed,
                          message.drive.steering_angle)
            elif name == '/livox/imu':
                values = (message.angular_velocity.z, message.linear_acceleration.x)
            else:
                values = (message.twist.twist.linear.x, message.twist.twist.angular.z)
            rows.append((receipt_ns * 1e-9, stamp, *values))
        arrays[name] = np.asarray(rows)
    return arrays


def response(path):
    data = read_bag(path)
    command = data['/ackermann_cmd']
    imu = data['/livox/imu']
    wheel = data['/rear_axle/wheel_odom']
    lio = data['/rear_axle/lio_odom']
    if not np.all(command[:, 2] == 0):
        raise ValueError('bag mixes speed mode with current/duty mode')
    t0 = command[0, 1]

    steer_at_lio = np.interp(lio[:, 1], command[:, 1], command[:, 4])
    wheel_at_lio = np.interp(lio[:, 1], wheel[:, 1], wheel[:, 2])
    straight = ((wheel_at_lio > .4) & (wheel_at_lio < 1.4) &
                (lio[:, 2] > .4) & (abs(steer_at_lio) < .04) &
                (abs(lio[:, 3]) < .12))
    scale_ratio = lio[straight, 2] / wheel_at_lio[straight]

    # Equivalent turn angle from yaw rate; wheel and LIO are independent speed
    # denominators for a useful sensitivity check, but IMU/LIO are correlated.
    t = np.arange(imu[0, 1], imu[-1, 1], .005)
    steering = np.interp(t, command[:, 1], command[:, 4])
    yaw_rate = np.interp(t, imu[:, 1], imu[:, 2])
    gyro_bias = float(np.median(yaw_rate[t < t[0] + 30]))
    tau_results = {}
    for name, samples in (('wheel', wheel), ('lio', lio)):
        speed = np.interp(t, samples[:, 1], samples[:, 2])
        turn = np.arctan(.36 * (yaw_rate - gyro_bias) / np.maximum(speed, .1))
        valid = ((speed > .55) & (speed < 1.45) & (abs(yaw_rate) < 2.5) &
                 (t > t0 + 32) & (t < t0 + 150))

        def steering_error(tau):
            decay = np.exp(-.005 / tau)
            predicted = lfilter([1 - decay], [1, -decay], steering)
            return float(np.mean((predicted[valid] - turn[valid]) ** 2))

        fit = minimize_scalar(steering_error, bounds=(.02, .25), method='bounded')
        tau_results[name] = dict(samples=int(valid.sum()), tau_s=float(fit.x),
                                 rms_rad=float(np.sqrt(fit.fun)))

    # The speed fit is diagnostic only: MPCC has no longitudinal delay state.
    grid = np.arange(command[0, 1], command[-1, 1], .02)
    speed_command = np.interp(grid, command[:, 1], command[:, 3])
    speed_observed = np.interp(grid, wheel[:, 1], wheel[:, 2])
    active = ((grid >= t0 + 33) & (grid < t0 + 150) &
              (speed_command >= 0) & (speed_observed >= -.03))

    def longitudinal_error(parameters):
        tau, delay, gain = parameters
        decay = np.exp(-.02 / tau)
        delayed = np.interp(grid - delay, grid, speed_command, left=0)
        predicted = lfilter([1 - decay], [1, -decay], gain * delayed)
        return float(np.mean((predicted[active] - speed_observed[active]) ** 2))

    fit_speed = minimize(longitudinal_error, [.15, .04, 1.], method='L-BFGS-B',
                         bounds=[(.02, 1.), (0., .2), (.7, 1.3)])

    # Strictly straight launches, with local acceleration bias/noise removal.
    launch_onsets = []
    acceleration = median_filter(imu[:, 3] * 9.80665, size=5, mode='nearest')
    launches = np.where((command[1:, 3] > .05) & (command[:-1, 3] <= .05))[0] + 1
    for index in launches:
        start = command[index, 1]
        if not t0 + 30 < start < t0 + 150:
            continue
        before = (command[:, 1] >= start - .3) & (command[:, 1] < start)
        around = before | ((command[:, 1] >= start) & (command[:, 1] < start + .4))
        if (before.sum() < 20 or np.max(abs(command[before, 3])) > .05 or
                np.max(abs(command[around, 4])) > .05 or
                abs(np.interp(start, wheel[:, 1], wheel[:, 2])) > .05):
            continue
        previous_imu = (imu[:, 1] >= start - .3) & (imu[:, 1] < start)
        baseline = np.median(acceleration[previous_imu])
        noise = 1.4826 * np.median(abs(acceleration[previous_imu] - baseline))
        threshold = max(.5, 5 * noise)
        future = np.where((imu[:, 1] >= start) & (imu[:, 1] < start + .3))[0]
        above = acceleration[future] - baseline > threshold
        sustained = np.where(above[:-2] & above[1:-1] & above[2:])[0]
        if len(sustained):
            launch_onsets.append((imu[future[sustained[0]], 1] - start) * 1e3)

    lio_age = lio[:, 0] - lio[:, 1]
    command_at_lio = np.interp(lio[:, 0], command[:, 0], command[:, 3])
    moving_age = lio_age[command_at_lio > .2]
    return dict(bag=str(Path(path).resolve()), speed_mode_messages=len(command),
                moving_wheel_median_mps=float(np.median(wheel[wheel[:, 2] > .55, 2])),
                straight_ratio_samples=int(straight.sum()),
                lio_over_wheel_ratio_median=float(np.median(scale_ratio)),
                straight_launch_count=len(launch_onsets),
                imu_onset_ms_median=float(np.median(launch_onsets)),
                imu_onset_ms_range=[float(min(launch_onsets)), float(max(launch_onsets))],
                gyro_bias_radps=gyro_bias, steering_fit=tau_results,
                longitudinal_fit=dict(tau_s=float(fit_speed.x[0]),
                                      delay_s=float(fit_speed.x[1]),
                                      gain=float(fit_speed.x[2]),
                                      rms_mps=float(np.sqrt(fit_speed.fun))),
                moving_lio_age_ms_p95=float(np.percentile(moving_age, 95) * 1e3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('db3')
    args = parser.parse_args()
    print(json.dumps(response(args.db3), indent=2))


if __name__ == '__main__':
    main()
