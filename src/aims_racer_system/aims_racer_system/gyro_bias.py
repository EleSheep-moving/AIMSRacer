"""Source-time stationary gyro calibration; no ROS or actuator access."""
from collections import deque
import copy
import math
import numpy as np


class GyroBiasCalibrator:
    def __init__(self, duration=10., min_samples=1000, max_observation_age=.25,
                 max_imu_gap=.05, max_wheel_speed=.02, max_lio_speed=.025,
                 max_position_change=.02, max_orientation_change=.01,
                 max_gyro_norm=.1, max_gyro_std=.005, max_accel_std=.02,
                 accel_norm_min=.75, accel_norm_max=1.25):
        values = locals().copy(); values.pop('self')
        if (type(min_samples) is not int or min_samples < 3 or
                any(not math.isfinite(v) or v <= 0 for v in values.values()) or
                accel_norm_min >= accel_norm_max):
            raise ValueError('Calibration parameters must be finite positive values')
        self.__dict__.update(values)
        self.wheels = deque(maxlen=200)
        self.lios = deque(maxlen=200)
        self.last_stamp = None
        self.ready = False
        self.bias = np.zeros(3)
        self.reason = 'waiting for stationary wheel and LIO observations'
        self.sample_count = 0
        self.gyro_std = np.zeros(3)
        self._clear_window()

    def _clear_window(self):
        self.samples = []
        self.anchor = None
        self.sample_count = 0

    def _reject(self, reason):
        self._clear_window()
        self.reason = reason
        return None

    def add_wheel(self, stamp, speed):
        if not math.isfinite(stamp) or not math.isfinite(speed):
            raise ValueError('Wheel input must be finite')
        if not self.wheels or stamp > self.wheels[-1][0]:
            self.wheels.append((stamp, speed))

    def add_lio(self, stamp, position, quaternion, velocity):
        p, q, v = (np.asarray(a, dtype=float) for a in (position, quaternion, velocity))
        if (not math.isfinite(stamp) or p.shape != (3,) or q.shape != (4,) or
                v.shape != (3,) or not all(np.isfinite(a).all() for a in (p, q, v)) or
                abs(np.linalg.norm(q)-1.) > .01):
            raise ValueError('LIO input must have a finite pose and normalized quaternion')
        if not self.lios or stamp > self.lios[-1][0]:
            self.lios.append((stamp, p, q/np.linalg.norm(q), v))

    def _past(self, records, stamp):
        for record in reversed(records):
            if record[0] <= stamp:
                return record if stamp-record[0] <= self.max_observation_age else None
        return None

    def process(self, stamp, gyro, accel):
        w, a = np.asarray(gyro, dtype=float), np.asarray(accel, dtype=float)
        if (not math.isfinite(stamp) or w.shape != (3,) or a.shape != (3,) or
                not np.isfinite(w).all() or not np.isfinite(a).all()):
            return self._reject('invalid IMU input')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.ready = False
            self.bias = np.zeros(3)
            self.wheels.clear(); self.lios.clear()
            self.last_stamp = stamp
            return self._reject('source clock reset; new calibration required')
        gap = self.last_stamp is not None and stamp-self.last_stamp > self.max_imu_gap
        self.last_stamp = stamp
        if self.ready:
            return w-self.bias
        if gap:
            self._reject('IMU gap; restarting stationary window')
        wheel, lio = self._past(self.wheels, stamp), self._past(self.lios, stamp)
        if wheel is None or lio is None:
            return self._reject('missing or stale source-time wheel/LIO observations')
        if abs(wheel[1]) > self.max_wheel_speed or np.linalg.norm(lio[3]) > self.max_lio_speed:
            return self._reject('wheel/LIO motion; stationary calibration pending')
        if (np.linalg.norm(w) > self.max_gyro_norm or
                not self.accel_norm_min <= np.linalg.norm(a) <= self.accel_norm_max):
            return self._reject('IMU motion or invalid raw acceleration units')
        if self.anchor is None:
            self.anchor = lio
        else:
            angle = 2.*math.acos(float(np.clip(abs(np.dot(self.anchor[2], lio[2])), 0., 1.)))
            if (np.linalg.norm(lio[1]-self.anchor[1]) > self.max_position_change or
                    angle > self.max_orientation_change):
                return self._reject('LIO pose changed during calibration')
        if len(self.samples) >= max(10000, self.min_samples*2):
            return self._reject('calibration sample budget exceeded')
        self.samples.append((stamp, w.copy(), a.copy()))
        self.sample_count = len(self.samples)
        self.reason = 'collecting stationary samples'
        elapsed = stamp-self.samples[0][0]
        if elapsed < self.duration or self.sample_count < self.min_samples:
            return None
        gyros = np.asarray([s[1] for s in self.samples])
        accels = np.asarray([s[2] for s in self.samples])
        self.gyro_std = gyros.std(axis=0, ddof=1)
        if self.gyro_std.max() > self.max_gyro_std or accels.std(axis=0, ddof=1).max() > self.max_accel_std:
            return self._reject('sensor variation too high; restarting stationary window')
        self.bias = gyros.mean(axis=0)
        self.ready = True
        self.reason = 'calibrated; bias frozen until restart or source-clock reset'
        self.samples = []
        self.anchor = None
        return w-self.bias


def correct_imu(msg, bias):
    """Subtract in Livox axes, preserving timing, accel and unknown covariance."""
    bias = np.asarray(bias, dtype=float)
    if bias.shape != (3,) or not np.isfinite(bias).all() or msg.angular_velocity_covariance[0] == -1.:
        raise ValueError('Finite gyro bias and available angular velocity required')
    out = copy.deepcopy(msg)
    w = out.angular_velocity
    w.x, w.y, w.z = (np.asarray([w.x,w.y,w.z])-bias).tolist()
    # All-zero covariance is unknown per sensor_msgs/Imu. Keep downstream floors.
    if any(v != 0. for v in out.angular_velocity_covariance):
        for index in (0, 4, 8):
            out.angular_velocity_covariance[index] += 1e-6
    return out
