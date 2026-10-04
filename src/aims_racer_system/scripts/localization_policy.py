"""Measurement freshness policy; bridge publications cannot call observe()."""
from collections import deque
import math
import bisect
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class MapOdomPackets:
    """A discrete correction edge: latest arrival wins at each integer stamp.

    Map corrections are discontinuous. Do not interpolate across them. A source
    scan proves its accepted anchor with an exact packet; diagnostic cloud
    checks may hold the most recent packet whose source stamp is not later.
    """
    def __init__(self, duration_ns=5000000000):
        self.duration_ns = duration_ns
        self.samples = {}
        self.latest = None

    def add(self, stamp_ns, transform):
        self.latest = stamp_ns if self.latest is None else max(self.latest, stamp_ns)
        cutoff = self.latest - self.duration_ns
        if stamp_ns < cutoff:
            return
        self.samples[stamp_ns] = transform.copy()
        for stamp in list(self.samples):
            if stamp < cutoff:
                self.samples.pop(stamp)

    def exact(self, stamp_ns):
        sample = self.samples.get(stamp_ns)
        return None if sample is None else sample.copy()

    def held(self, stamp_ns):
        stamp = self.held_stamp(stamp_ns)
        return None if stamp is None else self.samples[stamp].copy()

    def held_stamp(self, stamp_ns):
        stamps = sorted(self.samples)
        index = bisect.bisect_right(stamps, stamp_ns) - 1
        return None if index < 0 else stamps[index]

    def clear(self):
        self.samples.clear()
        self.latest = None


def trustworthy_alignment(values, anchor_matches):
    try:
        fitness = float(values.get('fitness_score', 'nan'))
        rotation = float(values.get('correction_rotation_deg', 'nan'))
        correction = float(values.get('correction_translation_m', 'nan'))
        yaw = float(values.get('correction_yaw_deg', 'nan'))
        return (anchor_matches and values.get('has_converged') == 'true' and
                all(math.isfinite(value) for value in (fitness, rotation, correction, yaw)) and
                0. <= fitness <= 1.5 and 0. <= rotation <= 10.)
    except (ValueError, TypeError):
        return False


def interpolate_pose(history, stamp):
    """Interpolate translation and rotation at source time, never extrapolate."""
    if not history or stamp < history[0][0] or stamp > history[-1][0]:
        return None
    index = bisect.bisect_left([item[0] for item in history], stamp)
    if history[index][0] == stamp:
        return history[index][1].copy()
    previous, following = history[index - 1], history[index]
    fraction = (stamp - previous[0]) / (following[0] - previous[0])
    pose = np.eye(4)
    pose[:3, 3] = previous[1][:3, 3] + fraction * (following[1][:3, 3] - previous[1][:3, 3])
    rotations = Rotation.from_matrix([previous[1][:3, :3], following[1][:3, :3]])
    pose[:3, :3] = Slerp([previous[0], following[0]], rotations)(stamp).as_matrix()
    return pose


class MeasurementHealth:
    def __init__(self):
        self.accepts = 0
        self.last_scan = None
        self.last_trustworthy = None
        self.reason = 'waiting_for_measurement'
        self.rejected = False
        self.corrections = deque()

    def observe(self, stamp, accepted, reason, translation=0., yaw=0.):
        if not math.isfinite(stamp):
            return
        if self.last_scan is not None and stamp < self.last_scan:
            self.__init__()
        if stamp == self.last_scan:
            return
        self.last_scan = stamp
        self.reason = reason
        self.rejected = not accepted
        if accepted:
            self.accepts += 1
            self.last_trustworthy = stamp
            self.corrections.append((stamp, abs(translation), abs(yaw)))
        else:
            self.accepts = 0

    def state(self, now, inputs_present):
        if not inputs_present or self.last_trustworthy is None:
            return 'lost'
        age = now - self.last_trustworthy
        if age < -.001 or age > 2.:
            return 'lost'
        if self.rejected or age >= .5:
            return 'degraded'
        return 'tracking' if self.accepts >= 3 else 'initializing'

    def correction_totals(self, now, window=10.):
        while self.corrections and now - self.corrections[0][0] > window:
            self.corrections.popleft()
        return tuple(sum(item[index] for item in self.corrections) for index in (1, 2))
