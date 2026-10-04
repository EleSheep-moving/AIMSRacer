"""Health derived solely from explicit NDT commits, never TF timer publication."""
import math
import re


def unsigned(value):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9]+', value):
        raise ValueError('expected unsigned decimal integer')
    result = int(value)
    if result > (1 << 64) - 1:
        raise ValueError('unsigned protocol integer overflow')
    return result


def boolean(value):
    if value not in ('true', 'false'):
        raise ValueError('expected protocol boolean')
    return value == 'true'


class AnchorHealth:
    def __init__(self, max_age=.5, recovery_commits=3):
        self.max_age = max_age
        self.recovery_commits = recovery_commits
        self.epoch = ''
        self.retired_epochs = set()
        self.event_sequence = -1
        self.anchor_sequence = 0
        self.last_anchor_stamp_ns = 0
        self.last_commit_received = None
        self.last_clock = None
        self.ready = False
        self.recovering = False
        self.accepts = 0
        self.rejected = False
        self.clock_invalid = False
        self.reason = 'waiting_for_anchor'

    def invalidate(self, reason):
        self.ready = False
        self.recovering = bool(self.last_anchor_stamp_ns)
        self.accepts = 0
        self.reason = reason

    def observe(self, values, attempt_stamp_ns, now_ns, received):
        try:
            if values['protocol_version'] != '1' or not values['epoch']:
                raise ValueError('invalid anchor protocol')
            epoch = values['epoch']
            seq = unsigned(values['event_sequence'])
            anchor = unsigned(values['anchor_sequence'])
            stamp = unsigned(values['last_anchor_stamp_ns'])
            committed = boolean(values['anchor_committed'])
            ready = boolean(values['ready'])
        except (ValueError, KeyError, TypeError):
            self.invalidate('invalid_anchor_protocol')
            return False
        if epoch in self.retired_epochs:
            return False
        if epoch != self.epoch:
            if self.epoch:
                self.retired_epochs.add(self.epoch)
            self.epoch = epoch
            self.event_sequence = -1
            self.anchor_sequence = 0
            self.last_anchor_stamp_ns = 0
            self.last_commit_received = None
            self.ready = False
            self.accepts = 0
            self.recovering = False
            self.rejected = False
            self.clock_invalid = False
            self.last_clock = None
            self.reason = 'new_epoch'
        if seq <= self.event_sequence:
            return False
        self.event_sequence = seq
        if now_ns < 0 or attempt_stamp_ns > now_ns or attempt_stamp_ns < 0:
            self.invalidate('future_or_invalid_attempt_stamp')
            return False
        if self.clock_invalid:
            return False
        if self.last_commit_received is not None and (received - self.last_commit_received > self.max_age or
                now_ns - self.last_anchor_stamp_ns > int(self.max_age * 1e9)):
            self.invalidate('anchor_timeout')
        if not committed:
            self.rejected = True
            self.accepts = 0
            self.reason = values.get('reason', 'anchor_rejected')
            if not ready:
                self.invalidate(self.reason)
            return True
        if (not ready or anchor <= self.anchor_sequence or stamp != attempt_stamp_ns or
                stamp <= self.last_anchor_stamp_ns or now_ns - stamp > int(self.max_age * 1e9)):
            self.invalidate('invalid_commit_provenance')
            return False
        self.anchor_sequence = anchor
        self.last_anchor_stamp_ns = stamp
        self.last_commit_received = received
        self.rejected = False
        self.reason = values.get('reason', 'ok')
        self.accepts += 1
        if not self.recovering or self.accepts >= self.recovery_commits:
            self.ready = True
            self.recovering = False
        return True

    def evaluate(self, now, monotonic_now, inputs_present, source_now_ns=None):
        now_ns = source_now_ns if source_now_ns is not None else (round(now * 1e9) if math.isfinite(now) else 0)
        if self.last_clock is not None and now_ns < self.last_clock:
            self.clock_invalid = True
            self.invalidate('clock_rewind')
        self.last_clock = now_ns
        age = (now_ns - self.last_anchor_stamp_ns) * 1e-9 if self.last_anchor_stamp_ns and math.isfinite(now) else float('inf')
        receive_age = monotonic_now - self.last_commit_received if self.last_commit_received is not None else float('inf')
        fresh = (math.isfinite(now) and 0. <= age <= self.max_age and
                 0. <= receive_age <= self.max_age and inputs_present and not self.clock_invalid)
        if not fresh:
            self.invalidate('inputs_stale' if not inputs_present else 'anchor_timeout')
        state = ('hold' if self.rejected else 'tracking') if self.ready and fresh else (
            'recovering' if fresh and self.accepts else 'lost')
        return dict(protocol_version='1', epoch=self.epoch, event_sequence=self.event_sequence,
                    anchor_sequence=self.anchor_sequence, state=state, ready=self.ready and fresh,
                    source_age=age, receive_age=receive_age, last_anchor_stamp_ns=self.last_anchor_stamp_ns,
                    accepted_streak=self.accepts, reason=self.reason)


def initialization_complete(values, previous_epoch, now, mono_now, status_received, source_now_ns=None):
    try:
        now_ns = source_now_ns if source_now_ns is not None else round(now * 1e9)
        return (values['protocol_version'] == '1' and values['epoch'] != previous_epoch and
                bool(values['epoch']) and unsigned(values['health_sequence']) > 0 and values['ready'] == 'true' and values['state'] == 'tracking' and
                0. <= float(values['source_age']) <= .5 and 0. <= float(values['receive_age']) <= .5 and
                0. <= (now_ns - unsigned(values['last_anchor_stamp_ns'])) * 1e-9 <= .5 and
                0. <= mono_now - status_received <= .5)
    except (KeyError, ValueError, TypeError):
        return False


def interpolate_pose(history, stamp):
    """Translation and SLERP at source time; extrapolation is forbidden."""
    import bisect
    import numpy as np
    from scipy.spatial.transform import Rotation, Slerp
    if not history or stamp < history[0][0] or stamp > history[-1][0]:
        return None
    index = bisect.bisect_left([item[0] for item in history], stamp)
    if history[index][0] == stamp:
        return history[index][1].copy()
    before, after = history[index - 1], history[index]
    fraction = (stamp - before[0]) / (after[0] - before[0])
    pose = np.eye(4)
    pose[:3, 3] = before[1][:3, 3] + fraction * (after[1][:3, 3] - before[1][:3, 3])
    rotations = Rotation.from_matrix([before[1][:3, :3], after[1][:3, :3]])
    pose[:3, :3] = Slerp([before[0], after[0]], rotations)(stamp).as_matrix()
    return pose


class MapOdomPackets:
    """Discrete committed correction snapshots; never interpolate corrections."""
    def __init__(self, duration_ns=5_000_000_000):
        self.duration_ns = duration_ns
        self.samples = {}

    def add(self, stamp_ns, transform):
        self.samples[stamp_ns] = transform.copy()
        cutoff = max(self.samples) - self.duration_ns
        self.samples = {key: value for key, value in self.samples.items() if key >= cutoff}

    def held_stamp(self, stamp_ns):
        prior = [key for key in self.samples if key <= stamp_ns]
        return max(prior) if prior else None

    def held(self, stamp_ns):
        stamp = self.held_stamp(stamp_ns)
        return self.samples[stamp].copy() if stamp is not None else None

    def clear(self):
        self.samples.clear()


def initial_pose_quaternion(frame, xyz, rpy):
    """An initializer describes T_map_base_link; rpy is in radians."""
    if frame != 'base_link' or len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
        raise ValueError('finite map/base_link xyz and roll/pitch/yaw required')
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
