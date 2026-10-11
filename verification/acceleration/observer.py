"""Causal scalar longitudinal Kalman observer with bounded source-time replay.

The optional disturbance is the effective offset in ``force = a + d``. Gravity
observations constrain this scalar at their historical source time; they do not
propagate orientation or LIO pose. This is an offline estimation candidate.
"""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from math import isfinite
from numbers import Integral, Real
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ObserverConfig:
    use_imu: bool = False
    use_gravity: bool = False
    jerk_psd: float = 1.0
    disturbance_psd: float = 0.01
    speed_variance: float = 0.0004
    force_variance: float = 0.49
    gravity_variance: float = 0.0225
    history_seconds: float = 2.0
    max_history_measurements: int = 4096

    def __post_init__(self):
        if not isinstance(self.use_imu, bool) or not isinstance(self.use_gravity, bool):
            raise ValueError("measurement switches must be bool")
        if self.use_gravity and not self.use_imu:
            raise ValueError("gravity constraints require the IMU disturbance state")
        if (not isinstance(self.max_history_measurements, Integral)
                or isinstance(self.max_history_measurements, (bool, np.bool_))
                or self.max_history_measurements < 2):
            raise ValueError("max_history_measurements must be an integer at least two")
        for name in ("jerk_psd", "disturbance_psd", "speed_variance", "force_variance",
                     "gravity_variance", "history_seconds"):
            value = getattr(self, name)
            if not _finite_scalar(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
            if name not in ("jerk_psd", "disturbance_psd") and value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.history_seconds * 1e9 < 1 or not np.isfinite(self.history_seconds * 1e9):
            raise ValueError("history_seconds must represent at least one finite nanosecond")


@dataclass(frozen=True)
class Estimate:
    stamp_ns: int
    velocity: float
    acceleration: float
    disturbance: Optional[float]
    acceleration_variance: float


@dataclass(frozen=True)
class _Measurement:
    stamp_ns: int
    kind: str
    value: float
    variance: float


@dataclass(frozen=True)
class _Checkpoint:
    stamp_ns: int
    state: np.ndarray
    covariance: np.ndarray


def _finite_scalar(value):
    if not isinstance(value, Real) or isinstance(value, (bool, np.bool_)):
        return False
    try:
        return isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _valid_stamp(stamp_ns):
    return (isinstance(stamp_ns, Integral)
            and not isinstance(stamp_ns, (bool, np.bool_))
            and 0 <= int(stamp_ns) <= 2 ** 63 - 1)


class LongitudinalObserver:
    """Admit observations in receipt order, update in integer source-time order.

    The initial prior is zero mean with variances [1, 4] (and 4 for d).
    It is placed at the earliest retained observation until a prefix has been
    trimmed. Equal-source-time measurements retain receipt order. Snapshots are
    frozen predictions, never stored as measurement checkpoints. Live snapshots
    remain available after long gaps; time and observation-count limits bound
    history, plus one prefix anchor. Inputs older than that anchor cannot replay.
    """

    def __init__(self, config: ObserverConfig):
        self.config = config
        self._dimension = 3 if config.use_imu else 2
        self._history_ns = int(round(config.history_seconds * 1e9))
        self._measurements = []
        self._stamps = []
        self._checkpoints = []
        self._anchor = None
        self._identity = np.eye(self._dimension)
        self._models = {"speed": np.array([1.0, 0.0] + ([0.0] if config.use_imu else []))}
        if config.use_imu:
            self._models["force"] = np.array([0.0, 1.0, 1.0])
        if config.use_gravity:
            self._models["gravity"] = np.array([0.0, 0.0, 1.0])
        self.counters = dict.fromkeys(("received", "admitted", "delayed", "too_old",
                                      "invalid", "disabled", "numerical_failure",
                                      "replayed", "trimmed", "count_trimmed", "max_history"), 0)

    @property
    def history_size(self):
        """Number of retained observations, excluding the single prefix anchor."""
        return len(self._measurements)

    def _prior(self, stamp_ns):
        return _Checkpoint(stamp_ns, np.zeros(self._dimension),
                           np.diag([1.0, 4.0] + ([4.0] if self.config.use_imu else [])))

    def _checked(self, state, covariance):
        covariance = (covariance + covariance.T) * 0.5
        if not np.isfinite(state).all() or not np.isfinite(covariance).all():
            raise FloatingPointError("nonfinite state or covariance")
        minimum = float(np.linalg.eigvalsh(covariance)[0])
        tolerance = 1e-12 * max(1.0, float(np.max(np.abs(covariance))))
        if minimum < -tolerance:
            raise FloatingPointError("covariance is not positive semidefinite")
        if minimum < 0:
            covariance = covariance + self._identity * (-minimum)
        return state, covariance

    def _predict(self, checkpoint, stamp_ns):
        # Subtract integer nanoseconds before conversion, preserving small dt at
        # UNIX-sized epochs. Snapshots may span gaps longer than history_seconds.
        dt = (stamp_ns - checkpoint.stamp_ns) * 1e-9
        if dt < 0:
            raise ValueError("cannot predict backwards")
        transition = self._identity.copy()
        transition[0, 1] = dt
        process = np.zeros((self._dimension, self._dimension))
        process[:2, :2] = self.config.jerk_psd * np.array(
            [[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        if self.config.use_imu:
            process[2, 2] = self.config.disturbance_psd * dt
        state, covariance = self._checked(transition @ checkpoint.state,
                                          transition @ checkpoint.covariance @ transition.T + process)
        return _Checkpoint(stamp_ns, state, covariance)

    def _update(self, checkpoint, measurement):
        prior = self._predict(checkpoint, measurement.stamp_ns)
        h = self._models[measurement.kind]
        projected = prior.covariance @ h
        innovation_variance = float(h @ projected + measurement.variance)
        if not np.isfinite(innovation_variance) or innovation_variance <= 0:
            raise FloatingPointError("invalid innovation variance")
        gain = projected / innovation_variance
        state = prior.state + gain * (measurement.value - h @ prior.state)
        residual = self._identity - np.outer(gain, h)
        # Joseph form keeps covariance symmetric and PSD under repeated updates.
        covariance = (residual @ prior.covariance @ residual.T
                      + measurement.variance * np.outer(gain, gain))
        state, covariance = self._checked(state, covariance)
        return _Checkpoint(measurement.stamp_ns, state, covariance)

    def receive(self, kind: str, stamp_ns: int, value: float,
                variance: Optional[float] = None) -> bool:
        """Return whether this scalar was admitted; rejection leaves state intact.

        No innovation gate is applied. ``received`` includes every attempt;
        ``delayed`` counts admitted observations older than newest source time,
        and ``replayed`` counts suffix updates caused by those late arrivals.
        ``trimmed`` counts all removed history entries; ``count_trimmed`` counts
        entries removed by the hard cap beyond those already outside the time
        window. Removal preserves their posterior in one checkpoint anchor.
        """
        self.counters["received"] += 1
        if kind not in ("speed", "force", "gravity") or not _valid_stamp(stamp_ns):
            self.counters["invalid"] += 1
            return False
        if kind not in self._models:
            self.counters["disabled"] += 1
            return False
        if variance is None:
            variance = getattr(self.config, f"{kind}_variance")
        if not _finite_scalar(value) or not _finite_scalar(variance) or variance <= 0:
            self.counters["invalid"] += 1
            return False
        stamp_ns = int(stamp_ns)
        newest = self._stamps[-1] if self._stamps else stamp_ns
        if (stamp_ns < newest - self._history_ns
                or (self._anchor is not None and stamp_ns < self._anchor.stamp_ns)):
            self.counters["too_old"] += 1
            return False
        measurement = _Measurement(stamp_ns, kind, float(value), float(variance))
        index = bisect_right(self._stamps, stamp_ns)
        delayed = index < len(self._measurements)
        checkpoint = (self._checkpoints[index - 1] if index else self._anchor)
        if checkpoint is None:
            checkpoint = self._prior(stamp_ns)
        suffix = []
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                checkpoint = self._update(checkpoint, measurement)
                suffix.append(checkpoint)
                if delayed:
                    for existing in self._measurements[index:]:
                        checkpoint = self._update(checkpoint, existing)
                        suffix.append(checkpoint)
        except (FloatingPointError, OverflowError, np.linalg.LinAlgError):
            self.counters["numerical_failure"] += 1
            return False
        # Commit only after the whole late suffix has been checked successfully.
        self._measurements.insert(index, measurement)
        self._stamps.insert(index, stamp_ns)
        self._checkpoints[index:] = suffix
        self.counters["admitted"] += 1
        if delayed:
            self.counters["delayed"] += 1
            self.counters["replayed"] += len(suffix)
        cutoff = self._stamps[-1] - self._history_ns
        time_trim = bisect_left(self._stamps, cutoff)
        count_trim = max(0, self.history_size - self.config.max_history_measurements)
        trim = max(time_trim, count_trim)
        if trim:
            self._anchor = self._checkpoints[trim - 1]
            del self._measurements[:trim]
            del self._stamps[:trim]
            del self._checkpoints[:trim]
            self.counters["trimmed"] += trim
            self.counters["count_trimmed"] += max(0, count_trim - time_trim)
        self.counters["max_history"] = max(self.counters["max_history"], self.history_size)
        return True

    def snapshot(self, stamp_ns: int) -> Estimate:
        """Predict at receipt time >= newest admitted source without mutating history."""
        if not _valid_stamp(stamp_ns):
            raise ValueError("snapshot requires a nonnegative signed-int64 nanosecond stamp")
        if not self._checkpoints:
            raise RuntimeError("no observation has been admitted")
        if stamp_ns < self._stamps[-1]:
            raise ValueError("snapshot precedes newest admitted source time")
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            predicted = self._predict(self._checkpoints[-1], int(stamp_ns))
        return Estimate(int(stamp_ns), float(predicted.state[0]), float(predicted.state[1]),
                        float(predicted.state[2]) if self.config.use_imu else None,
                        float(predicted.covariance[1, 1]))
