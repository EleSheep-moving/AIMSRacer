"""Source-time steady-speed and q-axis current evidence, without ROS dependencies."""
from collections import deque
from dataclasses import dataclass
import math


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _epsilon(*values):
    """Cover subtraction rounding at Unix-epoch timestamps without a broad epsilon."""
    return 4 * max(math.ulp(float(v)) for v in values)


@dataclass(frozen=True)
class SteadySettings:
    window_seconds: float = 1.
    confirm_seconds: float = 1.
    max_abs_acceleration: float = .1
    max_speed_span: float = .15
    min_samples: int = 40
    source_gap: float = .08
    current_settle_seconds: float = .25
    current_window_seconds: float = .5
    current_min_samples: int = 10
    current_absolute_tolerance: float = 1.
    current_relative_tolerance: float = .1
    current_good_fraction: float = .9

    def __post_init__(self):
        for name in ('window_seconds', 'confirm_seconds', 'source_gap', 'current_window_seconds'):
            value = getattr(self, name)
            if not _finite(value) or value <= 0:
                raise ValueError('invalid steady setting ' + name)
        for name in ('max_abs_acceleration', 'max_speed_span', 'current_settle_seconds',
                     'current_absolute_tolerance', 'current_relative_tolerance'):
            value = getattr(self, name)
            if not _finite(value) or value < 0:
                raise ValueError('invalid steady setting ' + name)
        for name in ('min_samples', 'current_min_samples'):
            value = getattr(self, name)
            if type(value) is not int or value < 2:
                raise ValueError('invalid steady sample count')
        if not _finite(self.current_good_fraction) or not 0 < self.current_good_fraction <= 1:
            raise ValueError('invalid current good fraction')


class SteadyDetector:
    """Observe native samples; repeated values with identical stamps add no evidence.

    The caller passes latest wheel and IQ samples on either callback. Both source
    clocks must be comparable to the command acknowledgement stamp. New commands,
    invalid samples, source discontinuities and mode changes discard eligibility.
    """
    def __init__(self, settings=None):
        self.settings = settings if settings is not None else SteadySettings()
        if not isinstance(self.settings, SteadySettings):
            raise ValueError('SteadySettings required')
        self.reset()

    def reset(self):
        self._wheel = deque()
        self._current = deque()
        self._last_wheel = None
        self._last_current = None
        self._context = None
        self._confirmation = None

    def _empty(self, reason, wheel_stamp=None, telemetry_stamp=None):
        return dict(steady=False, reason=reason, slope=None, speed_span=None,
                    sample_count=len(self._wheel), current_sample_count=len(self._current),
                    current_good_fraction=None, wheel_stamp=wheel_stamp,
                    telemetry_stamp=telemetry_stamp, window_start_stamp=None,
                    current_window_start_stamp=None, confirmation_start_stamp=None,
                    confirmed_seconds=0.)

    @staticmethod
    def _trim(ring, duration):
        cutoff = ring[-1][0] - duration
        # Retain one predecessor when the exact boundary is not sampled. Thus
        # the duration is genuinely covered rather than inferred from count.
        while len(ring) > 1 and ring[1][0] <= cutoff + _epsilon(cutoff, ring[1][0]):
            ring.popleft()

    def observe(self, wheel_stamp, velocity, telemetry_stamp, avg_iq,
                target_current, ack_stamp, current_mode=True):
        values = (wheel_stamp, velocity, telemetry_stamp, avg_iq, target_current, ack_stamp)
        if not all(_finite(v) for v in values) or current_mode is not True:
            self.reset()
            return self._empty('invalid_sample_or_mode')
        context = (ack_stamp, target_current)
        if context != self._context:
            self.reset()
            self._context = context
        s = self.settings
        if ((self._last_wheel is not None and wheel_stamp < self._last_wheel) or
                (self._last_current is not None and telemetry_stamp < self._last_current)):
            self.reset()
            return self._empty('backwards_source', wheel_stamp, telemetry_stamp)
        gap = ((self._last_wheel is not None and
                wheel_stamp-self._last_wheel > s.source_gap + _epsilon(wheel_stamp, self._last_wheel)) or
               (self._last_current is not None and
                telemetry_stamp-self._last_current > s.source_gap + _epsilon(telemetry_stamp, self._last_current)))
        stale = abs(wheel_stamp-telemetry_stamp) > s.source_gap + _epsilon(wheel_stamp, telemetry_stamp)
        if gap or stale:
            self._wheel.clear()
            self._current.clear()
            self._confirmation = None
        new_wheel = wheel_stamp != self._last_wheel
        new_current = telemetry_stamp != self._last_current
        self._last_wheel, self._last_current = wheel_stamp, telemetry_stamp
        settled = ack_stamp + s.current_settle_seconds
        if (stale or min(wheel_stamp, telemetry_stamp) <= ack_stamp or
                min(wheel_stamp, telemetry_stamp) < settled - _epsilon(settled, wheel_stamp, telemetry_stamp)):
            self._confirmation = None
            return self._empty('stale_source' if stale else 'current_settling', wheel_stamp, telemetry_stamp)
        if new_wheel:
            self._wheel.append((wheel_stamp, velocity))
            self._trim(self._wheel, s.window_seconds)
        if new_current:
            tolerance = max(s.current_absolute_tolerance, s.current_relative_tolerance * abs(target_current))
            self._current.append((telemetry_stamp, abs(avg_iq-target_current) <= tolerance))
            self._trim(self._current, s.current_window_seconds)
        evidence = self._empty('insufficient_window', wheel_stamp, telemetry_stamp)
        if not self._wheel or not self._current:
            self._confirmation = None
            return evidence
        first = self._wheel[0][0]
        current_first = self._current[0][0]
        fraction = sum(good for _, good in self._current) / len(self._current)
        speeds = [v for _, v in self._wheel]
        times = [stamp-first for stamp, _ in self._wheel]
        mean_t = sum(times)/len(times)
        mean_v = sum(speeds)/len(speeds)
        denominator = sum((t-mean_t)**2 for t in times)
        slope = (sum((t-mean_t)*(v-mean_v) for t,v in zip(times,speeds))/denominator
                 if denominator else None)
        span = max(speeds)-min(speeds)
        evidence.update(slope=slope, speed_span=span, current_good_fraction=fraction,
                        window_start_stamp=first, current_window_start_stamp=current_first)
        covered = (len(self._wheel) >= s.min_samples and
                   wheel_stamp-first + _epsilon(wheel_stamp, first) >= s.window_seconds and
                   len(self._current) >= s.current_min_samples and
                   telemetry_stamp-current_first + _epsilon(telemetry_stamp, current_first) >= s.current_window_seconds)
        eligible = (covered and slope is not None and abs(slope) <= s.max_abs_acceleration and
                    span <= s.max_speed_span and fraction >= s.current_good_fraction)
        if not eligible:
            self._confirmation = None
            evidence['reason'] = 'outside_steady_limits' if covered else 'insufficient_window'
            return evidence
        if self._confirmation is None:
            self._confirmation = wheel_stamp
        confirmed = wheel_stamp-self._confirmation
        evidence.update(confirmation_start_stamp=self._confirmation,
                        confirmed_seconds=confirmed,
                        steady=confirmed + _epsilon(wheel_stamp,self._confirmation) >= s.confirm_seconds,
                        reason='confirmed' if confirmed + _epsilon(wheel_stamp,self._confirmation) >= s.confirm_seconds else 'confirming')
        return evidence
