"""Offline wheel-acceleration queries from an evidence-supported current model.

This module intentionally depends only on the standard library. It predicts the
mean wheel-speed derivative represented by an artifact, not body acceleration.
Unsupported regions return an invalid result; they are never extrapolated.
"""
from bisect import bisect_left
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class QueryResult:
    value: Optional[float]
    valid: bool
    reason: str
    segment: str
    domain: Optional[dict] = None

    def to_dict(self):
        """Return an independent JSON-serializable query result."""
        return asdict(self)


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _numbers(values, minimum=1):
    if not isinstance(values, (list, tuple)) or len(values) < minimum:
        raise ValueError('expected a nonempty numeric array')
    if not all(_number(value) for value in values):
        raise ValueError('expected finite numbers, excluding booleans')


def _domain(values):
    _numbers(values, 2)
    if len(values) != 2 or values[0] >= values[1]:
        raise ValueError('invalid domain endpoints')


def _inside(value, domain):
    return domain[0] <= value <= domain[1]


def _curve(xs, ys):
    _numbers(xs, 2)
    _numbers(ys, 2)
    if len(xs) != len(ys):
        raise ValueError('curve arrays have different lengths')
    if any(a >= b for a, b in zip(xs, xs[1:])):
        raise ValueError('curve coordinates must strictly increase')
    if any(a > b for a, b in zip(ys, ys[1:])):
        raise ValueError('curve accelerations must be monotone')


def _supports(intervals, xs):
    if not isinstance(intervals, (list, tuple)):
        raise ValueError('support intervals must be an array')
    for interval in intervals:
        _numbers(interval, 2)
        if (len(interval) != 2 or interval[0] > interval[1]
                or interval[0] < xs[0] or interval[1] > xs[-1]):
            raise ValueError('invalid support interval')


def _supported(value, intervals):
    return any(lo <= value <= hi for lo, hi in intervals)


def _interpolate(value, xs, ys):
    """Evaluate a validated curve within its coordinate range."""
    i = bisect_left(xs, value)
    if i < len(xs) and value == xs[i]:
        return float(ys[i])
    t = (value-xs[i-1])/(xs[i]-xs[i-1])
    # Difference form preserves flat ordinates exactly, including at values
    # where two weighted copies of the same float would round differently.
    return ys[i-1]+t*(ys[i]-ys[i-1])


def _polynomial(coefficients, value):
    result = 0.
    for coefficient in reversed(coefficients):
        result = result*value+coefficient
    return result


def _baseline_extrema(coefficients, domain):
    """Cubic extrema from derivative roots, without a sampling-grid blind spot."""
    # c0 vanishes on differentiation. Including it in the normalization can
    # underflow a meaningful derivative term when coefficient scales differ.
    scale = max(abs(c) for c in coefficients[1:])
    if scale == 0.:
        return list(domain)
    c, b, a = coefficients[1]/scale, 2.*(coefficients[2]/scale), 3.*(coefficients[3]/scale)
    roots = []
    if a == 0.:
        if b != 0.:
            roots.append(-c/b)
    else:
        discriminant = b*b-4.*a*c
        if discriminant >= 0.:
            # Stable quadratic evaluation also retains a very small root.
            q = -.5*(b+math.copysign(math.sqrt(discriminant), b))
            if q == 0.:
                roots.append(-b/(2.*a))
            else:
                roots.extend((q/a, c/q))
    return list(domain)+[root for root in roots if _inside(root, domain)]


def _validate(payload):
    if not isinstance(payload, dict):
        raise ValueError('model must be an object')
    if (type(payload.get('schema_version')) is not int or payload['schema_version'] != 1
            or payload.get('kind') != 'wheel_current_model'
            or payload.get('current_signal') != 'avg_iq'
            or payload.get('outcome') != 'mean_wheel_acceleration'):
        raise ValueError('unsupported wheel-current model schema')
    threshold = payload['threshold_iq_a']
    if not _number(threshold) or threshold != 50.:
        raise ValueError('schema version 1 requires a 50 A threshold')
    baseline = payload['baseline']
    coefficients, domain = baseline['coefficients_ascending'], baseline['speed_domain_mps']
    _numbers(coefficients, 4)
    if len(coefficients) != 4:
        raise ValueError('baseline must be a cubic in ascending coefficient order')
    _domain(domain)
    if domain[0] <= 0.:
        raise ValueError('wheel-speed domain must be positive')
    for speed in _baseline_extrema(coefficients, domain):
        iq = _polynomial(coefficients, speed)
        if not _number(iq) or not 0. < iq < threshold:
            raise ValueError('baseline current must remain strictly between 0 and 50 A')
    high = payload['high']
    _curve(high['iq_a'], high['acceleration_mps2'])
    if high['iq_a'][0] != threshold:
        raise ValueError('high-current curve must start at 50 A')
    _supports(high['support_intervals'], high['iq_a'])
    _domain(high['speed_domain_mps'])
    if not all(_inside(v, domain) for v in high['speed_domain_mps']):
        raise ValueError('high-current speed domain exceeds baseline domain')
    if high['acceleration_mps2'][0] < 0.:
        raise ValueError('50 A acceleration must be nonnegative')
    rows = payload['rows']
    if not isinstance(rows, (list, tuple)) or not rows:
        raise ValueError('at least one low-current speed row is required')
    previous = None
    for row in rows:
        speed, xs, ys = row['speed_mps'], row['u'], row['acceleration_mps2']
        if not _number(speed) or not _inside(speed, domain) or (previous is not None and speed <= previous):
            raise ValueError('speed rows must strictly increase within baseline domain')
        previous = speed
        _curve(xs, ys)
        if 0. not in xs or xs[-1] != 1. or ys[xs.index(0.)] != 0.:
            raise ValueError('low-current row requires exact zero and 50 A anchors')
        if ys[-1] != high['acceleration_mps2'][0]:
            raise ValueError('low/high 50 A acceleration anchors disagree')
        _supports(row['support_intervals'], xs)
    if 'provenance' in payload and not isinstance(payload['provenance'], dict):
        raise ValueError('provenance must be an object')
    if 'validation' in payload and not isinstance(payload['validation'], dict):
        raise ValueError('validation must be an object')
    # Reject non-JSON metadata and nonfinite numbers anywhere in the artifact.
    json.dumps(payload, allow_nan=False)


class WheelCurrentModel:
    """Validated artifact with forward and piecewise-linear inverse queries."""

    def __init__(self, payload):
        try:
            _validate(payload)
        except (KeyError, TypeError, IndexError, AttributeError, OverflowError) as exc:
            raise ValueError('malformed wheel-current model') from exc
        self._payload = deepcopy(payload)
        self._baseline = self._payload['baseline']
        self._rows = self._payload['rows']
        self._speeds = [row['speed_mps'] for row in self._rows]
        self._high = self._payload['high']

    @classmethod
    def from_dict(cls, payload):
        return cls(payload)

    @classmethod
    def load(cls, path):
        """Load UTF-8 JSON and reject malformed model data with ValueError."""
        return cls.from_dict(json.loads(Path(path).read_text(encoding='utf-8')))

    def to_dict(self):
        """Preserve artifact metadata and return a deep copy."""
        return deepcopy(self._payload)

    def _iq0(self, speed):
        return _polynomial(self._baseline['coefficients_ascending'], speed)

    def _invalid(self, reason, segment='', domain=None):
        return QueryResult(None, False, reason, segment, deepcopy(domain))

    def _check(self, value, speed):
        if not _number(value) or not _number(speed):
            return self._invalid('invalid_input')
        domain = self._baseline['speed_domain_mps']
        if not _inside(speed, domain):
            return self._invalid('speed_out_of_domain', domain={'speed_mps': list(domain)})
        return None

    def _low_rows(self, speed):
        index = bisect_left(self._speeds, speed)
        if index < len(self._speeds) and self._speeds[index] == speed:
            return self._rows[index], self._rows[index], 0.
        if index == 0 or index == len(self._speeds):
            return None
        lo, hi = self._rows[index-1], self._rows[index]
        return lo, hi, (speed-lo['speed_mps'])/(hi['speed_mps']-lo['speed_mps'])

    @staticmethod
    def _low_intervals(rows):
        lo, hi, _ = rows
        intervals = []
        for alo, ahi in lo['support_intervals']:
            for blo, bhi in hi['support_intervals']:
                left, right = max(alo, blo), min(ahi, bhi)
                if left <= right:
                    intervals.append((left, right))
        return intervals

    @staticmethod
    def _low_value(u, rows):
        lo, hi, t = rows
        low = _interpolate(u, lo['u'], lo['acceleration_mps2'])
        high = _interpolate(u, hi['u'], hi['acceleration_mps2'])
        return low+t*(high-low)

    def predict_acceleration(self, iq_a, wheel_speed_mps):
        """Predict mean wheel acceleration without clipping or extrapolation."""
        error = self._check(iq_a, wheel_speed_mps)
        if error is not None:
            return error
        iq0 = self._iq0(wheel_speed_mps)
        if iq_a == iq0:
            return QueryResult(0., True, 'ok', 'baseline')
        if iq_a >= 50.:
            domain = {'iq_a': [self._high['iq_a'][0], self._high['iq_a'][-1]],
                      'speed_mps': list(self._high['speed_domain_mps']),
                      'support_intervals': deepcopy(self._high['support_intervals'])}
            if not _inside(wheel_speed_mps, self._high['speed_domain_mps']):
                return self._invalid('speed_out_of_domain', 'high', domain)
            if not _supported(iq_a, self._high['support_intervals']):
                return self._invalid('unsupported_current', 'high', domain)
            return QueryResult(_interpolate(iq_a, self._high['iq_a'], self._high['acceleration_mps2']),
                               True, 'ok', 'high', domain)
        rows = self._low_rows(wheel_speed_mps)
        if rows is None:
            return self._invalid('unsupported_low_speed', 'low', {'speed_mps': [self._speeds[0], self._speeds[-1]]})
        u = (iq_a-iq0)/(50.-iq0)
        supported = False
        for left, right in self._low_intervals(rows):
            iq_left, iq_right = iq0+(50.-iq0)*left, iq0+(50.-iq0)*right
            if iq_left <= iq_a <= iq_right:
                supported = True
                # These are exact coordinate identities, not tolerance-based
                # clipping: the immediately adjacent outside float is invalid.
                if iq_a == iq_left:
                    u = left
                elif iq_a == iq_right:
                    u = right
                break
        if not supported:
            return self._invalid('unsupported_current', 'low')
        # An exactly represented physical knot also keeps its original
        # normalized coordinate. Otherwise an inverse plateau endpoint can
        # round to the next slope when converted back from Iq to u.
        for knot in sorted(set(rows[0]['u']) | set(rows[1]['u'])):
            if iq_a == iq0+(50.-iq0)*knot:
                u = knot
                break
        return QueryResult(self._low_value(u, rows), True, 'ok', 'low')

    @staticmethod
    def _solutions(target, xs, ys, intervals, preferred):
        """Enumerate solutions on each supported closed piece, including flats."""
        candidates = []
        for lo, hi in intervals:
            knots = sorted({lo, hi, *(x for x in xs if lo < x < hi)})
            if len(knots) == 1:
                if _interpolate(lo, xs, ys) == target:
                    candidates.append(lo)
                continue
            for left, right in zip(knots, knots[1:]):
                yl = _interpolate(left, xs, ys)
                yr = _interpolate(right, xs, ys)
                if not yl <= target <= yr:
                    continue
                if yl == yr:
                    candidates.append(min(max(preferred, left), right))
                elif target == yl:
                    candidates.append(left)
                elif target == yr:
                    candidates.append(right)
                else:
                    fraction = (target-yl)/(yr-yl)
                    candidates.append(left+(right-left)*fraction)
        return candidates

    def current_for_acceleration(self, accel_mps2, wheel_speed_mps):
        """Invert supported pieces, preferring the current closest to Iq0(v).

        A candidate must reproduce the target on a forward query within
        ``max(1e-9 m/s², 1e-9 * max(abs(predicted), abs(target)))``. This permits
        floating-point interpolation roundoff, while rejecting a supported
        current whose representable value predicts a different acceleration.
        The tolerance never expands current or speed support.
        """
        error = self._check(accel_mps2, wheel_speed_mps)
        if error is not None:
            return error
        iq0 = self._iq0(wheel_speed_mps)
        if accel_mps2 == 0.:
            return QueryResult(float(iq0), True, 'ok', 'baseline')
        candidates = []
        rows = self._low_rows(wheel_speed_mps)
        if rows is not None:
            lo, hi, _ = rows
            intervals = self._low_intervals(rows)
            # Interpolate each row at the union of knots in normalized current.
            xs = sorted(set(lo['u']) | set(hi['u']) | {u for pair in intervals for u in pair})
            # Outside the common node range no support intersection can exist.
            xs = [u for u in xs if max(lo['u'][0], hi['u'][0]) <= u <= 1.]
            ys = [self._low_value(u, rows) for u in xs]
            for u in self._solutions(accel_mps2, xs, ys, intervals, 0.):
                candidates.append(iq0+(50.-iq0)*u)
        if _inside(wheel_speed_mps, self._high['speed_domain_mps']):
            candidates.extend(self._solutions(accel_mps2, self._high['iq_a'],
                                              self._high['acceleration_mps2'],
                                              self._high['support_intervals'], iq0))
        # A low endpoint at 50 A must also be legal under the high branch.
        # Affine conversion may collapse distinct normalized currents to one
        # float near 50 A; support alone does not prove it solves the inverse.
        matching = []
        for iq in candidates:
            forward = self.predict_acceleration(iq, wheel_speed_mps)
            if forward.valid and math.isclose(forward.value, accel_mps2, rel_tol=1e-9, abs_tol=1e-9):
                matching.append(iq)
        candidates = matching
        if not candidates:
            return self._invalid('unsupported_acceleration')
        iq = min(candidates, key=lambda current: (abs(current-iq0), current))
        result = self.predict_acceleration(iq, wheel_speed_mps)
        return QueryResult(float(iq), True, 'ok', result.segment, result.domain)
