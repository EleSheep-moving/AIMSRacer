"""Recovered enhanced PP equations; NumPy only, independent of ROS and solvers."""
from dataclasses import dataclass, fields
import math
from pathlib import Path

import numpy as np
import yaml


def wrap_angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class PPConfig:
    wheelbase: float = .36
    steer_limit: float = .45
    rear_offset: float = 0.
    half_length: float | None = None
    front_extent: float = .52
    rear_extent: float = .1
    half_width: float = .16
    max_speed: float = 4.
    geometry_verified: bool = True
    enforce_corridor: bool = False
    lookahead_gain: float = 1.
    min_lookahead: float = .3
    max_lookahead: float = 4.5
    lateral_error_gain: float = 1.
    heading_error_gain: float = .1
    curvature_ff_gain: float = .1
    steering_limit_start_speed: float = 4.
    steering_limit_full_speed: float = 6.
    high_speed_max_steering: float = .18

    def validate(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == 'geometry_verified':
                if value is not True:
                    raise ValueError('verified vehicle geometry required')
            elif field.name == 'enforce_corridor':
                if type(value) is not bool:
                    raise ValueError('boolean enforce_corridor required')
            elif field.name == 'half_length' and value is None:
                continue
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f'finite nonnegative {field.name} required')
        if min(self.wheelbase, self.steer_limit, self.half_width, self.front_extent, self.max_speed, self.min_lookahead) <= 0:
            raise ValueError('positive vehicle and controller limits required')
        if self.steer_limit >= math.pi/2 or self.max_lookahead < self.min_lookahead:
            raise ValueError('invalid steering or lookahead bounds')
        if self.steering_limit_full_speed < self.steering_limit_start_speed:
            raise ValueError('invalid speed steering transition')
        return self

    @classmethod
    def from_files(cls, vehicle_config, pp_config=''):
        vehicle = yaml.safe_load(Path(vehicle_config).read_text())
        if not isinstance(vehicle, dict):
            raise ValueError('vehicle configuration must be a mapping')
        required = {'wheelbase', 'steer_limit', 'rear_offset', 'half_length', 'front_extent', 'rear_extent', 'half_width', 'max_speed', 'geometry_verified'}
        geometry = required | {'enforce_corridor'}
        if required - vehicle.keys():
            raise ValueError(f'incomplete vehicle configuration: {sorted(required - vehicle.keys())}')
        values = {name: vehicle[name] for name in required}
        values['enforce_corridor'] = vehicle.get('enforce_corridor', False)
        if pp_config:
            tuning = yaml.safe_load(Path(pp_config).read_text())
            if not isinstance(tuning, dict):
                raise ValueError('PP configuration must be a mapping')
            names = {f.name for f in fields(cls)} - geometry
            if tuning.keys() - names:
                raise ValueError('PP tuning cannot override vehicle geometry or unknown fields')
            values.update(tuning)
        return cls(**values).validate()


class EnhancedPP:
    def __init__(self, config):
        self.config = config.validate()

    def compute_lookahead(self, velocity):
        if not math.isfinite(velocity):
            raise ValueError('finite velocity required')
        c = self.config
        return float(np.clip(c.lookahead_gain * abs(float(velocity)) + c.min_lookahead, c.min_lookahead, c.max_lookahead))

    def steering_limit_for_speed(self, velocity):
        self.compute_lookahead(velocity)
        c = self.config
        low, high = c.steer_limit, min(c.steer_limit, c.high_speed_max_steering)
        speed = abs(velocity)
        if speed <= c.steering_limit_start_speed:
            return low
        if c.steering_limit_full_speed <= c.steering_limit_start_speed + 1e-6 or speed >= c.steering_limit_full_speed:
            return high
        ratio = (speed-c.steering_limit_start_speed)/max(c.steering_limit_full_speed-c.steering_limit_start_speed, 1e-6)
        return low + ratio*(high-low)

    def compute_steering(self, current_pos, current_yaw, lookahead_point, lookahead_heading, path_curvature, velocity):
        current, target = np.asarray(current_pos, dtype=float), np.asarray(lookahead_point, dtype=float)
        if current.shape != (2,) or target.shape != (2,) or not np.isfinite([*current, *target, current_yaw, lookahead_heading, path_curvature, velocity]).all():
            raise ValueError('finite planar controller inputs required')
        ld = max(self.compute_lookahead(velocity), 1e-3)
        dx, dy = target-current
        y_ld = -math.sin(current_yaw)*dx + math.cos(current_yaw)*dy
        curvature_term = 2.*y_ld/max(ld*ld, 1e-6)
        heading_error = wrap_angle(lookahead_heading-current_yaw)
        c = self.config
        raw = math.atan(c.wheelbase*curvature_term)*c.lateral_error_gain + c.heading_error_gain*heading_error + c.curvature_ff_gain*path_curvature
        limit = self.steering_limit_for_speed(velocity)
        steering = float(np.clip(raw, -limit, limit))
        return steering, dict(lookahead_distance=ld, lateral_offset=float(y_ld), heading_error=heading_error,
                              curvature_term=float(curvature_term), path_curvature=path_curvature, raw_steering=raw,
                              steering_limit=limit, steering_clipped=abs(raw-steering)>1e-6)
