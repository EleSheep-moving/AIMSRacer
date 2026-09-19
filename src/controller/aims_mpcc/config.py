"""Explicit vehicle limits; unknown body geometry prevents controller activation."""
from dataclasses import dataclass, fields
import math

# Small signed EKF velocity noise is permitted near standstill (m/s).
REVERSE_SPEED_TOLERANCE = .05

@dataclass
class VehicleConfig:
    profile: str = 'measured'
    wheelbase: float = .36
    rear_offset: float | None = None
    half_length: float | None = None
    half_width: float | None = None
    geometry_verified: bool = False
    cruise_speed: float = .5
    max_speed: float = 1.
    steer_limit: float = .4
    steer_rate: float = .5
    steer_acceleration: float = 2.
    accel_limit: float = .5
    brake_limit: float = .5
    jerk_limit: float = 1.
    steering_tau: float = .115
    understeer_coefficient: float = 0.
    lateral_accel_limit: float = 1.

    def validate(self, require_verified=False, allow_synthetic=True):
        if type(self.geometry_verified) is not bool:
            raise ValueError('geometry_verified must be a boolean')
        if self.profile not in ('measured', 'synthetic'):
            raise ValueError('profile must be measured or synthetic')
        if self.profile == 'synthetic' and not allow_synthetic:
            raise ValueError('synthetic geometry is permitted only in simulation')
        optional = {'rear_offset', 'half_length', 'half_width'}
        for field in fields(self):
            if field.name in {'geometry_verified', 'profile'}: continue
            value = getattr(self, field.name)
            if value is None and field.name in optional: continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{field.name} must be finite')
            if field.name in {'rear_offset', 'understeer_coefficient', 'cruise_speed'}:
                if value < 0: raise ValueError(f'{field.name} must be nonnegative')
            elif value <= 0: raise ValueError(f'{field.name} must be positive')
        if not 0 < self.steer_limit < math.pi/2: raise ValueError('steer_limit must be below pi/2')
        if self.cruise_speed > self.max_speed: raise ValueError('cruise_speed exceeds max_speed')
        if self.geometry_verified or require_verified:
            if not self.geometry_verified or any(getattr(self,k) is None for k in optional):
                raise ValueError('verified rear_offset and body geometry required')
        return self
