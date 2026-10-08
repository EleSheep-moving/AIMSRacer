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
    front_extent: float | None = None
    rear_extent: float | None = None
    half_width: float | None = None
    geometry_verified: bool = False
    enforce_corridor: bool = True
    cruise_speed: float = .5
    max_speed: float = 1.
    minimum_drive_speed: float = 0.
    steer_limit: float = .4
    steer_rate: float = .5
    steer_acceleration: float = 2.
    accel_limit: float = .5
    brake_limit: float = .5
    jerk_limit: float = 1.
    steering_tau: float = .115
    understeer_coefficient: float = 0.
    lateral_accel_limit: float = 1.
    # Dimensionless objective weights; supplied as solver parameters so tuning
    # these values does not change the compiled symbolic graph.
    contour_weight: float = 1.
    heading_weight: float = 4.
    speed_weight: float = 4000 / 30
    steering_weight: float = 20 / 30
    steering_rate_weight: float = .3
    steering_acceleration_weight: float = .6
    terminal_weight: float = 3.
    solver_max_iterations: int = 35

    def validate(self, require_verified=False, allow_synthetic=True):
        if type(self.geometry_verified) is not bool:
            raise ValueError('geometry_verified must be a boolean')
        if type(self.enforce_corridor) is not bool:
            raise ValueError('enforce_corridor must be a boolean')
        if self.profile not in ('measured', 'synthetic'):
            raise ValueError('profile must be measured or synthetic')
        if self.profile == 'synthetic' and not allow_synthetic:
            raise ValueError('synthetic geometry is permitted only in simulation')
        optional = {'rear_offset', 'half_length', 'front_extent', 'rear_extent', 'half_width'}
        for field in fields(self):
            if field.name in {'geometry_verified', 'enforce_corridor', 'profile'}: continue
            value = getattr(self, field.name)
            if value is None and field.name in optional: continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{field.name} must be finite')
            if field.name in {'rear_offset', 'understeer_coefficient', 'cruise_speed', 'minimum_drive_speed'}:
                if value < 0: raise ValueError(f'{field.name} must be nonnegative')
            elif value <= 0: raise ValueError(f'{field.name} must be positive')
        if not 0 < self.steer_limit < math.pi/2: raise ValueError('steer_limit must be below pi/2')
        if type(self.solver_max_iterations) is not int or self.solver_max_iterations<1:
            raise ValueError('solver_max_iterations must be a positive integer')
        if self.cruise_speed > self.max_speed: raise ValueError('cruise_speed exceeds max_speed')
        if self.minimum_drive_speed > self.cruise_speed:
            raise ValueError('minimum_drive_speed exceeds cruise_speed')
        asymmetric = self.front_extent is not None or self.rear_extent is not None
        if asymmetric and (self.front_extent is None or self.rear_extent is None or
                           self.half_length is not None):
            raise ValueError('use both front/rear extents or a symmetric half_length')
        if self.geometry_verified or require_verified:
            if (not self.geometry_verified or self.rear_offset is None or
                    self.half_width is None or
                    (self.half_length is None and not asymmetric)):
                raise ValueError('verified rear_offset and body geometry required')
        return self

    def longitudinal_offsets(self):
        """Front and rear body edges measured from the rear-axle state origin."""
        self.validate(require_verified=True)
        if self.front_extent is not None:
            return self.front_extent, -self.rear_extent
        return self.rear_offset + self.half_length, self.rear_offset - self.half_length
