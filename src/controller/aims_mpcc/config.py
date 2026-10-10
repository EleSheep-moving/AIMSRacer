"""Explicit vehicle limits; unknown body geometry prevents controller activation."""
from dataclasses import dataclass, fields
import math

# Small signed EKF velocity noise is permitted near standstill (m/s).
REVERSE_SPEED_TOLERANCE = .05

@dataclass
class VehicleConfig:
    profile: str = 'measured'
    command_profile: str = 'legacy_bounded_v1'
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
    # Objective normalization only for rate_bounded_v2, never a hard limit.
    # None inherits the previous normalization to preserve existing costs.
    steering_acceleration_scale: float | None = None
    accel_limit: float = .5
    brake_limit: float = .5
    jerk_limit: float = 1.
    steering_tau: float = .115
    understeer_coefficient: float = 0.
    lateral_accel_limit: float = 1.
    # Explicit v2 experiment: independent actuator bounds and curvature speed
    # planning without a hard longitudinal/lateral acceleration ellipse.
    combined_accel_constraint_enabled: bool = True
    # Operating envelope halfaxes are independent of hard actuator capabilities.
    # None preserves the historical halfaxes for existing vehicle profiles.
    longitudinal_envelope_accel: float | None = None
    longitudinal_envelope_brake: float | None = None
    envelope_soft_enabled: bool = False
    envelope_slack_limit: float = .5  # excess of squared utilization E, not m/s^2
    envelope_slack_weight: float = 10000.
    envelope_recovery_time: float = .6
    # Acados single-RTI nonlinear ellipse reserve in squared utilization E.
    # Physical candidate validation still uses E<=1 and the original soft cap.
    acados_envelope_margin: float = .01
    # Explicit experiment: one or two full RTI passes on an unchanged OCP.
    acados_rti_steps: int = 1
    # Explicit common optimization-only E reserve; physical acceptance stays E<=1.
    optimization_envelope_margin: float = 0.
    recovery_jerk_enabled: bool = False
    recovery_jerk_limit: float = 2.
    contour_scale: float = .05
    lag_scale: float = .20
    heading_scale: float = .05
    speed_scale: float = .60
    steering_scale: float = .314159
    acceleration_scale: float = 2.
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
        if type(self.acados_rti_steps) is not int or self.acados_rti_steps not in (1, 2):
            raise ValueError('acados_rti_steps must be integer one or two')
        if type(self.geometry_verified) is not bool:
            raise ValueError('geometry_verified must be a boolean')
        boolean_fields = {'geometry_verified', 'enforce_corridor', 'envelope_soft_enabled', 'recovery_jerk_enabled',
                          'combined_accel_constraint_enabled'}
        for name in boolean_fields:
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be a boolean')
        if self.command_profile not in ('legacy_bounded_v1', 'rate_bounded_v2'):
            raise ValueError('command_profile must be legacy_bounded_v1 or rate_bounded_v2')
        if not self.combined_accel_constraint_enabled and (self.command_profile!='rate_bounded_v2' or self.envelope_soft_enabled):
            raise ValueError('disabled combined acceleration constraint requires strict rate_bounded_v2')
        if self.profile not in ('measured', 'synthetic'):
            raise ValueError('profile must be measured or synthetic')
        if self.profile == 'synthetic' and not allow_synthetic:
            raise ValueError('synthetic geometry is permitted only in simulation')
        optional = {'rear_offset', 'half_length', 'front_extent', 'rear_extent', 'half_width',
                    'longitudinal_envelope_accel', 'longitudinal_envelope_brake', 'steering_acceleration_scale'}
        for field in fields(self):
            if field.name in boolean_fields | {'profile', 'command_profile'}: continue
            value = getattr(self, field.name)
            if value is None and field.name in optional: continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{field.name} must be finite')
            if field.name in {'rear_offset', 'understeer_coefficient', 'cruise_speed', 'minimum_drive_speed', 'acados_envelope_margin', 'optimization_envelope_margin'}:
                if value < 0: raise ValueError(f'{field.name} must be nonnegative')
            elif value <= 0: raise ValueError(f'{field.name} must be positive')
        for name in ('acados_envelope_margin','optimization_envelope_margin'):
            if getattr(self,name)>=1:raise ValueError(f'{name} must be below one')
        if not 0 < self.steer_limit < math.pi/2: raise ValueError('steer_limit must be below pi/2')
        if type(self.solver_max_iterations) is not int or self.solver_max_iterations<1:
            raise ValueError('solver_max_iterations must be a positive integer')
        if self.cruise_speed > self.max_speed: raise ValueError('cruise_speed exceeds max_speed')
        if self.minimum_drive_speed > self.cruise_speed:
            raise ValueError('minimum_drive_speed exceeds cruise_speed')
        if self.recovery_jerk_enabled and self.recovery_jerk_limit < self.jerk_limit:
            raise ValueError('recovery_jerk_limit must not be below jerk_limit')
        if self.recovery_jerk_enabled and not self.envelope_soft_enabled:
            raise ValueError('recovery jerk requires experimental soft envelope')
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

    def require_legacy_command_profile(self):
        if self.command_profile != 'legacy_bounded_v1':
            raise ValueError('rate_bounded_v2 requires the native acados runtime; legacy runtime is incompatible')

    def steering_rate_change_scale(self):
        if self.command_profile == 'legacy_bounded_v1' or self.steering_acceleration_scale is None:
            return self.steer_acceleration
        return self.steering_acceleration_scale

    def envelope_halfaxes(self):
        return (self.accel_limit if self.longitudinal_envelope_accel is None else self.longitudinal_envelope_accel,
                self.brake_limit if self.longitudinal_envelope_brake is None else self.longitudinal_envelope_brake,
                self.lateral_accel_limit)

    def longitudinal_offsets(self):
        """Front and rear body edges measured from the rear-axle state origin."""
        self.validate(require_verified=True)
        if self.front_extent is not None:
            return self.front_extent, -self.rear_extent
        return self.rear_offset + self.half_length, self.rear_offset - self.half_length
