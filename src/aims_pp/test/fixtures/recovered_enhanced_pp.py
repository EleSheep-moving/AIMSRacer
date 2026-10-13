"""Verbatim recovered numeric helper and controller; provenance.json records origin."""
import math
import numpy as np


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuitController:
    def __init__(
        self,
        wheelbase: float = 0.36,
        lookahead_gain: float = 1.0,
        min_lookahead: float = 0.3,
        max_lookahead: float = 4.5,
        lateral_error_gain: float = 1.0,
        heading_error_gain: float = 0.1,
        curvature_ff_gain: float = 0.1,
        max_steering: float = 0.4751,
        steering_limit_start_speed: float = 4.0,
        steering_limit_full_speed: float = 6.0,
        high_speed_max_steering: float = 0.18,
    ) -> None:
        self.wheelbase = float(wheelbase)
        self.lookahead_gain = float(lookahead_gain)
        self.min_lookahead = float(min_lookahead)
        self.max_lookahead = float(max_lookahead)
        self.lateral_error_gain = float(lateral_error_gain)
        self.heading_error_gain = float(heading_error_gain)
        self.curvature_ff_gain = float(curvature_ff_gain)
        self.max_steering_angle = float(max_steering)
        self.steering_limit_start_speed = float(steering_limit_start_speed)
        self.steering_limit_full_speed = float(steering_limit_full_speed)
        self.high_speed_max_steering_angle = float(high_speed_max_steering)

    def compute_lookahead(self, velocity: float) -> float:
        ld = self.lookahead_gain * abs(float(velocity)) + self.min_lookahead
        return float(np.clip(ld, self.min_lookahead, self.max_lookahead))

    def steering_limit_for_speed(self, velocity: float) -> float:
        low_limit = abs(float(self.max_steering_angle))
        high_limit = min(low_limit, abs(float(self.high_speed_max_steering_angle)))
        start_speed = max(0.0, float(self.steering_limit_start_speed))
        full_speed = max(start_speed, float(self.steering_limit_full_speed))
        speed = abs(float(velocity))
        if speed <= start_speed:
            return low_limit
        if full_speed <= start_speed + 1e-6 or speed >= full_speed:
            return high_limit
        ratio = (speed - start_speed) / max(full_speed - start_speed, 1e-6)
        return float(low_limit + ratio * (high_limit - low_limit))

    def compute_steering(
        self,
        current_pos: np.ndarray,
        current_yaw: float,
        lookahead_point: np.ndarray,
        lookahead_heading: float,
        path_curvature: float,
        velocity: float,
    ) -> tuple:
        ld = max(self.compute_lookahead(velocity), 1e-3)
        dx = lookahead_point[0] - current_pos[0]
        dy = lookahead_point[1] - current_pos[1]
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        y_ld = -sin_yaw * dx + cos_yaw * dy

        curvature_term = 2.0 * y_ld / max(ld * ld, 1e-6)
        steering = math.atan(self.wheelbase * curvature_term) * self.lateral_error_gain
        heading_error = _angle_wrap(lookahead_heading - current_yaw)
        steering += self.heading_error_gain * heading_error
        steering += self.curvature_ff_gain * path_curvature
        raw_steering = float(steering)
        effective_limit = self.steering_limit_for_speed(velocity)
        steering = float(np.clip(raw_steering, -effective_limit, effective_limit))
        clipped = abs(raw_steering - steering) > 1e-6

        debug_info = {
            "lookahead_distance": ld,
            "lateral_offset": y_ld,
            "heading_error": heading_error,
            "curvature_term": curvature_term,
            "path_curvature": path_curvature,
            "raw_steering": raw_steering,
            "steering_limit": effective_limit,
            "steering_clipped": clipped,
        }
        return steering, debug_info
