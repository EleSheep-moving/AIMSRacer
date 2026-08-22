#!/usr/bin/env python3
"""Dedicated Pure Pursuit parameter tuning node.

Goal: adjust PP parameters so the vehicle hugs the trajectory at any speed without
oscillation or over-shooting corners. The node isolates PP logic from current
calibration and adds tooling for live tuning.

Features
========
- Internal figure-8 generator or optional external ``nav_msgs/Path`` input
- Adaptive lookahead with on-the-fly parameter updates via ``ros2 param set``
- Steering damping (rate limit + exponential smoothing) to avoid "扭来扭去"
- Manual speed control for incremental testing
- Rolling tracking metrics (cross-track RMS, heading RMS, mean steering)

Quick start::

    ros2 run aims_racer_system pp_param_tuner --ros-args \
        -p lookahead_gain:=1.8 -p steering_rate_limit:=3.0 \
        -p target_speed:=2.0

Adjust parameters live::

    ros2 param set /pp_param_tuner lookahead_gain 1.6
    ros2 param set /pp_param_tuner lateral_error_gain 1.2
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from visualization_msgs.msg import Marker


def _angle_wrap(angle: float) -> float:
    """Wrap any angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryHelper:
    """Utility for sampling closest/heading/curvature along a closed path."""

    def __init__(self, points: np.ndarray):
        self.update_points(points)

    def update_points(self, points: np.ndarray) -> None:
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Trajectory points must be of shape (N, 2)")
        if len(points) < 2:
            raise ValueError("Trajectory requires at least two points")
        self.points = points
        self.length = len(points)

    def closest_point(
        self, current_pos: np.ndarray, start_idx: int, window: int = 80
    ) -> Tuple[np.ndarray, int, float]:
        indices = (np.arange(start_idx, start_idx + window) % self.length).astype(int)
        segment = self.points[indices]
        distances = np.linalg.norm(segment - current_pos, axis=1)
        local = int(np.argmin(distances))
        global_idx = int(indices[local])
        return self.points[global_idx], global_idx, float(distances[local])

    def lookahead_point(
        self, current_idx: int, lookahead_distance: float
    ) -> Tuple[np.ndarray, int]:
        idx = current_idx % self.length
        distance = 0.0
        while distance < lookahead_distance:
            next_idx = (idx + 1) % self.length
            segment = np.linalg.norm(self.points[next_idx] - self.points[idx])
            if distance + segment >= lookahead_distance:
                ratio = (lookahead_distance - distance) / max(segment, 1e-6)
                point = self.points[idx] + ratio * (self.points[next_idx] - self.points[idx])
                return point, next_idx
            distance += segment
            idx = next_idx
        return self.points[idx], idx

    def heading(self, idx: int) -> float:
        prev_pt = self.points[(idx - 1) % self.length]
        next_pt = self.points[(idx + 1) % self.length]
        return math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])

    def curvature(self, idx: int) -> float:
        p_prev = self.points[(idx - 1) % self.length]
        p_curr = self.points[idx % self.length]
        p_next = self.points[(idx + 1) % self.length]
        a = np.linalg.norm(p_curr - p_prev)
        b = np.linalg.norm(p_next - p_curr)
        c = np.linalg.norm(p_next - p_prev)
        denom = max(a * b * c, 1e-6)
        if denom < 1e-6:
            return 0.0
        area = abs(
            p_prev[0] * (p_curr[1] - p_next[1])
            + p_curr[0] * (p_next[1] - p_prev[1])
            + p_next[0] * (p_prev[1] - p_curr[1])
        ) / 2.0
        return 4.0 * area / denom


class Figure8Trajectory(TrajectoryHelper):
    """Racetrack (stadium) path: straight lines for acceleration + semicircular turns."""

    def __init__(self, radius: float = 2.0, straight_length: float = 6.0, 
                 points_per_straight: int = 150, points_per_semicircle: int = 100):
        self.radius = radius
        self.straight_length = straight_length
        self.points_per_straight = points_per_straight
        self.points_per_semicircle = points_per_semicircle
        super().__init__(self._generate())

    def _generate(self) -> np.ndarray:
        # Bottom straight: left to right
        bottom_x = np.linspace(-self.straight_length/2, self.straight_length/2, 
                              self.points_per_straight, endpoint=False)
        bottom_y = np.zeros(self.points_per_straight)
        
        # Right semicircle: bottom to top, center at (straight_length/2, radius)
        theta_right = np.linspace(-np.pi/2, np.pi/2, self.points_per_semicircle, endpoint=False)
        right_x = self.straight_length/2 + self.radius * np.cos(theta_right)
        right_y = self.radius + self.radius * np.sin(theta_right)
        
        # Top straight: right to left
        top_x = np.linspace(self.straight_length/2, -self.straight_length/2, 
                           self.points_per_straight, endpoint=False)
        top_y = np.full(self.points_per_straight, 2 * self.radius)
        
        # Left semicircle: top to bottom, center at (-straight_length/2, radius)
        theta_left = np.linspace(np.pi/2, 3*np.pi/2, self.points_per_semicircle, endpoint=False)
        left_x = -self.straight_length/2 + self.radius * np.cos(theta_left)
        left_y = self.radius + self.radius * np.sin(theta_left)
        
        # Combine all sections
        return np.column_stack([
            np.concatenate([bottom_x, right_x, top_x, left_x]),
            np.concatenate([bottom_y, right_y, top_y, left_y])
        ])

    def regenerate(self, radius: float) -> None:
        self.radius = radius
        super().update_points(self._generate())


class AdaptivePurePursuit:
    """Pure Pursuit controller with tunable gains and feed-forward hooks."""

    def __init__(
        self,
        wheelbase: float,
        lookahead_gain: float,
        min_lookahead: float,
        max_lookahead: float,
        lateral_error_gain: float,
        heading_error_gain: float,
        curvature_ff_gain: float,
        max_steering: float,
        steering_limit_start_speed: float = 4.0,
        steering_limit_full_speed: float = 6.0,
        high_speed_max_steering: float = 0.18,
    ):
        self.wheelbase = wheelbase
        self.lookahead_gain = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lateral_error_gain = lateral_error_gain
        self.heading_error_gain = heading_error_gain
        self.curvature_ff_gain = curvature_ff_gain
        self.max_steering_angle = max_steering
        self.steering_limit_start_speed = steering_limit_start_speed
        self.steering_limit_full_speed = steering_limit_full_speed
        self.high_speed_max_steering_angle = high_speed_max_steering
        self._sanitize_limits()

    def update(self, **kwargs: float) -> None:
        for name, value in kwargs.items():
            if name == "max_steering":
                name = "max_steering_angle"
            if hasattr(self, name):
                setattr(self, name, float(value))
        self._sanitize_limits()

    def _sanitize_limits(self) -> None:
        self.max_steering_angle = abs(float(self.max_steering_angle))
        self.max_steering = self.max_steering_angle
        self.steering_limit_start_speed = max(0.0, float(self.steering_limit_start_speed))
        self.steering_limit_full_speed = max(
            self.steering_limit_start_speed,
            float(self.steering_limit_full_speed),
        )
        self.high_speed_max_steering_angle = max(
            0.0,
            min(self.max_steering_angle, abs(float(self.high_speed_max_steering_angle))),
        )

    def compute_lookahead(self, velocity: float) -> float:
        ld = self.lookahead_gain * abs(velocity) + self.min_lookahead
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

    def compute(
        self,
        current_pos: np.ndarray,
        current_yaw: float,
        lookahead_point: np.ndarray,
        lookahead_heading: float,
        path_curvature: float,
        velocity: float,
        cross_track_error: float,
    ) -> Tuple[float, Dict[str, float]]:
        ld = max(self.compute_lookahead(velocity), 1e-3)
        dx = lookahead_point[0] - current_pos[0]
        dy = lookahead_point[1] - current_pos[1]
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        y_ld = -sin_yaw * dx + cos_yaw * dy
        curvature_term = 2.0 * y_ld / max(ld**2, 1e-6)
        steering = math.atan(self.wheelbase * curvature_term)
        steering *= self.lateral_error_gain
        heading_error = _angle_wrap(lookahead_heading - current_yaw)
        steering += self.heading_error_gain * heading_error
        steering += self.curvature_ff_gain * path_curvature
        raw_steering = float(steering)
        steering_limit = self.steering_limit_for_speed(velocity)
        steering = float(np.clip(raw_steering, -steering_limit, steering_limit))
        clipped = abs(raw_steering - steering) > 1e-6
        speed_scale = steering_limit / max(self.max_steering_angle, 1e-6)
        debug = {
            "lookahead": ld,
            "heading_error": heading_error,
            "curvature_term": curvature_term,
            "cross_track_error": cross_track_error,
            "raw_steering": raw_steering,
            "steering_limit": steering_limit,
            "steering_clipped": clipped,
            "speed_scale": speed_scale,
        }
        return steering, debug


class TrackingMetrics:
    """Rolling statistics to visualize tuning progress in logs."""

    def __init__(self, window: int = 200):
        self.cte: Deque[float] = deque(maxlen=window)
        self.heading: Deque[float] = deque(maxlen=window)
        self.steering: Deque[float] = deque(maxlen=window)

    def update(self, cross_track: float, heading_error: float, steering: float) -> None:
        self.cte.append(abs(cross_track))
        self.heading.append(abs(heading_error))
        self.steering.append(abs(steering))

    def summary(self) -> str:
        def _rms(values: Deque[float]) -> float:
            if not values:
                return 0.0
            arr = np.array(values)
            return float(np.sqrt(np.mean(arr**2)))

        mean_steer = float(np.mean(self.steering)) if self.steering else 0.0
        return (
            f"cte_rms={_rms(self.cte):.03f}m | "
            f"heading_rms={_rms(self.heading):.03f}rad | "
            f"mean_steer={mean_steer:.03f}rad"
        )


class PPTuningNode(Node):
    """ROS2 node that concentrates purely on Pure Pursuit tuning."""

    def __init__(self) -> None:
        super().__init__('pp_param_tuner')
        self._declare_parameters()
        self._build_helpers()
        self._setup_interfaces()
        self._start_time = self.get_clock().now()
        self._trajectory_idx = 0
        self._last_log_time = 0.0
        self._odom_ready = False
        self.get_logger().info('PP tuner ready 🚗 start driving and tweak parameters!')

    # Parameter plumbing -------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Control parameters with descriptors for rqt_reconfigure
        lookahead_desc = ParameterDescriptor(
            description='Lookahead gain multiplier for velocity',
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=5.0, step=0.1)]
        )
        min_lookahead_desc = ParameterDescriptor(
            description='Minimum lookahead distance (m)',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.05)]
        )
        max_lookahead_desc = ParameterDescriptor(
            description='Maximum lookahead distance (m)',
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=10.0, step=0.1)]
        )
        lateral_desc = ParameterDescriptor(
            description='Lateral error gain',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=3.0, step=0.1)]
        )
        heading_desc = ParameterDescriptor(
            description='Heading error gain',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.05)]
        )
        curvature_desc = ParameterDescriptor(
            description='Curvature feedforward gain',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]
        )
        
        self.wheelbase = self.declare_parameter('wheelbase', 0.36).value
        self.max_steering_angle = self.declare_parameter('max_steering_angle', 0.4751).value
        self.max_steering = self.declare_parameter('max_steering', self.max_steering_angle).value
        self.max_steering_angle = float(max(0.0, self.max_steering))
        self.lookahead_gain = self.declare_parameter('lookahead_gain', 1.0, lookahead_desc).value
        self.min_lookahead = self.declare_parameter('min_lookahead', 0.30, min_lookahead_desc).value
        self.max_lookahead = self.declare_parameter('max_lookahead', 4.5, max_lookahead_desc).value
        self.lateral_error_gain = self.declare_parameter('lateral_error_gain', 1.0, lateral_desc).value
        self.heading_error_gain = self.declare_parameter('heading_error_gain', 0.1, heading_desc).value
        self.curvature_ff_gain = self.declare_parameter('curvature_ff_gain', 0.1, curvature_desc).value
        self.command_frequency = self.declare_parameter('command_frequency', 50.0).value
        
        # High-speed steering limit parameters.
        steer_limit_start_desc = ParameterDescriptor(
            description='Speed where steering limit starts shrinking (m/s)',
            floating_point_range=[FloatingPointRange(from_value=3.0, to_value=15.0, step=0.5)]
        )
        steer_limit_full_desc = ParameterDescriptor(
            description='Speed where high-speed steering limit is fully applied (m/s)',
            floating_point_range=[FloatingPointRange(from_value=5.0, to_value=20.0, step=0.5)]
        )
        high_speed_steer_desc = ParameterDescriptor(
            description='Absolute steering limit at and above steering_limit_full_speed (rad)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.6, step=0.01)]
        )
        self.steering_limit_start_speed = self.declare_parameter(
            'steering_limit_start_speed', 4.0, steer_limit_start_desc
        ).value
        self.steering_limit_full_speed = self.declare_parameter(
            'steering_limit_full_speed', 6.0, steer_limit_full_desc
        ).value
        self.high_speed_max_steering_angle = self.declare_parameter(
            'high_speed_max_steering_angle', 0.18, high_speed_steer_desc
        ).value
        self.steering_limit_start_speed = float(max(0.0, self.steering_limit_start_speed))
        self.steering_limit_full_speed = float(
            max(self.steering_limit_start_speed, self.steering_limit_full_speed)
        )
        self.high_speed_max_steering_angle = float(
            max(0.0, min(self.max_steering_angle, self.high_speed_max_steering_angle))
        )
        
        # Racetrack trajectory parameters with descriptors for live tuning
        track_radius_desc = ParameterDescriptor(
            description='Radius of the semicircular turns (m)',
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=10.0, step=0.1)]
        )
        track_straight_desc = ParameterDescriptor(
            description='Length of straight sections (m)',
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=100.0, step=0.5)]
        )
        track_pts_straight_desc = ParameterDescriptor(
            description='Number of points per straight section',
            integer_range=[IntegerRange(from_value=10, to_value=500, step=10)]
        )
        track_pts_semicircle_desc = ParameterDescriptor(
            description='Number of points per semicircle section',
            integer_range=[IntegerRange(from_value=10, to_value=300, step=10)]
        )
        
        self.track_radius = self.declare_parameter('track_radius', 3.0, track_radius_desc).value
        self.track_straight_length = self.declare_parameter('track_straight_length', 20.0, track_straight_desc).value
        self.track_points_per_straight = self.declare_parameter('track_points_per_straight', 150, track_pts_straight_desc).value
        self.track_points_per_semicircle = self.declare_parameter('track_points_per_semicircle', 100, track_pts_semicircle_desc).value
        
        self.use_external_path = self.declare_parameter('use_external_path', False).value
        
        # Target speed with descriptor for easy tuning
        speed_desc = ParameterDescriptor(
            description='Target speed (m/s) for trajectory tracking',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)]
        )
        self.target_speed = self.declare_parameter('target_speed', 1.5, speed_desc).value
        self.log_interval = self.declare_parameter('log_interval', 1.0).value
        self.metrics_window = int(self.declare_parameter('metrics_window', 200).value)
        
        # Trajectory offset parameters for manual tuning
        traj_offset_x_desc = ParameterDescriptor(
            description='Trajectory X offset (m) - can be adjusted live',
            floating_point_range=[FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.05)]
        )
        traj_offset_y_desc = ParameterDescriptor(
            description='Trajectory Y offset (m) - can be adjusted live',
            floating_point_range=[FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.05)]
        )
        traj_offset_yaw_desc = ParameterDescriptor(
            description='Trajectory YAW offset (rad) - can be adjusted live',
            floating_point_range=[FloatingPointRange(from_value=-3.14, to_value=3.14, step=0.01)]
        )
        self.traj_offset_x = self.declare_parameter('traj_offset_x', 0.0, traj_offset_x_desc).value
        self.traj_offset_y = self.declare_parameter('traj_offset_y', 0.0, traj_offset_y_desc).value
        self.traj_offset_yaw = self.declare_parameter('traj_offset_yaw', 0.0, traj_offset_yaw_desc).value
        self.use_first_odom_as_origin = bool(self.declare_parameter(
            'use_first_odom_as_origin', True
        ).value)

        self.add_on_set_parameters_callback(self._on_parameter_change)

    def _build_helpers(self) -> None:
        self.figure8 = Figure8Trajectory(
            radius=self.track_radius,
            straight_length=self.track_straight_length,
            points_per_straight=self.track_points_per_straight,
            points_per_semicircle=self.track_points_per_semicircle
        )
        self.external_traj: Optional[TrajectoryHelper] = None
        self.controller = AdaptivePurePursuit(
            wheelbase=self.wheelbase,
            lookahead_gain=self.lookahead_gain,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lateral_error_gain=self.lateral_error_gain,
            heading_error_gain=self.heading_error_gain,
            curvature_ff_gain=self.curvature_ff_gain,
            max_steering=self.max_steering_angle,
            steering_limit_start_speed=self.steering_limit_start_speed,
            steering_limit_full_speed=self.steering_limit_full_speed,
            high_speed_max_steering=self.high_speed_max_steering_angle,
        )
        self.metrics = TrackingMetrics(window=self.metrics_window)
        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.current_velocity = 0.0
        self.trajectory_initialized = False
        self._first_odom_origin: Optional[np.ndarray] = None
        self.trajectory_offset = np.zeros(2, dtype=float)
        self.trajectory_rotation = 0.0
        self._update_trajectory_transform()

    def _setup_interfaces(self) -> None:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_callback, qos)
        self.create_subscription(Path, '/pp/reference_path', self._path_callback, qos)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.trajectory_pub = self.create_publisher(Path, '/calib/current_trajectory', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/calib/lookahead_point', 10)
        self.status_pub = self.create_publisher(Marker, '/calib/status_text', 10)
        self.create_timer(1.0 / max(self.command_frequency, 1.0), self._control_loop)
        self.create_timer(2.0, self._publish_trajectory)  # Publish trajectory every 5 seconds

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        controller_updates: Dict[str, float] = {}
        for param in params:
            name = param.name
            value = param.value
            if name in {
                'lookahead_gain', 'min_lookahead', 'max_lookahead', 'lateral_error_gain',
                'heading_error_gain', 'curvature_ff_gain'
            }:
                controller_updates[name] = float(value)
            elif name in {'max_steering', 'max_steering_angle'}:
                self.max_steering_angle = float(max(0.0, value))
                self.max_steering = self.max_steering_angle
                controller_updates['max_steering_angle'] = self.max_steering_angle
            elif name == 'steering_limit_start_speed':
                self.steering_limit_start_speed = float(max(0.0, value))
                if self.steering_limit_full_speed < self.steering_limit_start_speed:
                    self.steering_limit_full_speed = self.steering_limit_start_speed
                controller_updates[name] = self.steering_limit_start_speed
                controller_updates['steering_limit_full_speed'] = self.steering_limit_full_speed
            elif name == 'steering_limit_full_speed':
                self.steering_limit_full_speed = float(
                    max(self.steering_limit_start_speed, value)
                )
                controller_updates[name] = self.steering_limit_full_speed
            elif name == 'high_speed_max_steering_angle':
                self.high_speed_max_steering_angle = float(
                    max(0.0, min(self.max_steering_angle, value))
                )
                controller_updates[name] = self.high_speed_max_steering_angle
            elif name == 'target_speed':
                self.target_speed = float(value)
            elif name == 'log_interval':
                self.log_interval = max(float(value), 0.1)
            elif name == 'track_radius':
                self.track_radius = float(value)
                self.figure8.regenerate(self.track_radius)
            elif name in ['track_straight_length', 'track_points_per_straight', 'track_points_per_semicircle']:
                # Update trajectory parameters (requires regeneration with all params)
                if name == 'track_straight_length':
                    self.track_straight_length = float(value)
                elif name == 'track_points_per_straight':
                    self.track_points_per_straight = int(value)
                elif name == 'track_points_per_semicircle':
                    self.track_points_per_semicircle = int(value)
                # Recreate trajectory with updated parameters
                self.figure8 = Figure8Trajectory(
                    radius=self.track_radius,
                    straight_length=self.track_straight_length,
                    points_per_straight=self.track_points_per_straight,
                    points_per_semicircle=self.track_points_per_semicircle
                )
            elif name == 'metrics_window':
                self.metrics_window = int(max(value, 10))
                self.metrics = TrackingMetrics(window=self.metrics_window)
            elif name == 'use_external_path':
                self.use_external_path = bool(value)
                self._update_trajectory_transform()
            elif name == 'traj_offset_x':
                self.traj_offset_x = float(value)
                self._update_trajectory_transform()
            elif name == 'traj_offset_y':
                self.traj_offset_y = float(value)
                self._update_trajectory_transform()
            elif name == 'traj_offset_yaw':
                self.traj_offset_yaw = float(value)
                self._update_trajectory_transform()
            elif name == 'use_first_odom_as_origin':
                self.use_first_odom_as_origin = bool(value)
                self._update_trajectory_transform()

        if controller_updates:
            self.controller.update(**controller_updates)
            self.max_steering_angle = self.controller.max_steering_angle
            self.max_steering = self.max_steering_angle
            self.steering_limit_start_speed = self.controller.steering_limit_start_speed
            self.steering_limit_full_speed = self.controller.steering_limit_full_speed
            self.high_speed_max_steering_angle = self.controller.high_speed_max_steering_angle

        return SetParametersResult(successful=True)

    # Subscriptions ------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        quat = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2),
        )
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        if self._first_odom_origin is None:
            self._first_odom_origin = self.current_pos.astype(float).copy()
            self._update_trajectory_transform()
        self._odom_ready = True

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        points = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        if len(points) < 2:
            self.get_logger().warning('Received external path with <2 points, ignoring')
            return
        self.external_traj = TrajectoryHelper(points)
        self._update_trajectory_transform()
        self.get_logger().info(f'Loaded external path with {len(points)} points')

    # Core control loop -------------------------------------------------------
    def _active_trajectory(self) -> TrajectoryHelper:
        if self.use_external_path and self.external_traj is not None:
            return self.external_traj
        return self.figure8
    
    def _transform_trajectory_point(self, point: np.ndarray) -> np.ndarray:
        """Transform trajectory point from local frame to odom frame."""
        # Rotate by initial yaw
        cos_yaw = np.cos(self.trajectory_rotation)
        sin_yaw = np.sin(self.trajectory_rotation)
        rotated = np.array([
            cos_yaw * point[0] - sin_yaw * point[1],
            sin_yaw * point[0] + cos_yaw * point[1]
        ])
        # Translate by initial position
        return rotated + self.trajectory_offset

    def _trajectory_origin_base(self) -> np.ndarray:
        if self.use_external_path and self.external_traj is not None:
            return np.zeros(2, dtype=float)
        if self.use_first_odom_as_origin and self._first_odom_origin is not None:
            return self._first_odom_origin
        return np.zeros(2, dtype=float)

    def _update_trajectory_transform(self) -> None:
        base = self._trajectory_origin_base()
        manual_offset = np.array([self.traj_offset_x, self.traj_offset_y], dtype=float)
        self.trajectory_offset = base + manual_offset
        self.trajectory_rotation = float(self.traj_offset_yaw)

    def _control_loop(self) -> None:
        if not self._odom_ready:
            return
        
        # Initialize trajectory at first odometry message (add user offset on top)
        if not self.trajectory_initialized:
            self._update_trajectory_transform()
            self.trajectory_initialized = True
            origin = (
                self._first_odom_origin
                if self._first_odom_origin is not None
                else np.zeros(2, dtype=float)
            )
            self.get_logger().info(
                f'Trajectory initialized at pos=({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}), '
                f'yaw={self.current_yaw:.2f}rad with first_odom_origin=({origin[0]:.2f}, {origin[1]:.2f}), '
                f'offset=({self.trajectory_offset[0]:.2f}, {self.trajectory_offset[1]:.2f}), '
                f'yaw_offset={self.trajectory_rotation:.2f}rad'
            )
        else:
            # Update offset from current parameters on every cycle (enables live tuning)
            self._update_trajectory_transform()
        
        now = self.get_clock().now().nanoseconds * 1e-9
        trajectory = self._active_trajectory()
        
        # Transform current position to trajectory's local frame
        dx = self.current_pos[0] - self.trajectory_offset[0]
        dy = self.current_pos[1] - self.trajectory_offset[1]
        cos_yaw = np.cos(-self.trajectory_rotation)
        sin_yaw = np.sin(-self.trajectory_rotation)
        local_pos = np.array([
            cos_yaw * dx - sin_yaw * dy,
            sin_yaw * dx + cos_yaw * dy
        ])
        
        closest, self._trajectory_idx, _ = trajectory.closest_point(
            local_pos, self._trajectory_idx
        )
        heading_at_closest = trajectory.heading(self._trajectory_idx)
        dx_local = local_pos[0] - closest[0]
        dy_local = local_pos[1] - closest[1]
        cross_track = -math.sin(heading_at_closest) * dx_local + math.cos(heading_at_closest) * dy_local
        
        lookahead_distance = self.controller.compute_lookahead(self.current_velocity)
        lookahead_point_local, lookahead_idx = trajectory.lookahead_point(
            self._trajectory_idx, lookahead_distance
        )
        lookahead_heading_local = trajectory.heading(lookahead_idx)
        path_curvature = trajectory.curvature(lookahead_idx)
        
        # Transform lookahead point to odom frame for visualization
        lookahead_point_odom = self._transform_trajectory_point(lookahead_point_local)
        
        # Transform lookahead heading to odom frame
        lookahead_heading = lookahead_heading_local + self.trajectory_rotation
        
        steering_cmd, debug = self.controller.compute(
            self.current_pos,
            self.current_yaw,
            lookahead_point_odom,
            lookahead_heading,
            path_curvature,
            self.current_velocity,
            cross_track,
        )
        
        # Publish lookahead point for visualization
        lookahead_msg = PointStamped()
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.header.frame_id = 'odom'
        lookahead_msg.point.x = float(lookahead_point_odom[0])
        lookahead_msg.point.y = float(lookahead_point_odom[1])
        lookahead_msg.point.z = 0.0
        self.lookahead_pub.publish(lookahead_msg)
        
        self.metrics.update(cross_track, debug['heading_error'], steering_cmd)
        
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = float(self.target_speed)
        cmd.drive.steering_angle = steering_cmd
        self.publisher.publish(cmd)
        self._publish_status_text(steering_cmd, cross_track, debug)

        if now - self._last_log_time > self.log_interval:
            self._last_log_time = now
            clipped = bool(debug.get('steering_clipped', False))
            self.get_logger().info(
                f"v={self.current_velocity:.2f}m/s -> target={self.target_speed:.2f} | "
                f"δ={steering_cmd:.3f}rad raw={debug['raw_steering']:.3f}rad | "
                f"steer_limit={debug['steering_limit']:.3f}rad clipped={clipped} | "
                f"ld={debug['lookahead']:.2f}m | "
                f"cte={cross_track:.3f}m | {self.metrics.summary()}"
            )

    def _publish_status_text(
        self,
        steering_cmd: float,
        cross_track: float,
        debug: Dict[str, float],
    ) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'pp_param_tuner'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.current_pos[0])
        marker.pose.position.y = float(self.current_pos[1])
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        clipped = bool(debug.get('steering_clipped', False))
        marker.text = (
            f"pp_param_tuner | target={self.target_speed:.2f}m/s\n"
            f"v={self.current_velocity:.2f}m/s | ld={debug['lookahead']:.2f}m | "
            f"cte={cross_track:.2f}m\n"
            f"steer={steering_cmd:.3f}rad raw={debug['raw_steering']:.3f}rad | "
            f"steer_limit={debug['steering_limit']:.3f}rad clipped={clipped}"
        )
        self.status_pub.publish(marker)

    def _publish_stop(self, repeat: int = 10) -> None:
        for _ in range(max(1, int(repeat))):
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            cmd.drive.steering_angle_velocity = 0.0
            cmd.drive.acceleration = 0.0
            cmd.drive.jerk = 0.0
            self.publisher.publish(cmd)

    def _publish_trajectory(self) -> None:
        """Publish the current active trajectory as a Path message for visualization."""
        if not self.trajectory_initialized:
            return
            
        trajectory = self._active_trajectory()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for point in trajectory.points:
            # Transform each point from local frame to odom frame
            transformed_point = self._transform_trajectory_point(point)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(transformed_point[0])
            pose.pose.position.y = float(transformed_point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.trajectory_pub.publish(path_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PPTuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Shutting down PP tuner...')
    finally:
        if rclpy.ok():
            try:
                node._publish_stop()
                node.get_logger().info('Published stop command: speed=0')
            except Exception as exc:
                node.get_logger().warn(f'Failed to publish stop command during shutdown: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
