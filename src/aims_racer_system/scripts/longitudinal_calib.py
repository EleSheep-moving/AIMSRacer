#!/usr/bin/env python3
"""Unified longitudinal calibration node.

This executable now covers localization-based and RC-intervention workflows:

- workflow=pp_speed_hold: Pure Pursuit Stage A, hold speeds and measure mean current.
- workflow=pp_accel_interval: Pure Pursuit Stage B, current-step accel trials.
- workflow=pp_decel_current: Pure Pursuit Stage C, negative-current decel trials.
- workflow=speed_hold: RC/no-localization Stage A.
- workflow=accel_interval: RC/no-localization Stage B.
- workflow=decel_current: RC/no-localization Stage C.

All workflows publish AckermannDriveStamped to /calib/ackermann_cmd by default.
The jerk field is used as the downstream mode flag:

- jerk=0.0: speed mode, drive.speed is target speed in m/s.
- jerk=2.0: current mode, drive.acceleration is motor current in A.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
from vesc_msgs.msg import VescStateStamped

try:
    from crsf_receiver_msg.msg import CRSFChannels16  # type: ignore
except Exception:  # pragma: no cover - allows importing without RC package.
    CRSFChannels16 = None  # type: ignore


def _try_get_attr(obj, path: str):
    cur = obj
    for part in path.split("."):
        if not hasattr(cur, part):
            return None
        cur = getattr(cur, part)
    return cur


def _extract_current_a(msg) -> Tuple[Optional[float], str]:
    candidates = [
        "state.avg_iq",
        "state.iq",
        "state.current_motor",
        "state.avg_motor_current",
        "state.current",
        "state.current_input",
        "state.avg_input_current",
    ]
    for field in candidates:
        value = _try_get_attr(msg, field)
        if value is None:
            continue
        try:
            val = float(value)
        except Exception:
            continue
        if math.isfinite(val):
            return val, field
    return None, "N/A"


def _extract_erpm(msg) -> float:
    candidates = [
        "state.electrical_rpm",
        "state.speed",
        "state.rpm",
        "electrical_rpm",
        "speed",
        "rpm",
    ]
    for field in candidates:
        value = _try_get_attr(msg, field)
        if value is None:
            continue
        try:
            val = float(value)
        except Exception:
            continue
        if math.isfinite(val):
            return val
    return 0.0


def _angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _load_base_currents(path: Path) -> Dict[float, float]:
    mapping: Dict[float, float] = {}
    if not path.exists():
        return mapping
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            v = float(parts[0])
            i = float(parts[1])
        except Exception:
            continue
        mapping[v] = i
    return mapping


def _normalize_workflow(workflow: str) -> str:
    aliases = {
        "pp_stage_a": "pp_speed_hold",
        "pp_hold": "pp_speed_hold",
        "pp_hold_current": "pp_speed_hold",
        "pp_speed_hold_current": "pp_speed_hold",
        "pp_stage_b": "pp_accel_interval",
        "pp_interval_accel": "pp_accel_interval",
        "pp_speed_interval": "pp_accel_interval",
        "pp_stage_c": "pp_decel_current",
        "pp_brake_interval": "pp_decel_current",
        "pp_braking_interval": "pp_decel_current",
        "stage_a": "speed_hold",
        "hold_current": "speed_hold",
        "speed_hold_current": "speed_hold",
        "stage_b": "accel_interval",
        "interval_accel": "accel_interval",
        "speed_interval": "accel_interval",
        "stage_c": "decel_current",
        "brake_interval": "decel_current",
        "braking_interval": "decel_current",
    }
    value = str(workflow).strip().lower()
    return aliases.get(value, value)


PP_WORKFLOWS = {"pp_speed_hold", "pp_accel_interval", "pp_decel_current"}
DATA_WORKFLOWS = {"speed_hold", "accel_interval", "decel_current"}
ALL_WORKFLOWS = PP_WORKFLOWS | DATA_WORKFLOWS
SPEED_HOLD_WORKFLOWS = {"speed_hold", "pp_speed_hold"}
ACCEL_INTERVAL_WORKFLOWS = {"accel_interval", "pp_accel_interval"}
DECEL_CURRENT_WORKFLOWS = {"decel_current", "pp_decel_current"}


def _workflow_stage(workflow: str) -> str:
    if workflow in SPEED_HOLD_WORKFLOWS:
        return "speed_hold"
    if workflow in ACCEL_INTERVAL_WORKFLOWS:
        return "accel_interval"
    if workflow in DECEL_CURRENT_WORKFLOWS:
        return "decel_current"
    return workflow


@dataclass
class SpeedHoldResult:
    target_speed: float
    mean_current: float
    std_current: float
    samples: int


@dataclass
class AccelTrialResult:
    v0: float
    v1: float
    current_a: float
    t_sec: float
    accel_mps2: float
    reached: bool


@dataclass
class DecelTrialResult:
    target_speed: float
    start_speed: float
    current_a: float
    t_sec: float
    decel_mps2: float
    stopped: bool


class Figure8Trajectory:
    """Generate a stadium trajectory: two straights and two semicircular turns."""

    def __init__(
        self,
        radius: float = 2.0,
        straight_length: float = 6.0,
        points_per_straight: int = 150,
        points_per_semicircle: int = 100,
    ) -> None:
        self.radius = float(radius)
        self.straight_length = float(straight_length)
        self.points_per_straight = int(points_per_straight)
        self.points_per_semicircle = int(points_per_semicircle)
        self.trajectory = self._generate_trajectory()
        self.trajectory_length = len(self.trajectory)

    def _generate_trajectory(self) -> np.ndarray:
        bottom_x = np.linspace(
            -self.straight_length / 2.0,
            self.straight_length / 2.0,
            self.points_per_straight,
            endpoint=False,
        )
        bottom_y = np.zeros(self.points_per_straight)

        theta_right = np.linspace(
            -math.pi / 2.0, math.pi / 2.0, self.points_per_semicircle, endpoint=False
        )
        right_x = self.straight_length / 2.0 + self.radius * np.cos(theta_right)
        right_y = self.radius + self.radius * np.sin(theta_right)

        top_x = np.linspace(
            self.straight_length / 2.0,
            -self.straight_length / 2.0,
            self.points_per_straight,
            endpoint=False,
        )
        top_y = np.full(self.points_per_straight, 2.0 * self.radius)

        theta_left = np.linspace(
            math.pi / 2.0, 3.0 * math.pi / 2.0, self.points_per_semicircle, endpoint=False
        )
        left_x = -self.straight_length / 2.0 + self.radius * np.cos(theta_left)
        left_y = self.radius + self.radius * np.sin(theta_left)

        return np.column_stack(
            [
                np.concatenate([bottom_x, right_x, top_x, left_x]),
                np.concatenate([bottom_y, right_y, top_y, left_y]),
            ]
        )

    def get_closest_point(self, current_pos: np.ndarray, start_idx: int = 0) -> tuple:
        search_range = 50
        indices = [(start_idx + i) % self.trajectory_length for i in range(search_range)]
        points = self.trajectory[indices]
        distances = np.linalg.norm(points - current_pos, axis=1)
        local_idx = int(np.argmin(distances))
        global_idx = indices[local_idx]
        return self.trajectory[global_idx], global_idx, float(distances[local_idx])

    def get_lookahead_point(self, current_idx: int, lookahead_distance: float) -> tuple:
        idx = current_idx % self.trajectory_length
        cumulative_dist = 0.0

        while cumulative_dist < lookahead_distance:
            next_idx = (idx + 1) % self.trajectory_length
            segment_dist = float(np.linalg.norm(self.trajectory[next_idx] - self.trajectory[idx]))
            if cumulative_dist + segment_dist >= lookahead_distance:
                ratio = (lookahead_distance - cumulative_dist) / max(segment_dist, 1e-6)
                point = self.trajectory[idx] + ratio * (
                    self.trajectory[next_idx] - self.trajectory[idx]
                )
                return point, next_idx
            cumulative_dist += segment_dist
            idx = next_idx
        return self.trajectory[idx], idx

    def get_heading(self, idx: int) -> float:
        prev_pt = self.trajectory[(idx - 1) % self.trajectory_length]
        next_pt = self.trajectory[(idx + 1) % self.trajectory_length]
        return math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])

    def get_curvature(self, idx: int) -> float:
        p_prev = self.trajectory[(idx - 1) % self.trajectory_length]
        p_curr = self.trajectory[idx % self.trajectory_length]
        p_next = self.trajectory[(idx + 1) % self.trajectory_length]
        a = float(np.linalg.norm(p_curr - p_prev))
        b = float(np.linalg.norm(p_next - p_curr))
        c = float(np.linalg.norm(p_next - p_prev))
        denom = max(a * b * c, 1e-6)
        area = (
            p_prev[0] * (p_curr[1] - p_next[1])
            + p_curr[0] * (p_next[1] - p_prev[1])
            + p_next[0] * (p_prev[1] - p_curr[1])
        ) / 2.0
        return float(4.0 * area / denom)

    def is_in_curve(self, trajectory_idx: int) -> bool:
        idx = trajectory_idx % self.trajectory_length
        bottom_end = self.points_per_straight
        right_end = bottom_end + self.points_per_semicircle
        top_end = right_end + self.points_per_straight
        if idx < bottom_end:
            return False
        if idx < right_end:
            return True
        if idx < top_end:
            return False
        return True


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


class LongitudinalCalibNode(Node):
    def __init__(self) -> None:
        super().__init__("longitudinal_calib")

        self.workflow = _normalize_workflow(
            self.declare_parameter("workflow", "pp_speed_hold").value
        )
        valid_workflows = ALL_WORKFLOWS
        if self.workflow not in valid_workflows:
            raise RuntimeError(
                f"Invalid workflow={self.workflow!r}; expected one of {sorted(valid_workflows)}"
            )

        default_odom = "/odometry/filtered" if self.workflow in PP_WORKFLOWS else "/odom"
        default_vesc = "/sensors/core" if self.workflow in PP_WORKFLOWS else "/sensors/core"
        self.odom_topic = self.declare_parameter("odom_topic", default_odom).value
        self.vesc_topic = self.declare_parameter("vesc_topic", default_vesc).value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/calib/ackermann_cmd").value

        self.command_frequency = float(self.declare_parameter("command_frequency", 50.0).value)
        self.steering_rad = float(self.declare_parameter("steering_rad", 0.0).value)
        self.speed_tolerance = float(self.declare_parameter("speed_tolerance", 0.20).value)
        self.stable_required_sec = float(
            self.declare_parameter("stable_required_sec", 1.0).value
        )

        self.current_pos = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.current_velocity = 0.0
        self.current_speed_x = 0.0
        self.current_erpm = 0.0
        self.has_received_odom = False
        self.has_received_vesc = False
        self.latest_current_a: Optional[float] = None
        self.latest_current_field = "N/A"

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_best_effort
        )
        self.vesc_subscription = self.create_subscription(
            VescStateStamped, self.vesc_topic, self.vesc_callback, qos_best_effort
        )
        self.cmd_publisher = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        self._done = False
        self._last_loop_time: Optional[float] = None
        self.start_time: Optional[float] = None
        self._phase = "RESET"
        self._phase_t0 = self.get_clock().now()
        self._stable_time_acc = 0.0
        self._csv_file = None
        self._csv_writer = None

        if self.workflow in PP_WORKFLOWS:
            self._init_pp_workflow()
        else:
            self._init_data_workflow(qos_best_effort)

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self.timer = self.create_timer(1.0 / max(self.command_frequency, 1e-3), self._on_timer)

        self.get_logger().info(
            f"longitudinal_calib started; workflow={self.workflow}, "
            f"odom={self.odom_topic}, vesc={self.vesc_topic}, cmd={self.cmd_topic}"
        )

    def _init_data_workflow(self, qos_best_effort: QoSProfile) -> None:
        default_rc = self.workflow == "speed_hold"
        self.use_rc_steering = bool(
            self.declare_parameter("use_rc_steering", default_rc).value
        )
        self.rc_topic = self.declare_parameter("rc_topic", "/rc/channels").value
        self.rc_timeout_sec = float(self.declare_parameter("rc_timeout_sec", 0.25).value)
        self.post_turn_settle_sec = float(
            self.declare_parameter("post_turn_settle_sec", 0.8).value
        )
        self.steering_channel = int(self.declare_parameter("steering_channel", 4).value)
        self.steering_limit = float(self.declare_parameter("steering_limit", 0.4751).value)
        self.steering_reverse = bool(self.declare_parameter("steering_reverse", True).value)
        self.channel_mid = int(self.declare_parameter("steering_channel_mid", 968).value)
        self.channel_deadzone = int(self.declare_parameter("channel_deadzone", 100).value)
        self.channel_max_range = int(self.declare_parameter("channel_max_range", 2000).value)
        self.channel_min_range = int(self.declare_parameter("channel_min_range", 0).value)

        self._rc_channels: Optional[List[int]] = None
        self._last_rc_time = self.get_clock().now()
        self._rc_sub = None
        if self.use_rc_steering:
            if CRSFChannels16 is None:
                raise RuntimeError(
                    "crsf_receiver_msg is not available (needed for use_rc_steering)"
                )
            self._rc_sub = self.create_subscription(
                CRSFChannels16, self.rc_topic, self._rc_cb, qos_best_effort
            )

        self._init_stage_output()
        stage = _workflow_stage(self.workflow)
        if stage == "speed_hold":
            self._init_speed_hold_state()
        elif stage == "accel_interval":
            self._init_interval_state()
        else:
            self._init_decel_state()

        if self.use_rc_steering:
            self.get_logger().info(
                f"RC steering enabled: rc={self.rc_topic}, channel={self.steering_channel}, "
                f"mid={self.channel_mid}, deadzone={self.channel_deadzone}, "
                f"post_turn_settle={self.post_turn_settle_sec}s"
            )

    def _init_stage_output(self) -> None:
        self.output_path = Path(
            str(self.declare_parameter("output_path", self._default_output_path()).value)
        ).expanduser()
        self.csv_path = str(self.declare_parameter("csv_path", "").value).strip()
        self._open_data_csv()

    def _default_output_path(self) -> str:
        stage = _workflow_stage(self.workflow)
        if stage == "speed_hold":
            return "speed_hold_current_results.txt"
        if stage == "decel_current":
            return "decel_current_sweep_results.txt"
        return "speed_interval_accel_results.txt"

    def _open_data_csv(self) -> None:
        if not self.csv_path:
            return
        csv_path = Path(self.csv_path).expanduser()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = csv_path.open("w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        stage = _workflow_stage(self.workflow)
        if stage == "speed_hold":
            self._csv_writer.writerow(
                ["t_ros_sec", "target_speed", "measured_speed", "current_A", "current_field"]
            )
        elif stage == "accel_interval":
            self._csv_writer.writerow(
                ["v0", "v1", "current_A", "t_sec", "accel_mps2", "reached", "current_field", "speed_field"]
            )
        else:
            self._csv_writer.writerow(
                [
                    "target_speed",
                    "start_speed",
                    "current_A",
                    "t_sec",
                    "decel_mps2",
                    "stopped",
                    "current_field",
                    "speed_field",
                ]
            )

    def _init_speed_hold_state(self) -> None:
        self.hold_speeds: List[float] = [
            float(v)
            for v in list(self.declare_parameter("speeds", [1, 2, 3, 4, 5, 6, 7, 8]).value)
        ]
        self.hold_time_sec = float(self.declare_parameter("hold_time_sec", 10.0).value)
        self.require_speed_stable = bool(
            self.declare_parameter("require_speed_stable", True).value
        )
        self._hold_stage_idx = 0
        self._hold_target_reached = False
        self._hold_sample_time_acc = 0.0
        self._hold_currents: List[float] = []
        self._hold_results: List[SpeedHoldResult] = []
        self._was_turning = False
        self._post_turn_time_acc = 0.0
        self._phase_t0 = self.get_clock().now()

    def _init_interval_state(self) -> None:
        self.v_start = float(self.declare_parameter("v_start", 1.0).value)
        self.v_end = float(self.declare_parameter("v_end", 8.0).value)
        self.dv = float(self.declare_parameter("dv", 1.0).value)
        self.current_step = float(self.declare_parameter("current_step", 3.0).value)
        self.current_max = float(self.declare_parameter("current_max", 80.0).value)
        self.current_start_step_index = int(
            max(0, self.declare_parameter("current_start_step_index", 1).value)
        )
        self.current_min_override = float(
            self.declare_parameter("current_min_override", float("nan")).value
        )
        self.reset_hold_time_sec = float(
            self.declare_parameter("reset_hold_time_sec", 4.0).value
        )
        self.reset_settle_time_sec = float(
            self.declare_parameter("reset_settle_time_sec", 2.0).value
        )
        self.reach_tolerance = float(self.declare_parameter("reach_tolerance", 0.05).value)
        self.trial_timeout_sec = float(
            self.declare_parameter("trial_timeout_sec", 8.0).value
        )
        self.base_current_file = Path(
            str(self.declare_parameter("base_current_file", "speed_hold_current_results.txt").value)
        ).expanduser()
        self.base_currents = _load_base_currents(self.base_current_file)
        self._v0 = float(self.v_start)
        self._v1 = float(self._v0 + self.dv)
        self._current_list: List[float] = []
        self._current_idx = 0
        self._trial_t0_ros: Optional[float] = None
        self._accel_results: List[AccelTrialResult] = []
        self._was_turning = False
        self._post_turn_time_acc = 0.0
        self._phase = "RESET"
        self._phase_t0 = self.get_clock().now()
        self._prepare_currents_for_interval()

    def _init_decel_state(self) -> None:
        self.v_start = float(self.declare_parameter("v_start", 3.0).value)
        self.v_end = float(self.declare_parameter("v_end", 8.0).value)
        self.dv = float(self.declare_parameter("dv", 1.0).value)
        self.reset_hold_time_sec = float(
            self.declare_parameter("reset_hold_time_sec", 4.0).value
        )
        self.reset_settle_time_sec = float(
            self.declare_parameter("reset_settle_time_sec", 2.0).value
        )
        self.decel_baseline_speed = float(
            self.declare_parameter("decel_baseline_speed", float("nan")).value
        )
        self.decel_low_speed = float(self.declare_parameter("decel_low_speed", 1.0).value)
        self.decel_current_min = float(
            self.declare_parameter("decel_current_min", -20.0).value
        )
        self.decel_current_step = float(
            self.declare_parameter("decel_current_step", 3.0).value
        )
        self.decel_stop_speed_threshold = float(
            self.declare_parameter("decel_stop_speed_threshold", float("nan")).value
        )
        self.max_speed_during_rc = float(
            self.declare_parameter("max_speed_during_rc", 5.0).value
        )
        self.decel_timeout_sec = float(self.declare_parameter("decel_timeout_sec", 8.0).value)
        self._target_speeds = self._build_target_speeds()
        self._target_idx = 0
        self._decel_currents = self._prepare_decel_currents()
        self._decel_current_idx = 0
        self._decel_trial_start_speed: Optional[float] = None
        self._decel_results: List[DecelTrialResult] = []
        self._was_turning = False
        self._post_turn_time_acc = 0.0
        self._rc_paused = False
        self._rc_pause_speed_ref: Optional[float] = None
        self._rc_pause_was_over_max = False
        self._rc_interrupted_decel = False
        self._phase = "RAMP"
        self._phase_t0 = self.get_clock().now()

    def _init_pp_workflow(self) -> None:
        self.wheelbase = self.declare_parameter("wheelbase", 0.36).value
        self.lookahead_gain = self.declare_parameter("lookahead_gain", 1.0).value
        self.min_lookahead = self.declare_parameter("min_lookahead", 0.3).value
        self.max_lookahead = self.declare_parameter("max_lookahead", 4.5).value
        self.lateral_error_gain = self.declare_parameter("lateral_error_gain", 1.0).value
        self.heading_error_gain = self.declare_parameter("heading_error_gain", 0.1).value
        self.curvature_ff_gain = self.declare_parameter("curvature_ff_gain", 0.1).value
        self.max_steering_angle = float(
            self.declare_parameter("max_steering_angle", 0.4751).value
        )
        self.steering_limit_start_speed = float(
            self.declare_parameter("steering_limit_start_speed", 4.0).value
        )
        self.steering_limit_full_speed = float(
            self.declare_parameter("steering_limit_full_speed", 6.0).value
        )
        self.high_speed_max_steering_angle = float(
            self.declare_parameter("high_speed_max_steering_angle", 0.18).value
        )
        self.max_steering_angle = float(max(0.0, self.max_steering_angle))
        self.steering_limit_start_speed = float(max(0.0, self.steering_limit_start_speed))
        self.steering_limit_full_speed = float(
            max(self.steering_limit_start_speed, self.steering_limit_full_speed)
        )
        self.high_speed_max_steering_angle = float(
            max(0.0, min(self.max_steering_angle, self.high_speed_max_steering_angle))
        )
        self.track_radius = self.declare_parameter("track_radius", 7.4).value
        self.track_straight_length = self.declare_parameter("track_straight_length", 70.0).value
        self.track_points_per_straight = self.declare_parameter(
            "track_points_per_straight", 150
        ).value
        self.track_points_per_semicircle = self.declare_parameter(
            "track_points_per_semicircle", 100
        ).value
        self.curve_speed_cap = float(self.declare_parameter("curve_speed_cap", 5.0).value)
        self.curve_exit_confirm_cycles = int(
            self.declare_parameter("curve_exit_confirm_cycles", 10).value
        )

        traj_offset_x_desc = ParameterDescriptor(
            description="Trajectory X offset (m) - can be adjusted live",
            floating_point_range=[
                FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.05)
            ],
        )
        traj_offset_y_desc = ParameterDescriptor(
            description="Trajectory Y offset (m) - can be adjusted live",
            floating_point_range=[
                FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.05)
            ],
        )
        traj_offset_yaw_desc = ParameterDescriptor(
            description="Trajectory YAW offset (rad) - can be adjusted live",
            floating_point_range=[
                FloatingPointRange(from_value=-3.14, to_value=3.14, step=0.01)
            ],
        )
        self.traj_offset_x = self.declare_parameter(
            "traj_offset_x", 0.0, traj_offset_x_desc
        ).value
        self.traj_offset_y = self.declare_parameter(
            "traj_offset_y", 0.0, traj_offset_y_desc
        ).value
        self.traj_offset_yaw = self.declare_parameter(
            "traj_offset_yaw", 0.0, traj_offset_yaw_desc
        ).value
        self.use_first_odom_as_origin = bool(
            self.declare_parameter("use_first_odom_as_origin", True).value
        )
        self._first_odom_origin: Optional[np.ndarray] = None
        self.trajectory_offset = np.zeros(2, dtype=float)
        self.trajectory_rotation = 0.0
        self._update_trajectory_transform()

        self.trajectory_pub = self.create_publisher(PathMsg, "/calib/current_trajectory", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/calib/lookahead_point", 10)
        self.status_pub = self.create_publisher(Marker, "/calib/status_text", 10)
        self.trajectory = Figure8Trajectory(
            radius=self.track_radius,
            straight_length=self.track_straight_length,
            points_per_straight=self.track_points_per_straight,
            points_per_semicircle=self.track_points_per_semicircle,
        )
        self.controller = PurePursuitController(
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
        self.trajectory_idx = 0
        self.trajectory_timer = self.create_timer(2.0, self._publish_trajectory)
        self._in_curve: Optional[bool] = None
        self._straight_confirm_counter = 0

        post_curve_settle = float(
            self.declare_parameter("post_curve_settle_sec", 0.8).value
        )
        self.post_turn_settle_sec = float(
            self.declare_parameter("post_turn_settle_sec", post_curve_settle).value
        )
        self._init_stage_output()
        stage = _workflow_stage(self.workflow)
        if stage == "speed_hold":
            self._init_speed_hold_state()
        elif stage == "accel_interval":
            self._init_interval_state()
        else:
            self._init_decel_state()

    def _rebuild_trajectory(self) -> None:
        self.trajectory = Figure8Trajectory(
            radius=self.track_radius,
            straight_length=self.track_straight_length,
            points_per_straight=self.track_points_per_straight,
            points_per_semicircle=self.track_points_per_semicircle,
        )
        self.trajectory_idx = 0
        self._straight_confirm_counter = 0
        self._publish_trajectory()
        self.get_logger().info(
            "Updated calibration trajectory: "
            f"radius={self.track_radius:.2f}m, "
            f"straight_length={self.track_straight_length:.2f}m, "
            f"points={self.trajectory.trajectory_length}"
        )

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        if self.workflow not in PP_WORKFLOWS:
            return SetParametersResult(successful=True)
        trajectory_dirty = False
        for param in params:
            name = param.name
            value = param.value
            if name == "traj_offset_x":
                self.traj_offset_x = float(value)
                self._update_trajectory_transform()
            elif name == "traj_offset_y":
                self.traj_offset_y = float(value)
                self._update_trajectory_transform()
            elif name == "traj_offset_yaw":
                self.traj_offset_yaw = float(value)
                self._update_trajectory_transform()
            elif name == "use_first_odom_as_origin":
                self.use_first_odom_as_origin = bool(value)
                self._update_trajectory_transform()
                self.trajectory_idx = 0
                self._publish_trajectory()
            elif name == "track_radius":
                self.track_radius = float(max(0.1, value))
                trajectory_dirty = True
            elif name == "track_straight_length":
                self.track_straight_length = float(max(0.0, value))
                trajectory_dirty = True
            elif name == "track_points_per_straight":
                self.track_points_per_straight = int(max(2, value))
                trajectory_dirty = True
            elif name == "track_points_per_semicircle":
                self.track_points_per_semicircle = int(max(4, value))
                trajectory_dirty = True
            elif name == "wheelbase":
                self.wheelbase = float(max(1e-3, value))
                self.controller.wheelbase = self.wheelbase
            elif name == "lookahead_gain":
                self.lookahead_gain = float(max(0.0, value))
                self.controller.lookahead_gain = self.lookahead_gain
            elif name == "min_lookahead":
                self.min_lookahead = float(max(0.01, value))
                self.max_lookahead = max(self.max_lookahead, self.min_lookahead)
                self.controller.min_lookahead = self.min_lookahead
                self.controller.max_lookahead = self.max_lookahead
            elif name == "max_lookahead":
                self.max_lookahead = float(max(self.min_lookahead, value))
                self.controller.max_lookahead = self.max_lookahead
            elif name == "lateral_error_gain":
                self.lateral_error_gain = float(value)
                self.controller.lateral_error_gain = self.lateral_error_gain
            elif name == "heading_error_gain":
                self.heading_error_gain = float(value)
                self.controller.heading_error_gain = self.heading_error_gain
            elif name == "curvature_ff_gain":
                self.curvature_ff_gain = float(value)
                self.controller.curvature_ff_gain = self.curvature_ff_gain
            elif name == "max_steering_angle":
                self.max_steering_angle = float(max(0.0, value))
                self.controller.max_steering_angle = self.max_steering_angle
                self.high_speed_max_steering_angle = float(
                    min(self.high_speed_max_steering_angle, self.max_steering_angle)
                )
                self.controller.high_speed_max_steering_angle = (
                    self.high_speed_max_steering_angle
                )
            elif name == "steering_limit_start_speed":
                self.steering_limit_start_speed = float(max(0.0, value))
                self.controller.steering_limit_start_speed = self.steering_limit_start_speed
                if self.steering_limit_full_speed < self.steering_limit_start_speed:
                    self.steering_limit_full_speed = self.steering_limit_start_speed
                    self.controller.steering_limit_full_speed = self.steering_limit_full_speed
            elif name == "steering_limit_full_speed":
                self.steering_limit_full_speed = float(
                    max(self.steering_limit_start_speed, value)
                )
                self.controller.steering_limit_full_speed = self.steering_limit_full_speed
            elif name == "high_speed_max_steering_angle":
                self.high_speed_max_steering_angle = float(
                    max(0.0, min(self.max_steering_angle, value))
                )
                self.controller.high_speed_max_steering_angle = (
                    self.high_speed_max_steering_angle
                )
            elif name == "curve_exit_confirm_cycles":
                self.curve_exit_confirm_cycles = int(max(1, value))
                self._straight_confirm_counter = 0
            elif name == "curve_speed_cap":
                self.curve_speed_cap = float(max(0.0, value))
            elif name == "speed_tolerance":
                self.speed_tolerance = float(max(0.0, value))
            elif name == "stable_required_sec":
                self.stable_required_sec = float(max(0.0, value))
        if trajectory_dirty:
            self._rebuild_trajectory()
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry) -> None:
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        quat = msg.pose.pose.orientation
        self.current_yaw = self._quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
        try:
            self.current_speed_x = float(msg.twist.twist.linear.x)
        except Exception:
            self.current_speed_x = 0.0
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x * msg.twist.twist.linear.x
            + msg.twist.twist.linear.y * msg.twist.twist.linear.y
        )
        self._capture_first_odom_origin_if_needed()
        self.has_received_odom = True

    def vesc_callback(self, msg: VescStateStamped) -> None:
        self.current_erpm = _extract_erpm(msg)
        self.latest_current_a, self.latest_current_field = _extract_current_a(msg)
        self.has_received_vesc = True

    @staticmethod
    def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _rc_cb(self, msg) -> None:
        self._rc_channels = [
            0,
            msg.ch1,
            msg.ch2,
            msg.ch3,
            msg.ch4,
            msg.ch5,
            msg.ch6,
            msg.ch7,
            msg.ch8,
            msg.ch9,
            msg.ch10,
            msg.ch11,
            msg.ch12,
            msg.ch13,
            msg.ch14,
            msg.ch15,
            msg.ch16,
        ]
        self._last_rc_time = self.get_clock().now()

    def _steering_from_rc(self) -> Tuple[float, bool]:
        if self._rc_channels is None:
            return 0.0, True
        idx = self.steering_channel
        if idx < 1 or idx >= len(self._rc_channels):
            return 0.0, True
        raw = int(self._rc_channels[idx])
        if abs(raw - self.channel_mid) <= int(self.channel_deadzone * 0.2):
            return 0.0, True
        if raw > self.channel_mid:
            denom = max(self.channel_max_range - self.channel_mid, 1)
            normalized = (raw - self.channel_mid) / denom
        else:
            denom = max(self.channel_mid - self.channel_min_range, 1)
            normalized = (raw - self.channel_mid) / denom
        steering = max(-self.steering_limit, min(self.steering_limit, normalized * self.steering_limit))
        if self.steering_reverse:
            steering = -steering
        return float(steering), False

    def _publish_trajectory(self) -> None:
        if self.workflow not in PP_WORKFLOWS or not hasattr(self, "trajectory"):
            return
        if self.use_first_odom_as_origin and self._first_odom_origin is None:
            return
        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for point in self.trajectory.trajectory:
            transformed_point = self._transform_trajectory_point(point)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(transformed_point[0])
            pose.pose.position.y = float(transformed_point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.trajectory_pub.publish(path_msg)

    def _trajectory_origin_base(self) -> np.ndarray:
        if self.use_first_odom_as_origin and self._first_odom_origin is not None:
            return self._first_odom_origin
        return np.zeros(2, dtype=float)

    def _update_trajectory_transform(self) -> None:
        base = self._trajectory_origin_base()
        manual_offset = np.array([self.traj_offset_x, self.traj_offset_y], dtype=float)
        self.trajectory_offset = base + manual_offset
        self.trajectory_rotation = float(self.traj_offset_yaw)

    def _capture_first_odom_origin_if_needed(self) -> None:
        if self.workflow not in PP_WORKFLOWS or not hasattr(self, "trajectory_offset"):
            return
        if self._first_odom_origin is not None:
            return
        self._first_odom_origin = self.current_pos.astype(float).copy()
        self._update_trajectory_transform()
        self.trajectory_idx = 0
        origin = self._first_odom_origin
        origin_mode = "enabled" if self.use_first_odom_as_origin else "disabled"
        self.get_logger().info(
            "PP first odom origin captured: "
            f"x={origin[0]:.3f}, y={origin[1]:.3f}; "
            f"use_first_odom_as_origin={origin_mode}; "
            f"manual offset=({self.traj_offset_x:.3f}, {self.traj_offset_y:.3f}), "
            f"yaw_offset={self.traj_offset_yaw:.3f}rad"
        )
        self._publish_trajectory()

    def _publish_status_text(
        self,
        now_ros,
        stage_name: str,
        in_curve: bool,
        target_speed: float,
        current_cmd_a: float,
        steering_angle: float,
        cross_track_error: float,
        mode_str: str,
        steering_debug: Optional[Dict[str, float]] = None,
    ) -> None:
        if self.workflow not in PP_WORKFLOWS or not hasattr(self, "status_pub"):
            return
        marker = Marker()
        marker.header.stamp = now_ros.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "longitudinal_calib"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.current_pos[0])
        marker.pose.position.y = float(self.current_pos[1])
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        measured_current = (
            f"{self.latest_current_a:.1f}A"
            if self.latest_current_a is not None
            else "nan"
        )
        segment = "CURVE" if in_curve else "STRAIGHT"
        steering_debug = steering_debug or {}
        raw_steering = float(steering_debug.get("raw_steering", steering_angle))
        steering_limit = float(steering_debug.get("steering_limit", abs(steering_angle)))
        lookahead_distance = float(steering_debug.get("lookahead_distance", 0.0))
        clipped = bool(steering_debug.get("steering_clipped", False))
        marker.text = (
            f"{self.workflow} | {stage_name} | {segment}\n"
            f"v={self.current_velocity:.2f}m/s -> target={float(target_speed):.2f}m/s | {mode_str}\n"
            f"I_cmd={float(current_cmd_a):.1f}A | I_meas={measured_current} ({self.latest_current_field})\n"
            f"steer={float(steering_angle):.3f}rad raw={raw_steering:.3f}rad | "
            f"steer_limit={steering_limit:.3f}rad clipped={clipped} | "
            f"ld={lookahead_distance:.2f}m cte={float(cross_track_error):.2f}m"
        )
        self.status_pub.publish(marker)

    def _publish_speed_mode(self, speed_mps: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed_mps)
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.cmd_publisher.publish(msg)

    def _publish_current_mode(self, current_a: float, steering_rad: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = float(steering_rad)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = float(current_a)
        msg.drive.jerk = 2.0
        self.cmd_publisher.publish(msg)

    def _publish_stop(self, repeat: int = 10) -> None:
        for _ in range(max(1, int(repeat))):
            self._publish_current_mode(0.0, 0.0)
            self._publish_speed_mode(0.0, 0.0)

    def _finish(self, reason: str) -> None:
        if self._done:
            return
        self._done = True
        self._publish_stop()
        stage = _workflow_stage(self.workflow)
        if stage == "speed_hold":
            self._write_speed_hold_results()
        elif stage in {"accel_interval", "decel_current"}:
            self._write_interval_results()
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
        if getattr(self, "_event_csv_f", None) is not None:
            self._event_csv_f.flush()
            self._event_csv_f.close()
        if getattr(self, "trajectory_timer", None) is not None:
            self.trajectory_timer.cancel()
        self.timer.cancel()
        self.get_logger().info(f"[DONE] {reason}")

    def _on_timer(self) -> None:
        if self._done:
            return
        now_ros = self.get_clock().now()
        now_sec = time.time()
        if self.start_time is None:
            self.start_time = now_sec
            self.get_logger().info("Calibration started")
        if self._last_loop_time is None:
            dt = 1.0 / max(self.command_frequency, 1e-3)
        else:
            dt = max(0.0, min(now_sec - self._last_loop_time, 0.2))
        self._last_loop_time = now_sec

        if self.workflow in PP_WORKFLOWS:
            self._on_pp_timer(now_ros, dt, now_sec)
        elif self.workflow == "speed_hold":
            self._on_speed_hold_timer(now_ros, dt)
        elif self.workflow == "accel_interval":
            self._on_interval_accel_timer(now_ros, dt)
        else:
            self._on_interval_decel_timer(now_ros, dt)

    def _get_steering_command(self, now_ros) -> Tuple[float, float, bool]:
        steering_cmd = float(self.steering_rad)
        steering_rc = 0.0
        in_deadzone = True
        if self.use_rc_steering:
            if (now_ros - self._last_rc_time).nanoseconds / 1e9 > self.rc_timeout_sec:
                self._finish(f"RC timeout (> {self.rc_timeout_sec}s)")
                return 0.0, 0.0, True
            steering_rc, in_deadzone = self._steering_from_rc()
            steering_cmd = 0.0 if in_deadzone else float(steering_rc)
        return float(steering_cmd), float(steering_rc), bool(in_deadzone)

    def _on_speed_hold_timer(self, now_ros, dt: float) -> None:
        steering_cmd, _, in_deadzone = self._get_steering_command(now_ros)
        if self._done:
            return
        if self._hold_stage_idx >= len(self.hold_speeds):
            self._finish("All speed-hold stages complete")
            return

        target_speed = float(self.hold_speeds[self._hold_stage_idx])
        self._publish_speed_mode(target_speed, steering_cmd)
        measured_speed = self.current_speed_x if self.has_received_odom else None

        if not self._hold_target_reached:
            if measured_speed is not None and abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._hold_target_reached = True
                self._phase_t0 = now_ros
                self._stable_time_acc = 0.0
                self._hold_sample_time_acc = 0.0
                self._post_turn_time_acc = 0.0
                self._was_turning = False
            return

        if self.require_speed_stable and measured_speed is not None:
            if abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return

        if self.latest_current_a is None:
            return
        if self.use_rc_steering and not in_deadzone:
            self._was_turning = True
            self._post_turn_time_acc = 0.0
            return
        if self.use_rc_steering and self._was_turning and self.post_turn_settle_sec > 0.0:
            self._post_turn_time_acc += dt
            if self._post_turn_time_acc < self.post_turn_settle_sec:
                return
            self._was_turning = False

        self._hold_currents.append(float(self.latest_current_a))
        self._hold_sample_time_acc += dt
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                [
                    f"{now_ros.nanoseconds / 1e9:.6f}",
                    f"{target_speed:.3f}",
                    "" if measured_speed is None else f"{measured_speed:.3f}",
                    f"{float(self.latest_current_a):.6f}",
                    self.latest_current_field,
                ]
            )

        if self._hold_sample_time_acc < self.hold_time_sec:
            return

        values = self._hold_currents
        if values:
            n = len(values)
            mean = sum(values) / n
            var = sum((v - mean) ** 2 for v in values) / max(n - 1, 1)
            std = math.sqrt(var)
        else:
            n = 0
            mean = float("nan")
            std = float("nan")
        self._hold_results.append(SpeedHoldResult(target_speed, mean, std, n))
        self.get_logger().info(
            f"[STAGE {self._hold_stage_idx + 1}/{len(self.hold_speeds)}] "
            f"v={target_speed:.1f} m/s -> I={mean:.2f}A (std={std:.2f}, n={n})"
        )
        self._hold_stage_idx += 1
        self._hold_target_reached = False
        self._stable_time_acc = 0.0
        self._hold_sample_time_acc = 0.0
        self._post_turn_time_acc = 0.0
        self._was_turning = False
        self._hold_currents = []

    def _prepare_currents_for_interval(self) -> None:
        base = self.base_currents.get(round(self._v0, 3))
        if base is None:
            base = self.base_currents.get(self._v0)
        step = abs(float(self.current_step))
        if step < 1e-9:
            step = 3.0
        if math.isfinite(self.current_min_override):
            start = float(self.current_min_override)
            start_source = "override"
        else:
            start = float(base) if base is not None else 0.0
            start += float(self.current_start_step_index) * step
            start_source = f"base+{self.current_start_step_index}*step"
        currents: List[float] = []
        cur = max(0.0, start)
        while cur <= self.current_max + 1e-6:
            currents.append(float(cur))
            cur += step
        if not currents:
            currents = [float(min(max(0.0, start), self.current_max))]
        self._current_list = sorted(set(min(self.current_max, max(0.0, c)) for c in currents))
        self._current_idx = 0
        self.get_logger().info(
            f"Interval {self._v0:.1f}->{self._v1:.1f} m/s: "
            f"base_I={base if base is not None else float('nan'):.2f}A, "
            f"start={self._current_list[0]:.1f}A ({start_source}), "
            f"sweep={self._current_list[0]:.1f}..{self._current_list[-1]:.1f}A"
        )

    def _advance_interval(self) -> None:
        self._v0 += self.dv
        self._v1 = self._v0 + self.dv
        if self._v1 <= self.v_end + 1e-6:
            self._prepare_currents_for_interval()

    def _on_interval_accel_timer(self, now_ros, dt: float) -> None:
        steering_cmd, _, in_deadzone = self._get_steering_command(now_ros)
        if self._done:
            return
        if self.use_rc_steering:
            if not in_deadzone:
                self._was_turning = True
                self._post_turn_time_acc = 0.0
                if self._phase != "RESET":
                    self.get_logger().info("Turning detected -> abort trial and reset to v0")
                self._phase = "RESET"
                self._phase_t0 = now_ros
                self._stable_time_acc = 0.0
                self._publish_speed_mode(self._v0, steering_cmd)
                return
            if self._was_turning and self.post_turn_settle_sec > 0.0:
                self._post_turn_time_acc += dt
                self._publish_speed_mode(self._v0, 0.0)
                if self._post_turn_time_acc < self.post_turn_settle_sec:
                    return
                self._was_turning = False
                self._phase = "RESET"
                self._phase_t0 = now_ros
                self._stable_time_acc = 0.0
                return

        if not self.has_received_odom:
            self._publish_speed_mode(self._v0, steering_cmd)
            return
        if self._v1 > self.v_end + 1e-6:
            self._finish("All intervals complete")
            return

        t_phase = (now_ros - self._phase_t0).nanoseconds / 1e9
        if self._phase == "RESET":
            self._publish_speed_mode(self._v0, steering_cmd)
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return
            if abs(self.current_speed_x - self._v0) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return
            if t_phase < self.reset_hold_time_sec:
                return
            self._phase = "TRIAL"
            self._phase_t0 = now_ros
            self._trial_t0_ros = now_ros.nanoseconds / 1e9
            return

        if self._phase != "TRIAL":
            return
        if self._current_idx >= len(self._current_list):
            self._advance_interval()
            self._phase = "RESET"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return

        current_a = float(self._current_list[self._current_idx])
        self._publish_current_mode(current_a, steering_cmd)
        t_trial = t_phase
        reached = self.current_speed_x >= (self._v1 - self.reach_tolerance)
        timed_out = t_trial >= self.trial_timeout_sec
        if not reached and not timed_out:
            return

        dv = self._v1 - self._v0
        accel = dv / max(t_trial, 1e-3)
        self._accel_results.append(
            AccelTrialResult(self._v0, self._v1, current_a, t_trial, accel, reached)
        )
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                [
                    f"{self._v0:.3f}",
                    f"{self._v1:.3f}",
                    f"{current_a:.3f}",
                    f"{t_trial:.4f}",
                    f"{accel:.4f}",
                    "1" if reached else "0",
                    self.latest_current_field,
                    "odom.twist.twist.linear.x",
                ]
            )
        tag = "OK" if reached else "TIMEOUT"
        self.get_logger().info(
            f"[{tag}] {self._v0:.1f}->{self._v1:.1f} m/s @ {current_a:.1f}A: "
            f"t={t_trial:.3f}s, a={accel:.3f} m/s^2"
        )
        self._current_idx += 1
        self._phase = "RESET"
        self._phase_t0 = now_ros
        self._stable_time_acc = 0.0

    def _build_target_speeds(self) -> List[float]:
        if self.dv <= 0.0:
            return [float(self.v_start)]
        speeds: List[float] = []
        cur = float(self.v_start)
        while cur <= self.v_end + 1e-6:
            speeds.append(float(round(cur, 6)))
            cur += float(self.dv)
        return speeds

    def _prepare_decel_currents(self) -> List[float]:
        step = abs(float(self.decel_current_step))
        if step < 1e-9:
            step = 3.0
        current_min = float(self.decel_current_min)
        if current_min > 0.0:
            current_min = -abs(current_min)
        currents: List[float] = [0.0]
        cur = -step
        while cur >= current_min + 1e-6:
            currents.append(float(cur))
            cur -= step
        if currents[-1] > current_min + 1e-6:
            currents.append(float(current_min))
        out: List[float] = []
        seen = set()
        for current in currents:
            key = round(float(current), 6)
            if key in seen:
                continue
            seen.add(key)
            out.append(float(current))
        return out

    def _on_interval_decel_timer(self, now_ros, dt: float) -> None:
        steering_cmd, steering_rc, in_deadzone = self._get_steering_command(now_ros)
        if self._done:
            return
        if not self.has_received_odom:
            self._publish_speed_mode(self.v_start, 0.0 if in_deadzone else steering_rc)
            return
        if self._target_idx >= len(self._target_speeds):
            self._finish("All target speeds complete")
            return

        target_speed = float(self._target_speeds[self._target_idx])
        stop_thr = (
            float(self.decel_stop_speed_threshold)
            if math.isfinite(self.decel_stop_speed_threshold)
            else float(self.decel_low_speed)
        )
        recover_speed = (
            float(self.decel_baseline_speed)
            if math.isfinite(self.decel_baseline_speed)
            else target_speed
        )
        stop_thr = max(0.0, stop_thr)
        current_a = (
            float(self._decel_currents[self._decel_current_idx])
            if self._decel_current_idx < len(self._decel_currents)
            else 0.0
        )

        if target_speed <= stop_thr + self.speed_tolerance:
            self.get_logger().warning(
                f"decel_current: skip target_speed={target_speed:.2f} <= stop_thr={stop_thr:.2f}"
            )
            self._target_idx += 1
            self._decel_current_idx = 0
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return

        if self.use_rc_steering and not in_deadzone:
            if not self._rc_paused:
                self._rc_paused = True
                self._rc_pause_was_over_max = self.current_speed_x > self.max_speed_during_rc + 1e-6
                self._rc_pause_speed_ref = min(self.current_speed_x, self.max_speed_during_rc)
                self._rc_interrupted_decel = self._phase in {"DECEL", "REGAIN"}
            hold_speed = min(
                float(self._rc_pause_speed_ref if self._rc_pause_speed_ref is not None else self.current_speed_x),
                self.max_speed_during_rc,
            )
            self._publish_speed_mode(hold_speed, steering_rc)
            return

        if self.use_rc_steering and in_deadzone and self._rc_paused:
            was_over_max = self._rc_pause_was_over_max
            self._rc_paused = False
            self._rc_pause_speed_ref = None
            self._rc_pause_was_over_max = False
            if self._rc_interrupted_decel:
                self._rc_interrupted_decel = False
                if was_over_max and self.current_speed_x < target_speed - self.speed_tolerance:
                    self._phase = "REGAIN"
                    self._phase_t0 = now_ros
                    self._stable_time_acc = 0.0
                    return
                self._phase = "DECEL"
                self._phase_t0 = now_ros
                self._decel_trial_start_speed = float(self.current_speed_x)
                return

        t_phase = (now_ros - self._phase_t0).nanoseconds / 1e9
        if self._phase == "RAMP":
            self._publish_speed_mode(target_speed, steering_cmd)
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return
            if abs(self.current_speed_x - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return
            if t_phase < self.reset_hold_time_sec:
                return
            self._phase = "DECEL"
            self._phase_t0 = now_ros
            self._decel_trial_start_speed = float(self.current_speed_x)
            return

        if self._phase == "REGAIN":
            self._publish_speed_mode(target_speed, 0.0)
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return
            if abs(self.current_speed_x - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return
            if t_phase < self.reset_hold_time_sec:
                return
            self._phase = "DECEL"
            self._phase_t0 = now_ros
            self._decel_trial_start_speed = float(self.current_speed_x)
            return

        if self._phase == "DECEL":
            if self._decel_trial_start_speed is None:
                self._decel_trial_start_speed = float(self.current_speed_x)
            start_speed = float(self._decel_trial_start_speed)
            stopped = self.current_speed_x < stop_thr
            t_trial = t_phase
            if not stopped:
                self._publish_current_mode(current_a, steering_cmd)
            if stopped or t_trial >= self.decel_timeout_sec:
                decel = max(0.0, start_speed - stop_thr) / max(t_trial, 1e-3)
                self._publish_speed_mode(recover_speed, steering_cmd)
                self._decel_results.append(
                    DecelTrialResult(target_speed, start_speed, current_a, t_trial, decel, stopped)
                )
                if self._csv_writer is not None:
                    self._csv_writer.writerow(
                        [
                            f"{target_speed:.3f}",
                            f"{start_speed:.3f}",
                            f"{current_a:.3f}",
                            f"{t_trial:.4f}",
                            f"{decel:.4f}",
                            "1" if stopped else "0",
                            self.latest_current_field,
                            "odom.twist.twist.linear.x",
                        ]
                    )
                tag = "OK" if stopped else "TIMEOUT"
                self.get_logger().info(
                    f"[{tag}] {target_speed:.1f}->{stop_thr:.1f} m/s @ {current_a:.1f}A: "
                    f"t={t_trial:.3f}s, decel={decel:.3f} m/s^2"
                )
                self._decel_current_idx += 1
                self._decel_trial_start_speed = None
                self._phase = "RECOVER"
                self._phase_t0 = now_ros
                self._stable_time_acc = 0.0
            return

        if self._phase == "RECOVER":
            self._publish_speed_mode(recover_speed, steering_cmd)
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return
            if abs(self.current_speed_x - recover_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return
            if t_phase < self.reset_hold_time_sec:
                return
            if self._decel_current_idx >= len(self._decel_currents):
                self._decel_current_idx = 0
                self._target_idx += 1
                if self._target_idx >= len(self._target_speeds):
                    self._finish("All target speeds complete")
                    return
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0

    def _write_speed_hold_results(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w") as handle:
            handle.write("# longitudinal_calib speed_hold results\n")
            handle.write(f"# generated_by=longitudinal_calib workflow={self.workflow}\n")
            handle.write(f"# vesc_topic={self.vesc_topic} odom_topic={self.odom_topic}\n")
            handle.write(f"# current_field={self.latest_current_field}\n")
            handle.write(f"# hold_time_sec={self.hold_time_sec}\n")
            handle.write("# format: speed_mps\tmean_current_A\tstd_current_A\tsamples\n")
            for result in self._hold_results:
                handle.write(
                    f"{result.target_speed:.3f}\t{result.mean_current:.4f}\t"
                    f"{result.std_current:.4f}\t{result.samples}\n"
                )
        self.get_logger().info(f"Wrote {self.output_path}")

    def _write_interval_results(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w") as handle:
            handle.write("# longitudinal_calib interval_sweep results\n")
            handle.write(f"# generated_by=longitudinal_calib workflow={self.workflow}\n")
            handle.write(f"# vesc_topic={self.vesc_topic} odom_topic={self.odom_topic}\n")
            handle.write(
                f"# current_field={self.latest_current_field} speed_field=odom.twist.twist.linear.x\n"
            )
            stage = _workflow_stage(self.workflow)
            if stage == "accel_interval":
                handle.write(
                    f"# dv={self.dv} current_step={self.current_step} "
                    f"current_start_step_index={self.current_start_step_index} "
                    f"current_max={self.current_max}\n"
                )
                handle.write("# format: v0\tv1\tcurrent_A\tt_sec\taccel_mps2\treached\n")
                for result in self._accel_results:
                    handle.write(
                        f"{result.v0:.3f}\t{result.v1:.3f}\t{result.current_a:.3f}\t"
                        f"{result.t_sec:.4f}\t{result.accel_mps2:.4f}\t{int(result.reached)}\n"
                    )
            else:
                handle.write(
                    f"# decel_low_speed={self.decel_low_speed} dv={self.dv} "
                    f"decel_current_min={self.decel_current_min} "
                    f"decel_current_step={self.decel_current_step} "
                    f"max_speed_during_rc={self.max_speed_during_rc}\n"
                )
                handle.write("# format: target_speed\tstart_speed\tcurrent_A\tt_sec\tdecel_mps2\tstopped\n")
                for result in self._decel_results:
                    handle.write(
                        f"{result.target_speed:.3f}\t{result.start_speed:.3f}\t"
                        f"{result.current_a:.3f}\t{result.t_sec:.4f}\t"
                        f"{result.decel_mps2:.4f}\t{int(result.stopped)}\n"
                    )
        self.get_logger().info(f"Wrote {self.output_path}")

    def _update_in_curve(self, desired_in_curve: bool) -> bool:
        if self._in_curve is None:
            self._in_curve = bool(desired_in_curve)
            self._straight_confirm_counter = 0
            return self._in_curve
        if desired_in_curve:
            self._in_curve = True
            self._straight_confirm_counter = 0
            return True
        if self._in_curve:
            self._straight_confirm_counter += 1
            if self._straight_confirm_counter >= max(1, self.curve_exit_confirm_cycles):
                self._in_curve = False
                self._straight_confirm_counter = 0
        else:
            self._straight_confirm_counter = 0
        return bool(self._in_curve)

    def _transform_trajectory_point(self, point_local: np.ndarray) -> np.ndarray:
        cos_yaw = math.cos(self.trajectory_rotation)
        sin_yaw = math.sin(self.trajectory_rotation)
        rotated = np.array(
            [
                cos_yaw * point_local[0] - sin_yaw * point_local[1],
                sin_yaw * point_local[0] + cos_yaw * point_local[1],
            ],
            dtype=float,
        )
        return rotated + self.trajectory_offset

    def _to_trajectory_local(self, point_odom: np.ndarray) -> np.ndarray:
        dx = float(point_odom[0] - self.trajectory_offset[0])
        dy = float(point_odom[1] - self.trajectory_offset[1])
        cos_yaw = math.cos(-self.trajectory_rotation)
        sin_yaw = math.sin(-self.trajectory_rotation)
        return np.array(
            [cos_yaw * dx - sin_yaw * dy, sin_yaw * dx + cos_yaw * dy], dtype=float
        )

    def _pp_curve_hold_speed(self, reference_speed: float) -> float:
        return min(float(self.curve_speed_cap), max(0.0, float(reference_speed)))

    def _compute_pp_speed_hold_command(self, now_ros, dt: float, in_curve: bool) -> tuple:
        if self._hold_stage_idx >= len(self.hold_speeds):
            return "PP_A_DONE", 0.0, 0.0, 0.0, True

        target_speed = float(self.hold_speeds[self._hold_stage_idx])
        measured_speed = float(self.current_velocity)

        if in_curve:
            self._was_turning = True
            self._post_turn_time_acc = 0.0
            return "PP_A_CURVE_SPEED", 0.0, 0.0, self._pp_curve_hold_speed(target_speed), False

        if self._was_turning and self.post_turn_settle_sec > 0.0:
            self._post_turn_time_acc += dt
            if self._post_turn_time_acc < self.post_turn_settle_sec:
                return "PP_A_POST_CURVE_SETTLE", 0.0, 0.0, target_speed, False
            self._was_turning = False

        if not self._hold_target_reached:
            if abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._hold_target_reached = True
                self._phase_t0 = now_ros
                self._stable_time_acc = 0.0
                self._hold_sample_time_acc = 0.0
                self._post_turn_time_acc = 0.0
            return "PP_A_REACH_SPEED", 0.0, 0.0, target_speed, False

        if self.require_speed_stable:
            if abs(measured_speed - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return "PP_A_STABILIZE", 0.0, 0.0, target_speed, False

        if self.latest_current_a is None:
            return "PP_A_WAIT_CURRENT", 0.0, 0.0, target_speed, False

        self._hold_currents.append(float(self.latest_current_a))
        self._hold_sample_time_acc += dt
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                [
                    f"{now_ros.nanoseconds / 1e9:.6f}",
                    f"{target_speed:.3f}",
                    f"{measured_speed:.3f}",
                    f"{float(self.latest_current_a):.6f}",
                    self.latest_current_field,
                ]
            )

        if self._hold_sample_time_acc < self.hold_time_sec:
            return "PP_A_SAMPLE_CURRENT", 0.0, 0.0, target_speed, False

        values = self._hold_currents
        if values:
            n = len(values)
            mean = sum(values) / n
            var = sum((v - mean) ** 2 for v in values) / max(n - 1, 1)
            std = math.sqrt(var)
        else:
            n = 0
            mean = float("nan")
            std = float("nan")
        self._hold_results.append(SpeedHoldResult(target_speed, mean, std, n))
        self.get_logger().info(
            f"[PP STAGE A {self._hold_stage_idx + 1}/{len(self.hold_speeds)}] "
            f"v={target_speed:.1f} m/s -> I={mean:.2f}A (std={std:.2f}, n={n})"
        )
        self._hold_stage_idx += 1
        self._hold_target_reached = False
        self._stable_time_acc = 0.0
        self._hold_sample_time_acc = 0.0
        self._post_turn_time_acc = 0.0
        self._was_turning = False
        self._hold_currents = []
        return "PP_A_NEXT_SPEED", 0.0, 0.0, target_speed, False

    def _compute_pp_accel_interval_command(self, now_ros, dt: float, in_curve: bool) -> tuple:
        if self._v1 > self.v_end + 1e-6:
            return "PP_B_DONE", 0.0, 0.0, 0.0, True

        if in_curve:
            if self._phase == "TRIAL":
                self.get_logger().info("PP curve entered -> abort accel trial and reset to v0")
            self._was_turning = True
            self._post_turn_time_acc = 0.0
            self._phase = "RESET"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            self._trial_t0_ros = None
            curve_speed = self._pp_curve_hold_speed(max(self.current_velocity, self._v0))
            return "PP_B_CURVE_SPEED", 0.0, 0.0, curve_speed, False

        if self._was_turning and self.post_turn_settle_sec > 0.0:
            self._post_turn_time_acc += dt
            if self._post_turn_time_acc < self.post_turn_settle_sec:
                return "PP_B_POST_CURVE_SETTLE", 0.0, 0.0, self._v0, False
            self._was_turning = False
            self._phase = "RESET"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_B_RESET_AFTER_CURVE", 0.0, 0.0, self._v0, False

        if self._current_idx >= len(self._current_list):
            self._advance_interval()
            if self._v1 > self.v_end + 1e-6:
                return "PP_B_DONE", 0.0, 0.0, 0.0, True
            self._phase = "RESET"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_B_NEXT_INTERVAL", 0.0, 0.0, self._v0, False

        t_phase = (now_ros - self._phase_t0).nanoseconds / 1e9
        if self._phase == "RESET":
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return "PP_B_RESET_SPEED", 0.0, 0.0, self._v0, False
            if abs(self.current_velocity - self._v0) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return "PP_B_STABILIZE_V0", 0.0, 0.0, self._v0, False
            if t_phase < self.reset_hold_time_sec:
                return "PP_B_HOLD_V0", 0.0, 0.0, self._v0, False
            self._phase = "TRIAL"
            self._phase_t0 = now_ros
            self._trial_t0_ros = now_ros.nanoseconds / 1e9
            return "PP_B_START_TRIAL", 0.0, 0.0, self._v0, False

        if self._phase != "TRIAL":
            return "PP_B_WAIT", 0.0, 0.0, self._v0, False

        current_a = float(self._current_list[self._current_idx])
        t_trial = t_phase
        reached = self.current_velocity >= (self._v1 - self.reach_tolerance)
        timed_out = t_trial >= self.trial_timeout_sec
        if not reached and not timed_out:
            return "PP_B_CURRENT_TRIAL", 2.0, current_a, 0.0, False

        dv = self._v1 - self._v0
        accel = dv / max(t_trial, 1e-3)
        self._accel_results.append(
            AccelTrialResult(self._v0, self._v1, current_a, t_trial, accel, reached)
        )
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                [
                    f"{self._v0:.3f}",
                    f"{self._v1:.3f}",
                    f"{current_a:.3f}",
                    f"{t_trial:.4f}",
                    f"{accel:.4f}",
                    "1" if reached else "0",
                    self.latest_current_field,
                    "odom.twist.twist.linear.x",
                ]
            )
        tag = "OK" if reached else "TIMEOUT"
        self.get_logger().info(
            f"[PP STAGE B {tag}] {self._v0:.1f}->{self._v1:.1f} m/s @ "
            f"{current_a:.1f}A: t={t_trial:.3f}s, a={accel:.3f} m/s^2"
        )
        self._current_idx += 1
        self._phase = "RESET"
        self._phase_t0 = now_ros
        self._stable_time_acc = 0.0
        return "PP_B_RECORD_TRIAL", 0.0, 0.0, self._v0, False

    def _compute_pp_decel_current_command(self, now_ros, dt: float, in_curve: bool) -> tuple:
        if self._target_idx >= len(self._target_speeds):
            return "PP_C_DONE", 0.0, 0.0, 0.0, True

        target_speed = float(self._target_speeds[self._target_idx])
        stop_thr = (
            float(self.decel_stop_speed_threshold)
            if math.isfinite(self.decel_stop_speed_threshold)
            else float(self.decel_low_speed)
        )
        recover_speed = (
            float(self.decel_baseline_speed)
            if math.isfinite(self.decel_baseline_speed)
            else target_speed
        )
        stop_thr = max(0.0, stop_thr)
        current_a = (
            float(self._decel_currents[self._decel_current_idx])
            if self._decel_current_idx < len(self._decel_currents)
            else 0.0
        )

        if target_speed <= stop_thr + self.speed_tolerance:
            self.get_logger().warning(
                f"pp_decel_current: skip target_speed={target_speed:.2f} <= stop_thr={stop_thr:.2f}"
            )
            self._target_idx += 1
            self._decel_current_idx = 0
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_C_SKIP_TARGET", 0.0, 0.0, target_speed, False

        if in_curve:
            if self._phase == "DECEL":
                self.get_logger().info("PP curve entered -> abort decel trial and reset target speed")
            self._was_turning = True
            self._post_turn_time_acc = 0.0
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            self._decel_trial_start_speed = None
            curve_speed = self._pp_curve_hold_speed(max(self.current_velocity, stop_thr))
            return "PP_C_CURVE_SPEED", 0.0, 0.0, curve_speed, False

        if self._was_turning and self.post_turn_settle_sec > 0.0:
            self._post_turn_time_acc += dt
            if self._post_turn_time_acc < self.post_turn_settle_sec:
                return "PP_C_POST_CURVE_SETTLE", 0.0, 0.0, target_speed, False
            self._was_turning = False
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_C_RESET_AFTER_CURVE", 0.0, 0.0, target_speed, False

        t_phase = (now_ros - self._phase_t0).nanoseconds / 1e9
        if self._phase == "RAMP":
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return "PP_C_RAMP_SPEED", 0.0, 0.0, target_speed, False
            if abs(self.current_velocity - target_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return "PP_C_STABILIZE_TARGET", 0.0, 0.0, target_speed, False
            if t_phase < self.reset_hold_time_sec:
                return "PP_C_HOLD_TARGET", 0.0, 0.0, target_speed, False
            self._phase = "DECEL"
            self._phase_t0 = now_ros
            self._decel_trial_start_speed = float(self.current_velocity)
            return "PP_C_START_DECEL", 0.0, 0.0, target_speed, False

        if self._phase == "DECEL":
            if self._decel_trial_start_speed is None:
                self._decel_trial_start_speed = float(self.current_velocity)
            start_speed = float(self._decel_trial_start_speed)
            stopped = self.current_velocity < stop_thr
            t_trial = t_phase
            if not stopped and t_trial < self.decel_timeout_sec:
                return "PP_C_CURRENT_DECEL", 2.0, current_a, 0.0, False

            decel = max(0.0, start_speed - stop_thr) / max(t_trial, 1e-3)
            self._decel_results.append(
                DecelTrialResult(target_speed, start_speed, current_a, t_trial, decel, stopped)
            )
            if self._csv_writer is not None:
                self._csv_writer.writerow(
                    [
                        f"{target_speed:.3f}",
                        f"{start_speed:.3f}",
                        f"{current_a:.3f}",
                        f"{t_trial:.4f}",
                        f"{decel:.4f}",
                        "1" if stopped else "0",
                        self.latest_current_field,
                        "odom.twist.twist.linear.x",
                    ]
                )
            tag = "OK" if stopped else "TIMEOUT"
            self.get_logger().info(
                f"[PP STAGE C {tag}] {target_speed:.1f}->{stop_thr:.1f} m/s @ "
                f"{current_a:.1f}A: t={t_trial:.3f}s, decel={decel:.3f} m/s^2"
            )
            self._decel_current_idx += 1
            self._decel_trial_start_speed = None
            self._phase = "RECOVER"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_C_RECORD_DECEL", 0.0, 0.0, recover_speed, False

        if self._phase == "RECOVER":
            if t_phase < self.reset_settle_time_sec:
                self._stable_time_acc = 0.0
                return "PP_C_RECOVER_SPEED", 0.0, 0.0, recover_speed, False
            if abs(self.current_velocity - recover_speed) <= self.speed_tolerance:
                self._stable_time_acc += dt
            else:
                self._stable_time_acc = 0.0
            if self._stable_time_acc < self.stable_required_sec:
                return "PP_C_STABILIZE_RECOVER", 0.0, 0.0, recover_speed, False
            if t_phase < self.reset_hold_time_sec:
                return "PP_C_HOLD_RECOVER", 0.0, 0.0, recover_speed, False
            if self._decel_current_idx >= len(self._decel_currents):
                self._decel_current_idx = 0
                self._target_idx += 1
                if self._target_idx >= len(self._target_speeds):
                    return "PP_C_DONE", 0.0, 0.0, 0.0, True
            self._phase = "RAMP"
            self._phase_t0 = now_ros
            self._stable_time_acc = 0.0
            return "PP_C_NEXT_TRIAL", 0.0, 0.0, target_speed, False

        return "PP_C_WAIT", 0.0, 0.0, target_speed, False

    def _on_pp_timer(self, now_ros, dt: float, now_sec: float) -> None:
        if not self.has_received_odom:
            self.get_logger().warn("Waiting for odometry data...")
            return

        local_pos = self._to_trajectory_local(self.current_pos)
        _, self.trajectory_idx, cross_track_error = self.trajectory.get_closest_point(
            local_pos, self.trajectory_idx
        )
        lookahead_distance = self.controller.compute_lookahead(self.current_velocity)
        lookahead_point_local, lookahead_idx = self.trajectory.get_lookahead_point(
            self.trajectory_idx, lookahead_distance
        )
        lookahead_heading_local = self.trajectory.get_heading(lookahead_idx)
        path_curvature = self.trajectory.get_curvature(lookahead_idx)
        in_curve = self._update_in_curve(bool(self.trajectory.is_in_curve(lookahead_idx)))

        if self.workflow == "pp_speed_hold":
            stage_name, jerk_mode, current_cmd_a, target_speed, done = (
                self._compute_pp_speed_hold_command(now_ros, dt, in_curve)
            )
        elif self.workflow == "pp_accel_interval":
            stage_name, jerk_mode, current_cmd_a, target_speed, done = (
                self._compute_pp_accel_interval_command(now_ros, dt, in_curve)
            )
        elif self.workflow == "pp_decel_current":
            stage_name, jerk_mode, current_cmd_a, target_speed, done = (
                self._compute_pp_decel_current_command(now_ros, dt, in_curve)
            )
        else:
            self._finish(f"Unsupported PP workflow: {self.workflow}")
            return
        if done:
            self._finish(f"{self.workflow} calibration complete")
            return

        lookahead_point = self._transform_trajectory_point(lookahead_point_local)
        lookahead_heading = lookahead_heading_local + self.trajectory_rotation
        steering_angle, steering_debug = self.controller.compute_steering(
            self.current_pos,
            self.current_yaw,
            lookahead_point,
            lookahead_heading,
            path_curvature,
            self.current_velocity,
        )

        lookahead_msg = PointStamped()
        lookahead_msg.header.stamp = now_ros.to_msg()
        lookahead_msg.header.frame_id = "odom"
        lookahead_msg.point.x = float(lookahead_point[0])
        lookahead_msg.point.y = float(lookahead_point[1])
        lookahead_msg.point.z = 0.0
        self.lookahead_pub.publish(lookahead_msg)

        if float(jerk_mode) == 2.0:
            self._publish_current_mode(current_cmd_a, steering_angle)
            mode_str = f"CURRENT_CTRL ({float(current_cmd_a):.1f}A)"
        else:
            self._publish_speed_mode(target_speed, steering_angle)
            mode_str = f"SPEED_CTRL ({float(target_speed):.1f}m/s)"
        self._publish_status_text(
            now_ros=now_ros,
            stage_name=stage_name,
            in_curve=in_curve,
            target_speed=target_speed,
            current_cmd_a=current_cmd_a,
            steering_angle=steering_angle,
            cross_track_error=cross_track_error,
            mode_str=mode_str,
            steering_debug=steering_debug,
        )

        elapsed_time = now_sec - (self.start_time if self.start_time is not None else now_sec)
        if int(elapsed_time) != int(max(0.0, elapsed_time - dt)):
            segment = "CURVE" if in_curve else "STRAIGHT"
            raw_steering = float(steering_debug.get("raw_steering", steering_angle))
            steering_limit = float(steering_debug.get("steering_limit", abs(steering_angle)))
            lookahead_distance = float(steering_debug.get("lookahead_distance", 0.0))
            clipped = bool(steering_debug.get("steering_clipped", False))
            self.get_logger().info(
                f"[{stage_name}] t={elapsed_time:.1f}s | {segment} | "
                f"I_cmd={float(current_cmd_a):.1f}A | "
                f"I_meas={self.latest_current_a if self.latest_current_a is not None else float('nan'):.1f}A"
                f"({self.latest_current_field}) | v={self.current_velocity:.2f}m/s | "
                f"steer={steering_angle:.3f}rad raw={raw_steering:.3f}rad | "
                f"steer_limit={steering_limit:.3f}rad clipped={clipped} | "
                f"ld={lookahead_distance:.2f}m | cte={cross_track_error:.2f}m | {mode_str}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LongitudinalCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("KeyboardInterrupt -> stopping")
            node._finish("KeyboardInterrupt")
    finally:
        if rclpy.ok():
            try:
                node._publish_stop()
                node.get_logger().info("Published stop command: speed=0")
            except Exception as exc:
                node.get_logger().warn(f"Failed to publish stop command during shutdown: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
