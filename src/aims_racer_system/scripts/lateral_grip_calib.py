#!/usr/bin/env python3
"""Fixed-radius lateral grip calibration node.

The node actively drives constant-radius circles, steps speed upward, and
estimates lateral acceleration, lateral force, friction coefficient, and slip
onset from odometry speed plus IMU yaw rate.

It publishes AckermannDriveStamped commands to /calib/ackermann_cmd by default.
The command follows this repository's convention:

- drive.jerk = 0.0: speed mode
- drive.speed: target speed in m/s
- drive.steering_angle: target steering angle in rad
"""

from __future__ import annotations

import csv
import math
import statistics
import time
from dataclasses import asdict, dataclass, fields
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped
from visualization_msgs.msg import Marker


G_MPS2 = 9.80665
EPS = 1.0e-6


class CalibState(Enum):
    WAIT_SENSORS = "WAIT_SENSORS"
    BIAS_CALIBRATION = "BIAS_CALIBRATION"
    RAMP_TO_SPEED = "RAMP_TO_SPEED"
    SETTLE = "SETTLE"
    SAMPLE = "SAMPLE"
    SWITCH_DIRECTION = "SWITCH_DIRECTION"
    DONE = "DONE"
    ABORT = "ABORT"


@dataclass
class GripMetrics:
    time_sec: float
    state: str
    direction: str
    speed_target: float
    radius_cmd: float
    steering_cmd: float
    speed_odom: float
    yaw_rate_imu: float
    yaw_rate_cmd: float
    yaw_rate_odom: float
    yaw_rate_error: float
    ay_imu: float
    ay_yaw: float
    ay_cmd: float
    mu_y: float
    fy_n: float
    radius_est: float
    radius_error: float
    slip_flag: bool
    slip_reason: str
    vesc_current_motor: float
    vesc_current_input: float
    vesc_avg_iq: float
    vesc_duty_cycle: float
    vesc_voltage_input: float

    @classmethod
    def fieldnames(cls) -> List[str]:
        return [field.name for field in fields(cls)]


@dataclass
class StageResult:
    direction: str
    radius_cmd: float
    speed_target: float
    steering_cmd: float
    samples: int
    stable: bool
    slip_flag: bool
    slip_reason: str
    speed_mean: float
    speed_std: float
    yaw_rate_imu_mean: float
    yaw_rate_imu_std: float
    yaw_rate_odom_mean: float
    yaw_rate_odom_std: float
    ay_yaw_mean: float
    ay_yaw_std: float
    ay_imu_mean: float
    ay_imu_std: float
    ay_cmd_mean: float
    mu_y_mean: float
    mu_y_max: float
    fy_mean: float
    fy_max_abs: float
    radius_est_mean: float
    radius_error_mean: float

    @classmethod
    def fieldnames(cls) -> List[str]:
        return [field.name for field in fields(cls)]


def _finite(value: float, default: float = math.nan) -> float:
    try:
        numeric = float(value)
    except Exception:
        return default
    return numeric if math.isfinite(numeric) else default


def _mean(values: Sequence[float]) -> float:
    finite_values = [v for v in values if math.isfinite(v)]
    if not finite_values:
        return math.nan
    return float(sum(finite_values) / len(finite_values))


def _std(values: Sequence[float]) -> float:
    finite_values = [v for v in values if math.isfinite(v)]
    if len(finite_values) <= 1:
        return 0.0
    return float(statistics.pstdev(finite_values))


def _max_abs(values: Sequence[float]) -> float:
    finite_values = [abs(v) for v in values if math.isfinite(v)]
    if not finite_values:
        return math.nan
    return float(max(finite_values))


def _nested_float(obj, path: str) -> float:
    cur = obj
    for part in path.split("."):
        if not hasattr(cur, part):
            return math.nan
        cur = getattr(cur, part)
    return _finite(cur)


def _generate_speed_targets(start: float, end: float, step: float) -> List[float]:
    if start <= 0.0 or end <= 0.0 or step <= 0.0 or end < start:
        return []
    values: List[float] = []
    value = start
    while value <= end + 1.0e-9:
        values.append(round(value, 6))
        value += step
    return values


def _parse_directions(value: str) -> List[Tuple[str, float]]:
    aliases = {
        "left": ("left", 1.0),
        "ccw": ("left", 1.0),
        "counterclockwise": ("left", 1.0),
        "right": ("right", -1.0),
        "cw": ("right", -1.0),
        "clockwise": ("right", -1.0),
    }
    parsed: List[Tuple[str, float]] = []
    for token in str(value).replace(";", ",").split(","):
        key = token.strip().lower()
        if not key:
            continue
        if key not in aliases:
            continue
        candidate = aliases[key]
        if candidate not in parsed:
            parsed.append(candidate)
    return parsed


class LateralGripCalib(Node):
    """Active fixed-radius lateral grip calibration state machine."""

    def __init__(self) -> None:
        super().__init__("lateral_grip_calib")
        self._declare_parameters()
        self._read_parameters()

        self.state = CalibState.WAIT_SENSORS
        self.state_entry_time = self._now()
        self.finished = False
        self.final_reason = ""

        self.last_odom: Optional[Odometry] = None
        self.last_imu: Optional[Imu] = None
        self.last_vesc: Optional[VescStateStamped] = None
        self.last_odom_time = math.nan
        self.last_imu_time = math.nan
        self.last_vesc_time = math.nan

        self.yaw_bias = 0.0
        self.ay_bias = 0.0
        self.bias_yaw_samples: List[float] = []
        self.bias_ay_samples: List[float] = []

        self.direction_index = 0
        self.speed_index = 0
        self.command_speed = 0.0
        self.current_samples: List[GripMetrics] = []
        self.all_samples: List[GripMetrics] = []
        self.results: List[StageResult] = []
        self.sample_elapsed = 0.0
        self.stable_since: Optional[float] = None
        self.unstable_since: Optional[float] = None
        self.slip_since: Optional[float] = None
        self.slip_reason = ""
        self.last_loop_time: Optional[float] = None
        self.last_status_time = 0.0
        self.last_log_times: Dict[str, float] = {}
        self.outputs_written = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, self.cmd_topic, 10
        )
        self.status_pub = self.create_publisher(Marker, self.status_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, sensor_qos)
        self.create_subscription(
            VescStateStamped, self.vesc_topic, self._vesc_cb, sensor_qos
        )

        self.timer = self.create_timer(
            1.0 / max(self.command_frequency, 1.0), self._timer_cb
        )

        self._log_configuration()
        if self.config_error:
            self.get_logger().error(self.config_error)

    def _declare_parameters(self) -> None:
        self.declare_parameter("vehicle_mass", 4.5)
        self.declare_parameter("wheelbase", 0.36)
        self.declare_parameter("test_radius", 3.0)
        self.declare_parameter("speed_start", 0.5)
        self.declare_parameter("speed_end", 5.0)
        self.declare_parameter("speed_step", 0.25)
        self.declare_parameter("settle_time_sec", 2.0)
        self.declare_parameter("hold_time_sec", 5.0)
        self.declare_parameter("stable_required_sec", 1.0)
        self.declare_parameter("speed_tolerance", 0.15)
        self.declare_parameter("speed_ramp_rate", 0.8)
        self.declare_parameter("ramp_timeout_sec", 15.0)
        self.declare_parameter("settle_timeout_sec", 15.0)
        self.declare_parameter("sample_timeout_margin_sec", 10.0)
        self.declare_parameter("direction_settle_sec", 3.0)
        self.declare_parameter("command_frequency", 50.0)
        self.declare_parameter("odom_topic", "/rear_axle/wheel_odom")
        self.declare_parameter("imu_topic", "/rear_axle/imu")
        self.declare_parameter("vesc_topic", "/sensors/core")
        self.declare_parameter("cmd_topic", "/calib/ackermann_cmd")
        self.declare_parameter("status_topic", "/calib/lateral_status_text")
        self.declare_parameter("armed", False)
        self.declare_parameter("require_vesc", True)
        self.declare_parameter("directions", "left,right")
        self.declare_parameter("max_abs_steering", 0.4751)
        self.declare_parameter("max_speed", 6.0)
        self.declare_parameter("sensor_timeout_sec", 0.5)
        self.declare_parameter("stationary_speed_threshold", 0.05)
        self.declare_parameter("imu_bias_samples", 150)
        self.declare_parameter("bias_timeout_sec", 10.0)
        self.declare_parameter("imu_lateral_axis_sign", 1.0)
        self.declare_parameter("imu_yaw_axis_sign", 1.0)
        self.declare_parameter("use_raw_imu_ay_for_abort", False)
        self.declare_parameter("raw_imu_ay_abort", 8.0)
        self.declare_parameter("mu_abort", 0.8)
        self.declare_parameter("yaw_rate_error_threshold", 0.25)
        self.declare_parameter("radius_error_threshold", 0.30)
        self.declare_parameter("slip_confirm_sec", 0.30)
        self.declare_parameter("slip_min_speed", 0.8)
        self.declare_parameter("max_abs_yaw_rate", 3.5)
        self.declare_parameter("enable_plateau_detection", True)
        self.declare_parameter("plateau_min_mu", 0.25)
        self.declare_parameter("plateau_mu_margin", 0.02)
        self.declare_parameter("output_dir", ".")
        self.declare_parameter("samples_path", "lateral_grip_samples.csv")
        self.declare_parameter("results_path", "lateral_grip_results.csv")
        self.declare_parameter("summary_path", "lateral_grip_summary.yaml")
        self.declare_parameter("mu_safe_factor", 0.85)
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("status_frame", "base_link")

    def _read_parameters(self) -> None:
        self.vehicle_mass = float(self.get_parameter("vehicle_mass").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.test_radius = float(self.get_parameter("test_radius").value)
        self.speed_start = float(self.get_parameter("speed_start").value)
        self.speed_end = float(self.get_parameter("speed_end").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.hold_time_sec = float(self.get_parameter("hold_time_sec").value)
        self.stable_required_sec = float(
            self.get_parameter("stable_required_sec").value
        )
        self.speed_tolerance = float(self.get_parameter("speed_tolerance").value)
        self.speed_ramp_rate = float(self.get_parameter("speed_ramp_rate").value)
        self.ramp_timeout_sec = float(self.get_parameter("ramp_timeout_sec").value)
        self.settle_timeout_sec = float(self.get_parameter("settle_timeout_sec").value)
        self.sample_timeout_margin_sec = float(
            self.get_parameter("sample_timeout_margin_sec").value
        )
        self.direction_settle_sec = float(
            self.get_parameter("direction_settle_sec").value
        )
        self.command_frequency = float(self.get_parameter("command_frequency").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.vesc_topic = str(self.get_parameter("vesc_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.require_vesc = bool(self.get_parameter("require_vesc").value)
        self.directions = _parse_directions(str(self.get_parameter("directions").value))
        self.max_abs_steering = float(self.get_parameter("max_abs_steering").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.sensor_timeout_sec = float(self.get_parameter("sensor_timeout_sec").value)
        self.stationary_speed_threshold = float(
            self.get_parameter("stationary_speed_threshold").value
        )
        self.imu_bias_samples = int(self.get_parameter("imu_bias_samples").value)
        self.bias_timeout_sec = float(self.get_parameter("bias_timeout_sec").value)
        self.imu_lateral_axis_sign = float(
            self.get_parameter("imu_lateral_axis_sign").value
        )
        self.imu_yaw_axis_sign = float(self.get_parameter("imu_yaw_axis_sign").value)
        self.use_raw_imu_ay_for_abort = bool(
            self.get_parameter("use_raw_imu_ay_for_abort").value
        )
        self.raw_imu_ay_abort = float(self.get_parameter("raw_imu_ay_abort").value)
        self.mu_abort = float(self.get_parameter("mu_abort").value)
        self.yaw_rate_error_threshold = float(
            self.get_parameter("yaw_rate_error_threshold").value
        )
        self.radius_error_threshold = float(
            self.get_parameter("radius_error_threshold").value
        )
        self.slip_confirm_sec = float(self.get_parameter("slip_confirm_sec").value)
        self.slip_min_speed = float(self.get_parameter("slip_min_speed").value)
        self.max_abs_yaw_rate = float(self.get_parameter("max_abs_yaw_rate").value)
        self.enable_plateau_detection = bool(
            self.get_parameter("enable_plateau_detection").value
        )
        self.plateau_min_mu = float(self.get_parameter("plateau_min_mu").value)
        self.plateau_mu_margin = float(self.get_parameter("plateau_mu_margin").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.samples_path = self._resolve_output_path(
            str(self.get_parameter("samples_path").value)
        )
        self.results_path = self._resolve_output_path(
            str(self.get_parameter("results_path").value)
        )
        self.summary_path = self._resolve_output_path(
            str(self.get_parameter("summary_path").value)
        )
        self.mu_safe_factor = float(self.get_parameter("mu_safe_factor").value)
        self.stop_publish_count = int(self.get_parameter("stop_publish_count").value)
        self.status_frame = str(self.get_parameter("status_frame").value)

        self.speed_targets = _generate_speed_targets(
            self.speed_start, self.speed_end, self.speed_step
        )
        self.steering_abs = math.atan2(self.wheelbase, self.test_radius)
        self.config_error = self._validate_configuration()

    def _resolve_output_path(self, value: str) -> Path:
        path = Path(value).expanduser()
        if path.is_absolute():
            return path
        return self.output_dir.expanduser() / path

    def _validate_configuration(self) -> str:
        errors: List[str] = []
        if self.vehicle_mass <= 0.0:
            errors.append("vehicle_mass must be positive")
        if self.wheelbase <= 0.0:
            errors.append("wheelbase must be positive")
        if self.test_radius <= 0.0:
            errors.append("test_radius must be positive")
        if not self.speed_targets:
            errors.append("speed_start/speed_end/speed_step produce no targets")
        if not self.directions:
            errors.append("directions must include left and/or right")
        if self.steering_abs > self.max_abs_steering + 1.0e-9:
            errors.append(
                "required steering {:.3f} rad exceeds max_abs_steering {:.3f} rad"
                .format(self.steering_abs, self.max_abs_steering)
            )
        if self.speed_end > self.max_speed + 1.0e-9:
            errors.append("speed_end exceeds max_speed")
        if self.command_frequency <= 0.0:
            errors.append("command_frequency must be positive")
        return "; ".join(errors)

    def _log_configuration(self) -> None:
        direction_names = ",".join(name for name, _ in self.directions) or "none"
        self.get_logger().info(
            "lateral_grip_calib config: armed={} mass={:.2f}kg radius={:.2f}m "
            "speeds={:.2f}->{:.2f} step={:.2f} directions={} odom={} imu={} "
            "vesc={} cmd={}".format(
                self.armed,
                self.vehicle_mass,
                self.test_radius,
                self.speed_start,
                self.speed_end,
                self.speed_step,
                direction_names,
                self.odom_topic,
                self.imu_topic,
                self.vesc_topic,
                self.cmd_topic,
            )
        )
        if not self.armed:
            self.get_logger().warn(
                "armed=false: this node will check topics and write no motion commands."
            )

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_time = self._now()

    def _imu_cb(self, msg: Imu) -> None:
        self.last_imu = msg
        self.last_imu_time = self._now()

    def _vesc_cb(self, msg: VescStateStamped) -> None:
        self.last_vesc = msg
        self.last_vesc_time = self._now()

    def _timer_cb(self) -> None:
        now = self._now()
        if self.last_loop_time is None:
            dt = 1.0 / max(self.command_frequency, 1.0)
        else:
            dt = max(0.0, min(now - self.last_loop_time, 0.5))
        self.last_loop_time = now

        self._publish_status(now)

        if self.finished:
            return

        if self.config_error:
            self._abort("configuration error: " + self.config_error)
            return

        if self.state == CalibState.WAIT_SENSORS:
            self._handle_wait_sensors(now)
            return

        if not self._sensors_ready(now):
            self._abort("sensor timeout: " + ", ".join(self._missing_sensors(now)))
            return

        if self.state == CalibState.BIAS_CALIBRATION:
            self._handle_bias_calibration(now)
        elif self.state == CalibState.RAMP_TO_SPEED:
            self._handle_ramp_to_speed(now, dt)
        elif self.state == CalibState.SETTLE:
            self._handle_settle(now)
        elif self.state == CalibState.SAMPLE:
            self._handle_sample(now, dt)
        elif self.state == CalibState.SWITCH_DIRECTION:
            self._handle_switch_direction(now)

    def _handle_wait_sensors(self, now: float) -> None:
        if not self._sensors_ready(now):
            missing = ", ".join(self._missing_sensors(now))
            self._log_periodic(
                "wait_sensors", 1.0, "info", "waiting for sensors: " + missing
            )
            return

        if not self.armed:
            self.get_logger().info(
                "dry check passed with armed=false; no command was published."
            )
            self._finish("dry check complete", write_outputs=False)
            return

        self._transition(CalibState.BIAS_CALIBRATION, "sensors ready")

    def _handle_bias_calibration(self, now: float) -> None:
        speed = self._odom_speed()
        if abs(speed) > self.stationary_speed_threshold:
            self._publish_stop()
            self._log_periodic(
                "bias_moving",
                1.0,
                "warn",
                "waiting for vehicle to stop before IMU bias calibration",
            )
            if now - self.state_entry_time > self.bias_timeout_sec:
                self._abort("vehicle did not become stationary for IMU bias")
            return

        self._publish_stop()
        if self.imu_bias_samples <= 0:
            self.yaw_bias = 0.0
            self.ay_bias = 0.0
            self._transition(CalibState.RAMP_TO_SPEED, "IMU bias disabled")
            return

        assert self.last_imu is not None
        self.bias_yaw_samples.append(float(self.last_imu.angular_velocity.z))
        self.bias_ay_samples.append(float(self.last_imu.linear_acceleration.y))

        if len(self.bias_yaw_samples) >= self.imu_bias_samples:
            self.yaw_bias = _mean(self.bias_yaw_samples)
            self.ay_bias = _mean(self.bias_ay_samples)
            self.get_logger().info(
                "IMU bias complete: yaw_bias={:.5f} rad/s ay_bias={:.5f} m/s^2"
                .format(self.yaw_bias, self.ay_bias)
            )
            self._transition(CalibState.RAMP_TO_SPEED, "IMU bias complete")

    def _handle_ramp_to_speed(self, now: float, dt: float) -> None:
        target = self._target_speed()
        direction, sign = self._current_direction()
        steering = self._steering_cmd(sign)
        step = self.speed_ramp_rate * dt
        self.command_speed = min(target, self.command_speed + step)
        self._publish_speed_command(self.command_speed, steering)

        metrics = self._compute_metrics(
            now, direction, target, steering, slip_flag=False, slip_reason=""
        )
        reason = self._update_slip_detector(now, metrics)
        if reason:
            self._finish_direction_due_to_limit(now, reason)
            return

        if (
            abs(self.command_speed - target) <= 0.01
            and abs(metrics.speed_odom - target) <= self.speed_tolerance
        ):
            self._transition(CalibState.SETTLE, "target speed reached")
            return

        if now - self.state_entry_time > self.ramp_timeout_sec:
            self._abort("ramp timeout at target speed {:.2f}".format(target))

    def _handle_settle(self, now: float) -> None:
        target = self._target_speed()
        direction, sign = self._current_direction()
        steering = self._steering_cmd(sign)
        self._publish_speed_command(target, steering)

        metrics = self._compute_metrics(
            now, direction, target, steering, slip_flag=False, slip_reason=""
        )
        reason = self._update_slip_detector(now, metrics)
        if reason:
            self._finish_direction_due_to_limit(now, reason)
            return

        if self._is_sample_stable(metrics):
            if self.stable_since is None:
                self.stable_since = now
            stable_time = now - self.stable_since
        else:
            self.stable_since = None
            stable_time = 0.0

        required = max(self.settle_time_sec, self.stable_required_sec)
        if stable_time >= required:
            self.current_samples = []
            self.sample_elapsed = 0.0
            self.unstable_since = None
            self._transition(CalibState.SAMPLE, "settled")
            return

        if now - self.state_entry_time > self.settle_timeout_sec:
            self._abort("settle timeout at target speed {:.2f}".format(target))

    def _handle_sample(self, now: float, dt: float) -> None:
        target = self._target_speed()
        direction, sign = self._current_direction()
        steering = self._steering_cmd(sign)
        self._publish_speed_command(target, steering)

        metrics = self._compute_metrics(
            now, direction, target, steering, slip_flag=False, slip_reason=""
        )
        reason = self._update_slip_detector(now, metrics)
        if reason:
            self._finish_direction_due_to_limit(now, reason)
            return

        if self._is_sample_stable(metrics):
            self.unstable_since = None
            self.current_samples.append(metrics)
            self.all_samples.append(metrics)
            self.sample_elapsed += dt
        else:
            if self.unstable_since is None:
                self.unstable_since = now

        if self.sample_elapsed >= self.hold_time_sec:
            result = self._summarize_current_stage(False, "")
            self.results.append(result)
            self.get_logger().info(
                "{} {:.2f}m/s: mu_mean={:.3f} mu_max={:.3f} radius_err={:.2f}"
                .format(
                    result.direction,
                    result.speed_target,
                    result.mu_y_mean,
                    result.mu_y_max,
                    result.radius_error_mean,
                )
            )
            plateau_reason = self._plateau_limit_reason(result)
            if plateau_reason:
                result.slip_flag = True
                result.slip_reason = plateau_reason
                self._finish_direction_due_to_limit(now, plateau_reason, append=False)
            else:
                self._advance_speed_or_direction()
            return

        sample_timeout = self.hold_time_sec + self.sample_timeout_margin_sec
        if now - self.state_entry_time > sample_timeout:
            self._abort("sample timeout at target speed {:.2f}".format(target))

    def _handle_switch_direction(self, now: float) -> None:
        self._publish_stop()
        if now - self.state_entry_time < self.direction_settle_sec:
            return
        self.speed_index = 0
        self.command_speed = 0.0
        self._transition(CalibState.RAMP_TO_SPEED, "next direction")

    def _advance_speed_or_direction(self) -> None:
        self.speed_index += 1
        if self.speed_index < len(self.speed_targets):
            self._reset_trial_state()
            self._transition(CalibState.RAMP_TO_SPEED, "next speed")
            return
        self._advance_direction("completed speed schedule")

    def _advance_direction(self, reason: str) -> None:
        self.get_logger().info(
            "direction {} finished: {}".format(self._current_direction()[0], reason)
        )
        self.direction_index += 1
        self.speed_index = 0
        self.command_speed = 0.0
        self._reset_trial_state()
        if self.direction_index >= len(self.directions):
            self._finish("completed all directions")
        else:
            self._transition(CalibState.SWITCH_DIRECTION, reason)

    def _finish_direction_due_to_limit(
        self, now: float, reason: str, append: bool = True
    ) -> None:
        if append and self.current_samples:
            result = self._summarize_current_stage(True, reason)
            self.results.append(result)
        self._publish_stop()
        self.get_logger().warn(
            "lateral limit detected in direction {}: {}".format(
                self._current_direction()[0], reason
            )
        )
        self._advance_direction(reason)

    def _reset_trial_state(self) -> None:
        self.current_samples = []
        self.sample_elapsed = 0.0
        self.stable_since = None
        self.unstable_since = None
        self.slip_since = None
        self.slip_reason = ""

    def _transition(self, state: CalibState, reason: str) -> None:
        self.state = state
        self.state_entry_time = self._now()
        self.stable_since = None
        self.unstable_since = None
        self.slip_since = None
        self.slip_reason = ""
        self.get_logger().info("state -> {} ({})".format(state.value, reason))

    def _sensors_ready(self, now: float) -> bool:
        return not self._missing_sensors(now)

    def _missing_sensors(self, now: float) -> List[str]:
        missing: List[str] = []
        if self.last_odom is None or now - self.last_odom_time > self.sensor_timeout_sec:
            missing.append(self.odom_topic)
        if self.last_imu is None or now - self.last_imu_time > self.sensor_timeout_sec:
            missing.append(self.imu_topic)
        if self.require_vesc:
            if (
                self.last_vesc is None
                or now - self.last_vesc_time > self.sensor_timeout_sec
            ):
                missing.append(self.vesc_topic)
        return missing

    def _current_direction(self) -> Tuple[str, float]:
        return self.directions[self.direction_index]

    def _target_speed(self) -> float:
        return self.speed_targets[self.speed_index]

    def _steering_cmd(self, direction_sign: float) -> float:
        return direction_sign * self.steering_abs

    def _odom_speed(self) -> float:
        if self.last_odom is None:
            return 0.0
        return _finite(self.last_odom.twist.twist.linear.x, 0.0)

    def _odom_yaw_rate(self) -> float:
        if self.last_odom is None:
            return math.nan
        return _finite(self.last_odom.twist.twist.angular.z)

    def _compute_metrics(
        self,
        now: float,
        direction: str,
        target_speed: float,
        steering: float,
        slip_flag: bool,
        slip_reason: str,
    ) -> GripMetrics:
        assert self.last_imu is not None
        _, direction_sign = self._current_direction()

        speed = self._odom_speed()
        yaw_rate_raw = float(self.last_imu.angular_velocity.z)
        ay_raw = float(self.last_imu.linear_acceleration.y)
        yaw_rate_imu = self.imu_yaw_axis_sign * (yaw_rate_raw - self.yaw_bias)
        ay_imu = self.imu_lateral_axis_sign * (ay_raw - self.ay_bias)

        yaw_rate_cmd = direction_sign * speed / max(self.test_radius, EPS)
        yaw_rate_odom = self._odom_yaw_rate()
        ay_yaw = speed * yaw_rate_imu
        ay_cmd = direction_sign * speed * speed / max(self.test_radius, EPS)
        mu_y = abs(ay_yaw) / G_MPS2
        fy_n = self.vehicle_mass * ay_yaw

        if abs(yaw_rate_imu) > EPS:
            radius_est = speed / yaw_rate_imu
            radius_error = abs(abs(radius_est) - self.test_radius) / self.test_radius
        else:
            radius_est = math.nan
            radius_error = math.nan

        if abs(yaw_rate_cmd) > EPS:
            yaw_rate_error = abs(yaw_rate_imu - yaw_rate_cmd) / abs(yaw_rate_cmd)
        else:
            yaw_rate_error = math.nan

        return GripMetrics(
            time_sec=now,
            state=self.state.value,
            direction=direction,
            speed_target=target_speed,
            radius_cmd=self.test_radius,
            steering_cmd=steering,
            speed_odom=speed,
            yaw_rate_imu=yaw_rate_imu,
            yaw_rate_cmd=yaw_rate_cmd,
            yaw_rate_odom=yaw_rate_odom,
            yaw_rate_error=yaw_rate_error,
            ay_imu=ay_imu,
            ay_yaw=ay_yaw,
            ay_cmd=ay_cmd,
            mu_y=mu_y,
            fy_n=fy_n,
            radius_est=radius_est,
            radius_error=radius_error,
            slip_flag=slip_flag,
            slip_reason=slip_reason,
            vesc_current_motor=self._vesc_field("state.current_motor"),
            vesc_current_input=self._vesc_field("state.current_input"),
            vesc_avg_iq=self._vesc_field("state.avg_iq"),
            vesc_duty_cycle=self._vesc_field("state.duty_cycle"),
            vesc_voltage_input=self._vesc_field("state.voltage_input"),
        )

    def _vesc_field(self, path: str) -> float:
        if self.last_vesc is None:
            return math.nan
        return _nested_float(self.last_vesc, path)

    def _is_sample_stable(self, metrics: GripMetrics) -> bool:
        if abs(metrics.speed_odom - metrics.speed_target) > self.speed_tolerance:
            return False
        if abs(metrics.speed_odom) < self.slip_min_speed:
            return True
        if math.isfinite(metrics.yaw_rate_error):
            if metrics.yaw_rate_error > self.yaw_rate_error_threshold:
                return False
        if math.isfinite(metrics.radius_error):
            if metrics.radius_error > self.radius_error_threshold:
                return False
        return True

    def _update_slip_detector(
        self, now: float, metrics: GripMetrics
    ) -> Optional[str]:
        violations: List[str] = []
        if metrics.mu_y > self.mu_abort:
            violations.append("mu_y {:.3f} > {:.3f}".format(metrics.mu_y, self.mu_abort))
        if abs(metrics.yaw_rate_imu) > self.max_abs_yaw_rate:
            violations.append(
                "|yaw_rate_imu| {:.3f} > {:.3f}".format(
                    abs(metrics.yaw_rate_imu), self.max_abs_yaw_rate
                )
            )
        if (
            self.use_raw_imu_ay_for_abort
            and abs(metrics.ay_imu) > self.raw_imu_ay_abort
        ):
            violations.append(
                "|ay_imu| {:.3f} > {:.3f}".format(
                    abs(metrics.ay_imu), self.raw_imu_ay_abort
                )
            )
        if abs(metrics.speed_odom) >= self.slip_min_speed:
            if (
                math.isfinite(metrics.yaw_rate_error)
                and metrics.yaw_rate_error > self.yaw_rate_error_threshold
            ):
                violations.append(
                    "yaw rate error {:.2f} > {:.2f}".format(
                        metrics.yaw_rate_error, self.yaw_rate_error_threshold
                    )
                )
            if (
                math.isfinite(metrics.radius_error)
                and metrics.radius_error > self.radius_error_threshold
            ):
                violations.append(
                    "radius error {:.2f} > {:.2f}".format(
                        metrics.radius_error, self.radius_error_threshold
                    )
                )

        if violations:
            reason = "; ".join(violations)
            if self.slip_since is None:
                self.slip_since = now
                self.slip_reason = reason
            elif now - self.slip_since >= self.slip_confirm_sec:
                return self.slip_reason
        else:
            self.slip_since = None
            self.slip_reason = ""
        return None

    def _plateau_limit_reason(self, current: StageResult) -> str:
        if not self.enable_plateau_detection or not math.isfinite(current.mu_y_mean):
            return ""
        same_direction = [
            result
            for result in self.results[:-1]
            if result.direction == current.direction and not result.slip_flag
        ]
        if not same_direction:
            return ""
        previous = same_direction[-1]
        if previous.mu_y_mean < self.plateau_min_mu:
            return ""
        if current.mu_y_mean + self.plateau_mu_margin < previous.mu_y_mean:
            return (
                "lateral acceleration plateau: mu {:.3f} after previous {:.3f}"
                .format(current.mu_y_mean, previous.mu_y_mean)
            )
        return ""

    def _summarize_current_stage(
        self, slip_flag: bool, slip_reason: str
    ) -> StageResult:
        target = self._target_speed()
        direction, sign = self._current_direction()
        steering = self._steering_cmd(sign)
        samples = self.current_samples
        speeds = [s.speed_odom for s in samples]
        yaw_imu = [s.yaw_rate_imu for s in samples]
        yaw_odom = [s.yaw_rate_odom for s in samples]
        ay_yaw = [s.ay_yaw for s in samples]
        ay_imu = [s.ay_imu for s in samples]
        ay_cmd = [s.ay_cmd for s in samples]
        mu_y = [s.mu_y for s in samples]
        fy = [s.fy_n for s in samples]
        radius_est_abs = [abs(s.radius_est) for s in samples]
        radius_error = [s.radius_error for s in samples]
        min_samples = max(1, int(0.5 * self.hold_time_sec * self.command_frequency))

        return StageResult(
            direction=direction,
            radius_cmd=self.test_radius,
            speed_target=target,
            steering_cmd=steering,
            samples=len(samples),
            stable=len(samples) >= min_samples,
            slip_flag=slip_flag,
            slip_reason=slip_reason,
            speed_mean=_mean(speeds),
            speed_std=_std(speeds),
            yaw_rate_imu_mean=_mean(yaw_imu),
            yaw_rate_imu_std=_std(yaw_imu),
            yaw_rate_odom_mean=_mean(yaw_odom),
            yaw_rate_odom_std=_std(yaw_odom),
            ay_yaw_mean=_mean(ay_yaw),
            ay_yaw_std=_std(ay_yaw),
            ay_imu_mean=_mean(ay_imu),
            ay_imu_std=_std(ay_imu),
            ay_cmd_mean=_mean(ay_cmd),
            mu_y_mean=_mean(mu_y),
            mu_y_max=_max_abs(mu_y),
            fy_mean=_mean(fy),
            fy_max_abs=_max_abs(fy),
            radius_est_mean=_mean(radius_est_abs),
            radius_error_mean=_mean(radius_error),
        )

    def _publish_speed_command(self, speed: float, steering: float) -> None:
        if not self.armed:
            return
        speed = max(0.0, min(float(speed), self.max_speed))
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.jerk = 0.0
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        msg.drive.acceleration = 0.0
        self.cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        if not self.armed:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.jerk = 0.0
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        msg.drive.acceleration = 0.0
        self.cmd_pub.publish(msg)

    def _publish_repeated_stop(self) -> None:
        if not self.armed:
            return
        count = max(1, self.stop_publish_count)
        for _ in range(count):
            self._publish_stop()
            time.sleep(0.01)

    def _publish_status(self, now: float) -> None:
        if now - self.last_status_time < 0.2:
            return
        self.last_status_time = now

        target = math.nan
        direction = "none"
        if self.directions and self.direction_index < len(self.directions):
            direction = self.directions[self.direction_index][0]
        if self.speed_targets and self.speed_index < len(self.speed_targets):
            target = self.speed_targets[self.speed_index]

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.status_frame
        marker.ns = "lateral_grip_calib"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.7
        marker.color.a = 1.0
        marker.text = (
            "lateral_grip_calib\n"
            "state: {}\n"
            "dir: {} target: {:.2f} m/s\n"
            "v: {:.2f} m/s mu: {:.3f}\n"
            "{}"
        ).format(
            self.state.value,
            direction,
            target,
            self._odom_speed(),
            self._latest_mu_estimate(),
            self.final_reason or self.slip_reason,
        )
        self.status_pub.publish(marker)

    def _latest_mu_estimate(self) -> float:
        if self.last_imu is None:
            return math.nan
        yaw_rate = self.imu_yaw_axis_sign * (
            float(self.last_imu.angular_velocity.z) - self.yaw_bias
        )
        return abs(self._odom_speed() * yaw_rate) / G_MPS2

    def _finish(self, reason: str, write_outputs: bool = True) -> None:
        self.final_reason = reason
        self.state = CalibState.DONE
        self.finished = True
        self._publish_repeated_stop()
        if write_outputs:
            self._write_outputs(reason, aborted=False)
        self.get_logger().info("lateral grip calibration finished: " + reason)

    def _abort(self, reason: str) -> None:
        self.final_reason = reason
        self.state = CalibState.ABORT
        self.finished = True
        self._publish_repeated_stop()
        self._write_outputs(reason, aborted=True)
        self.get_logger().error("lateral grip calibration aborted: " + reason)

    def _write_outputs(self, reason: str, aborted: bool) -> None:
        if self.outputs_written:
            return
        self.outputs_written = True
        paths = [self.samples_path, self.results_path, self.summary_path]
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)

        with self.samples_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=GripMetrics.fieldnames())
            writer.writeheader()
            for sample in self.all_samples:
                writer.writerow(asdict(sample))

        with self.results_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=StageResult.fieldnames())
            writer.writeheader()
            for result in self.results:
                writer.writerow(asdict(result))

        self.summary_path.write_text(self._summary_yaml(reason, aborted))
        self.get_logger().info(
            "wrote lateral grip outputs: {}, {}, {}".format(
                self.samples_path, self.results_path, self.summary_path
            )
        )

    def _summary_yaml(self, reason: str, aborted: bool) -> str:
        left_mu = self._direction_mu("left")
        right_mu = self._direction_mu("right")
        valid_mu = [value for value in [left_mu, right_mu] if math.isfinite(value)]
        if valid_mu:
            mu_safe = min(valid_mu) * self.mu_safe_factor
            max_ay_safe = mu_safe * G_MPS2
        else:
            mu_safe = math.nan
            max_ay_safe = math.nan

        lines = [
            "# Generated by lateral_grip_calib.py",
            "aborted: {}".format(str(aborted).lower()),
            "reason: {!r}".format(reason),
            "vehicle_mass: {:.6g}".format(self.vehicle_mass),
            "test_radius: {:.6g}".format(self.test_radius),
            "mu_left: {}".format(self._yaml_float(left_mu)),
            "mu_right: {}".format(self._yaml_float(right_mu)),
            "mu_safe: {}".format(self._yaml_float(mu_safe)),
            "max_lateral_accel_safe: {}".format(self._yaml_float(max_ay_safe)),
            "recommended_curve_speed_cap:",
            "  formula: 'sqrt(max_lateral_accel_safe * radius_m)'",
            "  note: 'Apply additional controller margin before racing.'",
        ]
        return "\n".join(lines) + "\n"

    def _direction_mu(self, direction: str) -> float:
        values = [
            result.mu_y_max
            for result in self.results
            if result.direction == direction and result.samples > 0
        ]
        return _max_abs(values)

    def _yaml_float(self, value: float) -> str:
        if not math.isfinite(value):
            return "null"
        return "{:.6g}".format(value)

    def _log_periodic(
        self, key: str, period: float, level: str, message: str
    ) -> None:
        now = self._now()
        last = self.last_log_times.get(key, -math.inf)
        if now - last < period:
            return
        self.last_log_times[key] = now
        logger = self.get_logger()
        if level == "warn":
            logger.warn(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def shutdown_safely(self) -> None:
        if not self.finished:
            self._abort("shutdown")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LateralGripCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().warn("keyboard interrupt")
            node.shutdown_safely()
    finally:
        node.destroy_node()
        # ROS launch may already have shut down the shared context on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
