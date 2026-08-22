#!/usr/bin/env python3
"""Launch fixed-radius lateral grip calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def _float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    """Generate launch description for the lateral grip calibration node."""

    args = [
        _arg("armed", "false", "If false, only checks topics and publishes no motion commands"),
        _arg("vehicle_mass", "4.5", "Vehicle mass in kg"),
        _arg("wheelbase", "0.36", "Vehicle wheelbase in m"),
        _arg("test_radius", "3.0", "Fixed circle radius in m"),
        _arg("speed_start", "0.5", "First target speed in m/s"),
        _arg("speed_end", "5.0", "Last target speed in m/s"),
        _arg("speed_step", "0.25", "Speed increment in m/s"),
        _arg("settle_time_sec", "2.0", "Stable settle time before sampling"),
        _arg("hold_time_sec", "5.0", "Sampling time per speed point"),
        _arg("stable_required_sec", "1.0", "Minimum continuous stable time"),
        _arg("speed_tolerance", "0.15", "Allowed speed error during settle/sample"),
        _arg("speed_ramp_rate", "0.8", "Commanded speed ramp rate in m/s^2"),
        _arg("direction_settle_sec", "3.0", "Stop time before switching direction"),
        _arg("command_frequency", "50.0", "Command loop frequency in Hz"),
        _arg("odom_topic", "/odom", "Odometry input topic"),
        _arg("imu_topic", "/livox/imu_ekf", "IMU input topic"),
        _arg("vesc_topic", "/sensors/core", "VESC telemetry topic"),
        _arg("cmd_topic", "/calib/ackermann_cmd", "Ackermann command output topic"),
        _arg("status_topic", "/calib/lateral_status_text", "RViz text marker topic"),
        _arg("require_vesc", "true", "Require VESC telemetry before motion"),
        _arg("directions", "left,right", "Comma-separated directions: left,right"),
        _arg("max_abs_steering", "0.4751", "Maximum allowed steering command in rad"),
        _arg("max_speed", "6.0", "Hard speed command limit in m/s"),
        _arg("sensor_timeout_sec", "0.5", "Sensor freshness timeout in seconds"),
        _arg("imu_bias_samples", "150", "Stationary IMU samples for bias estimation"),
        _arg("imu_lateral_axis_sign", "1.0", "Sign correction for IMU lateral accel"),
        _arg("imu_yaw_axis_sign", "1.0", "Sign correction for IMU yaw rate"),
        _arg("use_raw_imu_ay_for_abort", "false", "Use IMU ay as extra abort check"),
        _arg("mu_abort", "0.8", "Abort threshold for estimated lateral mu"),
        _arg("yaw_rate_error_threshold", "0.25", "Slip yaw-rate relative error threshold"),
        _arg("radius_error_threshold", "0.30", "Slip radius relative error threshold"),
        _arg("slip_confirm_sec", "0.30", "Required slip violation duration"),
        _arg("slip_min_speed", "0.8", "Minimum speed for yaw/radius slip checks"),
        _arg("output_dir", ".", "Directory for CSV/YAML outputs"),
        _arg("samples_path", "lateral_grip_samples.csv", "Per-sample CSV output path"),
        _arg("results_path", "lateral_grip_results.csv", "Per-stage CSV output path"),
        _arg("summary_path", "lateral_grip_summary.yaml", "Summary YAML output path"),
    ]

    node = Node(
        package="aims_racer_system",
        executable="lateral_grip_calib.py",
        name="lateral_grip_calib",
        output="screen",
        parameters=[
            {
                "armed": _bool_param("armed"),
                "vehicle_mass": _float_param("vehicle_mass"),
                "wheelbase": _float_param("wheelbase"),
                "test_radius": _float_param("test_radius"),
                "speed_start": _float_param("speed_start"),
                "speed_end": _float_param("speed_end"),
                "speed_step": _float_param("speed_step"),
                "settle_time_sec": _float_param("settle_time_sec"),
                "hold_time_sec": _float_param("hold_time_sec"),
                "stable_required_sec": _float_param("stable_required_sec"),
                "speed_tolerance": _float_param("speed_tolerance"),
                "speed_ramp_rate": _float_param("speed_ramp_rate"),
                "direction_settle_sec": _float_param("direction_settle_sec"),
                "command_frequency": _float_param("command_frequency"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "vesc_topic": LaunchConfiguration("vesc_topic"),
                "cmd_topic": LaunchConfiguration("cmd_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "require_vesc": _bool_param("require_vesc"),
                "directions": LaunchConfiguration("directions"),
                "max_abs_steering": _float_param("max_abs_steering"),
                "max_speed": _float_param("max_speed"),
                "sensor_timeout_sec": _float_param("sensor_timeout_sec"),
                "imu_bias_samples": _int_param("imu_bias_samples"),
                "imu_lateral_axis_sign": _float_param("imu_lateral_axis_sign"),
                "imu_yaw_axis_sign": _float_param("imu_yaw_axis_sign"),
                "use_raw_imu_ay_for_abort": _bool_param("use_raw_imu_ay_for_abort"),
                "mu_abort": _float_param("mu_abort"),
                "yaw_rate_error_threshold": _float_param("yaw_rate_error_threshold"),
                "radius_error_threshold": _float_param("radius_error_threshold"),
                "slip_confirm_sec": _float_param("slip_confirm_sec"),
                "slip_min_speed": _float_param("slip_min_speed"),
                "output_dir": LaunchConfiguration("output_dir"),
                "samples_path": LaunchConfiguration("samples_path"),
                "results_path": LaunchConfiguration("results_path"),
                "summary_path": LaunchConfiguration("summary_path"),
            }
        ],
    )

    ld = LaunchDescription()
    for action in args:
        ld.add_action(action)

    ld.add_action(LogInfo(msg=["Starting Lateral Grip Calibration"]))
    ld.add_action(LogInfo(msg=["armed: ", LaunchConfiguration("armed")]))
    ld.add_action(LogInfo(msg=["vehicle_mass: ", LaunchConfiguration("vehicle_mass"), " kg"]))
    ld.add_action(LogInfo(msg=["test_radius: ", LaunchConfiguration("test_radius"), " m"]))
    ld.add_action(LogInfo(msg=["speed_end: ", LaunchConfiguration("speed_end"), " m/s"]))
    ld.add_action(LogInfo(msg=["odom_topic: ", LaunchConfiguration("odom_topic")]))
    ld.add_action(LogInfo(msg=["imu_topic: ", LaunchConfiguration("imu_topic")]))
    ld.add_action(LogInfo(msg=["cmd_topic: ", LaunchConfiguration("cmd_topic")]))
    ld.add_action(node)
    return ld
