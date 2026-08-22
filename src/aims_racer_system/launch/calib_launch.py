#!/usr/bin/env python3
"""
Launch file for longitudinal calibration.

Starts longitudinal_calib.py, which supports Pure Pursuit auto-loop
Stage A/B/C calibration plus RC-intervention Stage A/B/C workflows.

Usage:
  ros2 launch aims_racer_system calib_launch.py
  
Optional arguments:
  workflow:=pp_speed_hold         # pp_speed_hold, pp_accel_interval, pp_decel_current
                                  # speed_hold, accel_interval, decel_current
  max_steering_angle:=0.4751      # atan(0.36 / 0.70), radians
  track_radius:=2.0               # Racetrack semicircle radius (m)
  track_straight_length:=6.0      # Racetrack straight length (m)
  command_frequency:=50           # Control loop frequency (Hz)
  odom_topic:=/odometry/filtered  # Odometry topic
  vesc_topic:=/sensors/core       # VESC telemetry topic
  cmd_topic:=/calib/ackermann_cmd # Calibration command output
  
Example:
  ros2 launch aims_racer_system calib_launch.py workflow:=pp_accel_interval
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def _float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    """Generate launch description for the unified calibration node."""
    
    # Declare launch arguments
    workflow_arg = DeclareLaunchArgument(
        'workflow',
        default_value='pp_speed_hold',
        description='Workflow: pp_speed_hold, pp_accel_interval, pp_decel_current, speed_hold, accel_interval, or decel_current'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.36',
        description='Wheelbase of the vehicle (meters)'
    )

    max_steering_angle_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.4751',
        description='Maximum steering angle from the measured 0.70 m turn radius (radians)'
    )
    
    lookahead_gain_arg = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='1.5',
        description='Pure Pursuit lookahead gain (ld = k*v + min_lookahead)'
    )
    
    track_radius_arg = DeclareLaunchArgument(
        'track_radius',
        default_value='2.0',
        description='Racetrack semicircle turn radius (meters)'
    )
    
    track_straight_length_arg = DeclareLaunchArgument(
        'track_straight_length',
        default_value='6.0',
        description='Racetrack straight section length (meters)'
    )
    
    track_points_per_straight_arg = DeclareLaunchArgument(
        'track_points_per_straight',
        default_value='150',
        description='Number of points per straight section'
    )
    
    track_points_per_semicircle_arg = DeclareLaunchArgument(
        'track_points_per_semicircle',
        default_value='100',
        description='Number of points per semicircular turn'
    )
    
    command_frequency_arg = DeclareLaunchArgument(
        'command_frequency',
        default_value='50',
        description='Control loop frequency (Hz)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Odometry input topic'
    )

    vesc_topic_arg = DeclareLaunchArgument(
        'vesc_topic',
        default_value='/sensors/core',
        description='VESC telemetry topic'
    )

    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/calib/ackermann_cmd',
        description='Calibration command output topic'
    )
    
    # Unified longitudinal calibration node
    calib_node = Node(
        package='aims_racer_system',
        executable='longitudinal_calib.py',
        name='longitudinal_calib',
        output='screen',
        parameters=[
            {
                'workflow': LaunchConfiguration('workflow'),
                'wheelbase': _float_param('wheelbase'),
                'max_steering_angle': _float_param('max_steering_angle'),
                'lookahead_gain': _float_param('lookahead_gain'),
                'track_radius': _float_param('track_radius'),
                'track_straight_length': _float_param('track_straight_length'),
                'track_points_per_straight': _int_param('track_points_per_straight'),
                'track_points_per_semicircle': _int_param('track_points_per_semicircle'),
                'command_frequency': _float_param('command_frequency'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'vesc_topic': LaunchConfiguration('vesc_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
            }
        ],
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(workflow_arg)
    ld.add_action(wheelbase_arg)
    ld.add_action(max_steering_angle_arg)
    ld.add_action(lookahead_gain_arg)
    ld.add_action(track_radius_arg)
    ld.add_action(track_straight_length_arg)
    ld.add_action(track_points_per_straight_arg)
    ld.add_action(track_points_per_semicircle_arg)
    ld.add_action(command_frequency_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(vesc_topic_arg)
    ld.add_action(cmd_topic_arg)
    
    # Add info messages
    ld.add_action(LogInfo(msg=['Starting Unified Longitudinal Calibration']))
    ld.add_action(LogInfo(msg=['Workflow: ', LaunchConfiguration('workflow')]))
    ld.add_action(LogInfo(msg=['Odometry Topic: ', LaunchConfiguration('odom_topic')]))
    ld.add_action(LogInfo(msg=['VESC Topic: ', LaunchConfiguration('vesc_topic')]))
    ld.add_action(LogInfo(msg=['Command Topic: ', LaunchConfiguration('cmd_topic')]))
    ld.add_action(LogInfo(msg=['Racetrack Radius: ', LaunchConfiguration('track_radius'), ' m']))
    ld.add_action(LogInfo(msg=['Straight Length: ', LaunchConfiguration('track_straight_length'), ' m']))
    
    # Add node
    ld.add_action(calib_node)
    
    return ld
