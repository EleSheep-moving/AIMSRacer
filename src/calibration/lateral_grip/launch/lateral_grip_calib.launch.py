"""Starts an idle recorder/controller; never auto-enables a trial or launches hardware."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config = Path(get_package_share_directory('aims_lateral_grip')) / 'config/disabled.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='aims_lateral_grip', executable='lateral_grip_calib',
             name='lateral_grip_calib', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
