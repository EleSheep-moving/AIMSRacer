"""Independent wheel/IMU acceleration estimate; starts no hardware or control."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = Path(get_package_share_directory('aims_racer_system')) / 'params/longitudinal_acceleration.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(default_config)),
        Node(package='aims_racer_system', executable='longitudinal_acceleration.py',
             name='longitudinal_acceleration', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
