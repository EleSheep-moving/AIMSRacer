import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('aims_mpcc_sim'), 'config', 'numerical.yaml'])
    vesc = PathJoinSubstitution([FindPackageShare('aims_mpcc_sim'), 'config', 'vesc_sim.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('path_directory'),
        DeclareLaunchArgument('vehicle_config', default_value=config),
        DeclareLaunchArgument('log_directory', default_value=''),
        DeclareLaunchArgument('initial_x', default_value='2.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value=str(math.pi / 2.0)),
        Node(package='aims_mpcc_sim', executable='numerical_plant', output='screen', parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }]),
        Node(package='ackermann_mux', executable='joystick_control_v2_ch3_ch1.py', output='screen'),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node', output='screen', parameters=[vesc]),
        Node(package='aims_mpcc', executable='mpcc_node', output='screen', parameters=[{
            'path_directory': LaunchConfiguration('path_directory'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'output_mode': 'drive',
            'simulation': True,
            'log_directory': LaunchConfiguration('log_directory'),
        }]),
    ])
