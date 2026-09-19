from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_directory'),
        DeclareLaunchArgument('vehicle_config',default_value=PathJoinSubstitution([FindPackageShare('aims_mpcc'),'config','vehicle.yaml'])),
        DeclareLaunchArgument('output_mode',default_value='shadow'),
        DeclareLaunchArgument('simulation',default_value='false'),
        DeclareLaunchArgument('odom_topic',default_value='/odometry/filtered'),
        DeclareLaunchArgument('log_directory',default_value=''),
        Node(package='aims_mpcc',executable='mpcc_node',output='screen',parameters=[{
            key:LaunchConfiguration(key) for key in ('path_directory','vehicle_config','output_mode','simulation','odom_topic','log_directory')
        }]),
    ])
