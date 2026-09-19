from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output',default_value='/results/nominal'),
        Node(package='aims_mpcc',executable='closed_loop_test',output='screen',
             arguments=['--output',LaunchConfiguration('output')]),
    ])
