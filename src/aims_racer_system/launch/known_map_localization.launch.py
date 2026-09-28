"""Add one verified PGO-map localizer to an existing V2/V3 driving launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('aims_racer_system'),
                          'params', 'fastlio_localizer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='saved PGO map.pcd, absolute path'),
        DeclareLaunchArgument('localizer_config', default_value=config),
        Node(package='localizer', executable='localizer_node', namespace='localizer',
             name='localizer_node', output='screen',
             parameters=[{'config_path': LaunchConfiguration('localizer_config')}],
             remappings=[('/tf', '/localizer/raw_tf')]),
        Node(package='aims_racer_system', executable='map_tf_gate.py',
             name='map_tf_gate', output='screen',
             parameters=[{'map_file': LaunchConfiguration('map_file')}]),
    ])
