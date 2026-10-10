"""NDT map anchoring add-on for the existing FAST-LIO2/EKF vehicle graph."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    def get(name):
        return LaunchConfiguration(name).perform(context)
    path = Path(get('map_file')).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValueError('map_file must be an existing immutable PCD')
    sim = get('use_sim_time').lower()
    if sim not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    common = {'use_sim_time': sim == 'true'}
    return [
        Node(package='lidar_localization_ros2', executable='lidar_localization_node', name='lidar_localization',
             parameters=[get('ndt_config'), common, {'map_path': str(path)}], output='screen',
             remappings=[('cloud', '/fastlio2/body_cloud'), ('anchor_status', '/localization/anchor_status'),
                         ('pcl_pose', '/localization/ndt_pose'), ('alignment_status', '/localization/ndt_status'),
                         ('odom_bridge_pose', '/localization/odom_bridge_pose'),
                         ('initial_map', '/localization/map_cloud')]),
        Node(package='aims_racer_system', executable='activate_ndt.py', name='activate_ndt',
             parameters=[common], output='screen'),
        Node(package='aims_racer_system', executable='localization_monitor', name='localization_monitor',
             parameters=[get('monitor_config'), common, {'map_file': str(path)}],
             additional_env={'OPENBLAS_NUM_THREADS': '1'}, output='screen'),
    ]


def generate_launch_description():
    params = os.path.join(get_package_share_directory('aims_racer_system'), 'params')
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='Required immutable PCD map, absolute path'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ndt_config', default_value=os.path.join(params, 'ndt_fastlio.yaml')),
        DeclareLaunchArgument('monitor_config', default_value=os.path.join(params, 'localization_monitor.yaml')),
        OpaqueFunction(function=build_nodes),
    ])
