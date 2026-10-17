"""Independent observer only; uses the shared external Livox mounting geometry."""
from pathlib import Path
import math
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_node(context):
    geometry_path = LaunchConfiguration('geometry_config').perform(context)
    with open(geometry_path) as stream:
        geometry = yaml.safe_load(stream)
    for key in ('livox_translation', 'livox_rpy'):
        values = geometry.get(key)
        if not isinstance(values, list) or len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f'{geometry_path}: {key} requires three finite values')
    quaternion = Rotation.from_euler('xyz', geometry['livox_rpy']).as_quat().tolist()
    sim = LaunchConfiguration('use_sim_time').perform(context).lower()
    if sim not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    return [Node(package='aims_racer_system', executable='body_acceleration',
        name='body_acceleration', output='screen',
        parameters=[LaunchConfiguration('config').perform(context),
                    {'livox_translation': geometry['livox_translation'],
                     'livox_quaternion': quaternion, 'use_sim_time': sim == 'true'}])]


def generate_launch_description():
    params = Path(get_package_share_directory('aims_racer_system')) / 'params'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('geometry_config', default_value=str(params / 'rear_axle_geometry.yaml')),
        DeclareLaunchArgument('config', default_value=str(params / 'body_acceleration.yaml')),
        OpaqueFunction(function=build_node),
    ])
