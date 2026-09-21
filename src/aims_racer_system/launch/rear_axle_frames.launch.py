"""Shared V2 Livox mounting transform and rear-axle data adapters."""
import os
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    geometry_path = LaunchConfiguration('geometry_config').perform(context)
    with open(geometry_path) as stream:
        geometry = yaml.safe_load(stream)
    with open(LaunchConfiguration('lio_config').perform(context)) as stream:
        lio = yaml.safe_load(stream)
    if lio['body_frame'] != 'livox_frame' or lio['world_frame'] != 'odom':
        raise ValueError('Rear-axle pipeline requires raw LIO frames odom / livox_frame')
    for key in ('livox_translation', 'livox_rpy'):
        values = geometry.get(key)
        if not isinstance(values, list) or len(values) != 3 or not np.isfinite(values).all():
            raise ValueError(f'Confirm {key} in {geometry_path}: three finite values required')
    if lio.get('esti_il', False):
        raise ValueError('Static rear-frame conversion requires esti_il: false')
    # One approximate external Livox origin for cloud, raw IMU and LIO output.
    # FAST-LIO's r_il/t_il remain internal and do not alter this mounting offset.
    translation = geometry['livox_translation']
    quaternion = Rotation.from_euler('xyz', geometry['livox_rpy']).as_quat().tolist()

    def static(name, child, t, q):
        args = ['--x', str(t[0]), '--y', str(t[1]), '--z', str(t[2]),
                '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                '--frame-id', 'base_link', '--child-frame-id', child]
        return Node(package='tf2_ros', executable='static_transform_publisher', name=name, arguments=args)

    mapping = LaunchConfiguration('publish_odom_tf').perform(context).lower() == 'true'
    return [
        static('rear_to_livox', 'livox_frame', translation, quaternion),
        static('rear_to_footprint', 'base_footprint', [0., 0., 0.], [0., 0., 0., 1.]),
        Node(package='aims_racer_system', executable='lio_to_rear_axle.py',
             parameters=[{'livox_translation': translation, 'livox_quaternion': quaternion,
                          'publish_tf': mapping}], output='screen'),
        Node(package='aims_racer_system', executable='imu_to_rear_axle.py',
             parameters=[{'livox_translation': translation, 'livox_quaternion': quaternion}], output='screen'),
    ]


def generate_launch_description():
    params = os.path.join(get_package_share_directory('aims_racer_system'), 'params')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_config', default_value=os.path.join(params, 'rear_axle_geometry.yaml')),
        DeclareLaunchArgument('lio_config', default_value=os.path.join(params, 'fastlio_rear.yaml')),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
