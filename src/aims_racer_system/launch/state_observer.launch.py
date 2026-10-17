"""Independent dual-reference state observers; no sensors or control authority."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation


def vector(values, name):
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(name + ' requires three finite numbers')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(name + ' requires three finite numbers')
    return result


def build_node(context):
    geometry_path = LaunchConfiguration('geometry_config').perform(context)
    cg_path = LaunchConfiguration('cg_geometry_config').perform(context)
    with open(geometry_path) as stream:
        geometry = yaml.safe_load(stream)
    with open(cg_path) as stream:
        measurements = yaml.safe_load(stream)
    translation = vector(geometry['livox_translation'], 'livox_translation')
    mounting = vector(geometry['livox_rpy'], 'livox_rpy')
    cg = measurements['imu_to_center_of_mass']
    if (cg.get('from_point') != 'mid360_internal_imu' or
            cg.get('to_point') != 'vehicle_center_of_mass' or
            cg.get('expressed_in') != 'vehicle_body_axes'):
        raise ValueError('CG geometry must describe the physical IMU-to-CG body vector')
    # Unknown height must never silently become zero. The default measurement
    # record explicitly distinguishes the user estimate from a calibrated value.
    offset = vector([cg.get(key+'_m') for key in 'xyz'], 'imu_to_cg')
    estimated = cg.get('z_estimated', True)
    if not isinstance(estimated, bool):
        raise ValueError('z_estimated must be a boolean')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower()
    if sim not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    return [Node(package='aims_racer_system', executable='state_observer', name='state_observer',
        output='screen', parameters=[LaunchConfiguration('config').perform(context),
            dict(livox_translation=translation,
                 livox_quaternion=Rotation.from_euler('xyz', mounting).as_quat().tolist(),
                 imu_to_cg=offset, cg_height_estimated=estimated,
                 geometry_calibrated=False, use_sim_time=sim == 'true')])]


def generate_launch_description():
    params = Path(get_package_share_directory('aims_racer_system'))/'params'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('geometry_config', default_value=str(params/'rear_axle_geometry.yaml')),
        DeclareLaunchArgument('cg_geometry_config', default_value=str(params/'vehicle_geometry_measurements.yaml')),
        DeclareLaunchArgument('config', default_value=str(params/'state_observer.yaml')),
        OpaqueFunction(function=build_node),
    ])
