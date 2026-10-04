"""Wheel/gyro local odometry with scan-end deskew and optional NDT map alignment."""
import os
from pathlib import Path
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    map_file = Path(get('map_file'))
    if not map_file.is_file():
        raise ValueError(f'map_file must be an existing PCD: {map_file}')
    geometry = yaml.safe_load(Path(get('geometry_config')).read_text())
    quaternion = Rotation.from_euler('xyz', geometry['livox_rpy']).as_quat().tolist()
    sim = get('use_sim_time').lower() == 'true'
    package = get_package_share_directory('aims_racer_system')
    common = {'use_sim_time': sim}
    nodes = [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(package,'launch','rear_axle_frames.launch.py')),
        launch_arguments={'geometry_config':get('geometry_config'),'enable_lio':'false',
                          'use_sim_time':get('use_sim_time'),'imu_topic':get('imu_topic')}.items()),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[get('ekf_config'),common,{'odom0':get('wheel_topic')}], output='screen'),
        Node(package='aims_racer_system',executable='livox_ekf_deskew',
             parameters=[common,{'livox_translation':geometry['livox_translation'],'livox_quaternion':quaternion}],
             remappings=[('/livox/lidar',get('lidar_topic'))],output='screen')]
    if get('enable_ndt').lower() == 'true':
        nodes.extend([
            Node(package='lidar_localization_ros2',executable='lidar_localization_node',name='lidar_localization',
                 parameters=[get('ndt_config'),common,{'map_path':str(map_file)}],
                 remappings=[('cloud','/localization/deskewed_cloud'),('pcl_pose','/localization/ndt_pose'),
                             ('alignment_status','/localization/ndt_status'),('odom_bridge_pose','/localization/odom_bridge_pose')],output='screen'),
            Node(package='aims_racer_system',executable='activate_ndt.py',parameters=[common],output='screen'),
            Node(package='aims_racer_system',executable='localization_monitor.py',
                 parameters=[common,{'map_file':str(map_file),'livox_translation':geometry['livox_translation'],
                                     'livox_quaternion':quaternion,'imu_topic':get('imu_topic'),
                                     'wheel_topic':get('wheel_topic')}],output='screen')])
    return nodes


def generate_launch_description():
    params=os.path.join(get_package_share_directory('aims_racer_system'),'params')
    args=[DeclareLaunchArgument('map_file',description='Required immutable PCD map'),
          DeclareLaunchArgument('use_sim_time',default_value='false'),
          DeclareLaunchArgument('enable_ndt',default_value='true'),
          DeclareLaunchArgument('lidar_topic',default_value='/livox/lidar'),
          DeclareLaunchArgument('imu_topic',default_value='/livox/imu'),
          DeclareLaunchArgument('wheel_topic',default_value='/rear_axle/wheel_odom')]
    for name,filename in [('geometry_config','rear_axle_geometry.yaml'),('ekf_config','ekf_wheel_imu.yaml'),('ndt_config','ndt_wheel_imu.yaml')]:
        args.append(DeclareLaunchArgument(name,default_value=os.path.join(params,filename)))
    return LaunchDescription(args+[OpaqueFunction(function=build_nodes)])
