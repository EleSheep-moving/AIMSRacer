"""Replay only: FAST-LIO2 and rear-axle EKF with optional known-map NDT."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    share=get_package_share_directory('aims_racer_system')
    params=os.path.join(share,'params')
    return LaunchDescription([
        DeclareLaunchArgument('map_file',default_value=''),
        DeclareLaunchArgument('known_map',default_value='true'),
        DeclareLaunchArgument('timing_trace_path',default_value=''),
        SetParameter(name='use_sim_time',value=True),
        Node(package='fastlio2',executable='lio_node',namespace='fastlio2',name='lio_node',
             parameters=[{'config_path':os.path.join(params,'fastlio_rear.yaml'),
                          'timing_trace_path':LaunchConfiguration('timing_trace_path')}],
             remappings=[('/tf','/fastlio2/tf')],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','rear_axle_frames.launch.py'))),
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',
             parameters=[os.path.join(params,'ekf_rear.yaml')],output='screen'),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share,'launch','known_map_localization.launch.py')),
            launch_arguments={'map_file':LaunchConfiguration('map_file'),'use_sim_time':'true'}.items())],
            condition=IfCondition(LaunchConfiguration('known_map'))),
    ])
