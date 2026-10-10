"""Manual driving and FAST-LIO/PGO mapping, optionally record raw data."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import include,recording_actions


def build_mapping(context):
    share=Path(get_package_share_directory('aims_racer_system'))
    return [*recording_actions(context,share,mode='mapping'),
        include('aims_racer_system','vehicle.launch.py',{'mapping':'true'}),
        Node(package='pgo',namespace='pgo',executable='pgo_node',name='pgo_node',
             parameters=[{'config_path':context.launch_configurations['pgo_config']}],output='screen')]


def generate_launch_description():
    share=Path(get_package_share_directory('aims_racer_system'))
    return LaunchDescription([
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('session_directory',default_value='',description='New recording directory; default dated directory under ~/aimsracer-data/sessions'),
        DeclareLaunchArgument('pgo_config',default_value=str(share/'params/pgo_rear.yaml')),
        OpaqueFunction(function=build_mapping)])
