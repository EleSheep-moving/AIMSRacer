from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    truth = lambda name: value(name).lower() == 'true'
    return [Node(package='aims_pp',executable='pp_node',name='aims_pp',output='screen',parameters=[dict(
        reference_directory=value('reference_directory'),vehicle_config=value('vehicle_config'),
        pp_config=value('pp_config') or str(Path(get_package_share_directory('aims_pp'))/'config/pp.yaml'),
        log_directory=value('log_directory'),speed_limit=float(value('speed_limit')),
        repeat_laps=truth('repeat_laps'),auto_start=truth('auto_start'),use_sim_time=truth('use_sim_time'))])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference_directory',description='Directory containing canonical reference.json'),
        DeclareLaunchArgument('vehicle_config',description='Verified actual vehicle geometry and hard limits'),
        DeclareLaunchArgument('pp_config',default_value='',description='Enhanced PP tuning YAML; blank uses installed defaults'),
        DeclareLaunchArgument('log_directory',default_value=''),
        DeclareLaunchArgument('speed_limit',default_value='1.0'),
        DeclareLaunchArgument('repeat_laps',default_value='false'),
        DeclareLaunchArgument('auto_start',default_value='false',description='Must remain false; enable through /pp/enable'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),OpaqueFunction(function=_node)])
