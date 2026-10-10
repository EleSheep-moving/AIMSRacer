"""Native-only MPCC; immutable bundle owns model, reference and timing mesh."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node


def build_controller(context):
    c=context.launch_configurations
    root=Path(c['artifact_directory']).expanduser().resolve(strict=True)
    def boolean(key):
        value=c[key].lower()
        if value not in ('true','false'):raise ValueError(key+' must be true or false')
        return value=='true'
    return [Node(package='aims_mpcc_rt',executable='mpcc_rt_node',name='aims_mpcc_rt',output='screen',parameters=[{
        'artifact_directory':str(root),'path_directory':str(root/'input_reference'),
        'vehicle_config':str(root/'input_config.yaml'),'log_directory':c['log_directory'],
        'odom_topic':c['odom_topic'],'solve_frequency':float(c['solve_frequency']),
        'solver_timeout':float(c['solver_timeout']),'handover_delay':float(c['handover_delay']),
        'simulation':boolean('simulation'),'repeat_laps':boolean('repeat_laps'),
        'auto_start':boolean('auto_start'),'auto_start_timeout':float(c['auto_start_timeout']),
    }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('artifact_directory'),
        DeclareLaunchArgument('auto_start',default_value='false'),
        DeclareLaunchArgument('auto_start_timeout',default_value='60.0'),
        DeclareLaunchArgument('repeat_laps',default_value='false'),
        DeclareLaunchArgument('log_directory',default_value=''),
        DeclareLaunchArgument('simulation',default_value='false'),
        DeclareLaunchArgument('odom_topic',default_value='/odometry/filtered'),
        DeclareLaunchArgument('solve_frequency',default_value='20.0'),
        DeclareLaunchArgument('solver_timeout',default_value='0.05'),
        DeclareLaunchArgument('handover_delay',default_value='0.02'),
        OpaqueFunction(function=build_controller)])
