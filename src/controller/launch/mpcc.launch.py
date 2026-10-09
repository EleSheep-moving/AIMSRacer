import json
import math
from pathlib import Path
from launch import Substitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


class RuntimeProfile(Substitution):
    """Resolve timing defaults against the selected offline native artifact."""
    def __init__(self, key):
        super().__init__()
        self.key=key

    def perform(self, context):
        values=context.launch_configurations
        native=values.get('implementation','legacy')=='acados_cpp'
        horizon=10;dt=.1
        if native:
            directory=values.get('artifact_directory','')
            if not directory:raise ValueError('acados_cpp requires artifact_directory')
            manifest=json.loads((Path(directory)/'manifest.json').read_text())
            horizon=int(manifest['horizon']);dt=float(manifest['dt'])
            if horizon<=0 or not math.isfinite(dt) or dt<=0:raise ValueError('invalid artifact horizon/dt')
        explicit=values.get('horizon','')
        if explicit:
            requested=int(explicit)
            if native and requested!=horizon:raise ValueError('horizon must match the prepared native artifact')
            horizon=requested
        if self.key=='horizon':return str(horizon)
        defaults={'plan_ttl':.8*horizon*dt,'solver_timeout':.05 if native else .25}
        return values.get(self.key,'') or str(defaults[self.key])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_directory'),
        DeclareLaunchArgument('vehicle_config',default_value=PathJoinSubstitution([FindPackageShare('aims_mpcc'),'config','vehicle.yaml'])),
        DeclareLaunchArgument('simulation',default_value='false'),
        DeclareLaunchArgument('odom_topic',default_value='/odometry/filtered'),
        DeclareLaunchArgument('log_directory',default_value=''),
        DeclareLaunchArgument('backend',default_value='ipopt',description='Experimental alternatives: acados or qp'),
        DeclareLaunchArgument('artifact_directory',default_value='',description='Prepared acados artifact cache'),
        DeclareLaunchArgument('implementation',default_value='legacy',choices=['legacy','acados_cpp'],description='Select exactly one controller runtime'),
        DeclareLaunchArgument('cpp_solve_frequency',default_value='20.0',description='C++ optimization requests per second; publication remains 50 Hz'),
        DeclareLaunchArgument('shadow',default_value='true',description='Isolate C++ command/status/service outputs under /mpcc_rt_shadow'),
        DeclareLaunchArgument('horizon',default_value='',description='Legacy defaults to 10; native derives horizon from the prepared artifact'),
        DeclareLaunchArgument('solve_frequency',default_value='5.0',description='Optimization requests per second; command output stays at 50 Hz'),
        DeclareLaunchArgument('plan_ttl',default_value='',description='Default 0.8 * artifact horizon * dt for native; 0.8 * horizon * 0.1 for legacy'),
        DeclareLaunchArgument('solver_timeout',default_value='',description='Submission-to-delivery budget; native default 0.05 s, legacy default 0.25 s'),
        DeclareLaunchArgument('handover_delay',default_value='0.02',description='Forecast lead independent of solve frequency; late replies are realigned before activation'),
        Node(package='aims_mpcc',executable='mpcc_node',output='screen',condition=IfCondition(PythonExpression(["'",LaunchConfiguration('implementation'),"' == 'legacy'"])),parameters=[{
            **{key:LaunchConfiguration(key) for key in ('path_directory','vehicle_config','simulation','odom_topic','log_directory','backend','artifact_directory')},
            'horizon':ParameterValue(RuntimeProfile('horizon'),value_type=int),
            'solve_frequency':ParameterValue(LaunchConfiguration('solve_frequency'),value_type=float),
            'plan_ttl':ParameterValue(RuntimeProfile('plan_ttl'),value_type=float),
            'solver_timeout':ParameterValue(RuntimeProfile('solver_timeout'),value_type=float),
            'handover_delay':ParameterValue(LaunchConfiguration('handover_delay'),value_type=float),
        }]),
        Node(package='aims_mpcc_rt',executable='mpcc_rt_node',output='screen',
             prefix=['/usr/bin/python3 ',PathJoinSubstitution([FindPackageShare('aims_mpcc_rt'),'tools','runtime_supervisor.py']),
                     ' --status-topic ',PythonExpression(["'/mpcc_rt_shadow/mpcc/status' if '",LaunchConfiguration('shadow'),"'.lower() == 'true' else '/mpcc/status'"]),' -- '],
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('implementation'),"' == 'acados_cpp'"])),parameters=[{
            'artifact_directory':LaunchConfiguration('artifact_directory'),
            'path_directory':LaunchConfiguration('path_directory'),
            'vehicle_config':LaunchConfiguration('vehicle_config'),
            'odom_topic':LaunchConfiguration('odom_topic'),
            'log_directory':LaunchConfiguration('log_directory'),
            'simulation':ParameterValue(LaunchConfiguration('simulation'),value_type=bool),
            'shadow':ParameterValue(LaunchConfiguration('shadow'),value_type=bool),
            'solve_frequency':ParameterValue(LaunchConfiguration('cpp_solve_frequency'),value_type=float),
            'plan_ttl':ParameterValue(RuntimeProfile('plan_ttl'),value_type=float),
            'solver_timeout':ParameterValue(RuntimeProfile('solver_timeout'),value_type=float),
            'horizon':ParameterValue(RuntimeProfile('horizon'),value_type=int),
            'handover_delay':ParameterValue(LaunchConfiguration('handover_delay'),value_type=float),
        }]),
    ])
