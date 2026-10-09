from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument('horizon',default_value='10',description='Number of 0.1 s prediction intervals'),
        DeclareLaunchArgument('solve_frequency',default_value='5.0',description='Optimization requests per second; command output stays at 50 Hz'),
        DeclareLaunchArgument('plan_ttl',default_value=PythonExpression([
            LaunchConfiguration('horizon'), ' * 0.8 * 0.1']),
            description='Seconds from ORIGINAL EKF measurement, not receipt/takeover; defaults to horizon * 0.8 * 0.1 s'),
        DeclareLaunchArgument('solver_timeout',default_value='0.25',description='Seconds from request submission to parent reply; late results are discarded and the worker continues'),
        DeclareLaunchArgument('handover_delay',default_value='0.02',description='Forecast lead independent of solve frequency; late replies are realigned before activation'),
        Node(package='aims_mpcc',executable='mpcc_node',output='screen',condition=IfCondition(PythonExpression(["'",LaunchConfiguration('implementation'),"' == 'legacy'"])),parameters=[{
            **{key:LaunchConfiguration(key) for key in ('path_directory','vehicle_config','simulation','odom_topic','log_directory','backend','artifact_directory')},
            'horizon':ParameterValue(LaunchConfiguration('horizon'),value_type=int),
            'solve_frequency':ParameterValue(LaunchConfiguration('solve_frequency'),value_type=float),
            'plan_ttl':ParameterValue(LaunchConfiguration('plan_ttl'),value_type=float),
            'solver_timeout':ParameterValue(LaunchConfiguration('solver_timeout'),value_type=float),
            'handover_delay':ParameterValue(LaunchConfiguration('handover_delay'),value_type=float),
        }]),
        Node(package='aims_mpcc_rt',executable='mpcc_rt_node',output='screen',condition=IfCondition(PythonExpression(["'",LaunchConfiguration('implementation'),"' == 'acados_cpp'"])),parameters=[{
            'artifact_directory':LaunchConfiguration('artifact_directory'),
            'path_directory':LaunchConfiguration('path_directory'),
            'vehicle_config':LaunchConfiguration('vehicle_config'),
            'odom_topic':LaunchConfiguration('odom_topic'),
            'log_directory':LaunchConfiguration('log_directory'),
            'simulation':ParameterValue(LaunchConfiguration('simulation'),value_type=bool),
            'shadow':ParameterValue(LaunchConfiguration('shadow'),value_type=bool),
            'solve_frequency':ParameterValue(LaunchConfiguration('cpp_solve_frequency'),value_type=float),
            'plan_ttl':ParameterValue(LaunchConfiguration('plan_ttl'),value_type=float),
            'solver_timeout':0.05,
            'handover_delay':ParameterValue(LaunchConfiguration('handover_delay'),value_type=float),
        }]),
    ])
