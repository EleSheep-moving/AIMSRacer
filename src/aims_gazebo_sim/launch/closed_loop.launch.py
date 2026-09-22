"""Launch Gazebo Fortress plus the unmodified AIMSRacer MPCC ROS pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('aims_gazebo_sim')
    world = PathJoinSubstitution([share, 'worlds', 'aims_track.sdf'])
    vehicle = PathJoinSubstitution([share, 'worlds', 'aims_racer.sdf'])
    gazebo_config = PathJoinSubstitution([share, 'config', 'gazebo.yaml'])
    vesc = PathJoinSubstitution([share, 'config', 'vesc_sim.yaml'])
    bridge_config = PathJoinSubstitution([share, 'config', 'ros_gz_bridge.yaml'])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={'gz_args': PythonExpression(["'-r -s ", world, "'"])}.items(),
        condition=UnlessCondition(LaunchConfiguration('gazebo_gui')),
    )
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={'gz_args': PythonExpression(["'-r ", world, "'"])}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'aims_track', '-file', vehicle, '-name', 'aims_racer',
                   '-x', LaunchConfiguration('initial_x'), '-y', LaunchConfiguration('initial_y'),
                   '-z', '0.052', '-Y', LaunchConfiguration('initial_yaw')],
    )
    physical_chain = [
        Node(package='aims_gazebo_sim', executable='vesc_gazebo_bridge', output='screen', parameters=[{
            'wheelbase': 0.36, 'steer_limit': 0.4,
        }]),
        Node(package='ackermann_mux', executable='joystick_control_v2_ch3_ch1.py', output='screen'),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node', output='screen', parameters=[vesc]),
        Node(package='aims_mpcc', executable='mpcc_node', output='screen', parameters=[{
            'path_directory': LaunchConfiguration('path_directory'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'output_mode': 'drive', 'simulation': True,
            'log_directory': LaunchConfiguration('log_directory'),
        }]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('path_directory'),
        DeclareLaunchArgument('vehicle_config', default_value=gazebo_config),
        DeclareLaunchArgument('log_directory', default_value=''),
        DeclareLaunchArgument('initial_x', default_value='8.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='1.57079632679'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='false'),
        SetParameter(name='use_sim_time', value=True),
        gazebo_server,
        gazebo_gui,
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen', parameters=[{
            'config_file': bridge_config,
        }]),
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=physical_chain),
        TimerAction(period=5.0, actions=[Node(
            package='aims_gazebo_sim', executable='mpcc_enabler', output='screen',
            condition=IfCondition(LaunchConfiguration('autostart')),
        )]),
        Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'mpcc_gazebo.rviz'])]),
    ])
