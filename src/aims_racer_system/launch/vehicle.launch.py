"""Field-tested Livox/FAST-LIO/rear-axle EKF and RC/VESC vehicle graph."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import flag, include


def build_vehicle(context):
    cfg=context.launch_configurations
    share=Path(get_package_share_directory('aims_racer_system'))
    mapping=flag(context,'mapping')
    selector={'channel_profile':'steering_ch1_throttle_ch3_aux_ch5_to_ch10',
        'rc_timeout_sec':.2,'nav_timeout_sec':.2,'calib_timeout_sec':.2,
        'limit_min_value':172,'limit_max_value':1810,'channel_min_range':172,
        'channel_mid':992,'channel_max_range':1810,'switch_mid_value':992,
        'speed_limit_min_speed':2.,'speed_limit_max_speed':12.,
        'current_limit_min_current':3.,'current_limit_max_current':100.,
        'steering_limit':.4751,'steering_reverse':True,'steering_channel_mid':992,
        'channel_deadzone':50,'direction_reverse':False}
    livox={'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.,
        'output_data_type':0,'frame_id':'livox_frame','lvx_file_path':'/home/livox/livox_test.lvx',
        'user_config_path':cfg['mid360_config'],'cmdline_input_bd_code':'livox0000000001'}
    nodes=[
        Node(package='crsf_receiver',executable='crsf_receiver_node',name='crsf_receiver_node',
            parameters=[{'device':'/dev/ttyELRS','baudrate':420000,'link_stats':True}],output='screen'),
        Node(package='ackermann_mux',executable='joystick_control_v2',name='joystick_control_v2_ch3_ch1',
            parameters=[selector],output='screen'),
        Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',name='livox_lidar_publisher',
            parameters=[livox],output='screen'),
        Node(package='vesc_driver',executable='vesc_driver_node',name='vesc_driver_node',
            parameters=[cfg['vesc_config']],output='screen'),
        Node(package='vesc_ackermann',executable='ackermann_to_vesc_node',name='ackermann_to_vesc_node',
            parameters=[cfg['vesc_config']]),
        Node(package='vesc_ackermann',executable='vesc_to_odom_node',name='vesc_to_odom_node',
            parameters=[cfg['vesc_config']],remappings=[('odom','/rear_axle/wheel_odom')]),
        Node(package='fastlio2',namespace='fastlio2',executable='lio_node',name='lio_node',
            parameters=[{'config_path':cfg['lio_config']}],output='screen',
            remappings=[('/tf','/fastlio2/tf'),('world_cloud','/fastlio2/visualization/world_cloud')]),
        include('aims_racer_system','rear_axle_frames.launch.py',
            {'lio_config':cfg['lio_config'],'publish_odom_tf':str(mapping).lower()}),
    ]
    if not mapping:nodes.append(Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',
        parameters=[str(share/'params/ekf_rear.yaml')],output='screen'))
    return nodes


def generate_launch_description():
    params=Path(get_package_share_directory('aims_racer_system'))/'params'
    return LaunchDescription([
        DeclareLaunchArgument('mapping',default_value='false',description='Mapping TF arrangement; normally selected by mapping.launch.py'),
        DeclareLaunchArgument('vesc_config',default_value=str(params/'vesc.yaml')),
        DeclareLaunchArgument('lio_config',default_value=str(params/'fastlio_rear.yaml')),
        DeclareLaunchArgument('mid360_config',default_value=str(params/'MID360_config.json')),
        OpaqueFunction(function=build_vehicle),
    ])
