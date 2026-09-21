# MIT License

# Copyright (c) 2024 Zhihao Zhang

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    vesc_config = os.path.join(
        get_package_share_directory('aims_racer_system'),
        'params',
        'vesc.yaml'
    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')

    # Declare config paths for LIO and Localizer
    lio_config = os.path.join(
        get_package_share_directory('aims_racer_system'),
        'params',
        'fastlio_rear.yaml'
    )
    
    localizer_config = os.path.join(
        get_package_share_directory('aims_racer_system'),
        'params',
        'fastlio_localizer.yaml'
    )

    lio_la = DeclareLaunchArgument(
        'lio_config',
        default_value=lio_config,
        description='Configuration for LIO node'
    )

    localizer_la = DeclareLaunchArgument(
        'localizer_config',
        default_value=localizer_config,
        description='Configuration for Localizer node'
    )

    ld = LaunchDescription([vesc_la, lio_la, localizer_la])

    # CRSF Receiver (ELRS遥控器接收器)
    crsf_receiver_node = Node(
        package='crsf_receiver',
        executable='crsf_receiver_node',
        name='crsf_receiver_node',
        parameters=[
            {'device': '/dev/ttyELRS'},
            {'baudrate': 420000},
            {'link_stats': True}
        ],
        output='screen'
    )

    ################### Livox LiDAR Configuration ###################
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    user_config_path = os.path.join(
        get_package_share_directory("aims_racer_system"),
        'params', 
        'MID360_config.json'
    )

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    lidar_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    
    # 当前车辆：CH3 油门、CH1 转向、CH8 标定、CH10 限幅。
    joystick_control_v2_node = Node(
        package='ackermann_mux',
        executable='joystick_control_v2_ch3_ch1.py',
        name='joystick_control_v2_ch3_ch1',
        output='screen',
        parameters=[
            {'limit_min_value': 172},
            {'limit_max_value': 1810},
            {'channel_min_range': 172},
            {'channel_mid': 992},
            {'channel_max_range': 1810},
            {'switch_mid_value': 992},
            
            # 速度模式参数
            {'speed_limit_min_speed': 2.0},
            {'speed_limit_max_speed': 12.0},
            
            # 电流模式参数
            {'current_limit_min_current': 3.0},
            {'current_limit_max_current': 100.0},
            
            # 转向参数
            {'steering_limit': 0.4751},
            {'steering_reverse': True},
            {'steering_channel_mid': 992},
            {'channel_deadzone': 100},
            
            # 方向反转
            {'direction_reverse': False},
        ],
        remappings=[
            # joystick_control_v2输出 -> vesc输入
            ('/ackermann_cmd', '/ackermann_cmd'),
        ]
    )
    
    # VESC驱动节点
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[('odom', '/rear_axle/wheel_odom')]
    )
    
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    # EKF融合里程计
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory("aims_racer_system"),
            'params', 
            'ekf_rear.yaml'
        )],
    )
    
    # FAST-LIO2 (如果需要建图/定位)
    lio = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('lio_config')}],
        # Upstream FAST-LIO always broadcasts TF. Keep it off the global tree;
        # EKF owns global odom -> base_link in V2/V3.
        remappings=[('/tf', '/fastlio2/tf')]
    )

    # Localizer (重定位节点)
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('localizer_config')}]
    )

    # Rear-axle frame conversion and consistent sensor extrinsics.
    rear_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aims_racer_system'), 'launch', 'rear_axle_frames.launch.py')),
        launch_arguments={'lio_config': LaunchConfiguration('lio_config'),
                          'publish_odom_tf': 'false'}.items())

    # 添加所有节点到LaunchDescription
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(crsf_receiver_node)
    ld.add_action(joystick_control_v2_node)  # 🆕 使用v2版本，无需mux
    ld.add_action(lidar_driver)
    ld.add_action(robot_localization_node)
    ld.add_action(lio)

    ld.add_action(rear_frames)
    return ld
