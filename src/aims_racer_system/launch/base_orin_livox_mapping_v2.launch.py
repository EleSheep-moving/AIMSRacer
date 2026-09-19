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
    
    pgo_config = os.path.join(
        get_package_share_directory('aims_racer_system'),
        'params',
        'pgo_rear.yaml'
    )

    lio_la = DeclareLaunchArgument(
        'lio_config',
        default_value=lio_config,
        description='Configuration for LIO node'
    )

    pgo_la = DeclareLaunchArgument(
        'pgo_config',
        default_value=pgo_config,
        description='Configuration for PGO node'
    )

    ld = LaunchDescription([vesc_la, lio_la, pgo_la])

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
    frame_id      = 'laser'
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
            {'current_limit_max_current': 20.0},
            
            # 占空比模式参数
            {'duty_channel8_min_duty': 0.05},
            {'duty_channel8_max_duty': 0.3},
            
            # 转向参数
            {'steering_channel_8_min_value': -0.4751},
            {'steering_channel_8_max_value': 0.4751},
            
            # 控制模式选择 (SPEED, CURRENT, DUTY)
            {'esc_mode': 'SPEED'}
        ]
    )
    
    # VESC Ackermann Driver
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
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    # FAST-LIO2 Mapping Node
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('lio_config')}]
    )
    
    # PGO (Pose Graph Optimization) Node for Loop Closure
    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('pgo_config')}]
    )

    # Rear-axle frame conversion and consistent sensor extrinsics.
    rear_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aims_racer_system'), 'launch', 'rear_axle_frames.launch.py')),
        launch_arguments={'lio_config': LaunchConfiguration('lio_config'),
                          'publish_odom_tf': 'true'}.items())

    # Add all nodes to launch description
    ld.add_action(crsf_receiver_node)
    ld.add_action(lidar_driver)
    
    ld.add_action(joystick_control_v2_node)
    
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    
    # Note: robot_localization EKF is not needed for mapping
    # LIO provides high-rate odometry directly
    # If 3D terrain navigation is needed, consider adding EKF back
    
    ld.add_action(lio_node)
    ld.add_action(pgo_node)
    

    ld.add_action(rear_frames)
    return ld
