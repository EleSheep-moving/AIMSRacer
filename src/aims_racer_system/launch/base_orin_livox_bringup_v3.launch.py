"""FAST-LIO2 vehicle bringup with ZED 2i neural-depth perception.

V3 deliberately reuses the proven V2 control, Livox, FAST-LIO2, and EKF
bringup.  The ZED is a perception sensor only: its positional tracking and
dynamic TF output are disabled so it cannot compete with the FAST-LIO2/EKF
localization chain.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    zed_camera_name = LaunchConfiguration('zed_camera_name')
    zed_serial_number = LaunchConfiguration('zed_serial_number')
    zed_params_file = LaunchConfiguration('zed_params_file')

    zed_publish_base_tf = LaunchConfiguration('zed_publish_base_tf')
    zed_tf_x = LaunchConfiguration('zed_tf_x')
    zed_tf_y = LaunchConfiguration('zed_tf_y')
    zed_tf_z = LaunchConfiguration('zed_tf_z')
    zed_tf_roll = LaunchConfiguration('zed_tf_roll')
    zed_tf_pitch = LaunchConfiguration('zed_tf_pitch')
    zed_tf_yaw = LaunchConfiguration('zed_tf_yaw')

    v2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aims_racer_system'),
                'launch',
                'base_orin_livox_bringup_v2.launch.py',
            ])
        )
    )

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            ])
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': zed_camera_name,
            'serial_number': zed_serial_number,
            'ros_params_override_path': zed_params_file,
            # FAST-LIO2/EKF own the vehicle localization and map/odom TF.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
            # Publish the ZED's internal static frame tree only.
            'publish_urdf': 'true',
            'node_log_type': 'screen',
        }.items(),
    )

    # The camera mounting transform must be measured.  It is intentionally
    # disabled by default rather than publishing an assumed transform.
    zed_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_zed2i_camera',
        condition=IfCondition(zed_publish_base_tf),
        arguments=[
            zed_tf_x,
            zed_tf_y,
            zed_tf_z,
            zed_tf_roll,
            zed_tf_pitch,
            zed_tf_yaw,
            'base_link',
            'zed2i_camera_link',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'zed_camera_name',
            default_value='zed2i',
            description='Namespace and frame-name prefix for the ZED 2i.',
        ),
        DeclareLaunchArgument(
            'zed_serial_number',
            default_value='0',
            description='ZED serial number; 0 selects the first detected ZED 2i.',
        ),
        DeclareLaunchArgument(
            'zed_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('aims_racer_system'),
                'params',
                'zed2i_racing.yaml',
            ]),
            description='ZED 2i racing/perception parameter overrides.',
        ),
        DeclareLaunchArgument(
            'zed_publish_base_tf',
            default_value='false',
            description='Publish measured base_link -> zed2i_camera_link static TF.',
        ),
        DeclareLaunchArgument('zed_tf_x', default_value='0.0'),
        DeclareLaunchArgument('zed_tf_y', default_value='0.0'),
        DeclareLaunchArgument('zed_tf_z', default_value='0.0'),
        DeclareLaunchArgument('zed_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('zed_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('zed_tf_yaw', default_value='0.0'),
        v2_bringup,
        zed_camera,
        zed_base_tf,
    ])
