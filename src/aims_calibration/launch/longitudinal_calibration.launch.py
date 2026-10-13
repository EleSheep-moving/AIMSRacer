"""Calibration node only; the system launch owns vehicle and localization."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('aims_calibration'))
    defaults=dict(config=str(share/'config/calibration.yaml'),reference_directory='',vehicle_config='',
                  pp_config='',log_directory=str(Path.home()/'aimsracer-data'/'calibration'),
                  stage='speed_hold',steering_source='pp',auto_start='false',require_acceleration='false')
    arguments=[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()]
    parameters={name:ParameterValue(LaunchConfiguration(name),value_type=bool if name in ('auto_start','require_acceleration') else str)
                for name in defaults if name!='config'}
    parameters['config_file']=ParameterValue(LaunchConfiguration('config'),value_type=str)
    return LaunchDescription(arguments+[Node(package='aims_calibration',executable='longitudinal_calibration',
        name='longitudinal_calibration',output='screen',parameters=[LaunchConfiguration('config'),parameters])])
