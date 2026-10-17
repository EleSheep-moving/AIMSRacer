"""Only the VRPN client; no vehicle or controller launch."""
import json
import math
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    config=json.loads(Path(LaunchConfiguration('config').perform(context)).read_text())['vrpn']
    if not config.get('server'):raise ValueError('configure vrpn.server')
    frequency=float(config.get('update_freq',200.));port=int(config.get('port',3883))
    if not math.isfinite(frequency) or frequency<=0 or not 0<port<65536:raise ValueError('invalid VRPN frequency/port')
    if config.get('use_vrpn_timestamps') is not True:raise ValueError('validation requires use_vrpn_timestamps=true; configure clock mapping separately')
    return [Node(package='vrpn_mocap',executable='client_node',namespace='vrpn_mocap',name='vrpn_mocap_client_node',output='screen',parameters=[dict(server=config['server'],port=port,frame_id='vicon_world',update_freq=frequency,refresh_freq=1.,multi_sensor=False,use_vrpn_timestamps=True,sensor_data_qos=True)])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config',description='Absolute selected site JSON configuration'),OpaqueFunction(function=build)])
