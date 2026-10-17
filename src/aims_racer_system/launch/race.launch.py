"""One workspace: field vehicle + known-map NDT + native acados MPCC."""
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import include,flag,recording_actions


def build_race(context):
    cfg=context.launch_configurations
    share=Path(get_package_share_directory('aims_racer_system'))
    map_file=str(Path(cfg['map_file']).expanduser().resolve(strict=True))
    artifact=str(Path(cfg['artifact_directory']).expanduser().resolve(strict=True))
    flags={name:str(flag(context,name)).lower() for name in ('auto_start','repeat_laps')}
    acceleration=str(flag(context,'longitudinal_acceleration')).lower()
    pose=cfg.get('initial_pose','').strip()
    if pose:
        values=[float(item) for item in pose.replace(',',' ').split()]
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise ValueError('initial_pose requires six finite base_link map values: x y z roll pitch yaw')
    actions=recording_actions(context,share,mode='race')
    actions += [include('aims_racer_system','vehicle.launch.py',{'longitudinal_acceleration':acceleration,
            'body_acceleration':str(flag(context,'body_acceleration')).lower(),
            'planar_acceleration':str(flag(context,'planar_acceleration')).lower(),
            'vx_observer':str(flag(context,'vx_observer')).lower(),
            'yaw_rate':str(flag(context,'yaw_rate')).lower(),
            'state_observer':str(flag(context,'state_observer')).lower(),
            **{key:cfg.get(key,'') for key in ('state_observer_config','state_geometry_config','cg_geometry_config')}}),
        include('aims_racer_system','known_map_localization.launch.py',{'map_file':map_file}),
        include('aims_mpcc_rt','mpcc.launch.py',dict(artifact_directory=artifact,
            log_directory=cfg.get('log_directory',''),**flags))]
    if pose:
        actions.append(Node(package='aims_racer_system',executable='relocalize_known_map.py',
            arguments=[map_file,'--pose-frame','base_link',*[arg for k,v in zip(('x','y','z','roll','pitch','yaw'),values) for arg in ('--'+k,str(v))]],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file',description='Exact immutable PCD used for the saved reference'),
        DeclareLaunchArgument('artifact_directory',description='Prepared native bundle for this reference and vehicle'),
        DeclareLaunchArgument('initial_pose',default_value='',description='Optional base_link map x y z roll pitch yaw; angles in radians. Otherwise initialize through /initialpose.'),
        DeclareLaunchArgument('auto_start',default_value='false',description='One startup enable after existing prerequisites; false requires /mpcc/enable'),
        DeclareLaunchArgument('repeat_laps',default_value='false',description='False stops after one lap'),
        DeclareLaunchArgument('body_acceleration',default_value='false',description='Record independent raw IMU/LIO longitudinal acceleration'),
        DeclareLaunchArgument('planar_acceleration',default_value='false',description='Record independent matched-response planar acceleration and G'),
        DeclareLaunchArgument('vx_observer',default_value='false',description='Independent rear-axle Vx observation; no control consumers'),
        DeclareLaunchArgument('yaw_rate',default_value='false',description='Record independent gyro-led yaw rate and angular acceleration'),
        DeclareLaunchArgument('state_observer',default_value='false',description='Record both rear-axle and center-of-mass independent state estimates'),
        DeclareLaunchArgument('state_observer_config',default_value='',description='Optional independent state observer model configuration'),
        DeclareLaunchArgument('state_geometry_config',default_value='',description='Optional independent observer rear-to-IMU mounting geometry'),
        DeclareLaunchArgument('cg_geometry_config',default_value='',description='Optional physical IMU-to-CG geometry'),
        DeclareLaunchArgument('longitudinal_acceleration',default_value='false',description='Record independent longitudinal acceleration; does not change control'),
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('session_directory',default_value='',description='New recording directory; generated when omitted'),
        DeclareLaunchArgument('log_directory',default_value='',description='Optional native runtime.csv directory; record=true selects the recording session'),
        OpaqueFunction(function=build_race)])
