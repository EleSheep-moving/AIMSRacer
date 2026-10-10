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
    pose=cfg.get('initial_pose','').strip()
    if pose:
        values=[float(item) for item in pose.replace(',',' ').split()]
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise ValueError('initial_pose requires six finite base_link map values: x y z roll pitch yaw')
    actions=recording_actions(context,share,mode='race')
    actions += [include('aims_racer_system','vehicle.launch.py'),
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
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('session_directory',default_value='',description='New recording directory; generated when omitted'),
        DeclareLaunchArgument('log_directory',default_value='',description='Optional native runtime.csv directory; record=true selects the recording session'),
        OpaqueFunction(function=build_race)])
