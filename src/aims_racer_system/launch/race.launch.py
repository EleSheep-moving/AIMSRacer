"""Field vehicle + known-map NDT + one explicitly selected controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import include,flag,recording_actions,parse_initial_pose


def build_race(context):
    cfg=context.launch_configurations
    share=Path(get_package_share_directory('aims_racer_system'))
    controller=cfg.get('controller','mpcc')
    if controller not in ('mpcc','pp'):raise ValueError('controller must be mpcc or pp')
    map_file=str(Path(cfg['map_file']).expanduser().resolve(strict=True))
    artifact=cfg.get('artifact_directory','')
    if controller=='mpcc' and not artifact:raise ValueError('MPCC requires artifact_directory')
    artifact=str(Path(artifact).expanduser().resolve(strict=True)) if artifact else ''
    reference=cfg.get('reference_directory','')
    if controller=='pp':
        if not reference and artifact:reference=str(Path(artifact)/'input_reference')
        if not reference:raise ValueError('PP requires reference_directory or artifact_directory')
        cfg['reference_directory']=str(Path(reference).expanduser().resolve(strict=True))
        cfg['vehicle_config']=cfg.get('vehicle_config','') or (str(Path(artifact)/'input_config.yaml') if artifact else
            str(Path(get_package_share_directory('aims_mpcc'))/'config/vehicle.yaml'))
        cfg['pp_config']=cfg.get('pp_config','') or str(Path(get_package_share_directory('aims_pp'))/'config/pp.yaml')
    flags={name:str(flag(context,name)).lower() for name in ('auto_start','repeat_laps')}
    acceleration=str(flag(context,'longitudinal_acceleration')).lower()
    pose=cfg.get('initial_pose','').strip()
    values=parse_initial_pose(pose)
    actions=recording_actions(context,share,mode='race')
    actions += [include('aims_racer_system','vehicle.launch.py',{'longitudinal_acceleration':acceleration}),
        include('aims_racer_system','known_map_localization.launch.py',{'map_file':map_file})]
    if controller=='mpcc':
        actions.append(include('aims_mpcc_rt','mpcc.launch.py',dict(artifact_directory=artifact,
            log_directory=cfg.get('log_directory',''),**flags)))
    else:
        actions.append(include('aims_pp','pp.launch.py',dict(reference_directory=cfg['reference_directory'],
            vehicle_config=cfg['vehicle_config'],pp_config=cfg['pp_config'],
            speed_limit=cfg.get('speed_limit','1.0'),log_directory=cfg.get('log_directory',''),**flags)))
    if pose:
        actions.append(Node(package='aims_racer_system',executable='relocalize_known_map.py',
            arguments=[map_file,'--pose-frame','base_link',*[arg for k,v in zip(('x','y','z','roll','pitch','yaw'),values) for arg in ('--'+k,str(v))]],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file',description='Exact immutable PCD used for the saved reference'),
        DeclareLaunchArgument('controller',default_value='mpcc',description='Exactly one controller: mpcc or pp'),
        DeclareLaunchArgument('artifact_directory',default_value='',description='Prepared native bundle; required for MPCC, optional PP reference/config source'),
        DeclareLaunchArgument('reference_directory',default_value='',description='PP canonical reference with reference.json; optional when supplied by bundle'),
        DeclareLaunchArgument('vehicle_config',default_value='',description='PP vehicle YAML; defaults to bundle config or installed field config'),
        DeclareLaunchArgument('pp_config',default_value='',description='Enhanced PP YAML; defaults to installed pp.yaml'),
        DeclareLaunchArgument('speed_limit',default_value='1.0',description='PP speed cap in m/s; MPCC uses its immutable bundle'),
        DeclareLaunchArgument('initial_pose',default_value='',description='Optional base_link map x y z roll pitch yaw; angles in radians. Otherwise initialize through /initialpose.'),
        DeclareLaunchArgument('auto_start',default_value='false',description='One startup enable after existing prerequisites; false requires /mpcc/enable'),
        DeclareLaunchArgument('repeat_laps',default_value='false',description='False stops after one lap'),
        DeclareLaunchArgument('longitudinal_acceleration',default_value='false',description='Record independent longitudinal acceleration; does not change control'),
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('session_directory',default_value='',description='New recording directory; generated when omitted'),
        DeclareLaunchArgument('log_directory',default_value='',description='Optional native runtime.csv directory; record=true selects the recording session'),
        OpaqueFunction(function=build_race)])
