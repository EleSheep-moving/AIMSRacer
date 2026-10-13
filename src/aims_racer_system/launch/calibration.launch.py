"""Longitudinal trials with one native calibration command publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import flag, include, parse_initial_pose, recording_actions


def build_calibration(context):
    cfg=context.launch_configurations
    stage=cfg.get('stage','speed_hold')
    steering=cfg.get('steering_source','pp')
    if stage not in ('speed_hold','accel_interval','decel_current'):
        raise ValueError('stage must be speed_hold, accel_interval or decel_current')
    if steering not in ('pp','rc'):raise ValueError('steering_source must be pp or rc')
    if flag(context,'auto_start'):raise ValueError('calibration requires explicit /calib/enable; auto_start must be false')
    values=parse_initial_pose(cfg.get('initial_pose',''))
    map_file=''
    if steering=='pp':
        if not cfg.get('map_file') or not cfg.get('reference_directory'):
            raise ValueError('PP calibration requires map_file and reference_directory')
        map_file=str(Path(cfg['map_file']).expanduser().resolve(strict=True))
        cfg['reference_directory']=str(Path(cfg['reference_directory']).expanduser().resolve(strict=True))
    elif values:
        raise ValueError('initial_pose is only used for PP calibration')
    cfg['calibration_config']=str(Path(cfg.get('calibration_config') or
        Path(get_package_share_directory('aims_calibration'))/'config/calibration.yaml').expanduser().resolve(strict=True))
    cfg['vehicle_config']=cfg.get('vehicle_config') or str(Path(get_package_share_directory('aims_mpcc'))/'config/vehicle.yaml')
    cfg['pp_config']=cfg.get('pp_config') or str(Path(get_package_share_directory('aims_pp'))/'config/pp.yaml')
    cfg['current_startup_enabled']='false'
    share=Path(get_package_share_directory('aims_racer_system'))
    actions=recording_actions(context,share,mode='calibration')
    log=cfg.get('log_directory','')
    if not log:
        from datetime import datetime
        log=str(Path.home()/'aimsracer-data/sessions'/datetime.now().strftime('%Y-%m-%d')/
            (datetime.now().strftime('%H%M%S_%f')+'_calibration'))
        cfg['log_directory']=log
    actions.append(include('aims_racer_system','vehicle.launch.py',{
        'current_startup_enabled':'false',
        'longitudinal_acceleration':str(flag(context,'longitudinal_acceleration')).lower()}))
    if steering=='pp':
        actions.append(include('aims_racer_system','known_map_localization.launch.py',{'map_file':map_file}))
    actions.append(include('aims_calibration','longitudinal_calibration.launch.py',dict(
        config=cfg['calibration_config'],reference_directory=cfg.get('reference_directory',''),
        vehicle_config=cfg['vehicle_config'],pp_config=cfg['pp_config'],log_directory=log,
        stage=stage,steering_source=steering,auto_start='false')))
    if values:
        actions.append(Node(package='aims_racer_system',executable='relocalize_known_map.py',
            arguments=[map_file,'--pose-frame','base_link',*[arg for key,value in
                zip(('x','y','z','roll','pitch','yaw'),values) for arg in ('--'+key,str(value))]],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage',default_value='speed_hold'),
        DeclareLaunchArgument('steering_source',default_value='pp'),
        DeclareLaunchArgument('calibration_config',default_value=''),
        DeclareLaunchArgument('reference_directory',default_value=''),
        DeclareLaunchArgument('vehicle_config',default_value=''),
        DeclareLaunchArgument('pp_config',default_value=''),
        DeclareLaunchArgument('map_file',default_value=''),
        DeclareLaunchArgument('initial_pose',default_value=''),
        DeclareLaunchArgument('auto_start',default_value='false'),
        DeclareLaunchArgument('longitudinal_acceleration',default_value='true'),
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('session_directory',default_value=''),
        DeclareLaunchArgument('log_directory',default_value=''),
        OpaqueFunction(function=build_calibration)])
