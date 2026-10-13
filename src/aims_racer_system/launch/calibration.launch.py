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
    profile=cfg.get('profile','default')
    if profile not in ('default','speed_hold_1to5') or (profile=='speed_hold_1to5' and stage!='speed_hold'):
        raise ValueError('profile must be default or speed_hold_1to5 with stage speed_hold')
    calibration_share=Path(get_package_share_directory('aims_calibration'))
    if profile=='speed_hold_1to5':
        for key,name in (('calibration_config','speed_hold_1to5.yaml'),('vehicle_config','speed_hold_vehicle.yaml'),('pp_config','speed_hold_pp.yaml')):
            cfg[key]=cfg.get(key) or str(calibration_share/'config'/name)
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
        stage=stage,steering_source=steering,auto_start='false',
        require_acceleration=str(flag(context,'longitudinal_acceleration')).lower())))
    if values:
        actions.append(Node(package='aims_racer_system',executable='relocalize_known_map.py',
            arguments=[map_file,'--pose-frame','base_link',*[arg for key,value in
                zip(('x','y','z','roll','pitch','yaw'),values) for arg in ('--'+key,str(value))]],output='screen'))
    if flag(context,'rviz'):
        if steering!='pp': raise ValueError('calibration RViz requires PP map tracking')
        env={'LIBGL_ALWAYS_SOFTWARE':'1'} if flag(context,'rviz_software_rendering') else {}
        actions.append(Node(package='rviz2',executable='rviz2',name='calibration_rviz',
            arguments=['-d',str(share/'rviz/calibration.rviz')],additional_env=env,output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage',default_value='speed_hold'),
        DeclareLaunchArgument('profile',default_value='default',description='Optional speed_hold_1to5 preset with packaged calibration, vehicle and PP settings'),
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
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('rviz_software_rendering',default_value='true',description='Software rendering for NX displays that render black with GL acceleration'),
        DeclareLaunchArgument('session_directory',default_value=''),
        DeclareLaunchArgument('log_directory',default_value=''),
        OpaqueFunction(function=build_calibration)])
