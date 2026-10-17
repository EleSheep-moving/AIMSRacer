"""Longitudinal trials with one native calibration command publisher."""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from aims_racer_system.launch_support import flag, include, parse_initial_pose, recording_actions


def build_calibration(context):
    cfg=context.launch_configurations
    stage=cfg.get('stage','auto')
    steering=cfg.get('steering_source','pp')
    profile=cfg.get('profile','default')
    stages=dict(speed_hold_1to5='speed_hold',accel_sweep_1to5='accel_interval',decel_sweep_1to5='decel_current',
        accel_full_step_1to5='accel_interval',decel_full_step_5to1='decel_current')
    if profile not in ('default',*stages):raise ValueError('unknown calibration profile')
    if stage=='auto':stage=stages.get(profile,'speed_hold')
    if profile in stages and stage!=stages[profile]:raise ValueError('profile and stage conflict')
    cfg['stage']=stage
    calibration_share=Path(get_package_share_directory('aims_calibration'))
    if profile in stages:
        for key,name in (('calibration_config',profile+'.yaml'),('vehicle_config','speed_hold_vehicle.yaml'),('pp_config','speed_hold_pp.yaml')):
            cfg[key]=cfg.get(key) or str(calibration_share/'config'/name)
    if stage not in ('speed_hold','accel_interval','decel_current'):
        raise ValueError('stage must be speed_hold, accel_interval or decel_current')
    if steering not in ('pp','rc'):raise ValueError('steering_source must be pp or rc')
    if flag(context,'auto_start'):raise ValueError('calibration requires explicit /calib/enable; auto_start must be false')
    initial_pose=parse_initial_pose(cfg.get('initial_pose',''))
    map_file=''
    if steering=='pp':
        if not cfg.get('map_file') or not cfg.get('reference_directory'):
            raise ValueError('PP calibration requires map_file and reference_directory')
        map_file=str(Path(cfg['map_file']).expanduser().resolve(strict=True))
        cfg['reference_directory']=str(Path(cfg['reference_directory']).expanduser().resolve(strict=True))
    elif initial_pose:
        raise ValueError('initial_pose is only used for PP calibration')
    cfg['calibration_config']=str(Path(cfg.get('calibration_config') or
        Path(get_package_share_directory('aims_calibration'))/'config/calibration.yaml').expanduser().resolve(strict=True))
    cfg['vehicle_config']=cfg.get('vehicle_config') or str(Path(get_package_share_directory('aims_mpcc'))/'config/vehicle.yaml')
    cfg['pp_config']=cfg.get('pp_config') or str(Path(get_package_share_directory('aims_pp'))/'config/pp.yaml')
    loaded=yaml.safe_load(Path(cfg['calibration_config']).read_text()) or {}
    params=loaded.get('longitudinal_calibration',{}).get('ros__parameters',{})
    full_range=params.get('sweep_protocol')=='full_range_step'
    if full_range:
        cfg['longitudinal_acceleration']='false'
        cfg['response_limits_file']=''
    baseline=cfg.get('base_current_file') or params.get('base_current_file','')
    limits='' if full_range else cfg.get('response_limits_file') or params.get('response_limits_file','')
    if params.get('predictive_exit') and not limits:
        limits=str(calibration_share/'config/response_limits_20261015.json')
    if limits:cfg['response_limits_file']=str(Path(limits).expanduser().resolve(strict=True))
    if params.get('plan_mode')=='sweep':
        if not baseline:raise ValueError('B/C sweep requires base_current_file')
        from dataclasses import fields
        from aims_calibration.engine import Config
        from aims_calibration.configuration import prepare_engine,tracking_settings_from_parameters
        defaults=Config()
        values={f.name:params.get(f.name,getattr(defaults,f.name)) for f in fields(Config)}
        values.update(stage=stage,steering_source=steering)
        vehicle=yaml.safe_load(Path(cfg['vehicle_config']).read_text())
        values['max_speed']=min(values['max_speed'],float(vehicle['max_speed']))
        from dataclasses import asdict
        from aims_calibration.protection import ProtectionSettings
        protection_settings={key:params.get('protection_'+key,value) for key,value in asdict(ProtectionSettings()).items()}
        prepare_engine(values,plan_mode='sweep',base_current_file=baseline,current_step=params.get('current_step',5.),
            tracking_settings=tracking_settings_from_parameters(params),sweep_start_index=params.get('sweep_start_index',0),
            response_limits_file=cfg.get('response_limits_file',''),protection_settings=protection_settings)
    if baseline:cfg['base_current_file']=str(Path(baseline).expanduser().resolve(strict=True))
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
        base_current_file=cfg.get('base_current_file',''),response_limits_file=cfg.get('response_limits_file',''),
        require_acceleration=str(flag(context,'longitudinal_acceleration')).lower())))
    if initial_pose:
        actions.append(Node(package='aims_racer_system',executable='relocalize_known_map.py',
            arguments=[map_file,'--pose-frame','base_link',*[arg for key,value in
                zip(('x','y','z','roll','pitch','yaw'),initial_pose) for arg in ('--'+key,str(value))]],output='screen'))
    if flag(context,'rviz'):
        if steering!='pp': raise ValueError('calibration RViz requires PP map tracking')
        env={'LIBGL_ALWAYS_SOFTWARE':'1'} if flag(context,'rviz_software_rendering') else {}
        actions.append(Node(package='rviz2',executable='rviz2',name='calibration_rviz',
            arguments=['-d',str(share/'rviz/calibration.rviz')],additional_env=env,output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage',default_value='auto',description='auto selects the profile stage; default profile remains speed_hold'),
        DeclareLaunchArgument('profile',default_value='default',description='default, speed_hold_1to5, accel_sweep_1to5, decel_sweep_1to5, accel_full_step_1to5 or decel_full_step_5to1'),
        DeclareLaunchArgument('response_limits_file',default_value='',description='Empirical wheel response limits bound to the Stage A baseline; packaged default for recorded 2026-10-14 A'),
        DeclareLaunchArgument('base_current_file',default_value='',description='Validated Stage A baseline_current.json required for B/C sweeps'),
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
