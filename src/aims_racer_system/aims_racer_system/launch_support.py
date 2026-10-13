"""Shared launch recording; no controller authority or runtime polling."""
from datetime import datetime
import math
import hashlib
import json
from pathlib import Path
import shutil
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def flag(context,name):
    value=context.launch_configurations.get(name,'false').lower()
    if value not in ('true','false'):raise ValueError(f'{name} must be true or false')
    return value=='true'


def include(package,filename,arguments=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory(package))/'launch'/filename)),
        launch_arguments=(arguments or {}).items())


def parse_initial_pose(pose):
    if not pose.strip():return []
    values=[float(item) for item in pose.replace(',',' ').split()]
    if len(values)!=6 or not all(math.isfinite(v) for v in values):
        raise ValueError('initial_pose requires six finite base_link map values: x y z roll pitch yaw')
    return values


def recording_actions(context,share,mode='race'):
    if not flag(context,'record'):return []
    share=Path(share)
    requested=context.launch_configurations.get('session_directory','')
    session=Path(requested).expanduser() if requested else Path.home()/'aimsracer-data/sessions'/datetime.now().strftime('%Y-%m-%d')/(datetime.now().strftime('%H%M%S_%f')+'_'+mode)
    session=session.resolve();session.mkdir(parents=True,exist_ok=False)
    # Native node's optional runtime.csv accompanies this bag without another
    # process wrapper. Config and manifest remain owned by the immutable bundle.
    context.launch_configurations['log_directory']=str(session)
    snapshots=session/'params';snapshots.mkdir()
    for p in (share/'params').glob('*'):
        if p.is_file():shutil.copy2(p,snapshots/p.name)
    # Capture the selected VESC file as well when a launch overrides the default.
    selected_vesc=context.launch_configurations.get('vesc_config','')
    if selected_vesc and Path(selected_vesc).expanduser().is_file():
        shutil.copy2(Path(selected_vesc).expanduser(),snapshots/'vesc_selected.yaml')
    for key,label in (('vehicle_config','vehicle'),('pp_config','pp'),('calibration_config','calibration')):
        selected=context.launch_configurations.get(key,'')
        if selected and Path(selected).expanduser().is_file():
            shutil.copy2(Path(selected).expanduser(),snapshots/(label+'_selected.yaml'))
    baseline=context.launch_configurations.get('base_current_file','')
    if baseline:shutil.copy2(Path(baseline).expanduser(),snapshots/'baseline_current.json')
    reference_files={}
    ref=context.launch_configurations.get('reference_directory','')
    if ref:
        for name in ('metadata.json','reference.json','calibration_regions.json'):
            path=Path(ref).expanduser()/name
            if path.is_file():
                reference_files[name]=hashlib.sha256(path.read_bytes()).hexdigest()
                shutil.copy2(path,snapshots/name)
    (session/'session.json').write_text(json.dumps(dict(mode=mode,created=datetime.now().astimezone().isoformat(),
        launch_arguments=dict(context.launch_configurations),reference_files=reference_files),indent=2)+'\n')
    topics=['/livox/lidar','/livox/imu','/livox/imu_bias_corrected','/fastlio2/lio_odom',
        '/fastlio2/body_cloud','/rear_axle/lio_odom','/rear_axle/imu','/rear_axle/wheel_odom',
        '/odometry/filtered','/imu/gyro_bias/status','/sensors/core','/rc/channels',
        '/state_estimation/longitudinal_acceleration','/state_estimation/longitudinal_acceleration/status',
        '/vesc/command','/calib/vesc_command','/control/autonomy_speed_enabled',
        '/control/calibration_enabled','/sensors/servo_position_command',
        '/sensors/imu','/sensors/imu/raw','/tf','/tf_static','/fastlio2/tf']
    if mode in ('race','calibration'):topics+=['/drive','/mpcc/status','/mpcc/reference','/mpcc/prediction',
        '/pp/status','/pp/reference','/pp/lookahead_point',
        '/calib/status','/calib/reference','/calib/lookahead_point',
        '/localization/status','/localization/anchor_status','/localization/ndt_status',
        '/localization/map_sha256','/localization/map_valid','/localization/ndt_pose','/localization/odom_bridge_pose']
    else:topics+=['/pgo/loop_markers']
    return [ExecuteProcess(cmd=['ros2','bag','record','--qos-profile-overrides-path',
        str(share/'params/recording_qos.yaml'),'-o',str(session/'bag'),*topics],output='screen')]
