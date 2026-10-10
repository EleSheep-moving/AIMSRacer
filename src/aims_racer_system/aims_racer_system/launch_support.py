"""Shared launch recording; no controller authority or runtime polling."""
from datetime import datetime
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
    (session/'session.json').write_text(json.dumps(dict(mode=mode,created=datetime.now().astimezone().isoformat(),
        launch_arguments=dict(context.launch_configurations)),indent=2)+'\n')
    topics=['/livox/lidar','/livox/imu','/livox/imu_bias_corrected','/fastlio2/lio_odom',
        '/fastlio2/body_cloud','/rear_axle/lio_odom','/rear_axle/imu','/rear_axle/wheel_odom',
        '/odometry/filtered','/imu/gyro_bias/status','/sensors/core','/rc/channels',
        '/ackermann_cmd','/control/autonomy_speed_enabled','/commands/motor/speed',
        '/commands/motor/current','/commands/servo/position','/tf','/tf_static','/fastlio2/tf']
    if mode=='race':topics+=['/drive','/mpcc/status','/mpcc/reference','/mpcc/prediction',
        '/localization/status','/localization/anchor_status','/localization/ndt_status',
        '/localization/map_sha256','/localization/map_valid','/localization/ndt_pose','/localization/odom_bridge_pose']
    else:topics+=['/pgo/loop_markers']
    return [ExecuteProcess(cmd=['ros2','bag','record','--qos-profile-overrides-path',
        str(share/'params/recording_qos.yaml'),'-o',str(session/'bag'),*topics],output='screen')]
