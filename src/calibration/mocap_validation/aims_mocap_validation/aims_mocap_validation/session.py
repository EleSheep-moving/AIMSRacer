"""Lifecycle orchestration; numeric timing subscriptions run in native C++."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from .bag import topic_specifications
from .capture import inspect_live,record_topics,command_snapshot
from .reference import map_time,vector


def readiness(config):
    capture_missing=[];analysis_missing=[]
    bridge=config.get('vrpn',{})
    if bridge.get('start_bridge',False) and not bridge.get('server'):capture_missing.append('vrpn.server')
    try:topic_specifications(config)
    except (KeyError,ValueError):capture_missing.append('vicon.topic/message_type')
    try:map_time([0],config.get('clock',{}))
    except (ValueError,TypeError,KeyError):analysis_missing.append('clock')
    try:
        vector(config.get('body_orientation_in_marker_xyzw'),4,'body orientation')
        for point in config['references'].values():vector(point.get('origin_in_marker_m'),3,'origin')
    except (ValueError,TypeError,KeyError):analysis_missing.append('marker_geometry')
    return dict(capture_missing=capture_missing,analysis_missing=analysis_missing,can_record=not capture_missing,can_analyze=not analysis_missing)


def stop(process):
    if process is None or process.poll() is not None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)


def bridge_process(config,path,log):
    if not config.get('vrpn',{}).get('start_bridge',False):return None
    return subprocess.Popen(['ros2','launch','aims_mocap_validation','bridge.launch.py','config:='+str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)


def inspect(config):
    import tempfile
    ready=readiness(config)
    if not ready['can_record']:return dict(readiness=ready,capture_configuration_errors=ready['capture_missing'],analysis_configuration_errors=ready['analysis_missing'],missing_topics=[],type_mismatches={})
    with tempfile.TemporaryDirectory() as folder:
        path=Path(folder)/'selected.json';path.write_text(json.dumps(config))
        with (Path(folder)/'bridge.log').open('w') as log:
            process=bridge_process(config,path,log)
            try:
                if process:time.sleep(1.)
                result=inspect_live(config,3.);result['readiness']=ready;return result
            finally:stop(process)


def process_snapshots(output):
    processes=[]
    names={'state_observer','lio_node','ekf_node','ekf_localization_node','vesc_driver_node','client_node','mocap_capture'}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            exe=(path/'exe').resolve()
            if exe.name not in names:continue
            command=(path/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
            item=dict(pid=int(path.name),executable=str(exe),sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),command=command,parameter_files=[])
            for i,arg in enumerate(command[:-1]):
                if arg=='--params-file':
                    source=Path(command[i+1]);target=output/f'pid-{path.name}-params-{i}.yaml';target.write_bytes(source.read_bytes());item['parameter_files'].append(target.name)
            processes.append(item)
        except (OSError,ValueError):continue
    return processes


def record(config,output,seconds=None):
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    output=Path(output);ready=readiness(config)
    if not ready['can_record']:raise ValueError('configure capture: '+', '.join(ready['capture_missing']))
    if seconds is not None and (not isinstance(seconds,(int,float)) or not 0<seconds<86400):raise ValueError('seconds must be positive and finite')
    output.mkdir(parents=True,exist_ok=False)
    selected=output/'config.selected.json';selected.write_text(json.dumps(config,indent=2)+'\n')
    prefix=Path(get_package_prefix('aims_mocap_validation'));binary=prefix/'lib/aims_mocap_validation/mocap_capture'
    share=Path(get_package_share_directory('aims_mocap_validation'))
    manifest=dict(start_wall_ns=time.time_ns(),ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),collector_executable=str(binary),collector_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),tool_version='0.1.0',python_sources_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},physical_acceptance=False)
    lock=share/'sources.lock.json'
    if lock.exists():manifest['sources']=json.loads(lock.read_text())
    bag_process=collector=bridge=None;logs=[];failure=None
    try:
        bridge_log=(output/'bridge.log').open('w');logs.append(bridge_log);bridge=bridge_process(config,selected,bridge_log)
        if bridge:time.sleep(1.)
        inspection=inspect_live(config,2.);(output/'preflight.json').write_text(json.dumps(inspection,indent=2)+'\n')
        if inspection['type_mismatches']:raise ValueError('existing topic type conflict: '+str(inspection['type_mismatches']))
        specs=topic_specifications(config);topics=list(specs);types=[specs[t]['message_type'] for t in topics]
        for topic in ['/livox/imu','/livox/imu_bias_corrected','/rear_axle/imu','/drive','/sensors/servo_position_command']:
            actual=inspection['graph'].get(topic)
            if topic not in topics and actual and len(actual)==1:topics.append(topic);types.append(actual[0])
        gaps=[.25 if 'lio_odom' in topic else .04 for topic in topics]
        parameters=output/'collector.yaml';parameters.write_text(yaml.safe_dump({'mocap_capture':{'ros__parameters':dict(output_file=str(output/'timing.csv'),topics=topics,types=types,max_gaps_s=gaps,queue_capacity=int(config.get('capture',{}).get('queue_capacity',4096)))}}))
        collector_log=(output/'collector.log').open('w');logs.append(collector_log)
        collector=subprocess.Popen([str(binary),'--ros-args','--params-file',str(parameters)],stdout=collector_log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+10
        while not (output/'timing.csv').exists():
            if collector.poll() is not None:raise RuntimeError('collector failed; see collector.log')
            if time.monotonic()>deadline:raise RuntimeError('collector startup timeout')
            time.sleep(.02)
        manifest['parameters']={}
        for name in ['/state_observer','/ekf_filter_node','/fastlio2/lio_node','/vesc_driver_node','/vrpn_mocap/vrpn_mocap_client_node']:
            if name in inspection['nodes']:manifest['parameters'][name]=command_snapshot(['ros2','param','dump',name],output/(name.strip('/').replace('/','_')+'.params.txt'))
        manifest['local_processes']=process_snapshots(output)
        command=['ros2','bag','record','--storage','sqlite3','--output',str(output/'bag'),*record_topics(config),'/mocap_validation/capture_status']
        manifest['command']=command;(output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
        recorder_log=(output/'recorder.log').open('w');logs.append(recorder_log)
        bag_process=subprocess.Popen(command,stdout=recorder_log,stderr=subprocess.STDOUT,start_new_session=True)
        print('Recording '+str(output)+'; Ctrl-C finalizes bag and timing CSV.',flush=True)
        end=time.monotonic()+seconds if seconds is not None else float('inf')
        while bag_process.poll() is None and time.monotonic()<end:
            if collector.poll() is not None:raise RuntimeError('native timing collector stopped during recording')
            if bridge is not None and bridge.poll() is not None:raise RuntimeError('VRPN bridge stopped during recording')
            time.sleep(.05)
        if bag_process.poll() is not None and bag_process.returncode not in (0,-signal.SIGINT):raise RuntimeError('bag recorder failed; see recorder.log')
    except KeyboardInterrupt:pass
    except BaseException as error:failure=error
    finally:
        for process in [bag_process,collector,bridge]:stop(process)
        for stream in logs:stream.close()
        if failure is None and bag_process and bag_process.returncode not in (0,-signal.SIGINT):failure=RuntimeError('bag recorder finalization failed; see recorder.log')
        if failure is None and collector and collector.returncode!=0:failure=RuntimeError('collector finalization failed; see collector.log')
        health=output/'timing.health.json'
        if health.exists():
            try:manifest['collector_health']=json.loads(health.read_text())
            except (OSError,ValueError) as error:
                manifest['collector_health_error']=str(error)
                if failure is None:failure=RuntimeError('collector health is incomplete: '+str(error))
        manifest['degraded']=bool(manifest.get('collector_health',{}).get('queue_drops',0))
        manifest.update(end_wall_ns=time.time_ns(),returncode=bag_process.returncode if bag_process else None,collector_returncode=collector.returncode if collector else None,error=str(failure) if failure else None)
        if (output/'timing.csv').exists():manifest['timing_sha256']=hashlib.sha256((output/'timing.csv').read_bytes()).hexdigest()
        (output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if failure:raise failure
    if manifest['collector_returncode']!=0 or manifest.get('collector_health',{}).get('writer_failed',False):raise RuntimeError('collector exited unsuccessfully')
    return manifest
