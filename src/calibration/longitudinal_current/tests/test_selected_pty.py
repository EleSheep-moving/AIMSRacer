"""Installed calibration -> installed selector -> installed driver -> PTY VESC.

The emulator reports scripted ERPM and motor telemetry; this is protocol and
state-machine acceptance, not a vehicle acceleration/stop-distance experiment.
"""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import pytest


@pytest.mark.parametrize('stage,loss,hold_speed,use_cli', [('speed_hold',False,1.,False),('speed_hold',False,5.,False),
    ('accel_interval',False,1.,False),('decel_current',False,1.,False),('accel_interval',True,1.,False),
    ('speed_hold',False,1.,True),('speed_hold',False,1.,'interrupt')])
def test_installed_calibration_selected_signed_current_and_measured_trials(tmp_path,monkeypatch,stage,loss,hold_speed,use_cli):
    monkeypatch.setenv('ROS_DOMAIN_ID',str(196+['speed_hold','accel_interval','decel_current'].index(stage)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1'); monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros_logs'))
    from ament_index_python.packages import get_package_prefix
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    import rclpy
    from crsf_receiver_msg.msg import CRSFChannels16
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from vesc_msgs.msg import VescCommand, VescStateStamped

    source=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('calibration_pty_emulator',source/'vesc/vesc_driver/test/test_driver_link.py')
    helper=importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    original_frame=helper.frame
    def q_axis_fixture(payload):
        if payload[0]==4:
            payload=bytearray(payload);payload[17:21]=struct.pack('>i',123)
        return original_frame(payload)
    helper.frame=q_axis_fixture
    emulator=helper.VescEmulator(); emulator.rpm=0
    context=Context(); rclpy.init(context=context)
    peer=rclpy.create_node('calibration_selected_pty_test',context=context)
    executor=SingleThreadedExecutor(context=context); executor.add_node(peer)
    channels=CRSFChannels16()
    for i in range(1,17): setattr(channels,f'ch{i}',992)
    channels.ch5=1810; channels.ch8=1810
    rc=peer.create_publisher(CRSFChannels16,'/rc/channels',1)
    statuses=[]; final=[]; selected=[]; telemetry=[]
    peer.create_subscription(DiagnosticArray,'/calib/status',lambda m:statuses.append(m),10)
    peer.create_subscription(VescCommand,'/vesc/command',lambda m:final.append(m),10)
    peer.create_subscription(Bool,'/control/calibration_enabled',lambda m:selected.append(m.data),10)
    peer.create_subscription(VescStateStamped,'/sensors/core',lambda m:telemetry.append(m),10)
    enable=peer.create_client(SetBool,'/calib/enable')
    processes=[]; logs=[]; pump_rc=True; last_rc=0.; operator_process=None

    def binary(package,name): return str(Path(get_package_prefix(package))/f'lib/{package}/{name}')
    def spawn(package,name,parameters,extra=()):
        log=(tmp_path/f'{package}.log').open('w'); logs.append(log)
        args=[arg for key,value in parameters.items() for arg in ('-p',f'{key}:={value}')]
        process=subprocess.Popen([binary(package,name),'--ros-args',*args,*extra],stdout=log,stderr=subprocess.STDOUT)
        processes.append(process)
    def state():
        if not statuses: return ''
        return {v.key:v.value for v in statuses[-1].status[0].values}.get('state','')
    def spin_until(condition,timeout=8.):
        nonlocal last_rc
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            for process in processes:
                assert process.poll() is None, '\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
            if pump_rc and time.monotonic()-last_rc>=.01:
                last_rc=time.monotonic(); rc.publish(channels)
            executor.spin_once(timeout_sec=.001)
            if condition(): return
        raise AssertionError('timeout, state='+state()+'\nstatus='+str({v.key:v.value for v in statuses[-1].status[0].values} if statuses else {})+'\n'+'\n'.join('\n'.join(p.read_text().splitlines()[-12:]) for p in tmp_path.glob('*.log')))
    def trials():
        files=list((tmp_path/'results').glob('*/trials.json'))
        return json.loads(files[0].read_text())['trials'] if files else []
    def wire(value):
        return any(payload[0]==6 and int.from_bytes(payload[1:],'big',signed=True)==value for _,payload in emulator.packets)
    def call(value):
        request=SetBool.Request(); request.data=value; future=enable.call_async(request)
        spin_until(future.done,timeout=2.); assert future.result().success
    try:
        spawn('vesc_driver','vesc_driver_node',dict(port=emulator.port,telemetry_rate_hz=100.,
            speed_to_erpm_gain=1000.,speed_to_erpm_offset=0.,current_startup_enabled='false',
            speed_min=-6000.,speed_max=6000.,current_min=-100.,current_max=100.),('-r','__node:=vesc_driver_node'))
        spawn('vesc_ackermann','vesc_to_odom_node',dict(speed_to_erpm_gain=1000.,speed_to_erpm_offset=0.,
            use_servo_cmd_to_calc_angular_velocity='false'),('-r','odom:=/rear_axle/wheel_odom'))
        spawn('ackermann_mux','joystick_control_v2',dict(channel_profile='steering_ch1_throttle_ch3_aux_ch5_to_ch10',
            channel_deadzone=50,publish_rate_hz=100.,speed_limit_max_speed=5.,command_speed_limit=5.,
            current_limit_max_current=100.,command_current_limit=100.))
        vehicle=source/'controller/config/vehicle.yaml'
        if hold_speed==5.:
            import yaml
            values=yaml.safe_load(vehicle.read_text()); values['max_speed']=5.
            vehicle=tmp_path/'hold_vehicle.yaml'; vehicle.write_text(yaml.safe_dump(values))
        params=dict(vehicle_config=str(vehicle),steering_source='rc',stage=stage,max_speed=5. if stage=='speed_hold' else 4.,
            log_directory=str(tmp_path/'results'),speeds=f'[{hold_speed}]',stable_seconds=.08,
            measurement_seconds=5. if use_cli=='interrupt' else .25,trial_timeout=2.,start_speed=1.,
            end_speed=2. if stage=='accel_interval' else .5)
        if stage!='speed_hold': params['currents']='[5.0]' if stage=='accel_interval' else '[-5.0]'
        spawn('aims_calibration','longitudinal_calibration',params)
        def ready():
            return statuses and {v.key:v.value for v in statuses[-1].status[0].values}.get('start_ready')=='true'
        spin_until(lambda:state()=='IDLE' and ready() and selected and selected[-1] and telemetry and enable.service_is_ready())
        assert all(m.control_mode==0 and m.target_speed_mps==0 for m in final[-5:])
        assert any(p[0]==8 and int.from_bytes(p[1:],'big',signed=True)==0 for _,p in emulator.packets)
        assert not trials()
        inspector=subprocess.Popen([binary('aims_calibration','inspect')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            spin_until(lambda:inspector.poll() is not None)
            output,error=inspector.communicate()
            assert inspector.returncode==0, error+output
            assert json.loads(output)['preflight']['driver_startup_disabled'] is True
        finally:
            if inspector.poll() is None: inspector.kill();inspector.wait()
        if use_cli:
            operator_process=subprocess.Popen([binary('aims_calibration','start'),'--max-seconds','15'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            spin_until(lambda:statuses and {v.key:v.value for v in statuses[-1].status[0].values}.get('enable_count')=='1')
        else: call(True)
        duplicate=SetBool.Request();duplicate.data=True
        future=enable.call_async(duplicate);spin_until(future.done)
        assert not future.result().success and 'acquisition_already_started' in future.result().message
        emulator.rpm=int(1000*hold_speed)
        if use_cli=='interrupt':
            spin_until(lambda:state()=='MEASURE')
            operator_process.send_signal(signal.SIGINT)
            spin_until(lambda:operator_process.poll() is not None)
            output,error=operator_process.communicate()
            assert operator_process.returncode==3,error+output
            spin_until(lambda:state()=='STOPPING' and trials() and wire(0))
            assert not trials()[0]['success']
            assert 'Stop request failed' not in output,error+output
            return
        if stage!='speed_hold':
            wanted=5000 if stage=='accel_interval' else -5000
            spin_until(lambda:wire(wanted) and state()=='MEASURE' and any(m.control_mode==1 and m.target_current==wanted/1000 for m in final),timeout=3.)
            assert any(m.control_mode==1 and m.target_current==wanted/1000 for m in final)
            if loss:
                channels.ch5=172
                spin_until(lambda:state()=='FAULT' and trials(),timeout=2.)
                assert not trials()[-1]['success'] and any(s in trials()[-1]['reason'] for s in ('authority','rc'))
                channels.ch5=1810
                spin_until(lambda:selected and selected[-1],timeout=2.)
                assert state()=='FAULT' and all(not t['success'] for t in trials())
                return
            emulator.rpm=2200 if stage=='accel_interval' else 400
        spin_until(lambda:state()=='COMPLETE' and trials() and final and final[-1].control_mode==0 and final[-1].target_speed_mps==0,timeout=3.)
        if use_cli:
            emulator.rpm=0
            spin_until(lambda:operator_process.poll() is not None,timeout=8.)
            output,error=operator_process.communicate()
            assert operator_process.returncode==0,error+output
            assert '"stationary_verified": true' in output
        result_file=next((tmp_path/'results').glob('*/trials.json'))
        report=subprocess.run([binary('aims_calibration','results'),str(result_file)],capture_output=True,text=True,timeout=5.)
        assert report.returncode==0, report.stderr+report.stdout
        assert json.loads(report.stdout)['valid'] is True
        result=trials()[0]; assert result['current_signal']=='avg_iq' and result['mean_iq']==pytest.approx(1.23)
        assert result['success'] and result['mean_current_motor']==pytest.approx(12.34)
        assert result['start_speed']==pytest.approx(hold_speed)
        if stage=='speed_hold':
            assert result['hold_sampling']=='cumulative' and result['duration_s']>=.25
            assert result['segment_count']==1
            assert any(p[0]==8 and int.from_bytes(p[1:],'big',signed=True)==int(1000*hold_speed) for _,p in emulator.packets)
        if stage!='speed_hold':
            assert result['end_speed']==pytest.approx(2.2 if stage=='accel_interval' else .4)
            assert result['duration_s']>0 and result['mean_acceleration'] is not None
        assert final[-1].control_mode==0 and final[-1].target_speed_mps==0
        assert all(m.state.avg_iq==pytest.approx(1.23) and m.state.current_motor==pytest.approx(12.34) and m.state.current_input==0 for m in telemetry)
        assert not any(p[0]==7 for _,p in emulator.packets)
        assert not emulator.errors
        rows=list(csv.DictReader(next((tmp_path/'results').glob('*/observations.csv')).open()))
        stamps=[(r['wheel_stamp'],r['telemetry_stamp']) for r in rows]
        assert any(float(r['avg_iq'])==pytest.approx(1.23) for r in rows)
        assert len(stamps)==len(set(stamps)) and any(float(r['current_motor'])==pytest.approx(12.34) for r in rows)
        if use_cli:
            operator_process=subprocess.Popen([binary('aims_calibration','stop')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            spin_until(lambda:operator_process.poll() is not None)
            output,error=operator_process.communicate()
            assert operator_process.returncode==0,error+output
        else: call(False)
        spin_until(lambda:state()=='STOPPING' and wire(0),timeout=2.)
    finally:
        if operator_process is not None and operator_process.poll() is None:
            operator_process.send_signal(signal.SIGINT)
            try: operator_process.wait(timeout=6.)
            except subprocess.TimeoutExpired: operator_process.kill();operator_process.wait()
        for process in reversed(processes):
            if process.poll() is None: process.send_signal(signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=3.)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        for log in logs: log.close()
        executor.shutdown(); peer.destroy_node(); rclpy.shutdown(context=context); emulator.close()
