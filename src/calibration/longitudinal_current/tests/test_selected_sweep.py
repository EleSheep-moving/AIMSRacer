"""Generated B/C sweeps through installed nodes and an isolated PTY VESC.

Scripted ERPM/current telemetry proves command selection, native wire encoding,
tracking decisions, and saved evidence. It does not model vehicle dynamics.
"""
import binascii
from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import time

import pytest


def fake_baseline(directory, stage, decision):
    """Nine valid synthetic A points; only selected test speeds are exercised."""
    from aims_calibration.engine import Config
    from aims_calibration.operator import canonical_hash
    from aims_calibration.planner import export_baseline
    speeds=[1.+.5*i for i in range(9)]
    configuration=json.loads(json.dumps(asdict(Config(speeds=tuple(speeds),max_speed=5.,measurement_seconds=1.))))
    means=[.104]*9
    if stage=='decel_current': means[2]=10.104 if decision=='positive_limit' else 5.104
    payload=dict(configuration=configuration,
        metadata=dict(current_signal='avg_iq',evidence_valid=True,configuration_sha256=canonical_hash(configuration)),
        trials=[dict(trial_id=i+1,plan_index=i,stage='speed_hold',requested_target=speed,
            success=True,evidence_valid=True,reason='completed',duration_s=1.,samples=100,
            mean_current_motor=99.,std_current_motor=0.,mean_iq=means[i],std_iq=.01) for i,speed in enumerate(speeds)])
    source=directory/'synthetic_A_trials.json'; source.write_text(json.dumps(payload)+'\n')
    baseline=export_baseline(payload,source)
    path=directory/'synthetic_baseline.json'; path.write_text(json.dumps(baseline)+'\n')
    return path,baseline


def native_milliamps(current):
    # VescCommand.target_current is float32, then COMM_SET_CURRENT truncates
    # the driver double to signed integer milliamps.
    return int(struct.unpack('>f',struct.pack('>f',current))[0]*1000)


def scripted_emulator(source):
    spec=importlib.util.spec_from_file_location('sweep_pty_vesc_helper',source/'vesc/vesc_driver/test/test_driver_link.py')
    helper=importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)

    class ScriptedEmulator(helper.VescEmulator):
        def __init__(self):
            self.motor_current=99.
            self.iq=0.
            super().__init__()
            self.rpm=0

        def run(self):
            # Use the driver's existing PTY/CRC fixture, with a scripted current
            # q-axis field independently from the 99 A motor diagnostic.
            buffer=bytearray()
            while self.running:
                if not select.select([self.master],[],[],.01)[0]: continue
                buffer.extend(os.read(self.master,4096))
                while len(buffer)>=5:
                    if buffer[0]!=2:
                        self.errors.append('invalid start'); del buffer[0]; continue
                    size=buffer[1]
                    if len(buffer)<size+5: break
                    payload=bytes(buffer[2:2+size]); crc=int.from_bytes(buffer[2+size:4+size],'big')
                    end=buffer[4+size]; del buffer[:size+5]
                    if end!=3 or crc!=binascii.crc_hqx(payload,0):
                        self.errors.append('invalid CRC/end'); continue
                    self.packets.append((time.monotonic(),payload))
                    if payload[0]==0: response=b'\x00\x06\x05PTY_VESC\x00'+bytes(24)
                    elif payload[0]==4:
                        response=bytearray(74); response[0]=4
                        response[23:27]=struct.pack('>i',self.rpm)
                        response[5:9]=struct.pack('>i',round(self.motor_current*100))
                        response[17:21]=struct.pack('>i',round(self.iq*100))
                        response[27:29]=struct.pack('>h',198)
                        response=bytes(response)
                    elif payload[0]==65: response=bytes([65,255,255])+struct.pack('>16f',*([1.]*16))
                    else: continue
                    packet=helper.frame(response)
                    os.write(self.master,packet[:3]); os.write(self.master,packet[3:])

    return ScriptedEmulator()


def test_fixture_plans_include_fractional_current_coast_and_scoped_limits(tmp_path):
    from aims_calibration.planner import build_sweep_plan
    _,baseline=fake_baseline(tmp_path,'decel_current','complete')
    plan=build_sweep_plan('decel_current',[2.],baseline,max_current=10.)
    assert [p.current for p in plan]==pytest.approx([.104,0.,-5.,-10.])
    assert [p.kind for p in plan]==['positive_deceleration','coast','braking','braking']
    assert [native_milliamps(p.current) for p in plan]==[104,0,-5000,-10000]
    _,baseline=fake_baseline(tmp_path,'decel_current','positive_limit')
    assert [p.current for p in build_sweep_plan('decel_current',[2.],baseline,max_current=10.)]==pytest.approx([5.104,.104,0.,-5.,-10.])


SWEEP_CASES=[('accel_interval','complete',False),('accel_interval','limit',False),
    ('decel_current','complete',False),('decel_current','positive_limit',False),
    ('accel_interval','complete',True),('decel_current','complete',True),('decel_current','reverse',False)]


@pytest.mark.parametrize('stage,decision,polynomial',SWEEP_CASES)
def test_installed_generated_sweep_native_selected_commands_and_tracking_decisions(tmp_path,monkeypatch,stage,decision,polynomial):
    case=SWEEP_CASES.index((stage,decision,polynomial))
    monkeypatch.setenv('ROS_DOMAIN_ID',str(210+case))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1'); monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros_logs'))
    from ament_index_python.packages import get_package_prefix
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    import rclpy
    from crsf_receiver_msg.msg import CRSFChannels16
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from vesc_msgs.msg import VescCommand,VescStateStamped
    from aims_calibration.operator import summarize_results
    from aims_calibration.planner import build_sweep_plan

    source=Path(__file__).resolve().parents[3]
    baseline_path,baseline=fake_baseline(tmp_path,stage,decision)
    if polynomial:
        from aims_calibration.planner import with_polynomial_baseline
        # Distinct measured and planning currents ensure installed nodes really
        # use the formula, not the retained A point. Keep currents in test bounds.
        baseline=with_polynomial_baseline(baseline,[.204 if stage=='accel_interval' else 5.204,0.,0.,0.])
        baseline_path.write_text(json.dumps(baseline)+'\n')
    speeds=[1.,1.5,2.] if stage=='accel_interval' else [2.]
    if stage=='decel_current' and decision in ('positive_limit','reverse'):speeds=[2.,1.5]
    plan=build_sweep_plan(stage,speeds,baseline,max_current=10.)
    emulator=scripted_emulator(source)
    assert emulator.port.startswith('/dev/pts/')
    context=Context(); rclpy.init(context=context)
    peer=rclpy.create_node('calibration_selected_sweep_pty_test',context=context)
    executor=SingleThreadedExecutor(context=context); executor.add_node(peer)
    channels=CRSFChannels16()
    for i in range(1,17): setattr(channels,f'ch{i}',992)
    channels.ch5=channels.ch8=1810
    rc=peer.create_publisher(CRSFChannels16,'/rc/channels',1)
    statuses=[]; final=[]; producer=[]; authority=[]; telemetry=[]
    peer.create_subscription(DiagnosticArray,'/calib/status',lambda m:statuses.append(m),10)
    peer.create_subscription(VescCommand,'/vesc/command',lambda m:final.append(m),10)
    peer.create_subscription(VescCommand,'/calib/vesc_command',lambda m:producer.append(m),10)
    peer.create_subscription(Bool,'/control/calibration_enabled',lambda m:authority.append(m.data),10)
    peer.create_subscription(VescStateStamped,'/sensors/core',lambda m:telemetry.append(m),10)
    enable=peer.create_client(SetBool,'/calib/enable')
    processes=[]; logs=[]; last_rc=0.; active=None; active_since=None; start_speed=None
    reverse_injected=False

    def binary(package,name): return str(Path(get_package_prefix(package))/f'lib/{package}/{name}')
    def spawn(package,name,parameters,extra=()):
        log=(tmp_path/f'{package}.log').open('w'); logs.append(log)
        args=[arg for key,value in parameters.items() for arg in ('-p',f'{key}:={value}')]
        processes.append(subprocess.Popen([binary(package,name),'--ros-args',*args,*extra],stdout=log,stderr=subprocess.STDOUT))
    def status():
        return {v.key:v.value for v in statuses[-1].status[0].values} if statuses else {}
    def payload():
        paths=list((tmp_path/'results').glob('*/trials.json'))
        return json.loads(paths[0].read_text()) if paths else {}
    def decoded_currents():
        return [(stamp,int.from_bytes(p[1:],'big',signed=True)) for stamp,p in emulator.packets if p[0]==6]
    def drive_script():
        nonlocal active,active_since,start_speed,reverse_injected
        if not final: return
        command=final[-1]
        if command.control_mode==0:
            if command.target_speed_mps>0:
                start_speed=command.target_speed_mps
                emulator.rpm=round(start_speed*1000)
                active=None; active_since=None
            return
        if command.control_mode!=1 or start_speed is None: return
        current=command.target_current
        if active!=current:
            active=current; active_since=time.monotonic()
        unreachable=decision=='limit' or (decision=='positive_limit' and current>0)
        emulator.iq=0. if unreachable else current
        if decision=='reverse' and start_speed==2. and current<0 and not reverse_injected and time.monotonic()-active_since>=.08:
            emulator.rpm=-100;reverse_injected=True
            return
        if decision=='reverse' and reverse_injected and start_speed==2. and current<0:
            emulator.rpm=-100
            return
        if not unreachable and time.monotonic()-active_since>=.08:
            endpoint=start_speed+.5 if stage=='accel_interval' else (start_speed-.5 if current>0 else 1.)
            emulator.rpm=round(endpoint*1000)
    def spin_until(condition,timeout=10.,script=False):
        nonlocal last_rc
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            for process in processes:
                assert process.poll() is None,'\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
            if time.monotonic()-last_rc>=.01:
                last_rc=time.monotonic(); rc.publish(channels)
            executor.spin_once(timeout_sec=.001)
            if script: drive_script()
            if condition(): return
        raise AssertionError('timeout;status='+str(status())+'\n'+'\n'.join('\n'.join(p.read_text().splitlines()[-15:]) for p in tmp_path.glob('*.log')))
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
        # Tests pin faster tracking timing explicitly; production defaults stay
        # .25s settling + .5s rolling window + ten unique source observations.
        spawn('aims_calibration','longitudinal_calibration',dict(
            vehicle_config=str(source/'controller/config/vehicle.yaml'),steering_source='rc',stage=stage,
            plan_mode='sweep',base_current_file=str(baseline_path),current_step=5.,max_current=10.,max_speed=4.,
            speeds=json.dumps(speeds),log_directory=str(tmp_path/'results'),stable_seconds=.08,
            trial_timeout=2.,current_settle_seconds=.03,current_window_seconds=.25,current_min_samples=10,
            current_absolute_tolerance=.01,current_relative_tolerance=.1,current_failure_fraction=.9,max_attempts=3))
        spin_until(lambda:status().get('state')=='IDLE' and status().get('start_ready')=='true' and authority and authority[-1] and telemetry and enable.service_is_ready())
        assert not payload().get('trials') and all(c.control_mode==0 and c.target_speed_mps==0 for c in final[-5:])
        assert peer.count_publishers('/drive')==0
        call(True)
        # Source ERPM is scripted from native selected speed/current commands.
        spin_until(lambda:status().get('state')=='COMPLETE' and len(payload().get('trials',[]))+len(payload().get('skipped',[]))==len(plan) and final[-1].control_mode==0 and final[-1].target_speed_mps==0,
                   timeout=20.,script=True)
        recorded=payload(); assert recorded['current_signal']=='avg_iq'
        assert all(m.state.current_motor==pytest.approx(99.) for m in telemetry)
        assert any(abs(m.state.avg_iq-m.state.current_motor)>1. for m in telemetry)
        summary=summarize_results(recorded)
        assert summary['execution_complete'] and summary['evidence_valid'],json.dumps(summary,indent=2)
        assert summary['valid']==(decision=='complete')
        assert summary['coverage_complete']==(decision=='complete')
        assert status()['current_signal']=='avg_iq'
        assert recorded['metadata']['current_signal']=='avg_iq'
        assert all(r['current_signal']=='avg_iq' for r in recorded['trials'])
        assert all(r['mean_current_motor']==pytest.approx(99.) for r in recorded['trials'])
        assert recorded['schema_version']==2 and len(recorded['plan'])==len(plan)
        assert recorded['baseline']['schema_version']==(3 if polynomial else 2)
        assert [p['current'] for p in recorded['plan']]==pytest.approx([p.current for p in plan])
        rows=recorded['trials']; skips=recorded['skipped']
        observed=[value for _,value in decoded_currents()]
        # Preserve the planner mean and account for the actual ROS float32
        # representation before native signed milliamp packet encoding.
        wanted=[native_milliamps(p.current) for p in plan]
        if decision=='complete':
            assert len(rows)==len(plan) and all(r['success'] for r in rows) and not skips
            assert all(value in observed for value in wanted)
        elif decision=='limit':
            assert [r['plan_index'] for r in rows]==list(range(len(plan))) and not skips
            assert all(r['tracking_status']=='outside_tolerance' and r['reason']=='trial_timeout' for r in rows)
            assert all(value in observed for value in wanted)
            assert summary['accepted']==0 and summary['coverage_gaps']==list(range(len(plan)))
        elif decision=='positive_limit':
            assert rows[0]['reason']=='current_tracking_not_achieved'
            assert [r['plan_index'] for r in skips]==[1,2,3,4]
            assert all(r['start_target_speed']==1.5 and r['success'] for r in rows[1:])
            assert summary['accepted']==3 and summary['coverage_gaps']==[0,1,2,3,4]
            assert native_milliamps(plan[1].current) not in observed
        else:
            assert reverse_injected
            assert [r['reason'] for r in rows[:3]]==['completed','completed','reverse_motion']
            assert [r['plan_index'] for r in skips]==[3]
            assert all(r['start_target_speed']==1.5 and r['success'] for r in rows[3:])
            assert summary['accepted']==5 and summary['coverage_gaps']==[2,3]
            assert not any(s.status[0].message=='reverse_motion' and any(v.key=='state' and v.value=='FAULT' for v in s.status[0].values) for s in statuses)
        if stage=='decel_current' and decision=='complete':
            expected=.204 if polynomial else .104
            assert rows[0]['current']==pytest.approx(expected) and rows[0]['baseline_current']==pytest.approx(5.+expected)
            assert rows[1]['tracking_status']=='not_applicable' and rows[1]['kind']=='coast'
            firsts=[next(stamp for stamp,value in decoded_currents() if value==wanted_current) for wanted_current in (wanted[0],0,-5000,-10000)]
            assert firsts==sorted(firsts)
        for row in rows:
            assert row['first_current_request_stamp']<=row['end_stamp']
            if row['first_current_ack_stamp'] is not None:assert row['first_current_request_stamp']<=row['first_current_ack_stamp']<=row['end_stamp']
            assert row['response_duration_s']>0
        assert len({r['attempt_id'] for r in rows})==len(rows)
        assert all(c.target_duty==0 and c.target_brake_current==0 and c.target_acceleration_mps2==0 for c in producer+final)
        assert any(c.control_mode==1 for c in producer) and any(c.control_mode==1 for c in final)
        assert not any(p[0] in (5,7,9) for _,p in emulator.packets) and not emulator.errors
        assert final[-1].control_mode==0 and final[-1].target_speed_mps==final[-1].target_current==0
        call(False)
        spin_until(lambda:status().get('state')=='STOPPING',timeout=2.)
    finally:
        for process in reversed(processes):
            if process.poll() is None: process.send_signal(signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=3.)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        for log in logs: log.close()
        executor.shutdown(); peer.destroy_node(); rclpy.shutdown(context=context); emulator.close()
