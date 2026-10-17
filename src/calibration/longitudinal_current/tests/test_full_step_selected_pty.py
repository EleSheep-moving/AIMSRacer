"""Full-range ROS command/ack/Iq handoff through the native driver and PTY.

Wheel ERPM and actual Iq are scripted telemetry, not vehicle dynamics. Only the
qualified localization/path input is replaced; wheel, Iq, selection, parameter
preflight, timestamps, packet encoding and saved evidence run through ROS.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import time

import pytest

from test_selected_sweep import fake_baseline, native_milliamps, scripted_emulator


@pytest.mark.parametrize('outcome', ['steady', 'measured_boundary'])
@pytest.mark.parametrize('stage', ['accel_interval', 'decel_current'])
def test_full_step_real_selector_driver_native_packets_and_post_ack_iq_handoff(tmp_path,monkeypatch,stage,outcome):
    case=0 if stage=='accel_interval' else 1
    monkeypatch.setenv('ROS_DOMAIN_ID',str(228+case+2*(outcome=='measured_boundary')))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros_logs'))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from ament_index_python.packages import get_package_prefix
    from crsf_receiver_msg.msg import CRSFChannels16
    from std_srvs.srv import SetBool
    from vesc_msgs.msg import VescCommand,VescStateStamped
    from aims_calibration.node import CalibrationNode,seconds
    from aims_calibration.operator import canonical_hash,check_start,summarize_results
    from aims_calibration.planner import build_full_step_plan,with_polynomial_baseline

    source=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('full_step_pty_reference_fixture',source/'aims_pp/test/conftest.py')
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    ref=tmp_path/'reference';ref.mkdir()
    reference=ref/'reference.json';reference.write_text(json.dumps(fixture.circle_payload.__wrapped__()))
    regions=ref/'calibration_regions.json'
    regions.write_text(json.dumps(dict(schema_version=1,reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
        regions=[dict(id='qualified-straight',start_s=0.,end_s=6.,kind='straight')])))
    baseline_path,baseline=fake_baseline(tmp_path,stage,'complete')
    baseline=with_polynomial_baseline(baseline,[.104 if case==0 else 5.104,0.,0.,0.])
    baseline_path.write_text(json.dumps(baseline))
    start,end=(1.,5.) if case==0 else (5.,1.)
    plan=build_full_step_plan(stage,baseline,start_speed=start,end_speed=end,max_current=20.)
    limits=tmp_path/'intentionally-absent-response-limits.json'
    parameters=dict(stage=stage,steering_source='pp',vehicle_config=str(source/'calibration/longitudinal_current/config/speed_hold_vehicle.yaml'),
        pp_config=str(source/'calibration/longitudinal_current/config/speed_hold_pp.yaml'),reference_directory=str(ref),
        log_directory=str(tmp_path/'results'),plan_mode='sweep',sweep_protocol='full_range_step',
        base_current_file=str(baseline_path),response_limits_file=str(limits),predictive_exit=False,
        measured_current_handoff=True,max_speed=5.,max_current=20.,
        start_speed=start,end_speed=end,speeds=[1.,5.],require_acceleration=False,
        steady_window_seconds=.2,steady_confirm_seconds=.1,steady_min_samples=8,
        current_settle_seconds=.03,current_window_seconds=.1,current_min_samples=4,
        current_absolute_tolerance=.02,stable_seconds=.08,trial_timeout=3.)
    emulator=scripted_emulator(source)
    rclpy.init();node=None;peer=None;executor=None;processes=[];logs=[]
    final=[];telemetry=[];last_rc=0.;last_packet=0;pending_iq=None;native_current=0.
    pulse_currents={native_milliamps(item.current)/1000. for item in plan}
    active_pulse=None;ramp_started=None;near_endpoint={};release_stamps={}

    def spawn(package,name,params,extra=()):
        binary=str(Path(get_package_prefix(package))/f'lib/{package}/{name}')
        log=(tmp_path/f'{package}.log').open('w');logs.append(log)
        args=[arg for key,value in params.items() for arg in ('-p',f'{key}:={value}')]
        processes.append(subprocess.Popen([binary,'--ros-args',*args,*extra],stdout=log,stderr=subprocess.STDOUT))

    def script_native_feedback():
        nonlocal last_packet,pending_iq,native_current,active_pulse,ramp_started
        packets=emulator.packets[last_packet:];last_packet+=len(packets)
        for stamp,packet in packets:
            if packet[0]==8:
                # Speed-mode preparation/hold follows the actual native ERPM.
                emulator.rpm=int.from_bytes(packet[1:],'big',signed=True)
                active_pulse=None;ramp_started=None
            if packet[0]==6:
                value=int.from_bytes(packet[1:],'big',signed=True)/1000.
                if value!=native_current:
                    native_current=value;pending_iq=(stamp+.02,value)
                    if outcome=='measured_boundary':
                        if value in pulse_currents:
                            active_pulse=value;ramp_started=stamp+.04
                        elif active_pulse is not None:
                            release_stamps[active_pulse]=stamp
                            # Retain observed release tail above nominal 5 m/s
                            # for B, and below endpoint but forward for C.
                            emulator.rpm=5500 if case==0 else 800
                            active_pulse=None;ramp_started=None
        if pending_iq is not None and time.monotonic()>=pending_iq[0]:
            emulator.iq=pending_iq[1];pending_iq=None
        if ramp_started is not None and time.monotonic()>=ramp_started:
            # A fast scripted wheel ramp would cross the old 140 ms forecast
            # well before this measured endpoint. The current must continue.
            elapsed=time.monotonic()-ramp_started
            velocity=start+(1. if case==0 else -1.)*min(4.,16.*elapsed)
            emulator.rpm=round(1000.*velocity)
            if (case==0 and 4.<=velocity<5.) or (case==1 and 1.<velocity<=2.):
                near_endpoint.setdefault(active_pulse,time.monotonic())

    def spin_until(condition,timeout=12.):
        nonlocal last_rc
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            assert all(p.poll() is None for p in processes),'native process exited'
            if time.monotonic()-last_rc>=.01:
                last_rc=time.monotonic();rc.publish(channels)
            executor.spin_once(timeout_sec=.001)
            script_native_feedback()
            if condition():return
            if node.engine.state=='FAULT':break
        raise AssertionError('state='+node.engine.state+' reason='+node.engine.reason+' preflight='+str(node.preflight_snapshot())+
            '\n'+'\n'.join('\n'.join(p.read_text().splitlines()[-12:]) for p in tmp_path.glob('*.log')))

    try:
        node=CalibrationNode(parameter_overrides=[Parameter(k,value=v) for k,v in parameters.items()])
        # Static qualified PP localization/path is outside this protocol test.
        monkeypatch.setattr(node.tracking,'localization_reason',lambda *args:'')
        at=node.reference.at(.5)
        monkeypatch.setattr(node.association_runtime,'association',lambda:((at['x'],at['y'],at['yaw'],start),.5,0.))
        monkeypatch.setattr(node.regions,'remaining',lambda s:100.)
        peer=rclpy.create_node('full_step_native_pty_peer')
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(peer)
        channels=CRSFChannels16()
        for i in range(1,17):setattr(channels,f'ch{i}',992)
        channels.ch5=channels.ch8=1810
        rc=peer.create_publisher(CRSFChannels16,'/rc/channels',1)
        peer.create_subscription(VescCommand,'/vesc/command',lambda m:final.append(m),100)
        peer.create_subscription(VescStateStamped,'/sensors/core',lambda m:telemetry.append(m),100)
        enable=peer.create_client(SetBool,'/calib/enable')
        spawn('vesc_driver','vesc_driver_node',dict(port=emulator.port,telemetry_rate_hz=100.,speed_to_erpm_gain=1000.,
            speed_to_erpm_offset=0.,current_startup_enabled='false',speed_min=-6000.,speed_max=6000.,
            current_min=-100.,current_max=100.),('-r','__node:=vesc_driver_node'))
        spawn('vesc_ackermann','vesc_to_odom_node',dict(speed_to_erpm_gain=1000.,speed_to_erpm_offset=0.,
            use_servo_cmd_to_calc_angular_velocity='false'),('-r','odom:=/rear_axle/wheel_odom'))
        spawn('ackermann_mux','joystick_control_v2',dict(channel_profile='steering_ch1_throttle_ch3_aux_ch5_to_ch10',
            channel_deadzone=50,publish_rate_hz=100.,speed_limit_max_speed=5.,command_speed_limit=5.,
            current_limit_max_current=100.,command_current_limit=100.))
        spin_until(lambda:enable.service_is_ready() and not check_start(node.preflight_snapshot(verify_artifacts=True)))
        assert node.driver_startup_disabled and node.tracking is not None
        assert peer.count_publishers('/state_estimation/longitudinal_acceleration')==0
        assert peer.count_publishers('/state_estimation/longitudinal_acceleration/status')==0
        assert peer.count_publishers('/drive')==0
        request=SetBool.Request();request.data=True;future=enable.call_async(request)
        spin_until(future.done,timeout=3.);assert future.result().success,future.result().message
        spin_until(lambda:node.engine.state=='COMPLETE' and final and final[-1].control_mode==0 and final[-1].target_speed_mps==0.,timeout=20.)
        node.output.flush()
        recorded=json.loads((node.output.directory/'trials.json').read_text())
        (tmp_path/'native_packets.json').write_text(json.dumps([
            dict(monotonic_stamp=stamp,payload_hex=packet.hex()) for stamp,packet in emulator.packets]))
        (tmp_path/'protocol_trace.json').write_text(json.dumps(dict(stage=stage,outcome=outcome,
            selected=[dict(stamp=seconds(m.header.stamp),mode=m.control_mode,speed=m.target_speed_mps,
                current=m.target_current) for m in final],
            telemetry=[dict(stamp=seconds(m.header.stamp),erpm=m.state.speed,avg_iq=m.state.avg_iq,
                current_motor=m.state.current_motor) for m in telemetry])))
        summary=summarize_results(recorded)
        assert summary['execution_complete'] and summary['evidence_valid'],json.dumps(summary,indent=2)
        assert summary['valid'] and summary['coverage_complete'],json.dumps(summary,indent=2)
        assert recorded['schema_version']==3 and recorded['sweep_protocol']=='full_range_step'
        assert recorded['endpoint_exit_policy']=='measured_wheel_speed'
        assert recorded['operating_wheel_speed_limit_mps'] is None
        assert recorded['metadata']['current_handoff_enabled'] is True
        assert not recorded['configuration']['predictive_exit']
        assert recorded['configuration']['measured_current_handoff']
        assert recorded.get('response_limits') is None and 'response_limits_sha256' not in recorded['metadata']
        assert not limits.exists()
        assert recorded['steady_configuration']['window_seconds']==.2
        assert recorded['metadata']['steady_configuration_sha256']==canonical_hash(recorded['steady_configuration'])
        assert not node.require_acceleration
        assert len(recorded['trials'])==len(plan) and not recorded['skipped']
        native=[int.from_bytes(p[1:],'big',signed=True) for _,p in emulator.packets if p[0]==6]
        assert all(native_milliamps(item.current) in native for item in plan)
        if case:
            assert [item.current for item in plan]==pytest.approx([.104,0.,-5.,-10.,-15.,-20.])
            assert all(v in native for v in (104,0,-5000,-10000,-15000,-20000))
        assert all(m.state.current_motor==pytest.approx(99.) for m in telemetry)
        reason='steady_state' if outcome=='steady' else 'speed_boundary'
        assert all(row['stop_reason']==reason and row['data_status']=='complete' and row['success']
            for row in recorded['trials'])
        if outcome=='steady':
            assert all(row['steady_evidence']['steady'] and not row['endpoint_reached'] for row in recorded['trials'])
        else:
            assert all(row['endpoint_reached'] for row in recorded['trials'])
            assert all(row['wheel_speed_max']>=5.5 if case==0 else row['wheel_speed_min']<=.8
                for row in recorded['trials'])
        for row in recorded['trials']:
            h=row['handoff']
            assert native_milliamps(h['target_current']) in native
            assert h['actual_iq_confirm_stamp']>h['ack_stamp']>=h['started']
            assert h['hold_confirmed'] and h['hold_confirmed_stamp']>=h['actual_iq_confirm_stamp']
            confirmations=[m for m in telemetry if h['ack_stamp']<seconds(m.header.stamp)<=h['actual_iq_confirm_stamp'] and
                abs(m.state.avg_iq-h['target_current'])<=recorded['protection_configuration']['iq_tolerance']]
            assert len({seconds(m.header.stamp) for m in confirmations})>=2
            premature=[m for m in final if h['started']<=seconds(m.header.stamp)<h['actual_iq_confirm_stamp'] and m.control_mode==0]
            assert not premature
            assert row['first_current_ack_stamp']>=row['first_current_request_stamp']
            if outcome=='measured_boundary':
                if case==0:assert row['end_speed']>=5.
                else:assert row['end_speed']<=1.
                assert h['started']-row['end_stamp']<=.05
                before_terminal=[m for m in final if row['first_current_ack_stamp']<=seconds(m.header.stamp)<row['end_stamp']]
                assert before_terminal and all(m.control_mode==1 and m.target_current==pytest.approx(row['current'])
                    for m in before_terminal)
                current=native_milliamps(row['current'])/1000.
                assert current in near_endpoint and current in release_stamps
                assert any(near_endpoint[current]<=stamp<=release_stamps[current] and p[0]==6 and
                    int.from_bytes(p[1:],'big',signed=True)==native_milliamps(row['current']) for stamp,p in emulator.packets)
        assert not emulator.errors and not any(p[0] in (5,7,9) for _,p in emulator.packets)
    finally:
        for process in reversed(processes):
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=3.)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for log in logs:log.close()
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if peer:peer.destroy_node()
        rclpy.shutdown();emulator.close()
