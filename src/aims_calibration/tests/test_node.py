import time
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool
from vesc_msgs.msg import VescCommand, VescStateStamped
from aims_calibration.node import CalibrationNode


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    vehicle=Path(__file__).resolve().parents[2]/'controller/config/vehicle.yaml'
    n=CalibrationNode(parameter_overrides=[Parameter('steering_source',value='rc'),
        Parameter('vehicle_config',value=str(vehicle)),Parameter('log_directory',value=str(tmp_path))])
    yield n
    n.destroy_node(); rclpy.shutdown()


def test_node_idle_native_output_default_disabled_and_manual_enable_service(node):
    topics={p.topic_name for p in node.publishers}
    assert '/calib/vesc_command' in topics and '/calib/status' in topics
    assert '/drive' not in topics and '/vesc/command' not in topics
    node._tick(); assert node.last_request.mode==0 and node.last_request.speed==0
    first=node.last_request.stamp; time.sleep(.01); node._tick()
    assert node.last_request.stamp>first and not node.engine.enabled
    request=SetBool.Request(); request.data=True
    response=node._enable(request,SetBool.Response())
    assert response.success and node.engine.state=='PREPARE'
    request.data=False; node._enable(request,SetBool.Response()); node._tick()
    assert node.last_request.mode==1 and node.last_request.current==0


def test_ros_adapter_motor_current_and_reject_inactive_fields(node):
    msg=VescStateStamped(); msg.header.stamp=node.get_clock().now().to_msg()
    msg.state.current_motor=7.; msg.state.current_input=99.; msg.state.voltage_input=16.; msg.state.speed=400.
    node._telemetry(msg)
    assert node.obs.current_motor==7. and node.obs.voltage_input==16. and node.obs.erpm==400.
    command=VescCommand(); command.header.stamp=node.get_clock().now().to_msg()
    command.header.frame_id='base_link'; command.target_duty=.1
    node._final(command)
    assert node.obs.final_request is None


def test_diagnostics_report_stop_reason_and_separate_measured_current(node):
    message=node.status_message()
    values={v.key:v.value for v in message.status[0].values}
    assert values['enabled']=='false' and values['state']=='IDLE'
    assert 'current_motor' in values and 'request_current' in values


def test_command_qos_keeps_latest_request_with_short_lifespan(node):
    assert node.command_pub.qos_profile.depth==1
    assert node.command_pub.qos_profile.lifespan.nanoseconds==100_000_000
    latest={s.topic_name:s.qos_profile.depth for s in node.subscriptions}
    assert latest['/vesc/command']==latest['/control/calibration_enabled']==1


def test_node_rejects_auto_start_even_without_system_launch(tmp_path):
    rclpy.init()
    try:
        with pytest.raises(ValueError,match='auto_start'):
            CalibrationNode(parameter_overrides=[Parameter('auto_start',value=True)])
    finally:
        rclpy.shutdown()


def test_node_uses_bounded_background_writer_and_reports_runtime(node):
    from aims_calibration.io import AsyncOutputLog
    assert isinstance(node.output,AsyncOutputLog)
    assert node.output.queue.maxsize==128
    node._tick(); node.output.queue.join()
    assert (node.output.directory/'runtime.csv').read_text().count('\n')==2


def test_logging_error_latches_zero_current_and_remains_visible(node):
    node.output.error=OSError('disk full')
    node._tick()
    assert node.engine.state=='FAULT' and node.last_request.mode==1 and node.last_request.current==0
    node.output.error=None
    node._tick(); assert node.engine.state=='FAULT'
    values={v.key:v.value for v in node.status_message().status[0].values}
    assert values['state']=='FAULT'


def test_clock_freeze_still_runs_safety_timer_and_latches_stop(tmp_path):
    from rosgraph_msgs.msg import Clock as ClockMessage
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from dataclasses import replace
    from aims_calibration.engine import Observation, Request
    rclpy.init(); executor=SingleThreadedExecutor(); n=None; peer=None
    try:
        vehicle=Path(__file__).resolve().parents[2]/'controller/config/vehicle.yaml'
        n=CalibrationNode(parameter_overrides=[Parameter('steering_source',value='rc'),
            Parameter('vehicle_config',value=str(vehicle)),Parameter('log_directory',value=str(tmp_path)),
            Parameter('use_sim_time',value=True)])
        peer=Node('calibration_clock_test'); clock=peer.create_publisher(ClockMessage,'/clock',10)
        executor.add_node(n); executor.add_node(peer)
        msg=ClockMessage(); msg.clock.sec=5
        deadline=time.monotonic()+2.
        while n._now()!=5. and time.monotonic()<deadline:
            clock.publish(msg); executor.spin_once(timeout_sec=.01)
        assert n._now()==5.
        n.engine.enable(True,now=4.9); n.engine.selected=True
        requested=Request(5.,speed=1.); n.engine.sent(requested)
        n.obs=Observation(wheel_stamp=5.,telemetry_stamp=5.,velocity=1.,current_motor=7.,voltage_input=16.,erpm=100.,
            authority=True,authority_stamp=5.,final_request=requested,final_received=5.,rc_stamp=5.,rc_valid=True,straight=True)
        n.receipts={name:time.monotonic() for name in ('wheel','telemetry','rc','authority','final')}
        n._tick(); assert n.engine.state=='PREPARE'
        deadline=time.monotonic()+.35
        while time.monotonic()<deadline: executor.spin_once(timeout_sec=.01)
        assert n._now()==5. and n.engine.state=='FAULT'
        assert n.last_request.mode==1 and n.last_request.current==0
    finally:
        executor.shutdown()
        if n is not None: n.destroy_node()
        if peer is not None: peer.destroy_node()
        rclpy.shutdown()


def test_completion_boundary_enqueue_overflow_revokes_trial_acceptance(node):
    from threading import Event
    from aims_calibration.engine import Config, Engine, Observation, Request
    import json
    now=node._now(); steady=time.monotonic()
    node.engine=Engine(Config(steering_source='rc',speeds=(1.,),measurement_seconds=.01))
    node.engine.enable(True,now=now-.1); node.engine.state='MEASURE'; node.engine.selected=True
    node.engine.started=now-.02; node.engine.start_velocity=1.; node.engine.region=''
    node.engine.phase_started=now-.02
    node.engine.active_samples=[dict(current_motor=7.,wheel_stamp=now-.02,velocity=1.)]
    node.engine.sent(Request(now-.01,speed=1.))
    node.obs=Observation(wheel_stamp=now,telemetry_stamp=now,velocity=1.,current_motor=7.,voltage_input=16.,erpm=100.,
        authority=True,authority_stamp=now,final_request=Request(now,speed=1.),final_received=now,
        rc_stamp=now,rc_valid=True,straight=True)
    node.receipts={name:steady for name in ('wheel','telemetry','rc','authority','final')}
    entered,release=Event(),Event(); original=node.output.writer.runtime
    def blocked(row): entered.set(); release.wait(2.); original(row)
    node.output.writer.runtime=blocked
    row=dict(ros_stamp=now,callback_ms=.1,timer_interval_ms=20.,pending=0,dropped=0,state='MEASURE')
    try:
        node.output.runtime(row); assert entered.wait(1.)
        for _ in range(node.output.queue.maxsize): node.output.runtime(row)
        node._tick()
        assert node.engine.state=='FAULT' and node.last_request.mode==1 and node.last_request.current==0
        assert not node.engine.trials[-1]['success']
        assert node.engine.trials[-1]['response_completed'] and not node.engine.trials[-1]['evidence_valid']
    finally:
        release.set(); node.output.queue.join()
    node._tick(); node.output.queue.join()
    payload=json.loads((node.output.directory/'trials.json').read_text())
    assert not payload['trials'][-1]['success'] and payload['metadata']['evidence_valid'] is False
    assert 'queue' in payload['trials'][-1]['evidence_failure']
