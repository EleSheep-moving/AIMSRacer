"""ROS adapter tests: no physical driver and no IMU/acceleration publisher."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from aims_calibration.node import CalibrationNode
from aims_calibration.planner import export_baseline
from test_cli import stage_a_payload


@pytest.fixture
def full_node(tmp_path):
    root=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('full_step_pp_fixture',root/'aims_pp/test/conftest.py')
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    ref=tmp_path/'reference';ref.mkdir()
    path=ref/'reference.json';path.write_text(json.dumps(fixture.circle_payload.__wrapped__()))
    ref_hash=hashlib.sha256(path.read_bytes()).hexdigest()
    regions=ref/'calibration_regions.json';regions.write_text(json.dumps(dict(schema_version=1,
        reference_sha256=ref_hash,regions=[dict(id='test-straight',start_s=0.,end_s=6.,kind='straight')])))
    source=stage_a_payload();source['metadata'].update(map_sha256='a'*64,reference_sha256=ref_hash,
        calibration_regions_sha256=hashlib.sha256(regions.read_bytes()).hexdigest())
    baseline=export_baseline(source);a=tmp_path/'A.json';a.write_text(json.dumps(baseline))
    limits=tmp_path/'intentionally-absent-response-limits.json'
    params=dict(stage='accel_interval',steering_source='pp',vehicle_config=str(root/'calibration/longitudinal_current/config/speed_hold_vehicle.yaml'),
        pp_config=str(root/'calibration/longitudinal_current/config/speed_hold_pp.yaml'),reference_directory=str(ref),
        log_directory=str(tmp_path/'logs'),plan_mode='sweep',sweep_protocol='full_range_step',
        base_current_file=str(a),response_limits_file=str(limits),predictive_exit=False,measured_current_handoff=True,max_speed=5.,
        start_speed=1.,end_speed=5.,speeds=[1.,5.],require_acceleration=False)
    rclpy.init();node=None
    try:
        node=CalibrationNode(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
        yield node
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_full_node_idle_records_v3_without_acceleration_publisher(full_node):
    node=full_node;node._tick();node.output.flush()
    assert node.engine.state=='IDLE' and not node.engine.enabled
    assert node.last_request.mode==0 and node.last_request.speed==0.
    assert node.preflight_snapshot()['require_acceleration'] is False
    data=json.loads((node.output.directory/'trials.json').read_text())
    assert data['schema_version']==3 and data['sweep_protocol']=='full_range_step'
    assert data['steady_configuration']['min_samples']==40
    assert not node.engine.config.predictive_exit and node.engine.config.measured_current_handoff
    assert data['endpoint_exit_policy']=='measured_wheel_speed'
    assert data['operating_wheel_speed_limit_mps'] is None
    assert data['metadata']['current_handoff_enabled'] is True
    assert data.get('response_limits') is None and 'response_limits_sha256' not in data['metadata']
    assert not Path(node.get_parameter('response_limits_file').value).exists()


def test_source_distance_is_passed_to_engine_and_status(full_node,monkeypatch):
    node=full_node
    monkeypatch.setattr(node.tracking,'localization_reason',lambda *args:'')
    at=node.reference.at(.5)
    pose=(at['x'],at['y'],at['yaw'],1.)
    monkeypatch.setattr(node.association_runtime,'association',lambda:(pose,.5,0.))
    obs=node._tracking_observation(node.obs,node._now(),0.)
    assert obs.straight and obs.straight_remaining_m==pytest.approx(5.5)
    node._tick()
    status={v.key:v.value for v in node.status_message().status[0].values}
    assert float(status['straight_remaining_m'])==pytest.approx(5.5)
    assert status['sweep_protocol']=='full_range_step'


def test_full_node_accepts_actual_odometry_tail_above_command_speed_cap(full_node):
    from nav_msgs.msg import Odometry
    msg=Odometry();msg.header.stamp=full_node.get_clock().now().to_msg()
    msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
    msg.twist.twist.linear.x=5.6
    full_node._odometry(msg)
    assert full_node.tracking.allow_speed_overshoot
    assert full_node.tracking.pose is not None and full_node.tracking.pose[3]==5.6


@pytest.mark.parametrize('interval,should_fault',[(.04,False),(.05148,False),(.059,False),(.061,True)])
def test_protected_publication_timer_allows_60ms(full_node,monkeypatch,interval,should_fault):
    from aims_calibration.engine import Request
    node=full_node
    node.engine.enabled=True;node.engine.selected=True
    node.last_timer=100.-interval
    node.receipts={name:100. for name in ('wheel','telemetry','rc','authority','final')}
    faults=[]
    monkeypatch.setattr('aims_calibration.node.time.monotonic',lambda:100.)
    monkeypatch.setattr(node,'count_publishers',lambda topic:1)
    monkeypatch.setattr(node,'_tracking_observation',lambda obs,*args:obs)
    monkeypatch.setattr(node.engine,'fault',faults.append)
    monkeypatch.setattr(node.engine,'tick',lambda now,obs:Request(now,1))
    node._tick()
    assert ('publication_timer_stale' in faults)==should_fault
