"""ROS adapter acceptance without selection or any hardware command path."""
from pathlib import Path
import json
import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml
from aims_calibration.node import CalibrationNode
from aims_calibration.planner import export_baseline
from test_cli import stage_a_payload


@pytest.mark.parametrize('stage',['accel_interval','decel_current'])
def test_sweep_node_freezes_baseline_and_remains_idle(stage,tmp_path):
    root=Path(__file__).resolve().parents[3]
    vehicle=tmp_path/'vehicle.yaml'
    values=yaml.safe_load((root/'controller/config/vehicle.yaml').read_text());values['max_speed']=5.
    vehicle.write_text(yaml.safe_dump(values))
    baseline=tmp_path/'baseline.json';baseline.write_text(json.dumps(export_baseline(stage_a_payload())))
    rclpy.init();node=None
    try:
        overrides=dict(stage=stage,steering_source='rc',vehicle_config=str(vehicle),log_directory=str(tmp_path/'logs'),
            plan_mode='sweep',base_current_file=str(baseline),max_speed=5.,
            speeds=[1.+.5*i for i in range(0 if stage=='accel_interval' else 1,9)])
        node=CalibrationNode(parameter_overrides=[Parameter(k,value=v) for k,v in overrides.items()])
        node._tick();node.output.queue.join()
        assert not node.engine.enabled and node.engine.state=='IDLE'
        assert node.last_request.mode==0 and node.last_request.speed==0.
        assert node.output.writer.manifest['plan']==node.engine.serializable_plan
        report=json.loads((node.output.directory/'trials.json').read_text())
        assert report['schema_version']==2 and report['trials']==[]
        if stage=='decel_current':assert node.engine.trial_plan[0].current>0
        baseline.write_text('{}')
        assert 'base_current_file' in node.preflight_snapshot(verify_artifacts=True)['artifact_error']
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_rc_sweep_rejects_available_baseline_vehicle_identity_mismatch(tmp_path):
    root=Path(__file__).resolve().parents[3]
    source=stage_a_payload();source['metadata']['vehicle_sha256']='0'*64
    baseline=export_baseline(source)
    path=tmp_path/'baseline.json';path.write_text(json.dumps(baseline))
    vehicle=tmp_path/'vehicle.yaml'
    values=yaml.safe_load((root/'controller/config/vehicle.yaml').read_text());values['max_speed']=5.
    vehicle.write_text(yaml.safe_dump(values))
    rclpy.init()
    try:
        overrides=dict(stage='accel_interval',steering_source='rc',vehicle_config=str(vehicle),
            plan_mode='sweep',base_current_file=str(path),max_speed=5.,speeds=[1.,1.5])
        with pytest.raises(ValueError,match='vehicle_sha256'):
            CalibrationNode(parameter_overrides=[Parameter(k,value=v) for k,v in overrides.items()])
    finally:rclpy.shutdown()


def test_protected_sweep_remains_idle_and_audits_response_model(tmp_path):
    root=Path(__file__).resolve().parents[3]
    baseline=export_baseline(stage_a_payload());path=tmp_path/'A.json';path.write_text(json.dumps(baseline))
    limits=tmp_path/'limits.json';limits.write_text(json.dumps(dict(schema_version=1,current_signal='avg_iq',
        baseline_sha256=baseline['baseline_sha256'],limits=[dict(stage='accel_interval',start_speed=1.,current=13.9,acceleration_bound=2.)])))
    rclpy.init();node=None
    try:
        overrides=dict(stage='accel_interval',steering_source='rc',vehicle_config=str(root/'controller/config/vehicle.yaml'),
            log_directory=str(tmp_path/'logs'),plan_mode='sweep',base_current_file=str(path),speeds=[1.,1.5],
            predictive_exit=True,response_limits_file=str(limits))
        node=CalibrationNode(parameter_overrides=[Parameter(k,value=v) for k,v in overrides.items()])
        node._tick();node.output.flush()
        assert node.engine.state=='IDLE' and not node.engine.enabled and node.last_request.speed==0.
        values={v.key:v.value for v in node.status_message().status[0].values}
        assert values['predictive_exit']=='true' and values['handoff']=='null'
        assert node.output.writer.manifest['protection_configuration']['exit_horizon']==.14
        limits.write_text('{}')
        assert 'response_limits_file' in node.preflight_snapshot(verify_artifacts=True)['artifact_error']
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
