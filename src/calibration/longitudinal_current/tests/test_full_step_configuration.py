from dataclasses import asdict
import json
import pytest

from aims_calibration.configuration import prepare_engine
from aims_calibration.engine import Config
from aims_calibration.operator import canonical_hash
from aims_calibration.planner import export_baseline
from test_cli import stage_a_payload


def prepare_full(tmp_path, stage='accel_interval', **options):
    baseline=export_baseline(stage_a_payload())
    a=tmp_path/'A.json';a.write_text(json.dumps(baseline))
    rows=[dict(stage=stage,start_speed=(1.+i*.5 if stage=='accel_interval' else 1.5+i*.5),
               current=20. if stage=='accel_interval' else -5.,acceleration_bound=2.) for i in range(8)]
    limits=tmp_path/'limits.json'
    limits.write_text(json.dumps(dict(schema_version=1,current_signal='avg_iq',baseline_sha256=baseline['baseline_sha256'],limits=rows)))
    values=asdict(Config());values.update(stage=stage,max_speed=5.,speeds=(1.,5.),predictive_exit=False,measured_current_handoff=True,
        start_speed=1. if stage=='accel_interval' else 5.,end_speed=5. if stage=='accel_interval' else 1.,
        sweep_protocol='full_range_step')
    return prepare_engine(values,base_current_file=str(a),response_limits_file=str(limits),**dict(plan_mode='sweep',**options))


@pytest.mark.parametrize('stage',['accel_interval','decel_current'])
def test_full_protocol_freezes_long_plan_and_steady_settings(tmp_path,stage):
    engine,metadata,manifest=prepare_full(tmp_path,stage)
    assert len({(p.start_speed,p.end_speed) for p in engine.trial_plan})==1
    assert abs(engine.trial_plan[0].end_speed-engine.trial_plan[0].start_speed)==4.
    assert engine.protection is not None and engine.protection.response_limit is None
    assert not engine.config.predictive_exit and engine.config.measured_current_handoff
    assert metadata['sweep_protocol']==manifest['sweep_protocol']=='full_range_step'
    assert metadata['steady_configuration_sha256']==canonical_hash(manifest['steady_configuration'])
    if stage=='decel_current':
        assert metadata['start_trigger_policy']==manifest['start_trigger_policy']=='first_wheel_at_or_above_start_speed'
    # Schema 2 baseline also supports a natural steady endpoint between grid points.
    assert engine.protection.holding_current(2.25)>0
    assert engine.protection.holding_current(2.25)==pytest.approx(
        (engine.protection.holding_current(2.)+engine.protection.holding_current(2.5))/2)
    with pytest.raises(ValueError):engine.protection.holding_current(.99)


def test_manual_full_step_cannot_bypass_frozen_sweep_plan():
    values=asdict(Config());values.update(stage='accel_interval',sweep_protocol='full_range_step',predictive_exit=False,measured_current_handoff=True)
    with pytest.raises(ValueError,match='sweep'):
        prepare_engine(values,plan_mode='manual')


def test_disabled_reverse_guard_is_frozen_in_manifest_and_hash(tmp_path):
    engine,metadata,manifest=prepare_full(tmp_path,'decel_current',
        protection_settings=dict(reverse_guard_enabled=False))
    assert engine.protection.settings.reverse_guard_enabled is False
    assert manifest['protection_configuration']['reverse_guard_enabled'] is False
    assert metadata['protection_configuration_sha256']==canonical_hash(manifest['protection_configuration'])


@pytest.mark.parametrize('value',[0,1,'false',None])
def test_reverse_guard_switch_requires_boolean(value):
    from aims_calibration.protection import ProtectionSettings
    with pytest.raises(ValueError,match='reverse_guard_enabled'):
        ProtectionSettings(reverse_guard_enabled=value)


def test_full_step_does_not_require_or_read_response_limits(tmp_path):
    from aims_calibration.planner import export_baseline
    baseline=tmp_path/'A.json';baseline.write_text(json.dumps(export_baseline(stage_a_payload())))
    values=asdict(Config());values.update(stage='accel_interval',sweep_protocol='full_range_step',
        max_speed=5.,start_speed=1.,end_speed=5.,predictive_exit=False,measured_current_handoff=True)
    engine,metadata,manifest=prepare_engine(values,plan_mode='sweep',base_current_file=str(baseline),
        response_limits_file=str(tmp_path/'does-not-exist.json'))
    assert engine.protection is not None and manifest.get('response_limits') is None
    assert metadata['endpoint_exit_policy']=='measured_wheel_speed'
    assert metadata['operating_wheel_speed_limit_mps'] is None
    assert metadata['response_limits_role']=='not_used'
