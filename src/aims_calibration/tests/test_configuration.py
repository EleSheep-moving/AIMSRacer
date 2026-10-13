from dataclasses import asdict
import json
import pytest
from aims_calibration.engine import Config
from importlib import import_module


def prepare(*args,**kwargs):
    return import_module("aims_calibration.configuration").prepare_engine(*args,**kwargs)
from test_cli import stage_a_payload


def test_a_preparation_preserves_config_and_legacy_output():
    engine,metadata,manifest=prepare(asdict(Config()))
    assert asdict(engine.config)==asdict(Config())
    assert metadata=={} and manifest is None


@pytest.mark.parametrize('stage',['accel_interval','decel_current'])
def test_sweep_preparation_freezes_plan_and_source(stage,tmp_path):
    from aims_calibration.planner import export_baseline
    path=tmp_path/'baseline.json';path.write_text(json.dumps(export_baseline(stage_a_payload())))
    values=asdict(Config());values.update(stage=stage,max_speed=5.,
        speeds=tuple(1.+.5*i for i in range(0 if stage=='accel_interval' else 1,9)))
    engine,metadata,manifest=prepare(values,plan_mode='sweep',base_current_file=str(path))
    assert manifest['plan']==engine.serializable_plan and metadata['plan_mode']=='sweep'
    assert manifest['baseline']['baseline_sha256']==metadata['baseline_sha256']
    assert engine.config.max_speed==5.
    if stage=='decel_current':
        assert engine.trial_plan[0].kind=='positive_deceleration' and engine.trial_plan[0].current>0
    before=list(engine.plan);path.write_text('{}')
    assert engine.plan==before


def test_sweep_cannot_fall_back_to_zero_or_select_a():
    with pytest.raises(ValueError):prepare(asdict(Config()),plan_mode='sweep')
    values=asdict(Config());values['stage']='accel_interval'
    with pytest.raises(ValueError):prepare(values,plan_mode='sweep',base_current_file='')
