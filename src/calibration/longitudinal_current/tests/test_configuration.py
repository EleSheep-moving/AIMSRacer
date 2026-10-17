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


def test_C_resume_freezes_suffix_and_preserves_full_plan_identity(tmp_path):
    from aims_calibration.planner import export_baseline,with_polynomial_baseline
    from aims_calibration.operator import canonical_hash,summarize_results
    baseline=with_polynomial_baseline(export_baseline(stage_a_payload()),[5.77545,3.51367,-.594009,.0451659])
    path=tmp_path/'baseline.json';path.write_text(json.dumps(baseline))
    values=asdict(Config());values.update(stage='decel_current',max_speed=5.,speeds=(5.,4.5,4.,3.5,3.,2.5,2.,1.5))
    full,_,full_manifest=prepare(values,plan_mode='sweep',base_current_file=str(path))
    engine,metadata,manifest=prepare(values,plan_mode='sweep',base_current_file=str(path),sweep_start_index=14)
    assert len(full.plan)==183 and len(engine.plan)==169
    assert engine.trial_plan==full.trial_plan[14:]
    assert engine.trial_plan[0].start_speed==5. and engine.trial_plan[0].current==-60.
    assert engine.index==0 and engine.state=='IDLE' and not engine.trials and not engine.skipped
    assert manifest['plan'][0]['plan_index']==0
    assert metadata['sweep_start_index']==14 and metadata['sweep_full_plan_trials']==183
    assert metadata['sweep_full_plan_sha256']==full_manifest['plan_sha256']
    cfg=asdict(engine.config);metadata.update(evidence_valid=True,configuration_sha256=canonical_hash(cfg),
        map_sha256='map',reference_sha256='reference')
    data=dict(schema_version=2,configuration=cfg,metadata=metadata,trials=[],skipped=[],**manifest)
    result=summarize_results(data)
    assert result['evidence_valid'] and not result['execution_complete'] and not result['coverage_complete']
    assert 'baseline_or_sweep_plan_invalid' not in result['errors']
    for key,value in [('sweep_start_index',13),('sweep_start_index',14.),('sweep_start_index',-1),
                      ('sweep_full_plan_sha256','bad'),('sweep_full_plan_trials',182)]:
        original=metadata[key];metadata[key]=value
        assert 'baseline_or_sweep_plan_invalid' in summarize_results(data)['errors']
        metadata[key]=original
    for offset in (-1,True,14.,183,184):
        with pytest.raises(ValueError):prepare(values,plan_mode='sweep',base_current_file=str(path),sweep_start_index=offset)
    with pytest.raises(ValueError):prepare(values,sweep_start_index=14)
