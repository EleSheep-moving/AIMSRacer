from dataclasses import asdict
import hashlib
import json
import pytest
from aims_calibration.engine import Config


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def a_payload():
    speeds=[1+.5*i for i in range(9)]
    c=asdict(Config(speeds=tuple(speeds), max_speed=5., measurement_seconds=5.))
    # Artifact writer serializes tuples to lists.
    c=json.loads(json.dumps(c))
    return dict(schema_version=1, configuration=c,
        metadata=dict(evidence_valid=True, configuration_sha256=canonical_sha(c), reference_sha256='a'*64),
        trials=[dict(trial_id=i+1, plan_index=i, stage='speed_hold', requested_target=s,
                     success=True, evidence_valid=True, reason='completed', mean_current_motor=.104+5*i,
                     std_current_motor=.2, samples=100, duration_s=5.) for i,s in enumerate(speeds)])


def test_baseline_export_retains_fractional_point_provenance_and_missing_voltage(tmp_path):
    from aims_calibration import planner
    payload=a_payload(); source=tmp_path/'trials.json'
    source.write_text(json.dumps(payload, indent=2)+'\n')
    baseline=planner.export_baseline(payload,source)
    assert baseline['points'][0]['mean_current_motor']==.104
    assert baseline['points'][0]['voltage_input'] is None
    assert baseline['source']['sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
    assert baseline['source']['configuration_sha256']==payload['metadata']['configuration_sha256']
    assert baseline['source']['metadata_sha256']==canonical_sha(payload['metadata'])
    assert baseline['source']['context']==payload['metadata']
    destination=tmp_path/'baseline.json'; destination.write_text(json.dumps(baseline))
    assert planner.load_baseline(destination)==baseline
    baseline['points'][0]['mean_current_motor']=0.
    destination.write_text(json.dumps(baseline))
    with pytest.raises(ValueError): planner.load_baseline(destination)


@pytest.mark.parametrize('damage', ['missing','duplicate','failed','evidence','configuration_hash','zero','nan','samples','duration','stage'])
def test_baseline_refuses_invalid_or_incomplete_A_source(damage):
    from aims_calibration import planner
    p=a_payload()
    if damage=='missing': p['trials'].pop()
    elif damage=='duplicate': p['trials'].append(p['trials'][0].copy())
    elif damage=='failed': p['trials'][0]['success']=False
    elif damage=='evidence': p['metadata']['evidence_valid']=False
    elif damage=='configuration_hash': p['metadata']['configuration_sha256']='b'*64
    elif damage=='zero': p['trials'][0]['mean_current_motor']=0.
    elif damage=='nan': p['trials'][0]['mean_current_motor']=float('nan')
    elif damage=='samples': p['trials'][0]['samples']=0
    elif damage=='duration': p['trials'][0]['duration_s']=4.99
    elif damage=='stage': p['trials'][0]['stage']='accel_interval'
    with pytest.raises(ValueError): planner.export_baseline(p)


def test_B_grid_uses_start_baseline_and_appends_exact_100_without_rounding():
    from aims_calibration import planner
    b=planner.export_baseline(a_payload())
    plan=planner.build_sweep_plan('accel_interval',None,b)
    first=[x for x in plan if x.start_speed==1.]
    assert [x.current for x in first]==pytest.approx([.104+5*i for i in range(1,20)]+[100.])
    assert all(x.current>x.baseline_current and x.kind=='acceleration' for x in plan)
    assert sorted(set((x.start_speed,x.end_speed) for x in plan))==[(1+.5*i,1.5+.5*i) for i in range(8)]
    assert all(x.repeat==1 and x.stage=='accel_interval' for x in plan)


def test_C_grid_includes_positive_coast_negative_with_distinct_endpoints():
    from aims_calibration import planner
    b=planner.export_baseline(a_payload())
    plan=planner.build_sweep_plan('decel_current',None,b)
    first=[x for x in plan if x.start_speed==1.5]
    assert [x.current for x in first]==pytest.approx([.104,0]+[-5*i for i in range(1,21)])
    assert first[0].kind=='positive_deceleration' and first[0].end_speed==1.
    assert first[1].kind=='coast' and first[1].end_speed==1.
    second=[x for x in plan if x.start_speed==2.]
    assert [x.current for x in second[:3]]==pytest.approx([5.104,.104,0])
    assert [x.end_speed for x in second[:2]]==[1.5,1.5]
    assert all(x.end_speed==1. for x in second[2:])
    assert second[-1].current==-100. and second[-1].kind=='braking'


def test_planner_repeats_expand_each_item_and_refuse_uncovered_speed_or_tampered_baseline():
    from aims_calibration import planner
    b=planner.export_baseline(a_payload())
    plan=planner.build_sweep_plan('accel_interval',[1.,1.5],b,repeats=2)
    assert [x.repeat for x in plan[:4]]==[1,2,1,2]
    assert plan[0].current==plan[1].current and plan[0].current!=plan[2].current
    with pytest.raises(ValueError): planner.build_sweep_plan('accel_interval',[1.,1.7],b)
    b['points'][0]['mean_current_motor']=0.
    with pytest.raises(ValueError): planner.build_sweep_plan('accel_interval',[1.,1.5],b)


@pytest.mark.parametrize('item', [dict(stage='accel_interval', start_speed=1.,end_speed=1.5,current=.104,baseline_current=.104),
    dict(stage='decel_current',start_speed=2.,end_speed=1.5,current=2.,kind='positive_deceleration'),
    dict(stage='decel_current',start_speed=2.,end_speed=1.,current=0.,kind='braking'),
    dict(stage='decel_current',start_speed=2.,end_speed=2.5,current=-5.,kind='braking')])
def test_structured_items_require_physical_sign_and_positive_C_baseline(item):
    from aims_calibration import planner
    with pytest.raises(ValueError): planner.TrialPlan(**item)


def test_baseline_accepts_real_legacy_A_artifact_without_schema_version():
    from aims_calibration import planner
    payload=a_payload(); del payload['schema_version']
    assert len(planner.export_baseline(payload)['points'])==9
    payload['schema_version']=2
    with pytest.raises(ValueError): planner.export_baseline(payload)


def test_B_refuses_baseline_when_first_increment_exceeds_ceiling():
    from aims_calibration import planner
    payload=a_payload(); payload['trials'][0]['mean_current_motor']=98.
    b=planner.export_baseline(payload)
    with pytest.raises(ValueError): planner.build_sweep_plan('accel_interval',[1.,1.5],b)


@pytest.mark.parametrize('key,value', [('speeds',None),('speeds',{}),('speeds',['bad',1.]),('repeats',True)])
def test_malformed_source_configuration_refuses_with_value_error(key,value):
    from aims_calibration import planner
    payload=a_payload(); payload['configuration'][key]=value
    payload['metadata']['configuration_sha256']=canonical_sha(payload['configuration'])
    with pytest.raises(ValueError): planner.export_baseline(payload)
