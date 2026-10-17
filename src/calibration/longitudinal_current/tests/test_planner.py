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
        metadata=dict(current_signal="avg_iq", evidence_valid=True, configuration_sha256=canonical_sha(c), reference_sha256='a'*64),
        trials=[dict(trial_id=i+1, plan_index=i, stage='speed_hold', requested_target=s,
                     success=True, evidence_valid=True, reason='completed', mean_current_motor=90.+i, mean_iq=.104+5*i,
                     std_current_motor=9.,std_iq=.2, samples=100, duration_s=5.) for i,s in enumerate(speeds)])


def test_baseline_export_retains_fractional_point_provenance_and_missing_voltage(tmp_path):
    from aims_calibration import planner
    payload=a_payload(); source=tmp_path/'trials.json'
    source.write_text(json.dumps(payload, indent=2)+'\n')
    baseline=planner.export_baseline(payload,source)
    assert baseline['points'][0]['mean_iq']==.104
    assert baseline['points'][0]['voltage_input'] is None
    assert baseline['source']['sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
    assert baseline['source']['configuration_sha256']==payload['metadata']['configuration_sha256']
    assert baseline['source']['metadata_sha256']==canonical_sha(payload['metadata'])
    assert baseline['source']['context']==payload['metadata']
    destination=tmp_path/'baseline.json'; destination.write_text(json.dumps(baseline))
    assert planner.load_baseline(destination)==baseline
    baseline['points'][0]['mean_iq']=0.
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
    elif damage=='zero': p['trials'][0]['mean_iq']=0.
    elif damage=='nan': p['trials'][0]['mean_iq']=float('nan')
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
    b['points'][0]['mean_iq']=0.
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
    payload=a_payload(); payload['trials'][0]['mean_iq']=98.
    b=planner.export_baseline(payload)
    with pytest.raises(ValueError): planner.build_sweep_plan('accel_interval',[1.,1.5],b)


@pytest.mark.parametrize('key,value', [('speeds',None),('speeds',{}),('speeds',['bad',1.]),('repeats',True)])
def test_malformed_source_configuration_refuses_with_value_error(key,value):
    from aims_calibration import planner
    payload=a_payload(); payload['configuration'][key]=value
    payload['metadata']['configuration_sha256']=canonical_sha(payload['configuration'])
    with pytest.raises(ValueError): planner.export_baseline(payload)


@pytest.mark.parametrize('damage',['missing_signal','motor_only','nan_iq','missing_std'])
def test_baseline_requires_explicit_q_axis_source_without_motor_substitution(damage):
    from aims_calibration.planner import export_baseline
    payload=a_payload()
    if damage=='missing_signal':payload['metadata'].pop('current_signal')
    elif damage=='motor_only':payload['trials'][0].pop('mean_iq')
    elif damage=='nan_iq':payload['trials'][0]['mean_iq']=float('nan')
    else:payload['trials'][0].pop('std_iq')
    with pytest.raises(ValueError):export_baseline(payload)


def test_baseline_schema_two_q_axis_identity_rejects_old_motor_schema():
    from aims_calibration.planner import export_baseline,load_baseline
    baseline=export_baseline(a_payload())
    assert baseline['schema_version']==2 and baseline['current_signal']=='avg_iq'
    assert baseline['points'][0]['mean_iq']==.104
    assert 'mean_current_motor' not in baseline['points'][0]
    baseline['schema_version']=1;baseline.pop('current_signal')
    for point in baseline['points']:
        point['mean_current_motor']=point.pop('mean_iq')
        point['std_current_motor']=point.pop('std_iq')
    baseline['baseline_sha256']=canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    with pytest.raises(ValueError):load_baseline('',raw=json.dumps(baseline))


@pytest.mark.parametrize('location',['baseline','source','result'])
def test_baseline_schema_versions_require_actual_integer_not_equal_float(location):
    from aims_calibration.planner import export_baseline,load_baseline
    payload=a_payload()
    if location=='result':
        payload['schema_version']=1.0
        with pytest.raises(ValueError):export_baseline(payload)
        return
    baseline=export_baseline(payload)
    target=baseline if location=='baseline' else baseline['source']
    target['schema_version']=float(target['schema_version'])
    baseline['baseline_sha256']=canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    with pytest.raises(ValueError):load_baseline('',raw=json.dumps(baseline))


def test_baseline_q_axis_export_is_bound_to_exact_source_file_and_metadata(tmp_path):
    from aims_calibration.planner import export_baseline,load_baseline
    payload=a_payload();source=tmp_path/'A.json'
    source.write_text(json.dumps(payload))
    changed=a_payload();changed['trials'][0]['mean_iq']=.204
    with pytest.raises(ValueError):export_baseline(changed,source)
    baseline=export_baseline(payload,source)
    assert baseline['baseline_sha256']==canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    baseline['source']['context']['current_signal']='current_motor'
    baseline['source']['metadata_sha256']=canonical_sha(baseline['source']['context'])
    baseline['baseline_sha256']=canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    with pytest.raises(ValueError):load_baseline('',raw=json.dumps(baseline))


@pytest.mark.parametrize('scope',['trial','segment'])
def test_baseline_export_rejects_explicit_conflicting_A_current_identity(scope):
    from aims_calibration.planner import export_baseline
    payload=a_payload();trial=payload['trials'][0]
    if scope=='trial':trial['current_signal']='current_motor'
    else:trial['segments']=[dict(current_signal='current_motor')]
    with pytest.raises(ValueError):export_baseline(payload)


def test_baseline_export_accepts_explicit_q_axis_trial_and_segment_identity():
    from aims_calibration.planner import export_baseline
    payload=a_payload()
    for trial in payload['trials']:
        trial['current_signal']='avg_iq'
        trial['segments']=[dict(current_signal='avg_iq')]
    assert export_baseline(payload)['current_signal']=='avg_iq'


def test_polynomial_baseline_preserves_measured_points_and_freezes_formula_currents():
    from aims_calibration import planner
    measured=planner.export_baseline(a_payload())
    baseline=planner.with_polynomial_baseline(measured,[5.77545,3.51367,-.594009,.0451659])
    assert measured['schema_version']==2 and 'planning_model' not in measured
    assert baseline['schema_version']==3 and baseline['points']==measured['points']
    assert baseline['source']==measured['source']
    assert planner.load_baseline('',raw=json.dumps(baseline))==baseline
    b=planner.build_sweep_plan('accel_interval',None,baseline)
    c=planner.build_sweep_plan('decel_current',None,baseline)
    assert len(b)==146 and len(c)==183
    assert b[0].baseline_current==pytest.approx(8.7402769)
    assert b[0].current==pytest.approx(13.7402769)
    assert list(dict.fromkeys(p.start_speed for p in c))==[5.,4.5,4.,3.5,3.,2.5,2.,1.5]
    low=[p for p in c if p.start_speed==1.5]
    assert low[0].baseline_current==pytest.approx(9.8618696625)
    assert low[0].current==pytest.approx(4.8618696625)
    assert low[1].current==0 and low[2].current==-5.
    assert b[-1].current==100.


@pytest.mark.parametrize('coefficients',[[0.,0.,0.,0.],[101.,0.,0.,0.],
    [10.,float('nan'),0.,0.],[10.,float('inf'),0.,0.],[10.,0.,0.],
    # All half-m/s grid values are positive, but the minimum at 1.25 is negative.
    [1.5615,-2.5,1.,0.]])
def test_polynomial_baseline_rejects_invalid_coefficients_and_between_grid_extrema(coefficients):
    from aims_calibration import planner
    with pytest.raises(ValueError):
        planner.with_polynomial_baseline(planner.export_baseline(a_payload()),coefficients)


def test_polynomial_baseline_rejects_hash_tampering_wrong_schema_and_extrapolation():
    from aims_calibration import planner
    baseline=planner.with_polynomial_baseline(planner.export_baseline(a_payload()),[5.77545,3.51367,-.594009,.0451659])
    with pytest.raises(ValueError):planner._polynomial_current(baseline['planning_model'],.5)
    with pytest.raises(ValueError):planner._polynomial_current(baseline['planning_model'],5.5)
    baseline['planning_model']['coefficients'][0]+=1.
    with pytest.raises(ValueError):planner.load_baseline('',raw=json.dumps(baseline))
    baseline['baseline_sha256']=canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    baseline['schema_version']=2
    baseline['baseline_sha256']=canonical_sha({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    with pytest.raises(ValueError):planner.load_baseline('',raw=json.dumps(baseline))
