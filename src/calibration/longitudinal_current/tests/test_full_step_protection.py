"""Empirical response support must cover a whole fixed-current step."""
import json
from types import SimpleNamespace

import pytest

from aims_calibration.planner import TrialPlan
from aims_calibration.protection import ProtectionSettings, PulseProtection, load_response_limits


def row(stage, speed, current, bound):
    return dict(stage=stage, start_speed=speed, current=current, acceleration_bound=bound)


def load(rows, **kwargs):
    raw=json.dumps(dict(schema_version=1, current_signal='avg_iq', baseline_sha256='baseline',
        source_events_sha256='events', limits=rows)).encode()
    return load_response_limits('unused', 'baseline', raw=raw, **kwargs)[0]


def accel(current=20., start=1., end=5.):
    return TrialPlan('accel_interval', start, end, current)


def decel(current=-20., start=5., end=1.):
    kind='positive_deceleration' if current>0 else 'coast' if current==0 else 'braking'
    return TrialPlan('decel_current', start, end, current, kind=kind, baseline_current=20.)


def coverage(stage='accel_interval', current=20.):
    speeds=[1.+.5*i for i in range(8)] if stage=='accel_interval' else [1.5+.5*i for i in range(8)]
    return [row(stage,s,current,2.+i) for i,s in enumerate(speeds)]


def observation(velocity=1., acceleration=999.):
    return SimpleNamespace(velocity=velocity, wheel_stamp=1., acceleration=acceleration,
        acceleration_quality='good')


def test_full_path_bound_includes_later_larger_speed_cell_at_preflight_and_runtime():
    lookup=load(coverage(), full_range=True)
    item=accel()
    assert lookup(item)==9.
    assert lookup(item, 3.7)==9.
    p=PulseProtection(lambda v:9., lookup, ProtectionSettings(), full_range=True)
    prediction=p.prediction(item, 1., observation())
    assert prediction['response_limit']==9.
    assert prediction['predicted_velocity']==pytest.approx(2.26)
    assert prediction['response_limit_provenance']['source_events_sha256']=='events'
    assert len(prediction['response_limit_provenance']['speed_cells'])==8


@pytest.mark.parametrize('missing', range(8))
def test_any_missing_half_mps_speed_cell_rejects_entire_step(missing):
    rows=coverage();rows.pop(missing)
    lookup=load(rows, full_range=True)
    with pytest.raises(ValueError, match='response_limit_unavailable'):
        lookup(accel())


@pytest.mark.parametrize('stage,command,reference,allowed', [
    ('accel_interval',20.,25.,True), ('accel_interval',20.,19.,False),
    ('accel_interval',20.,25.01,False),
    ('decel_current',10.,5.,True), ('decel_current',10.,11.,False),
    ('decel_current',3.,0.,False), ('decel_current',3.,-1.,False),
    ('decel_current',0.,0.,True), ('decel_current',0.,-1.,False),
    ('decel_current',-20.,-25.,True), ('decel_current',-20.,-19.,False),
    ('decel_current',-20.,-25.01,False), ('decel_current',-100.,-95.,False),
])
def test_only_same_signed_stronger_neighbor_within_five_amps_is_supported(stage,command,reference,allowed):
    lookup=load(coverage(stage, reference), full_range=True)
    item=accel(command) if stage=='accel_interval' else decel(command)
    if allowed:assert lookup(item)==9.
    else:
        with pytest.raises(ValueError, match='response_limit_unavailable'):lookup(item)


def test_envelope_maximizes_all_eligible_reference_rows_in_each_cell():
    rows=coverage()+[row('accel_interval', 2., 24., 30.)]
    assert load(rows, full_range=True)(accel())==30.


def test_old_braking_start_to_one_trial_cannot_claim_unobserved_lower_speed_support():
    lookup=load([row('decel_current', 5., -20., 10.)], full_range=True)
    with pytest.raises(ValueError, match='response_limit_unavailable'):lookup(decel())
    assert lookup(decel(start=5., end=4.5))==10.


def test_runtime_velocity_outside_preflight_range_requires_additional_support():
    lookup=load([row('accel_interval', 2., 20., 2.)], full_range=True)
    item=accel(start=2., end=2.5)
    assert lookup(item)==2.
    with pytest.raises(ValueError, match='response_limit_unavailable'):lookup(item, 1.9)
    with pytest.raises(ValueError, match='response_limit_unavailable'):lookup(item, 5.01)


def test_derivative_and_source_age_can_only_raise_empirical_prediction():
    p=PulseProtection(lambda v:9., load(coverage(), full_range=True), ProtectionSettings(), full_range=True)
    p.observe(.98, .6);p.observe(1., 1.)
    result=p.prediction(accel(),1.02,observation(acceleration=-999.))
    assert result['directional_rate']==pytest.approx(20.)
    assert result['horizon_s']==pytest.approx(.16)
    assert result['predicted_velocity']==pytest.approx(4.2)


def test_deceleration_whole_path_bound_stays_signed_and_uses_larger_observed_derivative():
    p=PulseProtection(lambda v:9., load(coverage('decel_current', -20.), full_range=True),
        ProtectionSettings(), full_range=True)
    result=p.prediction(decel(),1.,observation(velocity=4.))
    assert result['predicted_velocity']==pytest.approx(2.74)
    p.observe(.98,4.4);p.observe(1.,4.)
    result=p.prediction(decel(),1.,observation(velocity=4.))
    assert result['response_limit']==9.
    assert result['directional_rate']==pytest.approx(20.)
    assert result['predicted_velocity']==pytest.approx(1.2)


def test_partial_bin_requires_its_nominal_directional_cell():
    lookup=load([row('accel_interval', 2., 20., 2.)], full_range=True)
    assert lookup(accel(start=2.1,end=2.4))==2.
    with pytest.raises(ValueError,match='response_limit_unavailable'):
        lookup(accel(start=2.1,end=2.6))


def test_mean_model_and_observer_acceleration_cannot_replace_missing_empirical_support():
    p=PulseProtection(lambda v:9., load([row('accel_interval', 1., 20., 2.)], full_range=True),
        ProtectionSettings(), full_range=True)
    with pytest.raises(ValueError, match='response_limit_unavailable'):
        p.prediction(accel(),1.,observation(acceleration=.01))


def test_default_lookup_preserves_exact_legacy_matching_and_baseline_binding():
    rows=[row('accel_interval',1.,20.,2.)]
    assert load(rows)(accel())==2.
    with pytest.raises(ValueError,match='response_limit_unavailable'):load(rows)(accel(19.))
    raw=json.dumps(dict(schema_version=1,current_signal='avg_iq',baseline_sha256='wrong',limits=rows)).encode()
    with pytest.raises(ValueError,match='baseline/schema mismatch'):
        load_response_limits('unused','baseline',raw=raw,full_range=True)
@pytest.mark.parametrize('payload',[[],None,{'schema_version':1,'current_signal':'avg_iq','baseline_sha256':'A','limits':[None]}])
def test_malformed_guard_json_is_refused_as_value_error(payload):
    import json
    with pytest.raises(ValueError):
        load_response_limits('', 'A', raw=json.dumps(payload).encode(), full_range=True)
