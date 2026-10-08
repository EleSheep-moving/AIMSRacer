"""Strict diagnostics retain the full evaluator's candidate bound decisions."""
from dataclasses import replace
import time

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc import envelope


def config(**changes):
    return VehicleConfig(profile='synthetic',geometry_verified=True,rear_offset=.15,
                         half_length=.28,half_width=.15,enforce_corridor=False,**changes)


def witness(seed):
    rng=np.random.default_rng(seed)
    cfg=config(longitudinal_envelope_accel=.25+rng.random()*.5,
               longitudinal_envelope_brake=.2+rng.random()*.7,
               lateral_accel_limit=.4+rng.random()*1.2,
               understeer_coefficient=rng.random()*.3,
               steering_tau=.08+rng.random()*.12)
    dt=float(rng.choice([.02,.06,.1,.2]));n=int(rng.integers(2,13))
    initial=np.r_[rng.normal(size=3),rng.uniform(-.1,1.1),0.,rng.uniform(-.5,.5)]
    applied=np.array([rng.uniform(-.7,.7),rng.uniform(-.5,.5),rng.uniform(-1.,1.)])
    controls=np.c_[rng.uniform(-.65,.65,n),rng.uniform(-.5,.5,n),rng.uniform(-.1,1.1,n)]
    if seed%4==0:
        initial[3]=.2;initial[5]=0.;applied[:]=0.;controls[:]=[0.,0.,.2]
    return initial,applied,controls,cfg,dt


def strict_decision(diagnostic,samples,cfg,tolerance):
    return bool(diagnostic['hard_control_violation']<=tolerance and
                diagnostic['speed_bound_violation']<=tolerance and
                np.max(np.abs(samples[:,5])-cfg.steer_limit)<=tolerance and
                diagnostic['initial_candidate_utilization']<=1.+tolerance and
                diagnostic['future_slack_max']<=tolerance)


@pytest.mark.parametrize('seed',range(72))
def test_strict_differential_against_full_candidate_diagnostics(seed):
    initial,applied,controls,cfg,dt=witness(seed)
    full=envelope.evaluate_envelope(initial,applied,controls,cfg,dt)
    diagnostic,samples=envelope.evaluate_strict_envelope(initial,applied,controls,cfg,dt)
    assert isinstance(samples,np.ndarray)
    assert np.array_equal(samples,np.asarray(full['candidate_samples']))
    for field in diagnostic.keys() & full.keys():
        assert diagnostic[field]==pytest.approx(full[field]),field
    assert strict_decision(diagnostic,samples,cfg,1e-4)==strict_decision(full,samples,cfg,1e-4)
    assert diagnostic['braking_comparison_performed'] is False
    assert diagnostic['execution_authorized'] is False
    assert not {'reference_samples','reference_controls','reference_feasible',
                'braking_comparison_satisfied','recovery_acceptable','candidate_samples'} & diagnostic.keys()


def test_strict_retains_start_of_new_acceleration_at_interval_boundary():
    cfg=config(max_speed=2.,lateral_accel_limit=.2,longitudinal_envelope_accel=.5,
               longitudinal_envelope_brake=.3)
    initial=np.array([0.,0.,0.,1.,0.,.1]);applied=np.array([0.,0.,0.])
    controls=np.array([[0.,0.,1.],[-.3,0.,1.],[0.,0.,1.]])
    full=envelope.evaluate_envelope(initial,applied,controls,cfg)
    diagnostic,samples=envelope.evaluate_strict_envelope(initial,applied,controls,cfg)
    boundary=envelope.utilization(samples[5],controls[1,0],cfg)
    assert boundary>1.
    assert diagnostic['future_slack_max']==pytest.approx(full['future_slack_max'])
    assert diagnostic['future_slack_max']>=boundary-1.


@pytest.mark.parametrize('changes',[dict(steer_limit=.05),dict(lateral_accel_limit=.01)])
def test_strict_checks_immutable_initial_physical_steering_and_lateral_envelope(changes):
    cfg=config(**changes);initial=np.array([0.,0.,0.,.5,0.,.2]);applied=np.zeros(3)
    controls=np.tile([0.,0.,.5],(10,1))
    full=envelope.evaluate_envelope(initial,applied,controls,cfg)
    diagnostic,samples=envelope.evaluate_strict_envelope(initial,applied,controls,cfg)
    assert not strict_decision(diagnostic,samples,cfg,1e-4)
    assert diagnostic['initial_candidate_utilization']==pytest.approx(full['initial_candidate_utilization'])
    assert diagnostic['actual_steering_bound_violation']==pytest.approx(
        max(0.,np.max(np.abs(samples[:,5])-cfg.steer_limit)))


def test_strict_preserves_deadline_and_initial_jerk_infeasibility_diagnostics():
    initial,applied,controls,cfg,dt=witness(7)
    cfg=replace(cfg,envelope_recovery_time=.1);applied[0]=3.
    full=envelope.evaluate_envelope(initial,applied,controls,cfg,dt)
    diagnostic,_=envelope.evaluate_strict_envelope(initial,applied,controls,cfg,dt)
    assert diagnostic['initial_actuator_jerk_feasible'] is False
    for key in ('initial_acceleration_interval','recovery_deadline_satisfied',
                'future_violation_duration_s','terminal_utilization','recovery_bounds_satisfied'):
        assert diagnostic[key]==pytest.approx(full[key])


@pytest.mark.parametrize('change', ['initial_shape','applied_shape','controls_shape','empty',
                                 'initial_nan','applied_inf','controls_nan','dt_zero',
                                 'dt_nan','dt_fraction','tolerance_negative','bias_nan'])
def test_strict_invalid_inputs_reject_like_full_evaluator(change):
    initial=np.zeros(6);applied=np.zeros(3);controls=np.zeros((2,3));kwargs={}
    if change=='initial_shape':initial=np.zeros(5)
    elif change=='applied_shape':applied=np.zeros(2)
    elif change=='controls_shape':controls=np.zeros((2,2))
    elif change=='empty':controls=np.zeros((0,3))
    elif change=='initial_nan':initial[0]=np.nan
    elif change=='applied_inf':applied[0]=np.inf
    elif change=='controls_nan':controls[0,0]=np.nan
    elif change=='dt_zero':kwargs['dt']=0.
    elif change=='dt_nan':kwargs['dt']=np.nan
    elif change=='dt_fraction':kwargs['dt']=.03
    elif change=='tolerance_negative':kwargs['tolerance']=-1.
    elif change=='bias_nan':kwargs['steering_bias']=np.nan
    for function in (envelope.evaluate_envelope,envelope.evaluate_strict_envelope):
        with pytest.raises(ValueError):function(initial,applied,controls,config(),**kwargs)


def test_strict_helper_rejects_soft_configuration():
    with pytest.raises(ValueError,match='strict|soft'):
        envelope.evaluate_strict_envelope(np.zeros(6),np.zeros(3),np.zeros((2,3)),
                                         config(envelope_soft_enabled=True))


def test_strict_helper_rolls_only_the_candidate_once(monkeypatch):
    calls=[];original=envelope.independent_rollout
    def recorded(initial,applied,controls,*args,**kwargs):
        calls.append(len(controls))
        return original(initial,applied,controls,*args,**kwargs)
    monkeypatch.setattr(envelope,'independent_rollout',recorded)
    initial,applied,controls,cfg,dt=witness(0)
    envelope.evaluate_strict_envelope(initial,applied,controls,cfg,dt)
    assert calls==[len(controls)]


def test_soft_validator_keeps_the_full_recovery_comparison(monkeypatch):
    from aims_mpcc import validation
    def forbidden(*args,**kwargs):raise AssertionError('soft recovery used strict helper')
    monkeypatch.setattr(validation,'evaluate_strict_envelope',forbidden)
    cfg=config(envelope_soft_enabled=True)
    initial=np.array([0.,0.,0.,.2,0.,0.]);applied=np.zeros(3)
    controls=np.tile([0.,0.,.2],(2,1));states=envelope.independent_rollout(initial,applied,controls,cfg)[::5]
    result=validation.validate_candidate(dict(states=states.tolist(),controls=controls.tolist(),
                                             validation_applied=applied.tolist()),cfg)
    assert result['accepted']
    assert 'braking_comparison_satisfied' in result['envelope']
    assert 'braking_comparison_performed' not in result['envelope']


def test_strict_validator_and_backend_skip_full_braking_comparator(monkeypatch):
    from aims_mpcc import validation,backend_models
    from aims_mpcc.path import ReferencePath
    def forbidden(*args,**kwargs):raise AssertionError('strict request invoked braking comparator')
    monkeypatch.setattr(validation,'evaluate_envelope',forbidden)
    monkeypatch.setattr(backend_models,'evaluate_envelope',forbidden)
    cfg=config();initial=np.array([0.,0.,0.,.2,0.,0.]);applied=np.zeros(3)
    controls=np.tile([0.,0.,.2],(2,1));states=envelope.independent_rollout(initial,applied,controls,cfg)[::5]
    candidate=dict(states=states.tolist(),controls=controls.tolist(),validation_applied=applied.tolist())
    checked=validation.validate_candidate(candidate,cfg)
    assert checked['accepted']
    assert checked['envelope']['braking_comparison_performed'] is False
    theta=np.arange(32)*2*np.pi/32;path=ReferencePath(3*np.c_[np.cos(theta),np.sin(theta)],1.,1.)
    backend=backend_models.NumericalBackend(path,cfg,horizon=2)
    result=dict(candidate,success=True,diagnostics={});now=time.perf_counter()
    finished=backend.finish(result,initial,applied,np.full(3,.2),np.zeros(3),now,now,now)
    assert finished['diagnostics']['envelope']['braking_comparison_performed'] is False
    assert 'candidate_samples' not in finished['diagnostics']['envelope']
