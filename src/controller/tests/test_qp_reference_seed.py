"""Refresh longitudinal targets only for a stopped-reference cache transition."""
import copy
import json

import numpy as np
import pytest

from aims_mpcc.backend_models import NumericalBackend
from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import evaluate_strict_envelope,jerk_limits
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver
from test_backend_geometry import legacy_seed


def fixture(cruise=1.):
    angles=np.linspace(0.,2*np.pi,40,endpoint=False)
    path=ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,
        half_width=.15,geometry_verified=True,cruise_speed=cruise,max_speed=1.5,
        accel_limit=.5,brake_limit=.5,jerk_limit=1.)
    steering=float(np.arctan(cfg.wheelbase/3.))
    state=dict(x=3.,y=0.,yaw=np.pi/2,speed=.5,steering=steering)
    applied=dict(acceleration=0.,steering=steering,steering_rate=0.)
    return QPSolver(path,cfg,10),state,applied


def stopped_success(solver,state,applied,refs=None):
    refs=np.zeros(solver.n+1) if refs is None else refs
    result=solver.solve(state,applied,refs)
    assert result['success'],result
    return result


@pytest.mark.parametrize('history',['cold','shifted','expired'])
def test_common_default_and_explicit_false_preserve_original_seed_bits(history):
    solver,state,applied=fixture()
    initial=np.array([state['x'],state['y'],state['yaw'],state['speed'],0.,state['steering']])
    prefix=np.array([applied[k] for k in ('acceleration','steering','steering_rate')])
    refs=np.ones(11)
    if history!='cold':
        solver.previous=dict(controls=np.tile([-.3,state['steering'],.5],(10,1)))
        solver.previous_elapsed=.1 if history=='shifted' else 1.
    expected=legacy_seed(solver,initial,prefix,refs)
    default=NumericalBackend.seed(solver,initial,prefix,refs)
    explicit=NumericalBackend.seed(solver,initial,prefix,refs,refresh_longitudinal=False)
    for k in (0,1):
        assert np.array_equal(default[k],expected[k])
        assert np.array_equal(explicit[k],expected[k])


def test_longitudinal_refresh_retains_actual_prefix_and_exact_steering_dynamics():
    solver,state,applied=fixture()
    stopped_success(solver,state,applied)
    initial=np.array([3.,0.,np.pi/2,.36,0.,state['steering']])
    prefix=np.array([-.3,state['steering'],0.])
    refs=np.ones(11)
    solver.previous_elapsed=.1
    before=(initial.copy(),prefix.copy(),copy.deepcopy(solver.previous))
    old=NumericalBackend.seed(solver,initial,prefix,refs)
    new=solver.seed(initial,prefix,refs)
    assert solver._longitudinal_refresh is True
    assert old[0][-1,3]<initial[3]
    assert new[0][-1,3]>initial[3]+.2
    assert np.array_equal(new[0][0],initial)
    assert np.array_equal(new[1][:,1],old[1][:,1])
    assert np.array_equal(new[0][:,5],old[0][:,5])
    assert np.array_equal(initial,before[0]) and np.array_equal(prefix,before[1])
    assert np.array_equal(solver.previous['controls'],before[2]['controls'])
    assert np.array_equal(solver.previous['speed_refs'],before[2]['speed_refs'])
    acceleration=new[1][:,0]
    assert np.max(acceleration)<=solver.config.accel_limit
    assert np.min(acceleration)>=-solver.config.brake_limit
    limits=jerk_limits(initial,prefix,solver.config,solver.dt,solver.n)
    assert np.max(np.abs(np.diff(np.r_[prefix[0],acceleration]))-limits*solver.dt)<1e-12


def test_successful_qp_cache_copies_complete_references_and_selects_refresh():
    solver,state,applied=fixture();refs=np.zeros(11)
    stopped_success(solver,state,applied,refs)
    assert 'speed_refs' in solver.previous,'cache is missing its successful reference provenance'
    assert np.array_equal(solver.previous['speed_refs'],refs)
    refs[:]=.2
    assert np.array_equal(solver.previous['speed_refs'],np.zeros(11))
    state=dict(state,speed=.36);applied=dict(applied,acceleration=-.3)
    result=solver.solve(state,applied,np.ones(11),elapsed=.1)
    diag=result['diagnostics']
    assert diag['warm_start_longitudinal_refresh'] is True
    assert diag['warm_start_longitudinal_refresh_reason']=='stopped_reference_to_cruise'
    assert diag['warm_start_longitudinal_source']=='current_reference'
    assert diag['warm_start_source']=='last_success'
    assert diag['warm_start_age_s']==pytest.approx(.1)
    assert diag['warm_start_shift_steps']==1
    json.dumps(diag,allow_nan=False)
    assert type(diag['warm_start_longitudinal_refresh']) is bool


@pytest.mark.parametrize('case',['first_zero_future_positive','first_not_cruise',
    'previous_preview_positive','previous_first_positive','expired','missing_provenance','zero_cruise'])
def test_narrow_policy_does_not_change_other_shifted_or_feedforward_seeds(case):
    solver,state,applied=fixture(cruise=0. if case=='zero_cruise' else 1.)
    old_refs=np.zeros(11)
    if case=='previous_preview_positive':old_refs[5]=np.nextafter(0.,1.)
    if case=='previous_first_positive':old_refs[0]=.1
    stopped_success(solver,state,applied,old_refs)
    solver.previous_elapsed=1. if case=='expired' else .1
    if case=='missing_provenance':solver.previous.pop('speed_refs',None)
    refs=np.ones(11)
    if case=='first_zero_future_positive':refs[0]=0.
    if case=='first_not_cruise':refs[0]=np.nextafter(1.,0.)
    if case=='zero_cruise':refs[:]=0.
    initial=np.array([3.,0.,np.pi/2,.36,0.,state['steering']]);prefix=np.array([-.3,state['steering'],0.])
    expected=NumericalBackend.seed(solver,initial,prefix,refs)
    actual=solver.seed(initial,prefix,refs)
    assert np.array_equal(actual[0],expected[0])
    assert np.array_equal(actual[1],expected[1])


def test_failed_request_retains_success_refs_and_age_then_reset_removes_eligibility(monkeypatch):
    solver,state,applied=fixture();stopped_success(solver,state,applied)
    retained=solver.previous;controls=retained['controls'].copy()
    assert 'speed_refs' in retained
    refs=retained['speed_refs'].copy()
    native_solve=solver._native.solve
    def max_iterations(**kwargs):
        result=native_solve(**kwargs)
        result.info.status_val=7;result.info.status='maximum iterations reached'
        return result
    monkeypatch.setattr(solver._native,'solve',max_iterations)
    failed=solver.solve(state,applied,np.ones(11),elapsed=.1)
    assert not failed['success']
    assert failed['diagnostics']['osqp_status_val']==7
    assert failed['diagnostics']['native_candidate_status_eligible'] is False
    assert failed['diagnostics']['warm_start_longitudinal_refresh'] is True
    assert solver.previous is retained
    assert np.array_equal(retained['controls'],controls)
    assert np.array_equal(retained['speed_refs'],refs)
    monkeypatch.setattr(solver._native,'solve',native_solve)
    again=solver.solve(state,applied,np.ones(11),elapsed=.1)
    assert again['diagnostics']['warm_start_age_s']==pytest.approx(.2)
    assert again['diagnostics']['warm_start_shift_steps']==2
    assert again['diagnostics']['warm_start_longitudinal_refresh'] is True
    solver.reset()
    cold=solver.solve(state,applied,np.ones(11))
    assert cold['diagnostics']['warm_start_longitudinal_refresh'] is False
    assert cold['diagnostics']['warm_start_source']=='feedforward'


def test_refresh_seed_never_clamps_negative_physical_initial_speed():
    solver,state,applied=fixture()
    initial=np.array([3.,0.,np.pi/2,-.05,0.,state['steering']]);prefix=np.array([0.,state['steering'],0.])
    solver.previous=dict(controls=np.tile([-.3,state['steering'],.2],(10,1)))
    seed=NumericalBackend.seed(solver,initial,prefix,np.ones(11),refresh_longitudinal=True)
    assert np.array_equal(seed[0][0],initial)
    assert initial[3]==-.05
    envelope,_=evaluate_strict_envelope(initial,prefix,seed[1],solver.config,solver.dt)
    assert envelope['speed_bound_violation']>=.05


def test_success_reference_copy_is_included_in_solver_diagnostics_phase(monkeypatch):
    import aims_mpcc.qp_backend as qp_module
    solver,_,_=fixture();refs=np.zeros(11);events=[]
    original=np.asarray
    class MeasuredCopy(np.ndarray):
        def copy(self,*args,**kwargs):
            events.append('reference_copy')
            return super().copy(*args,**kwargs)
    def asarray(value,*args,**kwargs):
        array=original(value,*args,**kwargs)
        return array.view(MeasuredCopy) if value is refs else array
    def clock():
        return 12. if 'reference_copy' in events else 10.
    result=dict(success=True,states=np.zeros((11,6)).tolist(),controls=np.zeros((10,3)).tolist(),
                diagnostics={'envelope':{}})
    with monkeypatch.context() as patched:
        patched.setattr(qp_module.np,'asarray',asarray)
        patched.setattr(qp_module.time,'perf_counter',clock)
        result=solver.finish(result,np.zeros(6),np.zeros(3),refs,np.zeros(3),0.,2.,3.)
    assert events==['reference_copy']
    assert result['solve_time_s']==12.
    assert result['diagnostics']['preparation_time_s']==2.
    assert result['diagnostics']['optimizer_time_s']==1.
    assert result['diagnostics']['diagnostics_time_s']==9.
    assert np.array_equal(solver.previous['speed_refs'],refs)
