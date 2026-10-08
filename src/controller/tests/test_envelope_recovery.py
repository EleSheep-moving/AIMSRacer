"""Numerical envelope repair tests; no ROS graph or driving output."""
from dataclasses import replace
import importlib.util
import math

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig


def envelope():
    assert importlib.util.find_spec('aims_mpcc.envelope') is not None, 'independent envelope validator is missing'
    from aims_mpcc import envelope as module
    return module


def config(**kwargs):
    return VehicleConfig(rear_offset=0., front_extent=.52, rear_extent=.10,
                         half_width=.16, geometry_verified=True, max_speed=1.5,
                         steer_limit=.45, steer_rate=2., steering_tau=.08,
                         enforce_corridor=False, **kwargs)


def test_defaults_retain_baseline_and_split_actuator_from_operating_halfaxes():
    c = config()
    assert hasattr(c, 'envelope_soft_enabled'), 'explicit envelope experiment configuration missing'
    assert not c.envelope_soft_enabled
    assert (c.accel_limit, c.brake_limit, c.jerk_limit) == (.5, .5, 1.)
    assert c.envelope_halfaxes() == (.5, .5, 1.)
    wider = replace(c, longitudinal_envelope_accel=1., longitudinal_envelope_brake=.8)
    assert wider.envelope_halfaxes() == (1., .8, 1.)
    assert (wider.accel_limit, wider.brake_limit) == (.5, .5)


@pytest.mark.parametrize('v,delta,previous,jerk,expected', [
    (.8143,.4478,.3834,1.,1.1041),
    (.8392,.4395,.3072,1.,1.0178),
    (.8762,.4175,.3047,1.,1.0621),
    (.9185,.4373,-.1761,2.,1.2006),
])
def test_audit_initial_conflicts_are_diagnosed(v, delta, previous, jerk, expected):
    result = envelope().initial_envelope_diagnostic(
        np.array([0.,0.,0.,v,0.,delta]), np.array([previous,delta,0.]), config(jerk_limit=jerk), .1)
    assert result['minimum_initial_utilization'] == pytest.approx(expected, abs=.001)
    assert result['initial_unavoidable_violation'] > 0.


def test_invalid_recovery_limits_fail_validation():
    c = config()
    assert hasattr(c, 'envelope_slack_limit'), 'finite recovery bounds missing'
    for changes in ({'envelope_slack_limit': math.inf}, {'envelope_recovery_time': 0.},
                    {'recovery_jerk_limit': .5, 'recovery_jerk_enabled': True,
                     'envelope_soft_enabled': True}, {'contour_scale': 0.}):
        with pytest.raises(ValueError):
            replace(c, **changes).validate()


def test_independent_rollout_respects_identical_control_prefix_and_hard_limits():
    module = envelope()
    c = config()
    initial = np.array([0.,0.,0.,.9185,0.,.4373])
    applied = np.array([-.1761,.4373,0.])
    controls = np.tile([0.,.4373,.5], (10,1))
    result = module.evaluate_envelope(initial, applied, controls, c)
    assert result['reference_control_prefix_intervals'] == 1
    assert result['reference_controls'][0] == pytest.approx(controls[0])
    assert result['reference_controls'][1][0] == pytest.approx(-.1)
    assert result['reference_braking_begins_s'] == pytest.approx(.1)
    assert not result['recovery_acceptable']
    assert not result['execution_authorized']
    controls[0,0] = -.6
    invalid = module.evaluate_envelope(initial, applied, controls, c)
    assert invalid['hard_control_violation'] >= .1 - 1e-12
    assert not invalid['recovery_acceptable']


def test_lower_total_slack_does_not_accept_slower_recovery_than_braking():
    module = envelope()
    c = config()
    initial = np.array([0.,0.,0.,.9185,0.,.4373])
    applied = np.array([0.,.4373,0.])
    # A short coasting horizon has lower total E excess while preserving ay.
    controls = np.tile([0.,.4373,.5], (4,1))
    result = module.evaluate_envelope(initial, applied, controls, c)
    assert result['candidate_excess_integral'] < result['reference_excess_integral']
    assert result['candidate_speed_above_reference_max'] > 0.
    assert result['candidate_lateral_excess_integral'] > result['reference_lateral_excess_integral']
    assert not result['braking_comparison_satisfied']
    assert not result['recovery_acceptable']


def test_finite_recovery_jerk_only_applies_to_initial_conflict():
    module = envelope()
    c = config(envelope_soft_enabled=True, recovery_jerk_enabled=True)
    ordinary = np.array([0.,0.,0.,.2,0.,0.])
    conflict = np.array([0.,0.,0.,.9185,0.,.4373])
    applied = np.array([0.,.4373,0.])
    assert module.effective_jerk_limit(ordinary, applied, c, .1) == 1.
    assert module.effective_jerk_limit(conflict, applied, c, .1) == 2.


def test_soft_solver_omits_only_fixed_initial_envelope_and_bounds_future_slack():
    envelope()
    from aims_mpcc.path import ReferencePath
    from aims_mpcc.solver import MPCCSolver
    angles = np.arange(32)*2*math.pi/32
    path = ReferencePath(np.c_[2*np.cos(angles),2*np.sin(angles)], 1.,1.)
    c = config(envelope_soft_enabled=True, solver_max_iterations=100)
    solver = MPCCSolver(path,c,horizon=10)
    state = dict(x=2.,y=0.,yaw=math.pi/2,speed=.9185,steering=.4373)
    previous = dict(acceleration=-.1761,steering=.4373,steering_rate=0.)
    result = solver.solve(state,previous, [.5]*11)
    assert result['success'], result['status']
    assert 'envelope' in result['diagnostics']
    d = result['diagnostics']['envelope']
    assert d['initial_unavoidable_violation'] > .19
    assert d['future_slack_max'] <= .5 + 1e-4
    assert d['future_violation_duration_s'] <= .6 + 1e-6
    assert not d['execution_authorized']
    assert not any(b['group']=='acceleration_ellipse' and b['interval']==0 and b['substep']==0
                   for b in solver.constraint_blocks)


def test_strict_jerk_above_optional_recovery_default_remains_valid():
    assert config(jerk_limit=3.).validate().jerk_limit == 3.


def test_rollout_rejects_nonfinite_dt_and_negative_tolerance():
    module = envelope()
    x, a, u = np.zeros(6),np.zeros(3),np.zeros((2,3))
    for kwargs in ({'dt': float('nan')}, {'tolerance': -1.}):
        with pytest.raises(ValueError, match='finite|nonnegative'):
            module.evaluate_envelope(x,a,u,config(),**kwargs)


def test_high_lateral_recovery_with_braking_is_numerically_acceptable():
    module = envelope()
    c = config(envelope_soft_enabled=True)
    x = np.array([0.,0.,0.,.9185,0.,.4373])
    applied = np.array([0.,.4373,0.])
    rates = np.array([-.2,-.4,-.6,-.8,-.6,-.4,-.2,0.,0.,0.])
    controls = np.c_[np.zeros(10), .4373+np.cumsum(rates)*.1, np.full(10,.5)]
    controls[0,0] = -.1
    # The independently computed bounded reference itself is one admissible
    # recovery candidate. A subsequent call must verify it, not trust a label.
    reference = module.evaluate_envelope(x,applied,controls,c)['reference_controls']
    result = module.evaluate_envelope(x,applied,reference,c)
    assert result['initial_lateral_violation'] > .19
    assert min(row[0] for row in reference) < -.2
    assert result['hard_control_violation'] <= 1e-8
    assert result['recovery_acceptable'], result
    assert not result['execution_authorized']


def test_braking_reference_never_silently_violates_low_speed_jerk():
    module = envelope()
    c = config()
    x = np.array([0.,0.,0.,.08,0.,0.])
    result = module.evaluate_envelope(x,np.zeros(3),np.zeros((10,3)),c)
    assert 'reference_feasible' in result
    if result['reference_feasible']:
        reference = np.asarray(result['reference_controls'])
        assert np.max(np.abs(np.diff(np.r_[0.,reference[:,0]]))) <= .1+1e-8
        assert np.min(np.asarray(result['reference_samples'])[:,3]) >= -1e-8
    else:
        assert not result['recovery_acceptable']


def recovery_witness(module):
    c = config(envelope_soft_enabled=True)
    x = np.array([0.,0.,0.,.9185,0.,.4373])
    applied = np.array([0.,.4373,0.])
    rates = np.array([-.2,-.4,-.6,-.8,-.6,-.4,-.2,0.,0.,0.])
    controls = np.c_[np.zeros(10),.4373+np.cumsum(rates)*.1,np.full(10,.5)]
    controls[0,0] = -.1
    return x,applied,np.asarray(module.evaluate_envelope(x,applied,controls,c)['reference_controls']),c


def test_recovery_deadline_sample_must_already_be_strictly_inside_envelope():
    module = envelope()
    x,a,u,c = recovery_witness(module)
    result = module.evaluate_envelope(x,a,u,replace(c,envelope_recovery_time=.2))
    assert result['future_violation_duration_s'] == pytest.approx(.2)
    assert result['terminal_utilization'] < 1.
    assert not result['recovery_bounds_satisfied']
    assert not result['recovery_acceptable']


def test_default_hard_profile_cannot_accept_soft_recovery_witness():
    module = envelope()
    x,a,u,c = recovery_witness(module)
    result = module.evaluate_envelope(x,a,u,replace(c,envelope_soft_enabled=False))
    assert not result['recovery_bounds_satisfied']
    assert not result['recovery_acceptable']


def test_hard_solver_uses_only_cheap_initial_diagnostic(monkeypatch):
    from aims_mpcc.path import ReferencePath
    from aims_mpcc import solver as solver_module
    calls = []
    def forbidden_rollout(*args, **kwargs):
        calls.append(True)
        raise AssertionError('baseline solver must not duplicate the full independent validator')
    monkeypatch.setattr(solver_module, 'evaluate_envelope', forbidden_rollout)
    angles = np.arange(32)*2*math.pi/32
    path = ReferencePath(np.c_[2*np.cos(angles),2*np.sin(angles)],1.,1.)
    solver = solver_module.MPCCSolver(path,config(),horizon=2)
    result = solver.solve(dict(x=2.,y=0.,yaw=math.pi/2,speed=.2,steering=0.),
                          dict(acceleration=0.,steering=0.,steering_rate=0.))
    assert result['success'], result['status']
    assert calls == []
    diagnostic = result['diagnostics']['envelope']
    assert diagnostic['minimum_initial_utilization'] == 0.
    assert not diagnostic['execution_authorized']
    assert diagnostic['diagnostic_scope'] == 'initial_state_only'
    assert 'candidate_samples' not in diagnostic


def test_soft_solver_compact_diagnostics_omit_full_rollout_traces():
    from aims_mpcc.path import ReferencePath
    from aims_mpcc.solver import MPCCSolver
    angles = np.arange(32)*2*math.pi/32
    path = ReferencePath(np.c_[2*np.cos(angles),2*np.sin(angles)],1.,1.)
    solver = MPCCSolver(path,config(envelope_soft_enabled=True),horizon=2)
    result = solver.solve(dict(x=2.,y=0.,yaw=math.pi/2,speed=.2,steering=0.),
                          dict(acceleration=0.,steering=0.,steering_rate=0.))
    assert result['success'], result['status']
    diagnostic = result['diagnostics']['envelope']
    assert diagnostic['future_slack_max'] <= 1e-4
    assert not {'candidate_samples','reference_samples','reference_controls'} & diagnostic.keys()


@pytest.mark.parametrize('index',range(7))
def test_captured_recovery_compares_terminal_excess_within_physical_bounds(index):
    import json
    from pathlib import Path
    cases=json.loads((Path(__file__).parent/'fixtures/nx_recovery_controls.json').read_text())
    case=cases[index]
    result=envelope().evaluate_envelope(np.asarray(case['initial']),np.asarray(case['applied']),
        np.asarray(case['controls']),VehicleConfig(**case['config']),dt=case['dt'])
    assert result['recovery_acceptable']==case['expected_acceptable'],case['id']
    if case['expected_acceptable']:
        assert result['recovery_bounds_satisfied'] and result['recovery_deadline_satisfied']
        assert result['terminal_utilization']<=1.+1e-4
        assert result['candidate_terminal_lateral_excess']==0.
        assert result['reference_terminal_lateral_excess']==0.
    else:
        assert result['braking_comparison_failures']
    assert not result['execution_authorized']
