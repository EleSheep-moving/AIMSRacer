"""Batch diagnostics must match the frozen scalar path, including invalid inputs."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def scalar_violations(solver, states, controls, params, initial, upper, terminal_upper):
    """Verbatim algorithm from acados_backend.py at frozen commit 9eb309d."""
    violations = {}
    for k in range(solver.n):
        value = np.asarray(solver._constraints(states[k], controls[k], params[k])).ravel()
        errors = np.maximum.reduce((solver._ocp.constraints.lh-value, value-upper, np.zeros(len(value))))
        for group, error in zip(solver._groups, errors):
            violations[group] = max(violations.get(group, 0.), float(error))
        dynamics = float(np.max(np.abs(states[k+1]-np.asarray(solver._transition(states[k], controls[k])).ravel())))
        violations['dynamics'] = max(violations.get('dynamics', 0.), dynamics)
    value = np.asarray(solver._terminal_constraints(states[-1], params[-1])).ravel()
    violations['terminal_constraints'] = float(np.max(np.maximum.reduce((solver._ocp.constraints.lh_e-value,
        value-terminal_upper, np.zeros(len(value))))))
    violations['input_bounds'] = max(0., float(np.max(solver._ocp.constraints.lbu-controls)), float(np.max(controls-solver._ocp.constraints.ubu)))
    violations['speed_bounds'] = max(0., float(np.max(-states[:, 3])), float(np.max(states[:, 3]-solver.config.max_speed)))
    violations['initial_state'] = float(np.max(np.abs(states[0]-initial)))
    return violations


def symbolic_solver(soft=False, corridor=False, horizon=10):
    pytest.importorskip('acados_template')
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.backend_models import NumericalBackend
    angles = np.linspace(0., 2*np.pi, 40, endpoint=False)
    path = ReferencePath(3*np.c_[np.cos(angles), np.sin(angles)], 1., 1., 'odom')
    cfg = VehicleConfig(profile='synthetic', rear_offset=.15, half_length=.28,
                        half_width=.15, geometry_verified=True, enforce_corridor=corridor,
                        envelope_soft_enabled=soft)
    solver = AcadosSolver.__new__(AcadosSolver)
    NumericalBackend.__init__(solver, path, cfg, horizon)
    solver.model_name = 'diagnostics_fixture'
    solver._expected_lib_path = None
    solver._ocp = solver._build_ocp()
    return solver


def trajectory(solver, rng):
    states = rng.uniform(-.5, .5, (solver.n+1, 9))
    states[:, 3] += .8
    controls = rng.uniform(-.8, .8, (solver.n, 4 if solver.config.envelope_soft_enabled else 3))
    params = rng.uniform(-.5, .5, (solver.n+1, 10))
    params[:, 5] = 1.
    params[:, 7] = solver.config.jerk_limit
    params[:, 8] = np.maximum(solver.config.envelope_recovery_time-np.arange(solver.n+1)*solver.dt, 0.)
    params[:, 9] = 0.
    params[0, 9] = 1.
    return states, controls, params, states[0].copy()


@pytest.mark.parametrize('soft,corridor', [(False, False), (False, True), (True, False), (True, True)])
@pytest.mark.parametrize('horizon', [1, 10, 15])
def test_batch_constraint_dictionary_matches_frozen_scalar_for_invalid_trajectories(soft, corridor, horizon):
    solver = symbolic_solver(soft, corridor, horizon)
    rng = np.random.default_rng(1381)
    for i in range(30):
        states, controls, params, initial = trajectory(solver, rng)
        if i % 5 == 0:
            # Consistent transitions exercise the exact zero-dynamics case.
            for k in range(solver.n):
                states[k+1] = np.asarray(solver._transition(states[k], controls[k])).ravel()
        elif i % 5 == 1:
            states[0, 0] += .4
        elif i % 5 == 2:
            states[solver.n//2, 3] = -1.
            controls[0, 0] = 2*solver.config.accel_limit
        elif i % 5 == 3:
            states[solver.n//2, 0] = np.nan
        else:
            controls[0, 0] = np.nan
        for upper, terminal_upper in ((solver._ocp.constraints.uh, solver._ocp.constraints.uh_e),
                                      (solver._physical_upper, solver._physical_terminal_upper)):
            expected = scalar_violations(solver, states, controls, params, initial, upper, terminal_upper)
            actual = solver._trajectory_violations(states, controls, params, initial, upper, terminal_upper)
            assert list(actual) == list(expected)
            np.testing.assert_allclose(list(actual.values()), list(expected.values()), rtol=0., atol=0., equal_nan=True)


class Counted:
    def __init__(self, function):
        self.function = function
        self.calls = 0

    def __call__(self, *args):
        self.calls += 1
        return self.function(*args)


@pytest.mark.parametrize('soft,corridor', [(False, False), (False, True), (True, False), (True, True)])
def test_diagnostics_dispatches_once_per_batched_function(soft, corridor):
    solver = symbolic_solver(soft, corridor)
    states, controls, params, initial = trajectory(solver, np.random.default_rng(8))
    solver._constraints = Counted(solver._constraints)
    solver._transition = Counted(solver._transition)
    solver._terminal_constraints = Counted(solver._terminal_constraints)
    for name in ('_constraints_batch', '_transition_batch'):
        if hasattr(solver, name):
            setattr(solver, name, Counted(getattr(solver, name)))
    solver._trajectory_violations(states, controls, params, initial,
                                  solver._physical_upper, solver._physical_terminal_upper)
    assert solver._constraints.calls == 0
    assert solver._transition.calls == 0
    assert solver._terminal_constraints.calls == 1
    assert solver._constraints_batch.calls == 1
    assert solver._transition_batch.calls == 1


@pytest.mark.parametrize('soft,corridor', [(False, False), (False, True), (True, False), (True, True)])
def test_accumulated_forward_reconstruction_is_exact(soft, corridor):
    solver = symbolic_solver(soft, corridor)
    rng = np.random.default_rng(919)
    assert hasattr(solver, '_forward_transition')
    for _ in range(20):
        states, controls, _, initial = trajectory(solver, rng)
        scalar = [initial]
        for control in controls:
            scalar.append(np.asarray(solver._transition(scalar[-1], control)).ravel())
        batch = np.vstack((initial, np.asarray(solver._forward_transition(initial, controls.T)).T))
        np.testing.assert_array_equal(batch, np.asarray(scalar))


def test_frozen_failure_request_pair_preserves_native_candidate_and_policy(tmp_path):
    if importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.validation import validate_candidate
    fixture = json.loads((Path(__file__).parent/'fixtures'/'acados_future_envelope_240.json').read_text())
    path = ReferencePath(**fixture['path'])
    request = fixture['request']
    for steps in (1, 2):
        cfg = VehicleConfig(**fixture['config'], acados_rti_steps=steps)
        solver = AcadosSolver(path, cfg, prepare=True, artifact_directory=tmp_path)
        outputs = []
        for scalar in (True, False):
            solver.reset()
            seed = fixture['seed']
            solver.previous = {key: np.asarray(seed[key]) for key in ('states', 'controls')}
            solver.previous_theta = seed['previous_theta']
            solver.previous_yaw = seed['previous_yaw']
            original = solver._trajectory_violations
            if scalar:
                solver._trajectory_violations = lambda *args: scalar_violations(solver, *args)
            result = solver.solve(request['state'], request['previous'], request['speed_refs'], request['elapsed'])
            solver._trajectory_violations = original
            candidate = dict(result, validation_applied=[request['previous'][key] for key in
                             ('acceleration', 'steering', 'steering_rate')], dt=.1)
            outputs.append((result, validate_candidate(candidate, cfg, path)))
        old, new = outputs
        for key in ('success', 'status', 'iterations', 'constraint_violation', 'states', 'controls'):
            assert old[0][key] == new[0][key]
        for key in ('constraint_violations', 'raw_native_constraint_violations', 'native_pass_statuses',
                    'candidate_finite', 'native_controls_modified'):
            assert old[0]['diagnostics'][key] == new[0]['diagnostics'][key]
        if not old[0]['success']:
            for key in ('initial_state', 'applied', 'speed_refs', 'map_alignment', 'raw_native_states',
                        'raw_native_controls', 'projected_states', 'frozen_parameters',
                        'physical_constraint_violations', 'raw_native_constraint_violations'):
                assert old[0]['failure_snapshot'][key] == new[0]['failure_snapshot'][key]
        assert old[1] == new[1]
