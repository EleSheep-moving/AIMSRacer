"""Bounded RTI passes against a captured nonlinear envelope rejection."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def test_acados_default_keeps_one_rti_pass():
    assert getattr(VehicleConfig(), 'acados_rti_steps', None) == 1


@pytest.mark.parametrize('value', [0, 3, -1, 1., 2., True, False, None,
                                       float('nan'), float('inf'), '2'])
def test_acados_pass_count_requires_integer_one_or_two(value):
    cfg = VehicleConfig()
    cfg.acados_rti_steps = value
    with pytest.raises(ValueError, match='acados_rti_steps'):
        cfg.validate()


def test_two_rti_passes_resolve_captured_future_envelope_rejection(tmp_path):
    if importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.validation import validate_candidate
    fixture = json.loads((Path(__file__).parent / 'fixtures' /
                         'acados_future_envelope_240.json').read_text())
    path = ReferencePath(**fixture['path'])
    results = []
    for steps in (1, 2):
        cfg = VehicleConfig(**fixture['config'], acados_rti_steps=steps)
        solver = AcadosSolver(path, cfg, prepare=True, artifact_directory=tmp_path)
        seed = fixture['seed']
        solver.previous = {key: np.asarray(seed[key]) for key in ('states', 'controls')}
        solver.previous_theta = seed['previous_theta']
        solver.previous_yaw = seed['previous_yaw']
        request = fixture['request']
        result = solver.solve(request['state'], request['previous'], request['speed_refs'],
                              request['elapsed'])
        candidate = dict(result, validation_applied=[request['previous'][key] for key in
                         ('acceleration', 'steering', 'steering_rate')], dt=.1)
        results.append((result, validate_candidate(candidate, cfg, path)))
    one, two = results
    assert not one[0]['success'] and not one[1]['accepted']
    assert one[0]['diagnostics']['constraint_violations']['operating_envelope'] > .006
    assert one[1]['hard_ok'] and not one[1]['envelope_ok']
    assert two[0]['success'], two[0]
    assert two[1]['accepted'], two[1]
    assert two[0]['diagnostics']['effective_envelope_margin'] == .01
    assert two[0]['diagnostics']['native_controls_modified'] is False
    assert one[0]['failure_snapshot']['raw_native_controls'] == one[0]['controls']
    assert 'failure_snapshot' not in two[0]
    assert one[0]['diagnostics']['artifact_fingerprint'] != two[0]['diagnostics']['artifact_fingerprint']
    for result, _ in results:
        diagnostic = result['diagnostics']
        assert len(diagnostic['native_passes']) == diagnostic['acados_rti_steps']
        assert diagnostic['native_pass_statuses'] == [0] * diagnostic['acados_rti_steps']
        assert result['iterations'] == diagnostic['acados_rti_steps']
        assert diagnostic['native_total_time_s'] == pytest.approx(sum(
            row['native_total_time_s'] for row in diagnostic['native_passes']))
        assert all(len(row['nlp_residuals']) == 4 for row in diagnostic['native_passes'])


def test_two_pass_budget_and_final_physical_rejection_survive_first_status_error(tmp_path):
    if importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.validation import validate_candidate
    from aims_mpcc.worker import _record_solve
    from io import StringIO
    angles = np.linspace(0, 2*np.pi, 40, endpoint=False)
    path = ReferencePath(3*np.c_[np.cos(angles), np.sin(angles)], 1., 1., 'odom')
    cfg = VehicleConfig(profile='synthetic', rear_offset=.15, half_length=.28,
                        half_width=.15, geometry_verified=True, acados_rti_steps=2)
    solver = AcadosSolver(path, cfg, prepare=True, artifact_directory=tmp_path)
    native_solve = solver._native.solve
    calls = []
    def first_error_then_invalid_control():
        calls.append(native_solve())
        if len(calls) == 1:
            return 4
        control = solver._native.get(0, 'u')
        control[0] = cfg.accel_limit + .1
        solver._native.set(0, 'u', control)
        return 0
    solver._native.solve = first_error_then_invalid_control
    state = dict(x=3., y=0., yaw=np.pi/2, speed=.5, steering=np.arctan(.36/3))
    applied = dict(acceleration=0., steering=state['steering'], steering_rate=0.)
    result = solver.solve(state, applied, np.full(11, .5))
    assert len(calls) == 2
    assert result['diagnostics']['native_pass_statuses'] == [4, 0]
    assert not result['success'] and solver.previous is None
    assert result['controls'][0][0] == pytest.approx(cfg.accel_limit + .1)
    candidate = dict(result, validation_applied=[0., applied['steering'], 0.], dt=.1)
    assert not validate_candidate(candidate, cfg, path)['accepted']
    log = StringIO()
    _record_solve(log, result)
    logged = json.loads(log.getvalue())
    assert 'states' not in logged and 'controls' not in logged
    assert logged['failure_snapshot']['raw_native_controls'][0][0] == pytest.approx(cfg.accel_limit + .1)
