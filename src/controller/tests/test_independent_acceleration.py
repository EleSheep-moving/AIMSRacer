"""Removing the acceleration ellipse must retain the independent bounds."""
from dataclasses import replace

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.validation import validate_candidate


def config():
    return VehicleConfig(command_profile='rate_bounded_v2', geometry_verified=True,
                         rear_offset=0., front_extent=.52, rear_extent=.1,
                         half_width=.16, enforce_corridor=False, max_speed=1.5,
                         combined_accel_constraint_enabled=False)


def test_explicit_flag_preserves_old_profiles_and_rejects_unsupported_modes():
    assert VehicleConfig().combined_accel_constraint_enabled is True
    config().validate()
    for changes in ({'command_profile': 'legacy_bounded_v1'},
                    {'envelope_soft_enabled': True}):
        with pytest.raises(ValueError, match='strict rate_bounded_v2'):
            replace(config(), **changes).validate()
    with pytest.raises(ValueError, match='must be a boolean'):
        replace(config(), combined_accel_constraint_enabled=0).validate()


def test_independent_validator_allows_ellipse_excess_but_keeps_actuator_bounds():
    cfg = config()
    controls = np.tile([0., .3, 1.2], (10, 1))
    initial = [0., 0., 0., 1.2, 0., .3]
    applied = [0., .3, 0.]

    def candidate(commands, state=initial):
        return dict(states=independent_rollout(state, applied, commands, cfg)[::5].tolist(),
                    controls=commands.tolist(), validation_applied=applied, dt=.1)

    plan = candidate(controls)
    result = validate_candidate(plan, cfg)
    assert result['accepted'], result
    assert result['envelope']['future_slack_max'] > .1
    assert not validate_candidate(plan, replace(cfg, combined_accel_constraint_enabled=True))['accepted']
    for column, value in ((0, cfg.accel_limit+.1), (0, -cfg.brake_limit-.1),
                          (1, cfg.steer_limit+.1), (2, cfg.max_speed+.1)):
        invalid = controls.copy()
        invalid[0, column] = value
        assert not validate_candidate(candidate(invalid), cfg)['accepted']
    assert not validate_candidate(candidate(controls, [0., 0., 0., cfg.max_speed+.1, 0., .3]), cfg)['accepted']
    plan['controls'][0][0] = np.nan
    assert not validate_candidate(plan, cfg)['accepted']


@pytest.mark.parametrize('corridor', [False, True])
def test_ocp_removes_only_ellipse_rows(corridor):
    pytest.importorskip('acados_template')
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.backend_models import NumericalBackend
    from aims_mpcc.path import ReferencePath
    angles = np.arange(40)*2*np.pi/40
    path = ReferencePath(3*np.c_[np.cos(angles), np.sin(angles)], 1., 1.)
    solver = AcadosSolver.__new__(AcadosSolver)
    cfg = replace(config(), enforce_corridor=corridor)
    NumericalBackend.__init__(solver, path, cfg, 15, .1)
    solver.model_name = 'independent_acceleration_test'
    solver._expected_lib_path = None
    ocp = solver._build_ocp()
    assert 'operating_envelope' not in solver._groups
    assert solver._groups.count('steering_rate') == 1
    assert len(ocp.constraints.uh_e) == (4 if corridor else 0)
    assert len(ocp.constraints.uh) == (13 if corridor else 1)
    np.testing.assert_array_equal(ocp.constraints.lbu, [-cfg.brake_limit, -cfg.steer_limit, 0.])
    np.testing.assert_array_equal(ocp.constraints.ubu, [cfg.accel_limit, cfg.steer_limit, cfg.max_speed])
    assert ocp.solver_options.nlp_solver_type == 'SQP_RTI'
