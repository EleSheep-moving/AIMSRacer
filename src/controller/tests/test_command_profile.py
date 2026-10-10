"""Versioned command profile and generated constraint contracts."""
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from aims_mpcc.config import VehicleConfig
from aims_mpcc.io import load_config


def test_old_configuration_has_explicit_legacy_contract():
    cfg = VehicleConfig()
    assert getattr(cfg, 'command_profile', None) == 'legacy_bounded_v1'


def test_profile_validates_and_cost_scale_is_independent():
    cfg = VehicleConfig()
    cfg.command_profile = 'rate_bounded_v2'
    cfg.steering_acceleration_scale = 2.
    cfg.validate()
    cfg.command_profile = 'unknown'
    with pytest.raises(ValueError, match='command_profile'):
        cfg.validate()


def test_v2_removes_only_two_hard_rows_and_preserves_objective():
    from aims_mpcc.acados_backend import AcadosSolver
    from aims_mpcc.backend_models import NumericalBackend
    from aims_mpcc.path import ReferencePath
    angles = np.linspace(0.,2*np.pi,40,endpoint=False)
    path = ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.,"odom")
    cfg = VehicleConfig(profile='synthetic', rear_offset=.15, half_length=.28,
                        half_width=.1, geometry_verified=True, enforce_corridor=False)
    def build(profile):
        solver = AcadosSolver.__new__(AcadosSolver)
        configuration = replace(cfg)
        configuration.command_profile = profile
        configuration.steering_acceleration_scale = cfg.steer_acceleration
        NumericalBackend.__init__(solver,path,configuration,10,.1)
        solver.model_name='profile_test'; solver._expected_lib_path=None
        return solver, solver._build_ocp()
    v1, old = build('legacy_bounded_v1'); v2, new = build('rate_bounded_v2')
    assert 'jerk' in v1._groups
    assert 'steering_acceleration' in v1._groups
    assert 'jerk' not in v2._groups
    assert 'steering_acceleration' not in v2._groups
    assert list(v2._groups) == [g for g in v1._groups if g not in ('jerk','steering_acceleration')]
    assert new.model.con_h_expr_0.numel() == new.model.con_h_expr.numel()
    assert new.model.con_h_expr_e.numel() == old.model.con_h_expr_e.numel()
    np.testing.assert_array_equal(new.cost.W,old.cost.W)
    import casadi as ca
    x=np.array([0.,0.,0.,.1,0.,.01,0.,.02,.5]);u=np.array([.5,.15,.3]);p=np.array([0.,0.,0.,0.,0.,1.,.5,1.,0.,1.])
    a=ca.Function('old_cost',[old.model.x,old.model.u,old.model.p],[old.model.cost_y_expr])
    b=ca.Function('new_cost',[new.model.x,new.model.u,new.model.p],[new.model.cost_y_expr])
    np.testing.assert_array_equal(np.asarray(a(x,u,p)),np.asarray(b(x,u,p)))
    # Removed bounds in seeds: acceleration responds immediately, a large
    # previous rate does not narrow the retained endpoint/rate interval.
    initial=np.zeros(6)
    states,controls=v2.seed(initial,np.array([-.5,0.,5.]),np.full(11,.5))
    assert controls[0,0] == .5
    assert abs(controls[0,1]) <= cfg.steer_rate*.1


def test_native_configuration_declares_selected_profile():
    cfg=load_config(Path(__file__).parents[1]/'config/vehicle.yaml')
    assert cfg.command_profile == 'rate_bounded_v2'
    assert cfg.steering_acceleration_scale == cfg.steer_acceleration


def test_legacy_startup_and_python_backend_reject_v2():
    cfg=VehicleConfig(command_profile='rate_bounded_v2')
    with pytest.raises(ValueError,match='native acados runtime'):
        cfg.require_legacy_command_profile()


def test_old_cost_normalization_is_inherited_for_nondefault_limit():
    cfg=VehicleConfig(command_profile='rate_bounded_v2',steer_acceleration=7.)
    assert cfg.steering_rate_change_scale()==7.
    cfg.steering_acceleration_scale=2.
    assert cfg.steering_rate_change_scale()==2.
    cfg.command_profile='legacy_bounded_v1'
    assert cfg.steering_rate_change_scale()==7.
