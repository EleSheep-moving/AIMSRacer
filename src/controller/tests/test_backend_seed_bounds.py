"""Warm steering targets must not manufacture an angular rate jump at a bound."""
import numpy as np
import pytest

from aims_mpcc.backend_models import NumericalBackend
from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.path import ReferencePath


def instance(dt=.1):
    angles=np.linspace(0,2*np.pi,40,endpoint=False)
    path=ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.,'odom')
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,
        half_width=.15,geometry_verified=True,enforce_corridor=False,
        cruise_speed=1.,max_speed=1.5,steer_limit=.45,steer_rate=2.,steer_acceleration=2.)
    return NumericalBackend(path,cfg,10,dt)


def assert_angular_bounds(backend,controls,applied):
    angles=controls[:,1]
    rates=np.diff(np.r_[applied[1],angles])/backend.dt
    changes=np.diff(np.r_[applied[2],rates])
    assert np.max(abs(angles))<=backend.config.steer_limit+1e-12
    assert np.max(abs(rates))<=backend.config.steer_rate+1e-12
    assert np.max(abs(changes))<=backend.config.steer_acceleration*backend.dt+1e-12


@pytest.mark.parametrize('sign',[-1.,1.])
@pytest.mark.parametrize('dt',[.1,.2])
def test_shifted_valid_warm_targets_brake_rate_before_angle_limit(sign,dt):
    backend=instance(dt)
    old_initial=np.array([3.,0.,np.pi/2,.2,0.,sign*.45])
    old_applied=np.array([0.,sign*.45,0.])
    warm=np.column_stack((np.zeros(10),np.full(10,sign*.45),np.full(10,.2)))
    backend.previous=dict(states=independent_rollout(old_initial,old_applied,warm,
                          backend.config,dt)[::round(dt/.02)],controls=warm)
    backend.previous_elapsed=.05
    initial=np.array([3.,0.,np.pi/2,1.,0.,-sign*.18])
    applied=np.array([0.,-sign*.18,0.])
    original=applied.copy()
    states,controls=backend.seed(initial,applied,np.ones(11))
    assert np.array_equal(applied,original)
    assert np.array_equal(states[0],initial)
    assert_angular_bounds(backend,controls,applied)
    # The seed still advances at the prescribed measured speed.
    assert states[-1,4]>states[0,4]+.5


def test_exactly_stoppable_prefix_can_reach_bound_then_hold():
    backend=instance()
    initial=np.array([3.,0.,np.pi/2,.5,0.,.44])
    applied=np.array([0.,.44,.3])
    original=applied.copy()
    backend.previous=dict(controls=np.column_stack((np.zeros(10),np.full(10,.45),np.full(10,.5))))
    states,controls=backend.seed(initial,applied,np.full(11,.5))
    assert np.array_equal(applied,original)
    assert np.array_equal(states[0],initial)
    assert controls[0,1]==pytest.approx(.45)
    assert_angular_bounds(backend,controls,applied)


@pytest.mark.parametrize('sign',[-1.,1.])
def test_impossible_initial_prefix_is_reported_without_altering_actual_history(sign):
    backend=instance()
    initial=np.array([3.,0.,np.pi/2,.5,0.,sign*.44])
    applied=np.array([0.,sign*.44,sign*1.])
    initial_before=initial.copy();applied_before=applied.copy()
    with pytest.raises(ValueError,match='bounded angular continuation'):
        backend.seed(initial,applied,np.full(11,.5))
    assert np.array_equal(initial,initial_before)
    assert np.array_equal(applied,applied_before)
