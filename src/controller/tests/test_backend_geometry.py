"""Differential checks against the scalar geometry/seed before batching."""
import math

import numpy as np
import pytest

from aims_mpcc.backend_models import NumericalBackend
from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout,jerk_limits
from aims_mpcc.path import ReferencePath


def legacy_geometry(backend,theta,alignment):
    ref=backend.path.at(theta);c,s=np.cos(alignment[2]),np.sin(alignment[2])
    rotation=np.array([[c,-s],[s,c]])
    xy=rotation.T@(np.array([ref['x'],ref['y']])-alignment[:2])
    return np.r_[xy,ref['yaw']-alignment[2],ref['curvature'],theta,
                 np.linalg.norm(backend.path.curve.numpy(theta,1))]


def legacy_seed(backend,initial,applied,refs):
    """Frozen original algorithm; retain its repeated geometry reads as oracle."""
    shifted=None
    if backend.previous is not None and backend.previous_elapsed<backend.n*backend.dt:
        shift=min(backend.n,max(1,round(backend.previous_elapsed/backend.dt)))
        shifted=np.vstack((backend.previous['controls'][shift:],np.repeat(backend.previous['controls'][-1:],shift,axis=0)))
    states=[initial];controls=[];acceleration,steering,rate=applied
    limits=jerk_limits(initial,applied,backend.config,backend.dt,backend.n)
    for k in range(backend.n):
        x=states[-1];ref=backend.path.at(x[4])
        desired_accel=(refs[k+1]-x[3])/backend.dt
        desired_steer=math.atan(backend.config.wheelbase*(1+backend.config.understeer_coefficient*x[3]**2)*ref['curvature'])
        if shifted is not None:desired_accel,desired_steer=shifted[k,:2]
        acceleration=float(np.clip(desired_accel,max(-backend.config.brake_limit,acceleration-limits[k]*backend.dt),
                                    min(backend.config.accel_limit,acceleration+limits[k]*backend.dt)))
        desired_rate=(np.clip(desired_steer,-backend.config.steer_limit,backend.config.steer_limit)-steering)/backend.dt
        rate=float(np.clip(desired_rate,max(-backend.config.steer_rate,rate-backend.config.steer_acceleration*backend.dt),
                            min(backend.config.steer_rate,rate+backend.config.steer_acceleration*backend.dt)))
        endpoint=float(np.clip(steering+rate*backend.dt,-backend.config.steer_limit,backend.config.steer_limit))
        progress=max(0.,x[3]+.5*acceleration*backend.dt)/np.linalg.norm(backend.path.curve.numpy(x[4],1))
        control=np.array([acceleration,endpoint,min(backend.config.max_speed,progress)])
        states.append(independent_rollout(x,[acceleration,steering,rate],[control],backend.config,backend.dt)[-1])
        controls.append(control);rate=(endpoint-steering)/backend.dt;steering=endpoint
    return np.asarray(states),np.asarray(controls)


def backend(seed):
    rng=np.random.default_rng(seed)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,half_width=.15,
                      geometry_verified=True,steering_tau=.08+rng.random()*.1,
                      understeer_coefficient=rng.random()*.2)
    angle=np.arange(64)*2*np.pi/64
    points=np.c_[(3.+.1*np.cos(3*angle))*np.cos(angle),(2.+.1*np.sin(2*angle))*np.sin(angle)]
    frame='map' if seed%2 else 'odom'
    path=ReferencePath(points,1.2,1.,frame,{'map_sha256':'a'*64} if frame=='map' else None)
    return NumericalBackend(path,cfg,horizon=int(rng.integers(2,21)),dt=float(rng.choice([.02,.06,.1,.2]))),rng


@pytest.mark.parametrize('seed',range(70))
def test_batched_geometry_matches_scalar_negative_progress_laps_and_alignment(seed):
    instance,rng=backend(seed)
    theta=np.r_[rng.uniform(-5*instance.path.length,5*instance.path.length,17),
                -instance.path.length,0.,instance.path.length,7*instance.path.length]
    alignment=np.r_[rng.normal(size=2)*3,rng.uniform(-3*np.pi,3*np.pi)]
    expected=np.asarray([legacy_geometry(instance,t,alignment) for t in theta])
    actual=instance.geometries(theta,alignment)
    assert actual.shape==(len(theta),6)
    assert np.allclose(actual,expected,rtol=1e-13,atol=1e-13)
    assert np.array_equal(actual[:,4],theta)


@pytest.mark.parametrize('seed',range(70))
def test_seed_matches_frozen_scalar_algorithm_for_feedforward_and_shifted_history(seed):
    instance,rng=backend(seed)
    initial=np.r_[rng.normal(size=3),rng.random(),rng.uniform(-3,4)*instance.path.length,rng.uniform(-.3,.3)]
    applied=np.array([rng.uniform(-.4,.4),rng.uniform(-.3,.3),rng.uniform(-.5,.5)])
    refs=rng.random(instance.n+1)
    expected=legacy_seed(instance,initial,applied,refs)
    actual=instance.seed(initial,applied,refs)
    assert np.array_equal(actual[0],expected[0])
    assert np.array_equal(actual[1],expected[1])
    instance.previous=dict(states=expected[0],controls=expected[1])
    instance.previous_elapsed=float(rng.choice([0.,.03,.1,.35,instance.n*instance.dt]))
    moved=initial.copy();moved[:3]+=rng.normal(size=3)*.03;moved[4]+=.2
    expected=legacy_seed(instance,moved,applied,refs)
    actual=instance.seed(moved,applied,refs)
    assert np.array_equal(actual[0],expected[0])
    assert np.array_equal(actual[1],expected[1])


def test_batch_geometry_reads_each_spline_derivative_once(monkeypatch):
    instance,_=backend(0);calls=[];original=instance.path.curve.numpy
    def recorded(theta,derivative=0):
        calls.append((np.asarray(theta).shape,derivative))
        return original(theta,derivative)
    monkeypatch.setattr(instance.path.curve,'numpy',recorded)
    instance.geometries(np.linspace(-10.,20.,21),np.array([1.,2.,.3]))
    assert calls==[((21,),0),((21,),1),((21,),2)]


def test_seed_omits_unused_position_geometry_and_reuses_tangent(monkeypatch):
    instance,_=backend(0);calls=[];original=instance.path.curve.numpy
    def recorded(theta,derivative=0):
        calls.append(derivative);return original(theta,derivative)
    monkeypatch.setattr(instance.path.curve,'numpy',recorded)
    initial=np.array([3.,0.,np.pi/2,.2,0.,0.]);applied=np.zeros(3);refs=np.full(instance.n+1,.2)
    result=instance.seed(initial,applied,refs)
    assert calls.count(0)==0
    assert calls.count(1)==instance.n
    assert calls.count(2)==instance.n
    instance.previous=dict(states=result[0],controls=result[1]);instance.previous_elapsed=instance.dt
    calls.clear();instance.seed(initial,applied,refs)
    assert calls==[1]*instance.n
