"""Theta-only projection preserves the original offline projector."""

import numpy as np
import pytest
from scipy.optimize import minimize_scalar

from aims_mpcc.path import ReferencePath


def frozen_project(path,xy):
    """Original scalar projector before the theta-only helper."""
    xy=np.asarray(xy,float)
    if xy.shape!=(2,) or not np.isfinite(xy).all():raise ValueError('finite xy required')
    guess=path._projection_grid[np.argmin(np.sum((path._projection_points-xy)**2,axis=1))]
    step=path._projection_step
    result=minimize_scalar(lambda s:float(np.sum((path.curve.numpy(s)-xy)**2)),bounds=(guess-step,guess+step),method='bounded',options={'xatol':1e-12})
    theta=float(result.x%path.length)
    ref=path.at(theta);normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
    return theta,float(np.dot(xy-np.array([ref['x'],ref['y']]),normal))


def reference(frame):
    angle=np.arange(48)*2*np.pi/48
    radius=3.+.15*np.cos(3*angle)
    return ReferencePath(np.c_[radius*np.cos(angle),radius*np.sin(angle)],1.,1.,frame,
                         {'map_sha256':'a'*64} if frame=='map' else None)


@pytest.mark.parametrize('frame',['odom','map'])
def test_theta_and_lateral_error_match_original_at_seams_grid_ties_and_random_positions(frame):
    path=reference(frame)
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    rng=np.random.default_rng(834)
    queries=np.vstack((path.curve.numpy(np.array([-1e-9,0.,1e-9,path.length-1e-9,path.length,path.length+1e-9])),
                       .5*(path._projection_points[::7]+np.roll(path._projection_points,-1,axis=0)[::7]),
                       path.curve.numpy(rng.uniform(-path.length,2*path.length,80))+rng.normal(size=(80,2))*.1))
    for xy in queries:
        expected=frozen_project(path,xy)
        assert path.project_theta(xy).hex()==expected[0].hex()
        assert tuple(value.hex() for value in path.project(xy))==tuple(value.hex() for value in expected)


@pytest.mark.parametrize('xy',[[0.],[0.,0.,0.],[np.nan,0.],[0.,np.inf]])
def test_theta_only_projector_keeps_input_validation(xy):
    path=reference('odom')
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    with pytest.raises(ValueError,match='finite xy required'):path.project_theta(xy)


def test_theta_only_projector_skips_unused_lateral_geometry(monkeypatch):
    path=reference('odom')
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    calls=[];original=path.at
    def observed(theta):
        calls.append(theta)
        return original(theta)
    monkeypatch.setattr(path,'at',observed)
    xy=path.points[0]+[.03,.02]
    theta=path.project_theta(xy)
    assert calls==[]
    assert path.project(xy)[0]==theta
    assert len(calls)==1
