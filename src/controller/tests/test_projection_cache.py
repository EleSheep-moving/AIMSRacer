"""Static reference sampling is reused without changing projection numerics."""
import numpy as np
import pytest
from scipy.optimize import minimize_scalar
from aims_mpcc.path import ReferencePath


def path(frame='odom'):
    a=np.linspace(0.,2*np.pi,64,endpoint=False)
    points=np.c_[3*np.cos(a)+.15*np.cos(3*a),2*np.sin(a)+.1*np.sin(2*a)]
    return ReferencePath(points,.9,.8,frame,{'map_sha256':'a'*64} if frame=='map' else None)


def frozen_project(reference,xy):
    grid=np.linspace(0,reference.length,max(100,int(reference.length/.05)),endpoint=False)
    guess=grid[np.argmin(np.sum((reference.curve.numpy(grid)-xy)**2,axis=1))]
    step=reference.length/len(grid)
    result=minimize_scalar(lambda s:float(np.sum((reference.curve.numpy(s)-xy)**2)),
                          bounds=(guess-step,guess+step),method='bounded',options={'xatol':1e-12})
    s=float(result.x%reference.length)
    p=reference.at(s);n=np.array([-np.sin(p['yaw']),np.cos(p['yaw'])])
    return s,float(np.dot(xy-np.array([p['x'],p['y']]),n))


@pytest.mark.parametrize('frame',['odom','map'])
def test_projection_matches_frozen_algorithm_exactly(frame):
    reference=path(frame);rng=np.random.default_rng(2828)
    points=np.vstack([rng.normal(size=(80,2))*4,reference.points,
                      reference.curve.numpy(np.array([-.02,0,.02,reference.length-.02]))])
    for xy in points:
        assert reference.project(xy)==frozen_project(reference,xy)


def test_repeated_projection_does_not_resample_full_static_reference(monkeypatch):
    reference=path();calls=[];original=reference.curve.numpy
    def recorded(theta,derivative=0):
        calls.append((np.asarray(theta).shape,derivative))
        return original(theta,derivative)
    monkeypatch.setattr(reference.curve,'numpy',recorded)
    for xy in ([3.,0.],[2.,1.],[-3.,0.]):reference.project(xy)
    assert not any(shape for shape,derivative in calls),calls
