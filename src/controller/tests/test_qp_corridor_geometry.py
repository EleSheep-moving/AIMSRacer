"""Physical corridor batching preserves scalar geometry and corner arithmetic."""
from types import SimpleNamespace

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver


def fixture(horizon,frame,asymmetric,path_class=ReferencePath):
    angles=np.linspace(0.,2*np.pi,80,endpoint=False)
    points=np.c_[3*np.cos(angles),2.5*np.sin(angles)]
    alignment=np.array([.7,-.4,.31]) if frame=='map' else np.zeros(3)
    c,s=np.cos(alignment[2]),np.sin(alignment[2]);rotation=np.array([[c,-s],[s,c]])
    path=path_class(points@rotation.T+alignment[:2],.9,1.,frame,
                    {'map_sha256':'a'*64} if frame=='map' else None)
    geometry=dict(front_extent=.43,rear_extent=.17) if asymmetric else dict(half_length=.28)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_width=.15,
                      geometry_verified=True,enforce_corridor=True,cruise_speed=.5,**geometry)
    # Start just before the periodic seam, so the physical trace crosses it.
    theta=path.length-.03;ref=path.at(theta)
    xy=rotation.T@(np.array([ref['x'],ref['y']])-alignment[:2])
    yaw=ref['yaw']-alignment[2]
    state=dict(x=xy[0],y=xy[1],yaw=yaw,speed=.5,
               steering=np.arctan(cfg.wheelbase*ref['curvature']))
    previous=dict(acceleration=0.,steering=state['steering'],steering_rate=0.)
    return QPSolver(path,cfg,horizon),state,previous,alignment


def scalar_margin(solver,result,alignment):
    microstates=independent_rollout(result['solve_input']['initial_state'],
                                   result['solve_input']['applied'],result['controls'],solver.config)
    path,cfg=solver.path,solver.config
    c,s=np.cos(alignment[2]),np.sin(alignment[2]);rotation=np.array([[c,-s],[s,c]])
    margin=np.inf
    for x in microstates:
        xy=rotation@x[:2]+alignment[:2];ref=path.at(x[4])
        normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
        yaw=x[2]+alignment[2];forward=np.array([np.cos(yaw),np.sin(yaw)])
        left=np.array([-np.sin(yaw),np.cos(yaw)])
        for along in cfg.longitudinal_offsets():
            for sign in (-1,1):
                lateral=np.dot(xy+along*forward+sign*cfg.half_width*left-
                               np.array([ref['x'],ref['y']]),normal)
                margin=min(margin,path.left_width-lateral,path.right_width+lateral)
    return margin


@pytest.mark.parametrize('horizon',[10,15,20])
@pytest.mark.parametrize('frame',['odom','map'])
@pytest.mark.parametrize('asymmetric',[False,True])
def test_production_physical_geometry_uses_two_batches_and_exact_scalar_outputs(horizon,frame,asymmetric,monkeypatch):
    solver,state,previous,alignment=fixture(horizon,frame,asymmetric)
    original=solver.path.curve.numpy;calls=[]
    def recorded(theta,derivative=0):
        calls.append((np.asarray(theta).size,derivative))
        return original(theta,derivative)
    monkeypatch.setattr(solver.path.curve,'numpy',recorded)
    result=solver.solve(state,previous,np.full(horizon+1,.5),map_alignment=alignment if frame=='map' else None)
    assert result['success'],result
    assert [call for call in calls if call[0]==5*horizon+1]==[(5*horizon+1,0),(5*horizon+1,1)]
    assert result['diagnostics']['minimum_predicted_margin_m']==scalar_margin(solver,result,alignment)
    assert np.asarray(result['states'])[-1,4]>solver.path.length
    # Execute the former scalar branch with the same native primal and info.
    baseline,_,_,_=fixture(horizon,frame,asymmetric)
    original_at=baseline.path.at;scalar_calls=[]
    def scalar_at(theta):
        scalar_calls.append(theta)
        return original_at(theta)
    monkeypatch.setattr(baseline.path,'at',scalar_at)
    monkeypatch.setattr(baseline._native,'solve',lambda **kwargs:SimpleNamespace(
        x=solver.last_native.x.copy(),info=solver.last_native.info))
    old=baseline.solve(state,previous,np.full(horizon+1,.5),map_alignment=alignment if frame=='map' else None)
    assert len(scalar_calls)==5*horizon+2  # input projection + physical rows
    assert old['success']==result['success'] and old['status']==result['status']
    assert old['constraint_violation']==result['constraint_violation']
    assert np.array_equal(old['controls'],result['controls'])
    assert np.array_equal(old['states'],result['states'])
    for key in ['minimum_predicted_margin_m','envelope','qp_constraint_violation','candidate_feasible']:
        assert old['diagnostics'][key]==result['diagnostics'][key]


@pytest.mark.parametrize('custom',['subclass','overridden_at','class_override'])
def test_custom_path_geometry_retains_scalar_fallback(custom,monkeypatch):
    class CustomPath(ReferencePath):
        pass
    solver,state,previous,alignment=fixture(10,'odom',False,CustomPath if custom=='subclass' else ReferencePath)
    if custom in ('overridden_at','class_override'):
        original_at=solver.path.at
        def altered(self,theta):
            ref=original_at(theta)
            return dict(ref,y=ref['y']+.02,yaw=ref['yaw']+.01)
        if custom=='class_override':monkeypatch.setattr(ReferencePath,'at',altered)
        else:monkeypatch.setattr(solver.path,'at',lambda theta:altered(solver.path,theta))
    original=solver.path.curve.numpy;calls=[]
    def recorded(theta,derivative=0):
        calls.append((np.asarray(theta).size,derivative))
        return original(theta,derivative)
    monkeypatch.setattr(solver.path.curve,'numpy',recorded)
    result=solver.solve(state,previous,np.full(11,.5))
    assert not [call for call in calls if call[0]==51]
    assert result['diagnostics']['minimum_predicted_margin_m']==scalar_margin(solver,result,alignment)
