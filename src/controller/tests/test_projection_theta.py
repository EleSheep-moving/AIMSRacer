"""Theta-only projection preserves the original projector and executor."""
import copy
from dataclasses import replace

import numpy as np
import pytest
from scipy.optimize import minimize_scalar

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.execution import execution_schedule
from aims_mpcc.path import ReferencePath
from aims_mpcc.runtime import Command,State,Supervisor


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


def scenario(frame,status):
    path=reference(frame);theta=path.length-.025;ref=path.at(theta)
    alignment=np.array([4.,-2.,.37]) if frame=='map' else None
    xy=np.array([ref['x'],ref['y']]);yaw=ref['yaw']
    if alignment is not None:
        c,s=np.cos(alignment[2]),np.sin(alignment[2])
        xy=np.array([[c,s],[-s,c]])@(xy-alignment[:2]);yaw-=alignment[2]
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,half_width=.15,
                      geometry_verified=True,cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=np.array([*xy,yaw,.35,theta,.05]);applied=dict(speed=.35,steering=.08,acceleration=.04,steering_rate=.02)
    supervisor=Supervisor(cfg,path.length,solve_period=.2,handover_delay=.02)
    supervisor.observe(State(*initial[:3],0.,initial[5],100.),10.,theta,0.)
    supervisor.set_mode(True,10.);supervisor.start(10.)
    supervisor.state=replace(supervisor.state,speed=initial[3]);supervisor.status=status
    supervisor.last_command=Command(.35,.08);supervisor.last_acceleration=.04
    supervisor.last_steering_rate=.02;supervisor.last_tick=9.98
    supervisor.progress=theta+3*path.length;supervisor.lap_goal=supervisor.progress+.65
    controls=[[.1,.08+i*.002,.3] for i in range(10)]
    applied3=[applied[key] for key in ('acceleration','steering','steering_rate')]
    states=independent_rollout(initial,applied3,controls,cfg)[::5]
    plan=dict(states=states.tolist(),controls=controls,previous_steering=.08,
              stamp=10.02,source_stamp=10.,dt=.1,map_alignment=alignment,
              execution_speed_targets=[.35+i*.01 for i in range(11)])
    return supervisor,plan,initial,applied,path


@pytest.mark.parametrize('frame',['odom','map'])
@pytest.mark.parametrize('status',['RUNNING','RECOVERING'])
def test_nominal_schedule_matches_frozen_full_projector(frame,status):
    supervisor,plan,initial,applied,path=scenario(frame,status)
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    snapshot=copy.deepcopy(supervisor.__dict__)
    actual=execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    path.project=lambda xy:frozen_project(path,xy)
    original=execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    for key in ('states','controls','internal','wire','elapsed','physical_progress'):
        assert np.array_equal(np.asarray(actual[key]),np.asarray(original[key])),(frame,status,key)
    assert actual['statuses']==original['statuses']
    assert supervisor.__dict__==snapshot


def test_execution_uses_theta_only_for_standard_projector(monkeypatch):
    supervisor,plan,initial,applied,path=scenario('map','RECOVERING')
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    calls=[];original=path.at
    monkeypatch.setattr(path,'at',lambda theta:calls.append(theta) or original(theta))
    execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    assert calls==[]  # execution consumes theta, not lateral error or curvature


def test_custom_projector_keeps_its_results_and_full_call_site():
    supervisor,plan,initial,applied,path=scenario('odom','RUNNING')
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    calls=[]
    def custom(xy):
        calls.append(True)
        theta,error=frozen_project(path,xy)
        return (theta+.003*np.sin(float(xy[0])))%path.length,error
    path.project=custom
    actual=execution_schedule(supervisor,plan,initial,applied,10.02,path)
    assert len(calls)==51
    assert not actual['context']['finish_independent_certificate']['proven']
    expected=execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    for key in ('states','controls','internal','wire','physical_progress'):
        assert np.array_equal(actual[key],expected[key])


def test_custom_theta_helper_cannot_receive_standard_finish_certificate():
    supervisor,plan,initial,applied,path=scenario('odom','RUNNING')
    assert hasattr(path,'project_theta'),'theta-only projector is missing'
    supervisor.lap_goal=supervisor.progress+path.length
    original=path.project_theta;calls=[]
    def custom(xy):
        calls.append(True)
        return (original(xy)+.003*np.sin(float(xy[0])))%path.length
    path.project_theta=custom
    actual=execution_schedule(supervisor,plan,initial,applied,10.02,path)
    assert len(calls)==51
    assert not actual['context']['finish_independent_certificate']['proven']
    expected=execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    for key in ('states','controls','internal','wire','physical_progress'):
        assert np.array_equal(actual[key],expected[key])


@pytest.mark.parametrize('method',['project','project_theta'])
def test_class_projection_override_cannot_receive_standard_finish_certificate(method,monkeypatch):
    supervisor,plan,initial,applied,path=scenario('odom','RUNNING')
    supervisor.lap_goal=supervisor.progress+path.length
    original=getattr(ReferencePath,method);calls=[]
    def custom(self,xy):
        calls.append(True)
        result=original(self,xy)
        offset=.003*np.sin(float(xy[0]))
        if method=='project':return (result[0]+offset)%self.length,result[1]
        return (result+offset)%self.length
    monkeypatch.setattr(ReferencePath,method,custom)
    actual=execution_schedule(supervisor,plan,initial,applied,10.02,path)
    assert not actual['context']['finish_independent_certificate']['proven']
    assert len(calls)==51
    expected=execution_schedule(supervisor,plan,initial,applied,10.02,path,force_slow=True)
    for key in ('states','controls','internal','wire','physical_progress'):
        assert np.array_equal(actual[key],expected[key])
