"""Corridor offsets are read per pass and retain native/physical geometry."""
import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver


def fixture(asymmetric, enabled):
    angle=np.arange(64)*2*np.pi/64
    path=ReferencePath(3*np.c_[np.cos(angle),np.sin(angle)],1.2,1.,'map',
                       {'map_sha256':'a'*64})
    geometry=dict(front_extent=.43,rear_extent=.17) if asymmetric else dict(half_length=.28)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_width=.15,
                      geometry_verified=True,enforce_corridor=enabled,**geometry)
    return QPSolver(path,cfg,horizon=3),cfg,path


def request(seed):
    rng=np.random.default_rng(seed)
    alignment=np.r_[rng.normal(size=2)*.03,rng.normal()*.02]
    state=dict(x=3.+rng.normal()*.02,y=rng.normal()*.02,
               yaw=np.pi/2+rng.normal()*.02,speed=rng.uniform(.15,.65),
               steering=np.arctan(.36/3)+rng.normal()*.01)
    previous=dict(acceleration=rng.uniform(-.05,.05),steering=state['steering'],
                  steering_rate=rng.uniform(-.05,.05))
    return state,previous,rng.uniform(.25,.75,4),alignment


@pytest.mark.parametrize('asymmetric',[False,True])
@pytest.mark.parametrize('enabled',[False,True])
@pytest.mark.parametrize('seed',range(4))
def test_native_corridor_rows_use_current_offsets_once_per_assembly(asymmetric,enabled,seed,monkeypatch):
    solver,cfg,path=fixture(asymmetric,enabled)
    state,previous,refs,alignment=request(seed)
    initial,applied,refs,alignment,warm=solver.inputs(state,previous,refs,.05,alignment)
    original=cfg.longitudinal_offsets;calls=[]
    def recorded():
        calls.append(True)
        return original()
    monkeypatch.setattr(cfg,'longitudinal_offsets',recorded)
    for changed in (False,True):
        if changed:
            if asymmetric:cfg.front_extent+=.04
            else:cfg.half_length+=.04
        offsets=original() if enabled else ()
        calls.clear()
        P,q,A,lower,upper=solver._assemble(initial,applied,refs,alignment,warm)
        assert len(calls)==int(enabled)
        width=2+len(offsets)
        for k in range(solver.n+1):
            for j,along in enumerate(offsets):
                row=4+4*solver.n+k*width+2+j
                expected=np.zeros(solver.dimension);expected[4*k]=1.;expected[4*k+1]=along
                assert np.array_equal(A[row],expected)
                assert lower[row]==-path.right_width+cfg.half_width
                assert upper[row]==path.left_width-cfg.half_width


@pytest.mark.parametrize('asymmetric',[False,True])
@pytest.mark.parametrize('seed',range(4))
def test_physical_corridor_margin_preserved_with_two_offset_reads(asymmetric,seed,monkeypatch):
    from aims_mpcc.envelope import independent_rollout
    solver,cfg,path=fixture(asymmetric,True)
    state,previous,refs,alignment=request(seed)
    original=cfg.longitudinal_offsets;offsets=original();calls=[]
    def recorded():
        calls.append(True)
        return original()
    monkeypatch.setattr(cfg,'longitudinal_offsets',recorded)
    result=solver.solve(state,previous,refs,.05,alignment)
    assert len(calls)==2  # native assembly and independent physical diagnostics
    assert result.get('states') is not None
    microstates=independent_rollout(result['solve_input']['initial_state'],
                                   result['solve_input']['applied'],result['controls'],cfg)
    c,s=np.cos(alignment[2]),np.sin(alignment[2]);rotation=np.array([[c,-s],[s,c]])
    margin=np.inf
    for x in microstates:
        xy=rotation@x[:2]+alignment[:2];ref=path.at(x[4])
        normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
        yaw=x[2]+alignment[2];forward=np.array([np.cos(yaw),np.sin(yaw)])
        left=np.array([-np.sin(yaw),np.cos(yaw)])
        for along in offsets:
            for sign in (-1,1):
                lateral=np.dot(xy+along*forward+sign*cfg.half_width*left-
                               np.array([ref['x'],ref['y']]),normal)
                margin=min(margin,path.left_width-lateral,path.right_width+lateral)
    assert result['diagnostics']['minimum_predicted_margin_m']==margin
