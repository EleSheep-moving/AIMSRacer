"""QP objective and fixed CSC ordering survive smaller numerical updates."""
import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver
from aims_mpcc.vendor.normalized_cost import RATIOS


def fixture(seed):
    rng=np.random.default_rng(seed)
    angle=np.arange(48)*2*np.pi/48
    path=ReferencePath(np.c_[3*np.cos(angle),2.5*np.sin(angle)],1.2,1.)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,
                      half_width=.15,geometry_verified=True,enforce_corridor=bool(seed%2))
    return QPSolver(path,cfg,int(rng.choice([2,3,10,15])),float(rng.choice([.02,.1,.2]))),rng


def dense_objective(solver,refs,applied,seed,alignment):
    """Frozen full-vector least-squares construction from eea2365."""
    cfg=solver.config;P=np.eye(solver.dimension)*1e-10;q=np.zeros(solver.dimension)
    def cost(values,target,weight):
        vector=np.zeros(solver.dimension)
        for i,v in values.items():vector[i]+=v
        P[:]+=2*weight*np.outer(vector,vector);q[:]-=2*weight*target*vector
    state=lambda k,j:4*k+j
    control=lambda k,j:4*(solver.n+1)+2*k+j
    geometries=solver.geometries(seed[0][:,4],alignment)
    for k in range(solver.n+1):
        terminal=cfg.terminal_weight if k==solver.n else 1.
        cost({state(k,0):1.},0.,terminal*cfg.contour_weight/cfg.contour_scale**2)
        cost({state(k,1):1.},0.,terminal*cfg.heading_weight/cfg.heading_scale**2)
        cost({state(k,3):1.},refs[k],terminal*cfg.speed_weight/cfg.speed_scale**2)
    for k in range(solver.n):
        accel,steer=control(k,0),control(k,1)
        delta={steer:1.}
        if k:delta[control(k-1,1)]=-1.;previous=0.
        else:previous=applied[1]
        second=dict(delta);bound_offset=0.
        if k==0:bound_offset=applied[1]+applied[2]*solver.dt
        elif k==1:second[control(k-1,1)]=-2.;bound_offset=-applied[1]
        else:second[control(k-1,1)]=-2.;second[control(k-2,1)]=1.
        ff=np.arctan(cfg.wheelbase*(1+cfg.understeer_coefficient*seed[0][k,3]**2)*geometries[k][3])
        cost({steer:1.},ff,cfg.steering_weight/cfg.steering_scale**2)
        cost({accel:1.},0.,RATIOS['accel']/cfg.acceleration_scale**2)
        cost(delta,previous,cfg.steering_rate_weight/(solver.dt*cfg.steer_rate)**2)
        cost(second,bound_offset,cfg.steering_acceleration_weight/(cfg.steer_acceleration*solver.dt**2)**2)
    return P,q


@pytest.mark.parametrize('seed',range(20))
def test_active_cost_matches_dense_objective_after_mutable_tuning(seed):
    solver,rng=fixture(seed);cfg=solver.config
    initial=np.array([3.,0.,np.pi/2,rng.uniform(.1,.8),rng.uniform(-2.,2.),rng.uniform(-.2,.2)])
    applied=np.array([rng.uniform(-.1,.1),rng.uniform(-.2,.2),rng.uniform(-.2,.2)])
    refs=rng.uniform(.1,.9,solver.n+1);alignment=np.r_[rng.normal(size=2),rng.uniform(-.5,.5)]
    for changed in (False,True):
        if changed:
            for name in ('contour_weight','heading_weight','speed_weight','steering_weight',
                         'steering_rate_weight','steering_acceleration_weight','terminal_weight',
                         'contour_scale','heading_scale','speed_scale','steering_scale','acceleration_scale'):
                setattr(cfg,name,getattr(cfg,name)*rng.uniform(.6,1.4))
        seed_states=solver.seed(initial,applied,refs)
        expected=dense_objective(solver,refs,applied,seed_states,alignment)
        P,q,*_=solver._assemble(initial,applied,refs,alignment,seed_states)
        assert np.array_equal(P,expected[0])
        assert np.array_equal(q,expected[1])


def test_cost_does_not_form_full_dimension_outer_products(monkeypatch):
    solver,_=fixture(0);calls=[];original=np.outer
    def recorded(a,b,*args,**kwargs):
        calls.append((len(a),len(b)))
        return original(a,b,*args,**kwargs)
    monkeypatch.setattr(np,'outer',recorded)
    initial=np.array([3.,0.,np.pi/2,.5,0.,0.]);applied=np.zeros(3);refs=np.full(solver.n+1,.5)
    solver._assemble(initial,applied,refs,np.zeros(3),solver.seed(initial,applied,refs))
    assert all(max(shape)<=3 for shape in calls)


@pytest.mark.parametrize('name',['p','a'])
def test_csc_values_use_one_gather_and_preserve_pattern_order(name):
    solver,rng=fixture(2);pattern=getattr(solver,'_'+name+'_pattern')
    base=rng.normal(size=pattern.shape);calls=[]
    class RecordedMatrix(np.ndarray):
        def __getitem__(self,index):
            calls.append(index)
            return super().__getitem__(index)
    values=getattr(solver,'_'+name+'_values')(base.view(RecordedMatrix))
    expected=np.concatenate([base[pattern.indices[pattern.indptr[col]:pattern.indptr[col+1]],col]
                             for col in range(pattern.shape[1])])
    assert np.array_equal(values.data,expected)
    assert np.array_equal(values.indices,pattern.indices)
    assert np.array_equal(values.indptr,pattern.indptr)
    assert len(calls)==1
