"""Fresh dense assembly must preserve every bit and mutable numeric input."""
import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver
from qp_assembly_858_oracle import original_assemble


def fixture(horizon=10,corridor=True,mapped=False):
    theta=np.linspace(0,2*np.pi,48,endpoint=False)
    points=np.c_[3*np.cos(theta),2.5*np.sin(theta)]
    alignment=np.array([1.2,-.7,.31]) if mapped else np.zeros(3)
    if mapped:
        c,s=np.cos(alignment[2]),np.sin(alignment[2]);rotation=np.array([[c,-s],[s,c]])
        points=points@rotation.T+alignment[:2]
    path=ReferencePath(points,1.1,.95,'map' if mapped else 'odom',
                       {'map_sha256':'a'*64} if mapped else None)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,half_width=.15,
                      geometry_verified=True,enforce_corridor=corridor,cruise_speed=.7)
    solver=QPSolver(path,cfg,horizon)
    initial=np.array([3.,0.,np.pi/2,.61,0.,.14]);applied=np.array([-.17,.13,-.08])
    refs=np.linspace(.7,.25,horizon+1)
    seed=solver.seed(initial,applied,refs)
    return solver,(initial,applied,refs,alignment,seed)


def assert_exact(solver,args):
    expected=original_assemble(solver,*args)
    structures=(solver._p_structure.copy(),solver._a_structure.copy())
    actual=solver._assemble(*args)
    for a,b in zip(actual,expected):
        assert a.dtype==b.dtype and a.shape==b.shape
        assert np.array_equal(a.view(np.uint64),b.view(np.uint64))
    assert np.array_equal(solver._p_structure,structures[0])
    assert np.array_equal(solver._a_structure,structures[1])
    return actual


@pytest.mark.parametrize('horizon',[10,15,20])
@pytest.mark.parametrize('corridor',[False,True])
@pytest.mark.parametrize('mapped',[False,True])
def test_all_five_dense_outputs_match_frozen_original_under_mutable_numeric_config(horizon,corridor,mapped):
    solver,args=fixture(horizon,corridor,mapped)
    assert_exact(solver,args)
    cfg=solver.config
    for name in ['contour_weight','heading_weight','speed_weight','steering_weight',
                 'steering_rate_weight','steering_acceleration_weight','terminal_weight',
                 'contour_scale','heading_scale','speed_scale','steering_scale','acceleration_scale']:
        setattr(cfg,name,getattr(cfg,name)*1.17)
    cfg.accel_limit=.6;cfg.brake_limit=.7;cfg.jerk_limit=1.2
    cfg.steer_rate=.6;cfg.steer_acceleration=2.2;cfg.steer_limit=.43
    cfg.max_speed=1.4;cfg.half_width=.18;cfg.half_length=.31
    cfg.longitudinal_envelope_accel=.62;cfg.longitudinal_envelope_brake=.68
    cfg.lateral_accel_limit=1.2;cfg.optimization_envelope_margin=.03
    solver.path.left_width=1.05;solver.path.right_width=.9
    assert_exact(solver,args)


def test_matching_runtime_layout_does_not_allocate_float_and_bool_constraint_rows(monkeypatch):
    solver,args=fixture();original=np.zeros;calls=[]
    def zeros(shape,*a,**kw):
        if shape==solver.dimension:calls.append(np.dtype(kw.get('dtype',float)))
        return original(shape,*a,**kw)
    monkeypatch.setattr(np,'zeros',zeros)
    output=solver._assemble(*args)
    assert output[2].shape[0]==458
    assert calls.count(np.dtype(bool))==0,'fixed sparsity is still rebuilt row by row'
    assert calls.count(np.dtype(float))<=1,'numeric constraint rows still allocate separately'


@pytest.mark.parametrize('change',['corridor','offset_count','same_count_new_values',
                                  'horizon_dimension','debug_constructor','template_shape','template_missing'])
def test_changed_layout_and_debug_assembly_use_exact_original_fallback(change,monkeypatch):
    solver,args=fixture()
    if change=='corridor':solver.config.enforce_corridor=False
    elif change=='offset_count':monkeypatch.setattr(solver.config,'longitudinal_offsets',lambda:(-.31,0.,.38))
    elif change=='same_count_new_values':monkeypatch.setattr(solver.config,'longitudinal_offsets',lambda:(-.34,.43))
    elif change=='horizon_dimension':
        solver.n=15;solver.dimension=4*16+2*15
        initial,applied,_,alignment,_=args
        refs=np.linspace(.7,.25,16);args=(initial,applied,refs,alignment,solver.seed(initial,applied,refs))
    elif change=='debug_constructor':
        # Exercise the original assembly path independently of native setup.
        clone=QPSolver.__new__(QPSolver)
        for key in ['path','config','n','dt','dimension','_transition']:
            setattr(clone,key,getattr(solver,key))
        solver=clone
    elif change=='template_shape':
        # An incompatible cached template must be ignored, not indexed.
        solver._fixed_a_structure=np.zeros((1,1),bool)
    else:del solver._fixed_a_structure
    assert_exact(solver,args)


def test_returned_arrays_and_public_structure_arrays_remain_independent_across_calls():
    solver,args=fixture();first=solver._assemble(*args)
    saved=tuple(a.copy() for a in first)
    p_structure=solver._p_structure;a_structure=solver._a_structure
    expected_p=p_structure.copy();expected_a=a_structure.copy()
    p_structure[:]=False;a_structure[:]=False
    solver.config.half_width+=.02
    second=solver._assemble(*args)
    for previous,current,expected in zip(first,second,saved):
        assert not np.shares_memory(previous,current)
        assert np.array_equal(previous.view(np.uint64),expected.view(np.uint64))
    assert not np.shares_memory(solver._p_structure,p_structure)
    assert not np.shares_memory(solver._a_structure,a_structure)
    assert np.array_equal(solver._p_structure,expected_p)
    assert np.array_equal(solver._a_structure,expected_a)


@pytest.mark.parametrize('case',['running','recovering'])
def test_captured_candidate_linearization_matches_all_original_outputs(case):
    from test_running_braking_capacity import FROZEN_RUNNING
    from test_recovery_braking_capacity import FROZEN,POINTS
    recorded=FROZEN_RUNNING if case=='running' else FROZEN
    cfg=VehicleConfig(**recorded['supervisor']['config'])
    states=np.asarray(recorded['candidate']['states']);controls=np.asarray(recorded['candidate']['controls'])
    solver=QPSolver(ReferencePath(POINTS,.9,.9),cfg,len(controls))
    command=recorded['args'][2]
    applied=np.array([command[k] for k in ('acceleration','steering','steering_rate')])
    # The captured candidate is a fixed numerical linearization input here,
    # not a claim to restore the original optimizer's unavailable workspace.
    assert_exact(solver,(states[0],applied,np.full(len(states),cfg.cruise_speed),
                         np.zeros(3),(states,controls)))


def test_geometry_override_can_update_offsets_before_their_original_evaluation_point(monkeypatch):
    solver,args=fixture();original=solver.geometries
    def changed_geometry(*a,**kw):
        solver.config.half_length=.41
        return original(*a,**kw)
    monkeypatch.setattr(solver,'geometries',changed_geometry)
    expected=original_assemble(solver,*args)
    solver.config.half_length=.28
    actual=solver._assemble(*args)
    for a,b in zip(actual,expected):
        assert np.array_equal(a.view(np.uint64),b.view(np.uint64))


def test_direct_debug_assembly_preserves_original_wider_bound_precision():
    solver,args=fixture(corridor=False)
    solver.config.max_speed=np.longdouble('1.4')
    expected=original_assemble(solver,*args)
    actual=solver._assemble(*args)
    assert expected[4].dtype==np.dtype(np.longdouble)
    for a,b in zip(actual,expected):
        assert a.dtype==b.dtype
        assert np.array_equal(a,b)
