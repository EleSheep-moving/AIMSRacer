"""Native status eligibility never replaces matrix or physical candidate gates."""
import copy

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath
from aims_mpcc.qp_backend import QPSolver
from aims_mpcc.runtime import Command,State,Supervisor
from aims_mpcc.validation import validate_candidate


def fixture():
    angles=np.linspace(0.,2*np.pi,40,endpoint=False)
    path=ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,
                      half_width=.15,geometry_verified=True,cruise_speed=.5)
    state=dict(x=3.,y=0.,yaw=np.pi/2,speed=.5,steering=np.arctan(.36/3))
    previous=dict(acceleration=0.,steering=state['steering'],steering_rate=0.)
    return QPSolver(path,cfg,horizon=3),state,previous


def native_status(solver,monkeypatch,status_val,damage=None):
    original=solver._native.solve;captured={}
    def solve(**kwargs):
        solution=original(**kwargs)
        assert solution.info.status_val==1
        captured['x']=solution.x.copy()
        if damage is not None:damage(solution)
        solution.info.status_val=status_val
        solution.info.status={1:'solved',2:'solved inaccurate',7:'maximum iterations reached'}[status_val]
        captured['info']=copy.copy(solution.info)
        return solution
    monkeypatch.setattr(solver._native,'solve',solve)
    return captured


@pytest.mark.parametrize('status_val',[1,2])
@pytest.mark.parametrize('matrix_residual',[0.,8.767800433462369e-7])
def test_feasible_native_candidate_preserves_accuracy_status_and_downstream_gates(status_val,matrix_residual,monkeypatch):
    solver,state,previous=fixture()
    def residual(solution):
        solution.x[3]+=matrix_residual
    captured=native_status(solver,monkeypatch,status_val,residual)
    result=solver.solve(state,previous,np.full(4,.5))
    assert result['success'],result
    diagnostic=result['diagnostics']
    assert diagnostic['osqp_status_val']==status_val
    assert result['status']==captured['info'].status==diagnostic['osqp_status']
    assert diagnostic['native_converged']==(status_val==1)
    assert diagnostic['native_candidate_status_eligible'] is True
    assert diagnostic['candidate_feasible'] is True
    assert diagnostic['success_scope']=='candidate_feasibility'
    assert diagnostic['osqp_primal_residual']==captured['info'].prim_res
    assert diagnostic['osqp_dual_residual']==captured['info'].dual_res
    assert diagnostic['qp_constraint_violation']==pytest.approx(matrix_residual,abs=1e-12)
    # Status handling cannot modify the returned native acceleration/steering.
    assert np.array_equal(np.asarray(result['controls'])[:,:2],captured['x'][16:].reshape(3,2))
    assert result['execution_authorized'] is False
    applied=[previous[k] for k in ('acceleration','steering','steering_rate')]
    candidate=dict(result,validation_applied=applied,dt=.1)
    candidate['validation']=validate_candidate(candidate,solver.config,solver.path)
    assert candidate['validation']['accepted']
    # The unchanged handover and execution gates still decide activation.
    supervisor=Supervisor(solver.config,solver.path.length,handover_delay=.02,solve_period=.05)
    supervisor.observe(State(state['x'],state['y'],state['yaw'],0.,state['steering'],100.),10.,0.,0.)
    supervisor.set_mode(True,10.);supervisor.start(10.)
    supervisor.state=State(**state,timestamp=100.02)
    supervisor.state_received=supervisor.mode_received=10.02
    supervisor.last_command=Command(state['speed'],previous['steering'])
    candidate.update(generation=supervisor.generation,source_stamp=10.,submitted_at=10.,stamp=10.02)
    assert supervisor.accept(candidate,10.02)
    actual=dict(speed=state['speed'],**previous)
    assert supervisor.activate(10.02,supervisor.state,actual,path=solver.path)
    assert supervisor.plan['execution_validation']['accepted']


def test_status_two_with_matrix_residual_outside_existing_bound_is_rejected(monkeypatch):
    solver,state,previous=fixture()
    def outside(solution):
        solution.x[3]+=1.01e-4
    native_status(solver,monkeypatch,2,outside)
    result=solver.solve(state,previous,np.full(4,.5))
    assert not result['success']
    assert result['diagnostics']['qp_constraint_violation']>1e-4
    assert result['diagnostics']['envelope']['hard_control_violation']<1e-4
    assert result['diagnostics']['native_candidate_status_eligible'] is True
    assert result['diagnostics']['candidate_feasible'] is False
    assert solver.previous is None


def test_status_two_with_physical_violation_is_rejected_even_with_permissive_matrix(monkeypatch):
    solver,state,previous=fixture()
    original=solver._assemble
    def permissive(*args):
        P,q,A,lower,upper=original(*args)
        return P,q,A,np.full_like(lower,-np.inf),np.full_like(upper,np.inf)
    monkeypatch.setattr(solver,'_assemble',permissive)
    # Retain the real nonlinear evaluator; violate its immutable applied jerk.
    native_status(solver,monkeypatch,2,lambda solution:solution.x.__setitem__(16,.100101))
    result=solver.solve(state,previous,np.full(4,.5))
    assert not result['success']
    assert result['diagnostics']['qp_constraint_violation']==0.
    assert result['diagnostics']['envelope']['hard_control_violation']>1e-4
    assert result['diagnostics']['candidate_feasible'] is False


def test_maximum_iterations_is_ineligible_even_when_candidate_is_feasible(monkeypatch):
    solver,state,previous=fixture()
    native_status(solver,monkeypatch,7)
    result=solver.solve(state,previous,np.full(4,.5))
    assert not result['success']
    assert result['constraint_violation']<1e-4
    assert result['diagnostics']['candidate_feasible'] is True
    assert result['diagnostics']['native_candidate_status_eligible'] is False
    assert result['diagnostics']['native_converged'] is False
    assert result['diagnostics']['osqp_status_val']==7
    assert result['status']=='qp_candidate_rejected:maximum iterations reached'
    assert solver.previous is None


@pytest.mark.parametrize('damage',[lambda solution:setattr(solution,'x',None),
                                   lambda solution:solution.x.__setitem__(0,np.nan)])
def test_status_two_requires_finite_primal_candidate(damage,monkeypatch):
    solver,state,previous=fixture()
    native_status(solver,monkeypatch,2,damage)
    result=solver.solve(state,previous,np.full(4,.5))
    assert not result['success']
    assert result['diagnostics']['candidate_feasible'] is False
    assert 'states' not in result and 'controls' not in result
