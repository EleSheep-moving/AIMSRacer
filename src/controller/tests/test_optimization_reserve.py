"""Configured optimization reserve preserves the separate physical gate."""
import importlib.util
import numpy as np
import pytest
from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


@pytest.mark.parametrize('backend',['ipopt','qp','acados'])
def test_accelerating_candidates_use_configured_reserve_and_physical_validation(backend,tmp_path):
    if backend=='acados' and importlib.util.find_spec('acados_template') is None:pytest.skip('acados native installation required')
    from aims_mpcc.backends import create_solver
    from aims_mpcc.envelope import independent_rollout,utilization
    from aims_mpcc.validation import validate_candidate
    angle=np.linspace(0,2*np.pi,40,endpoint=False);path=ReferencePath(3*np.c_[np.cos(angle),np.sin(angle)],1.,1.)
    cfg=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,half_width=.15,geometry_verified=True,
                      cruise_speed=1.,max_speed=1.5,optimization_envelope_margin=.02)
    if backend=='acados':create_solver(backend,path,cfg,prepare=True,artifact_directory=tmp_path)
    solver=create_solver(backend,path,cfg,artifact_directory=tmp_path)
    steering=float(np.arctan(cfg.wheelbase/3))
    result=solver.solve(dict(x=3.,y=0.,yaw=np.pi/2,speed=0. if backend=='acados' else 1.,steering=steering),
                        dict(acceleration=0.,steering=steering,steering_rate=0.),np.full(11,1.5))
    assert result['success'],result
    samples=independent_rollout(result['states'][0],[0.,steering,0.],result['controls'],cfg)
    peak=max(utilization(samples[k*5+j],u[0],cfg) for k,u in enumerate(result['controls']) for j in range(6))
    assert peak<=1.-cfg.optimization_envelope_margin+1e-3
    assert result['diagnostics']['optimization_envelope_margin']==.02
    assert validate_candidate(dict(result,validation_applied=[0.,steering,0.],dt=.1),cfg,path)['accepted']
    if backend=='acados':assert result['diagnostics']['effective_envelope_margin']==.02
