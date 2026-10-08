"""Real native cores and their numerical candidate contract (no output authority)."""
import importlib.util
from dataclasses import replace
import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def circle(frame='odom'):
    angles=np.linspace(0,2*np.pi,40,endpoint=False)
    return ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.,frame,
                         {'map_sha256':'a'*64} if frame=='map' else None)


def config(**values):
    return VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,
                         half_width=.15,geometry_verified=True,**values)


def inputs():
    return dict(x=3.,y=0.,yaw=np.pi/2,speed=.5,steering=np.arctan(.36/3)),dict(
        acceleration=0.,steering=np.arctan(.36/3),steering_rate=0.)


def test_factory_unknown_backend_is_explicit():
    from aims_mpcc.backends import create_solver
    with pytest.raises(ValueError,match='backend'):
        create_solver('imaginary',circle(),config())


@pytest.mark.parametrize('backend',['qp','acados'])
def test_native_backend_contract_and_rollout(backend,tmp_path):
    if backend=='acados' and importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.backends import create_solver
    if backend=='acados':
        create_solver(backend,circle(),config(),prepare=True,artifact_directory=tmp_path)
    solver=create_solver(backend,circle(),config(),artifact_directory=tmp_path)
    state,previous=inputs()
    result=solver.solve(state,previous,np.full(11,.5))
    assert result['success'], result
    assert result['execution_authorized'] is False
    assert np.asarray(result['states']).shape==(11,6)
    assert np.asarray(result['controls']).shape==(10,3)
    assert result['constraint_violation']<1e-4
    assert result['diagnostics']['native_core'] in ('acados/HPIPM','OSQP C')
    assert result['solve_time_s']>=result['diagnostics']['optimizer_time_s']
    from aims_mpcc.envelope import independent_rollout,evaluate_envelope
    numeric=independent_rollout(result['states'][0],[previous[k] for k in
        ('acceleration','steering','steering_rate')],result['controls'],config())[::5]
    assert np.allclose(numeric,result['states'],atol=2e-5)
    from aims_mpcc.validation import validate_candidate
    candidate=dict(result,validation_applied=[previous[k] for k in ('acceleration','steering','steering_rate')],
                   previous_steering=previous['steering'],dt=.1)
    assert validate_candidate(candidate,config(),circle())['accepted']
    solver.reset()
    assert solver.previous is None


def test_acados_online_never_compiles_missing_artifact(tmp_path):
    from aims_mpcc.backends import create_solver
    with pytest.raises((FileNotFoundError,RuntimeError),match='prepar|artifact'):
        create_solver('acados',circle(),config(),artifact_directory=tmp_path)


@pytest.mark.parametrize('backend',['qp','acados'])
def test_map_alignment_equivalent_candidates(backend,tmp_path):
    if backend=='acados' and importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.backends import create_solver
    alignment=np.array([4.,-2.,.3]); c,s=np.cos(alignment[2]),np.sin(alignment[2])
    rotation=np.array([[c,-s],[s,c]])
    path=circle(); mapped=ReferencePath(path.points@rotation.T+alignment[:2],1.,1.,'map',{'map_sha256':'b'*64})
    solvers=[]
    for reference in (path,mapped):
        if backend=='acados': create_solver(backend,reference,config(),prepare=True,artifact_directory=tmp_path)
        solvers.append(create_solver(backend,reference,config(),artifact_directory=tmp_path))
    state,previous=inputs()
    a=solvers[0].solve(state,previous,np.full(11,.5))
    b=solvers[1].solve(state,previous,np.full(11,.5),map_alignment=alignment)
    assert a['success'] and b['success'],(a,b)
    assert np.allclose(a['controls'],b['controls'],atol=2e-4)
    assert np.allclose(a['states'],b['states'],atol=2e-4)


@pytest.mark.parametrize('backend',['qp','acados'])
def test_corridor_switch_reaches_native_and_independent_validator(backend,tmp_path):
    if backend=='acados' and importlib.util.find_spec('acados_template') is None:
        pytest.skip('acados native installation required')
    from aims_mpcc.backends import create_solver
    from aims_mpcc.validation import validate_candidate
    state,previous=inputs();state.update(x=4.2,speed=0.);previous['steering']=state['steering']=0.
    for enabled in (False,True):
        cfg=config(enforce_corridor=enabled)
        if backend=='acados':create_solver(backend,circle(),cfg,prepare=True,artifact_directory=tmp_path)
        result=create_solver(backend,circle(),cfg,artifact_directory=tmp_path).solve(state,previous,np.zeros(11))
        plan=dict(result,validation_applied=[0.,0.,0.],dt=.1)
        if enabled:
            assert not result['success']
            assert not validate_candidate(plan,cfg,circle())['accepted']
        else:
            assert result['success'],result
            assert validate_candidate(plan,cfg,circle())['accepted']


def test_qp_soft_recovery_explicitly_unsupported():
    from aims_mpcc.backends import create_solver
    state,previous=inputs()
    result=create_solver('qp',circle(),config(envelope_soft_enabled=True)).solve(state,previous)
    assert not result['success'] and result['status']=='unsupported'
    assert 'soft_operating_envelope' in result['diagnostics']['unsupported_constraints']


def test_acados_soft_cap_is_utilization_cap_and_deadline_is_time(tmp_path):
    if importlib.util.find_spec('acados_template') is None:pytest.skip('acados native installation required')
    from aims_mpcc.backends import create_solver
    cfg=config(envelope_soft_enabled=True,envelope_slack_limit=.5,envelope_recovery_time=.6)
    solver=create_solver('acados',circle(),cfg,prepare=True,artifact_directory=tmp_path)
    state,previous=inputs();result=solver.solve(state,previous)
    assert result['success'],result
    x=np.r_[result['states'][5],0.,previous['steering'],0.]
    u=np.r_[0.,previous['steering'],.5,.4]
    p=np.r_[solver.geometry(x[4],np.zeros(3)),.5,1.,.1,0.]
    constraints=np.asarray(solver._constraints(x,u,p)).ravel()
    assert constraints[-1]==pytest.approx(-.1)  # slack .4 below E cap .5
    p[-2]=0.
    assert float(solver._constraints(x,u,p)[-1])==pytest.approx(.4)  # deadline forbids slack
    changed=replace(cfg,steering_tau=.12)
    with pytest.raises(FileNotFoundError,match='artifact'):
        create_solver('acados',circle(),changed,artifact_directory=tmp_path)


def _record_real_warm_start(solver):
    calls=[];native=solver._native.warm_start
    def capture(*,x,y):
        calls.append((np.asarray(x).copy(),np.asarray(y).copy()))
        return native(x=x,y=y)
    solver._native.warm_start=capture
    return calls


def test_qp_native_primal_shift_counts_failed_request_age():
    from aims_mpcc.backends import create_solver
    solver=create_solver('qp',circle(),config())
    state,previous=inputs();first=solver.solve(state,previous,np.full(11,.5))
    assert first['success']
    valid_controls=np.asarray(first['controls']).copy()
    calls=_record_real_warm_start(solver)
    outside=dict(state,x=4.2,speed=0.,steering=0.)
    applied=dict(acceleration=0.,steering=0.,steering_rate=0.)
    failed=solver.solve(outside,applied,np.zeros(11),elapsed=.1)
    assert not failed['success']
    assert np.allclose(solver.previous['controls'],valid_controls)
    final=solver.solve(state,previous,np.full(11,.5),elapsed=.2)
    assert final['success']
    assert len(calls)==2  # each real solve must explicitly initialize native primal
    assert final['diagnostics']['warm_start_age_s']==pytest.approx(.3)
    assert final['diagnostics']['warm_start_shift_steps']==3
    warm_controls=np.asarray(final['solve_input']['warm_controls'])
    shifted=np.vstack((valid_controls[3:,:2],np.repeat(valid_controls[-1:,:2],3,axis=0)))
    assert np.allclose(warm_controls[:,:2],shifted,atol=1e-10)
    assert np.allclose(calls[-1][0][4*11:].reshape(10,2),warm_controls[:,:2])
    assert np.all(calls[-1][1]==0.)  # dual stage rows cannot be shifted reliably


def test_qp_native_primal_reprojects_after_map_alignment_change():
    from aims_mpcc.backends import create_solver
    solver=create_solver('qp',circle('map'),config())
    state,previous=inputs();first=solver.solve(state,previous,np.full(11,.5),map_alignment=np.zeros(3))
    assert first['success']
    calls=_record_real_warm_start(solver)
    alignment=np.array([.07,-.03,.03])
    result=solver.solve(state,previous,np.full(11,.5),map_alignment=alignment)
    assert len(calls)==1
    seed=np.asarray(result['solve_input']['warm_states']);native=calls[0][0][:44].reshape(11,4)
    for k,x in enumerate(seed):
        g=solver.geometry(x[4],alignment);normal=np.array([-np.sin(g[2]),np.cos(g[2])])
        expected=np.r_[np.dot(x[:2]-g[:2],normal),(x[2]-g[2]+np.pi)%(2*np.pi)-np.pi,x[5],x[3]]
        assert np.allclose(native[k],expected,atol=1e-12)
    assert abs(native[0,0])>1e-3  # the former frame's zero lateral seed is invalid
    assert np.all(calls[0][1]==0.)


@pytest.mark.parametrize('update_json_hash',[False,True])
def test_acados_rejects_redirected_loader_metadata_without_online_compile(tmp_path,monkeypatch,update_json_hash):
    if importlib.util.find_spec('acados_template') is None:pytest.skip('acados native installation required')
    import hashlib,json
    from aims_mpcc.backends import create_solver
    from acados_template import AcadosOcpSolver
    solver=create_solver('acados',circle(),config(),prepare=True,artifact_directory=tmp_path)
    original_json=solver.json_file.read_bytes();original_manifest=solver._manifest.read_bytes()
    def forbidden_compile(*args,**kwargs):raise AssertionError('online compilation attempted')
    monkeypatch.setattr(AcadosOcpSolver,'generate',forbidden_compile)
    monkeypatch.setattr(AcadosOcpSolver,'build',forbidden_compile)
    mutations=[('name','aims_unverified'),('code_export_directory',str(tmp_path/'unverified')),
               ('acados_lib_path',str(tmp_path/'unverified-libraries')),
               ('shared_lib_ext','.unverified'),('dims.N',9),('solver_options.N_horizon',9),
               ('model.name','aims_unverified'),('solver_options.nlp_solver_type','SQP')]
    if not update_json_hash:mutations.insert(0,('unverified_annotation','changed bytes'))
    try:
        for field,value in mutations:
            document=json.loads(original_json)
            keys=field.split('.');node=document
            for key in keys[:-1]:node=node[key]
            node[keys[-1]]=value
            solver.json_file.write_text(json.dumps(document))
            manifest=json.loads(original_manifest)
            if update_json_hash:
                manifest['ocp_json_sha256']=hashlib.sha256(solver.json_file.read_bytes()).hexdigest()
            solver._manifest.write_text(json.dumps(manifest))
            with pytest.raises(FileNotFoundError,match='artifact'):
                create_solver('acados',circle(),config(),artifact_directory=tmp_path)
    finally:
        solver.json_file.write_bytes(original_json);solver._manifest.write_bytes(original_manifest)
