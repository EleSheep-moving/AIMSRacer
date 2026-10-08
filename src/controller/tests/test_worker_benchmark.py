"""Full asynchronous no-drive accounting, including failed and late replies."""
from dataclasses import asdict
import importlib.util
import numpy as np
import pytest
from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def synthetic_bundle(tmp_path):
    angle=np.linspace(0,2*np.pi,40,endpoint=False)
    config=VehicleConfig(profile='synthetic',rear_offset=.15,half_length=.28,half_width=.15,geometry_verified=True)
    path=ReferencePath(3*np.c_[np.cos(angle),np.sin(angle)],1.,1.)
    directory=tmp_path/'reference';path.save(directory)
    return directory,config


def test_full_accounting_keeps_failed_late_and_censored_requests():
    from aims_mpcc.worker_benchmark import summarize
    rows=[dict(success=True,full_request_s=.01,discarded=False,validation=dict(accepted=True),components={'optimizer_s':.002}),
          dict(success=False,full_request_s=.2,discarded=False,validation=dict(accepted=False),components={'optimizer_s':.18}),
          dict(success=True,full_request_s=.1,discarded=True,validation=dict(accepted=True),components={'optimizer_s':.03}),
          dict(success=False,full_request_s=.3,discarded=True,censored=True,validation=None,components={})]
    result=summarize(rows,.05)
    assert result['requests']==4 and result['failures']==2
    assert result['late_replies']==1 and result['censored_requests']==1
    assert result['executable_candidates']==1
    assert result['full_request_p95_s']>.28 and result['full_request_p99_s']>.29
    assert result['full_request_max_s']==.3
    assert result['components']['optimizer_s']['observations']==3
    assert result['components']['optimizer_s']['max_s']==.18
    assert summarize([])['full_request_max_s'] is None


def test_real_qp_worker_late_delivery_and_caller_validation_are_timed(tmp_path):
    if importlib.util.find_spec('osqp') is None:pytest.skip('native OSQP required')
    from aims_mpcc.worker_benchmark import run_case
    directory,config=synthetic_bundle(tmp_path)
    result=run_case(directory,config,'qp',10,samples=3,period=.005,deadline=.001,
                    budget=.001,startup_timeout=10.)
    assert result['status']=='completed',result
    assert result['summary']['requests']==3
    assert result['summary']['late_replies']==3
    assert result['summary']['executable_candidates']==0
    assert result['includes_ipc'] and result['includes_caller_validation']
    for row in result['requests']:
        assert row['delivered'] and row['discarded']
        assert row['validation'] is not None
        assert row['full_request_s']>=row['delivery_s']>0.
        assert row['components']['caller_validation_s']>=0.
        assert row['components']['worker_validation_s']>=0.
    assert result['blocked_submission_opportunities']>0


def test_acados_missing_prepared_artifact_is_reported_without_compilation(tmp_path):
    from aims_mpcc.worker_benchmark import run_case
    directory,config=synthetic_bundle(tmp_path);cache=tmp_path/'unprepared'
    result=run_case(directory,config,'acados',10,samples=1,artifact_directory=cache,startup_timeout=10.)
    assert result['status']=='startup_failed'
    assert 'artifact' in result['error'].lower() and 'prepare' in result['error'].lower()
    assert result['summary']['requests']==0
    assert not list(cache.rglob('*.so'))
    assert not list(cache.rglob('ocp.json'))


def test_cli_default_mode_cannot_prepare_and_keeps_startup_error(tmp_path,monkeypatch):
    import json,yaml
    from aims_mpcc.worker_benchmark import main
    from aims_mpcc import prepare_solver
    directory,config=synthetic_bundle(tmp_path)
    config_file=tmp_path/'vehicle.yaml';config_file.write_text(yaml.safe_dump(asdict(config)))
    output=tmp_path/'worker-report.json';cache=tmp_path/'unprepared'
    def forbidden_prepare(*args,**kwargs):raise AssertionError('default benchmark mode attempted offline preparation')
    monkeypatch.setattr(prepare_solver,'main',forbidden_prepare)
    code=main(['--reference',str(directory),'--vehicle-config',str(config_file),'--backend','acados',
               '--horizon','10','--samples','1','--artifact-directory',str(cache),
               '--startup-timeout','10','--output',str(output)])
    report=json.loads(output.read_text())
    assert code==1
    assert report['layer']=='full-async-worker' and not report['online_compilation_allowed']
    assert report['cases'][0]['status']=='startup_failed'
    assert report['cases'][0]['summary']['requests']==0
    assert not report['production_node_timer_included'] and not report['supervisor_handover_included']
    assert not report['execution_authorized']
    assert not list(cache.rglob('*.so'))


def test_duration_limit_drains_every_submitted_request(tmp_path):
    if importlib.util.find_spec('osqp') is None:pytest.skip('native OSQP required')
    from aims_mpcc.worker_benchmark import run_case
    directory,config=synthetic_bundle(tmp_path)
    result=run_case(directory,config,'qp',10,samples=None,duration=.005,period=.05,
                    startup_timeout=10.)
    assert result['status']=='completed',result
    assert result['submitted_requests']>=1
    assert result['submitted_requests']==len(result['requests'])
    assert all(r['delivered'] for r in result['requests'])
    assert result['collection_wall_time_s']>=result['requests'][0]['full_request_s']


def test_prepare_only_records_real_native_warmup_status_without_worker_collection(tmp_path):
    if importlib.util.find_spec('osqp') is None:pytest.skip('native OSQP required')
    import json,yaml
    from aims_mpcc.worker_benchmark import main
    directory,config=synthetic_bundle(tmp_path)
    config_file=tmp_path/'vehicle.yaml';config_file.write_text(yaml.safe_dump(asdict(config)))
    output=tmp_path/'prepared.json'
    code=main(['--reference',str(directory),'--vehicle-config',str(config_file),'--backend','qp',
               '--horizon','10','--prepare-only','--output',str(output)])
    report=json.loads(output.read_text());case=report['cases'][0]
    assert code==0 and report['layer']=='offline-preparation'
    assert report['offline_compilation_allowed'] and not report['online_compilation_allowed']
    assert case['status']=='prepared'
    assert case['warmup']['status']=='solved'
    assert case['warmup']['diagnostics']['native_core']=='OSQP C'
    assert 'requests' not in case


def test_optional_sample_lap_moves_nominal_reference_inputs(tmp_path):
    if importlib.util.find_spec('osqp') is None:pytest.skip('native OSQP required')
    from aims_mpcc.worker_benchmark import nominal_request,run_case
    directory,config=synthetic_bundle(tmp_path);path=ReferencePath.load(directory)
    request=nominal_request(path,config,10,2.,.1,theta=1.)
    ref=path.at(1.)
    assert request['state']['x']==ref['x'] and request['state']['y']==ref['y']
    result=run_case(directory,config,'qp',10,samples=3,period=.02,sample_lap=True,startup_timeout=10.)
    assert result['status']=='completed'
    assert result['scenario']=='synthetic-reference-following-states'
    theta=[r['reference_progress'] for r in result['requests']]
    assert theta[0]==0. and theta[0]<theta[1]<theta[2]
    assert all(r['validation']['accepted'] for r in result['requests'])
    assert not result['execution_authorized']
