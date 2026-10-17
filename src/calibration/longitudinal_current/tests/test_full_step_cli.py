import json
import pytest
from aims_calibration import cli,planner
from test_planner import a_payload


def baseline_path(tmp_path):
    path=tmp_path/'baseline.json';path.write_text(json.dumps(planner.export_baseline(a_payload())))
    return str(path)


@pytest.mark.parametrize('stage,start,end',[('accel_interval',1.,5.),('decel_current',5.,1.)])
def test_full_preview_defaults_and_measured_policy_are_explicit(tmp_path,capsys,stage,start,end):
    assert cli.plan_main(['--stage',stage,'--sweep-protocol','full_range_step','--base-current-file',baseline_path(tmp_path)])==0
    report=json.loads(capsys.readouterr().out)
    assert report['sweep_protocol']=='full_range_step'
    assert all((p['start_speed'],p['end_speed'])==(start,end) for p in report['plan'])
    assert report['acquisition_policy']['endpoint_exit_policy']=='measured_wheel_speed'
    assert report['acquisition_policy']['response_limits_used'] is False
    assert report['acquisition_policy']['operating_wheel_speed_limit_mps'] is None


def test_full_preview_custom_endpoints_ignores_response_artifact(tmp_path,capsys):
    path=baseline_path(tmp_path);base=json.loads(open(path).read())
    limits=tmp_path/'limits.json';limits.write_text(json.dumps(dict(schema_version=1,current_signal='avg_iq',baseline_sha256=base['baseline_sha256'],limits=[dict(stage='accel_interval',start_speed=1.,current=10.,acceleration_bound=1.)])))
    assert cli.plan_main(['--stage','accel_interval','--sweep-protocol','full_range_step','--base-current-file',path,'--start-speed','1.5','--end-speed','4.5','--response-limits-file',str(limits)])==0
    report=json.loads(capsys.readouterr().out)
    assert report['plan'][0]['start_speed']==1.5 and report['plan'][0]['end_speed']==4.5
    assert report['acquisition_policy']['ignored_response_limits_file']==str(limits)
    assert report['acquisition_policy']['response_limits_used'] is False


def test_interval_preview_default_is_unchanged(tmp_path,capsys):
    assert cli.plan_main(['--stage','accel_interval','--base-current-file',baseline_path(tmp_path)])==0
    report=json.loads(capsys.readouterr().out)
    assert report['plan'][0]['end_speed']==1.5


def test_full_results_cli_prints_response_categories(tmp_path,capsys):
    from test_full_step_io import payload,row
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    path=tmp_path/'trials.json';path.write_text(json.dumps(data))
    assert cli.results_main([str(path)])==0
    result=json.loads(capsys.readouterr().out)
    assert result['complete_responses']==len(data['plan'])


def test_measured_endpoint_preview_ignores_missing_response_artifact(tmp_path,capsys):
    assert cli.plan_main(['--stage','accel_interval','--sweep-protocol','full_range_step',
        '--base-current-file',baseline_path(tmp_path),'--response-limits-file',str(tmp_path/'does-not-exist.json')])==0
    report=json.loads(capsys.readouterr().out)
    assert report['acquisition_policy']['endpoint_exit_policy']=='measured_wheel_speed'
    assert report['acquisition_policy']['current_handoff_enabled'] is True
    assert report['acquisition_policy']['operating_wheel_speed_limit_mps'] is None
    assert report['acquisition_policy']['response_limits_used'] is False
    assert report['acquisition_policy']['ignored_response_limits_file'].endswith('does-not-exist.json')
    assert 'protection_preview' not in report
