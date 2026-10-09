import importlib.util
from pathlib import Path
import pytest
import os
import sys


def module():
    file=Path(__file__).resolve().parents[1]/'tools/qualification_plan.py'
    spec=importlib.util.spec_from_file_location('qualification_plan',file)
    answer=importlib.util.module_from_spec(spec);spec.loader.exec_module(answer)
    return answer


def test_prepared_matrix_covers_all_leads_rates_and_matched_profiles(tmp_path):
    plan=module().build_plan(Path('/workspace'),Path('/bundles'),tmp_path,Path('/vesc.yaml'))
    jobs=plan['groups']['protocol']
    def value(command,key):return command[command.index(key)+1]
    actual={(float(value(job['command'],'--handover-delay')),float(value(job['command'],'--frequency'))) for job in jobs}
    assert actual=={(lead,frequency) for lead in (0.,.02,.05,.1) for frequency in (10.,20.,40.)}
    assert all('qual-v2-circle-v05-n10' in value(job['command'],'--bundle') for job in jobs)
    assert {case['name'] for case in plan['comparison']['cases']}=={'circle-v05','circle-v10','route-v05','route-v10'}
    command=plan['groups']['matched'][0]['command']
    assert value(command,'--seconds')=='90'
    assert command[-3:]==['.75','1','1.25']


def test_frozen_source_changes_are_rejected_before_starting_a_case(tmp_path):
    file=tmp_path/'runtime';file.write_text('original')
    tool=module();frozen=tool.fingerprint([file])
    tool.verify_frozen(frozen)
    file.write_text('changed')
    with pytest.raises(RuntimeError,match='changed'):tool.verify_frozen(frozen)


def test_loaded_nx_plan_has_three_bounded_full180_second_runs(tmp_path):
    load=dict(replay_runner=Path('/replay.py'),bag=Path('/bag'),map=Path('/map'),seed=Path('/seed'))
    plan=module().build_plan(Path('/workspace'),Path('/bundles'),tmp_path,Path('/vesc.yaml'),load=load)
    jobs=plan['groups']['nx']
    assert len(jobs)==3
    for job in jobs:
        command=job['command']
        assert command[command.index('--seconds')+1]=='180'
        assert '--shared' in command and '--replay-runner' in command
        assert job['timeout_s']==360


def test_case_timeout_reaps_detached_descendants(tmp_path):
    ready=tmp_path/'child-pid'
    child='import os,pathlib,time;pathlib.Path('+repr(str(ready))+').write_text(str(os.getpid()));time.sleep(30)'
    launcher='import subprocess,sys,time;subprocess.Popen([sys.executable,"-c",'+repr(child)+'],start_new_session=True);time.sleep(30)'
    root=Path(__file__).resolve().parents[3]
    with (tmp_path/'log').open('w') as log:
        result=module().run_case([sys.executable,'-c',launcher],dict(os.environ),log,.15,root,cleanup_grace=.1)
    assert result['error']=='bounded case timeout'
    assert ready.exists() and not Path('/proc/'+ready.read_text()).exists()


def test_acceptance_tracking_success_cannot_hide_timing_gate_failure(tmp_path):
    import json
    report=dict(overall_pass=True,timing=dict(requests=100,measurement_s=5.,publications=250,
        failures=2,max_consecutive_failures=1,log_integrity_pass=True,
        complete=dict(p95=.001,p99=.002),publish_gap=dict(p99=.02,max=.021)))
    (tmp_path/'report.json').write_text(json.dumps(report))
    job=dict(command=['acceptance.py','--output',str(tmp_path)])
    result=module().assess_case('acceptance',job,Path(__file__).resolve().parents[3],dict(returncode=0))
    assert result['tracking_pass'] and not result['timing_pass'] and not result['qualified']
