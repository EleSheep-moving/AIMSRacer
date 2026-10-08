"""A detached replay child and an idle tail must not escape benchmark accounting."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


TOOL=Path(__file__).parents[1]/'tools/nx_load_matrix.py'
spec=importlib.util.spec_from_file_location('nx_load_matrix',TOOL)
matrix=importlib.util.module_from_spec(spec);spec.loader.exec_module(matrix)


def test_short_replay_cannot_qualify_an_idle_tail_as_shared_load():
    rows=[dict(submitted_at=.5,completed_at=1.,full_request_s=.5),
          dict(submitted_at=9.,completed_at=9.5,full_request_s=.5)]
    events=[dict(kind='native_timing',received=i/10) for i in range(101)]
    events.append(dict(kind='anchor',values={'anchor_committed':'true'}))
    result=matrix.load_coverage(rows,events,ended_monotonic=8.)
    assert not result['valid'] and result['total_requests']==2
    assert matrix.load_coverage(rows,events)['valid']
    early=[event for event in events if event.get('received',0)<=2.]
    partial=matrix.load_coverage(rows,early)
    assert not partial['valid'] and partial['fully_overlapping_requests']==1
    assert partial['shared_interval_s']==1.5


def test_censored_worker_failure_is_retained_in_load_window():
    rows=[dict(submitted_at=1.,full_request_s=10.,censored=True)]
    events=[dict(kind='native_timing',received=1.),dict(kind='native_timing',received=2.)]
    result=matrix.load_coverage(rows,events)
    assert not result['valid'] and result['measurement_end']==11.


@pytest.mark.skipif(sys.platform!='linux',reason='NX process ownership uses Linux subreaping')
def test_already_exited_parent_detached_child_is_terminated_and_reaped():
    # Run in a separate driver so subreaping never affects the pytest process.
    code='''import importlib.util,subprocess,sys,os,json,time
spec=importlib.util.spec_from_file_location('matrix',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.enable_subreaper()
child_code='import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)'
parent_code='import subprocess,sys; p=subprocess.Popen([sys.executable,"-c",sys.argv[1]],start_new_session=True); print(p.pid,flush=True)'
p=subprocess.Popen([sys.executable,'-c',parent_code,child_code],stdout=subprocess.PIPE,text=True)
pid=int(p.stdout.readline());p.wait();time.sleep(.1)
assert p.poll()==0 and os.path.exists('/proc/'+str(pid))
m.cleanup([p],grace=.1)
assert not os.path.exists('/proc/'+str(pid)), 'detached child survived cleanup'
print(json.dumps({'cleaned':True}))
'''
    result=subprocess.run([sys.executable,'-c',code,str(TOOL)],capture_output=True,text=True,timeout=10.)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['cleaned']


def test_native_callback_bursts_cannot_hide_an_interior_idle_gap():
    rows=[dict(submitted_at=1.,completed_at=299.,full_request_s=298.)]
    events=[dict(kind='native_timing',received=i/200+1.) for i in range(200)]
    events+=[dict(kind='native_timing',received=i/200+299.) for i in range(200)]
    events.append(dict(kind='anchor',values={'anchor_committed':'true'}))
    result=matrix.load_coverage(rows,events)
    assert not result['valid'] and result['max_native_gap_s']>297.
