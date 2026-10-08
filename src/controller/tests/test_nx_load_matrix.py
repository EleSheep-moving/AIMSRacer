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


@pytest.mark.skipif(sys.platform!='linux',reason='NX process ownership uses Linux subreaping')
@pytest.mark.parametrize('proc_children',['available','fallback'])
def test_live_secondary_thread_child_is_terminated_and_reaped(proc_children):
    code='''import importlib.util,subprocess,sys,os,json,time,signal
spec=importlib.util.spec_from_file_location('matrix',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.enable_subreaper()
if sys.argv[2]=='fallback':
 original=m.Path.read_text
 def read_text(path,*args,**kwargs):
  if path.name=='children':raise FileNotFoundError('CONFIG_PROC_CHILDREN unavailable')
  return original(path,*args,**kwargs)
 m.Path.read_text=read_text
child_code='import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print("ready",flush=True); time.sleep(60)'
parent_code='''+repr('''import signal,threading,subprocess,sys,time
signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
def run():
 p=subprocess.Popen([sys.executable,'-c',sys.argv[1]],stdout=subprocess.PIPE,text=True,start_new_session=True)
 assert p.stdout.readline().strip()=='ready'
 print(p.pid,flush=True)
 time.sleep(60)
threading.Thread(target=run).start()
time.sleep(60)
''')+'''
p=subprocess.Popen([sys.executable,'-c',parent_code,child_code],stdout=subprocess.PIPE,text=True,start_new_session=True)
pid=int(p.stdout.readline())
try:
 assert p.poll() is None and pid in m.owned_children(), 'live thread child absent from owned descendants'
 m.cleanup([p],grace=.1)
 assert p.poll() is not None and not os.path.exists('/proc/'+str(pid)), 'thread child survived cleanup'
 print(json.dumps({'cleaned':True}))
finally:
 for target in (pid,p.pid):
  try:os.kill(target,signal.SIGKILL)
  except ProcessLookupError:pass
 p.wait()
 until=time.monotonic()+2.
 while time.monotonic()<until:
  try:
   if os.waitpid(pid,os.WNOHANG)[0]:break
  except ChildProcessError:break
  time.sleep(.02)
'''
    result=subprocess.run([sys.executable,'-c',code,str(TOOL),proc_children],capture_output=True,text=True,timeout=10.)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['cleaned']


def test_proc_enumeration_failure_cannot_claim_successful_cleanup(monkeypatch):
    original=Path.iterdir
    def iterdir(path):
        if str(path).startswith('/proc'):raise PermissionError('proc unavailable')
        return original(path)
    monkeypatch.setattr(Path,'iterdir',iterdir)
    with pytest.raises(RuntimeError,match='enumerate owned'):matrix.cleanup([],grace=.01)


def test_cleanup_signals_a_child_first_seen_during_final_escalation(monkeypatch):
    import signal
    phase={'signal':None,'killed':False}
    def children():
        return [42] if phase['signal']==signal.SIGKILL and not phase['killed'] else [41]
    def kill(pid,sig):
        phase['signal']=sig
        if pid==42 and sig==signal.SIGKILL:phase['killed']=True
    def owned():
        return [] if phase['killed'] else children()
    monkeypatch.setattr(matrix,'owned_children',owned)
    monkeypatch.setattr(matrix.os,'kill',kill)
    monkeypatch.setattr(matrix.os,'waitpid',lambda *args:(0,0))
    matrix.cleanup([],grace=.01)
    assert phase['killed']


def test_cleanup_failure_is_retained_in_failed_matrix_entry(tmp_path,monkeypatch):
    inputs=tmp_path/'input';inputs.write_text('input')
    output=tmp_path/'matrix'
    monkeypatch.setattr(matrix,'enable_subreaper',lambda:None)
    def launch_failure(*args,**kwargs):raise OSError('tegrastats unavailable')
    def cleanup_failure(processes):raise RuntimeError('Owned load processes did not shut down')
    monkeypatch.setattr(matrix.subprocess,'Popen',launch_failure)
    monkeypatch.setattr(matrix,'cleanup',cleanup_failure)
    monkeypatch.setattr(sys,'argv',['matrix','--output',str(output),'--reference',str(inputs),
        '--vehicle-config',str(inputs),'--artifact-directory',str(tmp_path),'--conditions','idle',
        '--repeats','1','--replay-runner',str(inputs),'--bag',str(inputs),'--map',str(inputs),'--seed',str(inputs)])
    with pytest.raises(SystemExit,match='retained failed case'):matrix.main()
    entries=json.loads((output/'matrix.json').read_text())['results']
    assert len(entries)==1 and entries[0]['status']=='cleanup_failed'
    assert 'Owned load processes' in entries[0]['cleanup_error']
    assert 'tegrastats unavailable' in entries[0]['error']


@pytest.mark.parametrize('exit_code,summary,init_exit,intentional,expected',[
    (1,True,0,False,'invalid_replay'),
    (0,False,0,False,'invalid_replay'),
    (0,'malformed',0,False,'artifact_failed'),
    (0,'on_wait',0,False,'recorded'),
    (0,True,1,False,'invalid_replay'),
    (-2,True,0,True,'invalid_replay'),
    (130,True,0,True,'invalid_replay'),
    (-2,False,0,True,'invalid_replay'),
    (-2,True,0,False,'invalid_replay'),
])
def test_matrix_requires_replay_summary_and_successful_audits(
        tmp_path,monkeypatch,exit_code,summary,init_exit,intentional,expected):
    inputs=tmp_path/'input';inputs.write_text('input')
    output=tmp_path/'matrix'
    class Process:
        pid=42
        def __init__(self,name):
            self.name=name;self.calls=0;self.waited=False;self.wait_calls=0
            self.returncode=exit_code if name=='replay' else 0
        def poll(self):
            self.calls+=1
            if self.name=='replay' and (self.calls==1 or (intentional or summary=='on_wait') and not self.waited):return None
            return self.returncode
        def wait(self,timeout):
            self.wait_calls+=1
            if self.wait_calls==1:
                assert timeout==65.
                if intentional:raise subprocess.TimeoutExpired('replay',timeout)
            else:assert timeout==15.
            self.waited=True
            if summary=='on_wait':write_summary(output/'shared-1'/'replay')
            return self.returncode
    def write_summary(directory):
        report=dict(counts={'ekf':100,'body_cloud':10},
            tf_authorities={'odom/base_link':1,'map/odom':1},accepted=10,initializer_cli_exit=init_exit)
        (directory/'summary.json').write_text('{' if summary=='malformed' else json.dumps(report))
    def spawn(command,**kwargs):
        name=Path(kwargs['stdout'].name).stem
        if name=='replay':
            directory=output/'shared-1'/'replay';directory.mkdir()
            events=[dict(kind='native_timing',received=10.+i/10) for i in range(11)]
            events.append(dict(kind='anchor',values={'anchor_committed':'true'}))
            (directory/'events.jsonl').write_text(''.join(json.dumps(event)+'\n' for event in events))
            if summary and summary!='on_wait':write_summary(directory)
        if name=='benchmark':
            report={'cases':[{'summary':{},'requests':[dict(submitted_at=10.,completed_at=11.,full_request_s=1.)]}]}
            (output/'shared-1'/'benchmark.json').write_text(json.dumps(report))
        return Process(name)
    monkeypatch.setattr(matrix,'enable_subreaper',lambda:None)
    monkeypatch.setattr(matrix,'cleanup',lambda processes:None)
    monkeypatch.setattr(matrix.subprocess,'Popen',spawn)
    monkeypatch.setattr(matrix.os,'killpg',lambda *args:None)
    monkeypatch.setattr(matrix.time,'monotonic',lambda:10.)
    monkeypatch.setattr(sys,'argv',['matrix','--output',str(output),'--reference',str(inputs),
        '--vehicle-config',str(inputs),'--artifact-directory',str(tmp_path),'--conditions','shared',
        '--seconds','1','--repeats','1','--replay-runner',str(inputs),'--bag',str(inputs),'--map',str(inputs),'--seed',str(inputs)])
    if expected=='recorded':matrix.main()
    else:
        with pytest.raises(SystemExit,match='retained failed case'):matrix.main()
    entries=json.loads((output/'matrix.json').read_text())['results']
    assert len(entries)==1 and entries[0]['status']==expected
    assert entries[0]['load_coverage']['valid']
    assert entries[0]['load_qualified']==(expected=='recorded')
    if expected=='invalid_replay':assert entries[0]['replay_errors']
    if expected=='artifact_failed':assert entries[0]['artifact_error']
    if summary=='on_wait':
        assert entries[0]['replay_completed_normally']
        assert entries[0]['replay_drain_s']==0.
        assert not entries[0].get('intentional_replay_shutdown')
    if intentional:
        assert entries[0]['replay_drain_timeout']
        assert entries[0]['intentional_replay_shutdown']
