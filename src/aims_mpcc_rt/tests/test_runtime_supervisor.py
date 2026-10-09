"""Process lifecycle checks use disposable children and never publish drive."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def module():
    path=Path(__file__).resolve().parents[1]/'tools/runtime_supervisor.py'
    spec=importlib.util.spec_from_file_location('runtime_supervisor',path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


def test_worker_deadline_detects_native_stall_even_with_fresh_status():
    monitor=module().StallMonitor(started=10.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert monitor.observe(dict(status='READY',worker_busy=False,worker_elapsed_s=0.),10.1) is None
    assert monitor.observe(dict(status='RUNNING',worker_busy=True,worker_elapsed_s=.51),10.2)=='native_worker_stalled'


def test_each_started_child_must_report_disabled_and_missing_status_is_bounded():
    cls=module().StallMonitor
    monitor=cls(started=10.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert monitor.observe(dict(status='RUNNING',worker_busy=False,worker_elapsed_s=0.),10.1)=='startup_not_disabled'
    restarted=cls(started=11.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert restarted.observe(dict(status='READY',worker_busy=False,worker_elapsed_s=0.),11.1) is None
    assert restarted.check(11.51)=='status_stalled'
    assert cls(started=10.,startup_timeout=2.).check(12.01)=='startup_status_timeout'


def test_sigterm_then_sigkill_bounds_shutdown_of_unresponsive_process(tmp_path):
    ready=tmp_path/'ready'
    code='import signal,time,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path('+repr(str(ready))+').touch(); time.sleep(30)'
    child=subprocess.Popen([sys.executable,'-c',code],start_new_session=True)
    try:
        until=time.monotonic()+2.
        while not ready.exists() and time.monotonic()<until:time.sleep(.005)
        assert ready.exists()
        begin=time.monotonic()
        result=module().terminate_child(child,grace_s=.05,kill_s=.2)
        assert time.monotonic()-begin<.5
        assert result['signals']==['SIGTERM','SIGKILL']
        assert child.returncode==-signal.SIGKILL
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait()


def test_launcher_exit_does_not_hide_a_wedged_native_descendant(tmp_path):
    supervisor=module()
    supervisor.enable_subreaper()
    ready=tmp_path/'native-pid'
    native='import os,signal,time,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path('+repr(str(ready))+').write_text(str(os.getpid())); time.sleep(30)'
    launcher='import subprocess,time,sys; subprocess.Popen([sys.executable,"-c",'+repr(native)+']); time.sleep(30)'
    child=subprocess.Popen([sys.executable,'-c',launcher],start_new_session=True)
    try:
        until=time.monotonic()+2.
        while not ready.exists() and time.monotonic()<until:time.sleep(.005)
        assert ready.exists()
        native_pid=int(ready.read_text())
        result=supervisor.terminate_child(child,grace_s=.05,kill_s=.2)
        assert result['signals']==['SIGTERM','SIGKILL']
        assert not Path('/proc/'+str(native_pid)).exists()
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGKILL);child.wait()
