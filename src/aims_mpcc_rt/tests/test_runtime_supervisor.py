"""Process lifecycle checks use disposable children and never publish drive."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest


def module():
    path=Path(__file__).resolve().parents[1]/'tools/runtime_supervisor.py'
    spec=importlib.util.spec_from_file_location('runtime_supervisor',path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


def status(**changes):
    values=dict(status='READY',enabled=0,speed_command=0.,worker_busy=False,worker_elapsed_s=0.,
                startup_instance='child-one',startup_disabled=True,explicit_enable_count=0)
    values.update(changes)
    return values


def test_late_first_running_status_proves_disabled_birth_and_explicit_enable():
    monitor=module().StallMonitor(started=10.)
    assert monitor.observe(status(status='RUNNING',enabled=1,speed_command=.2,explicit_enable_count=1),10.2) is None
    assert monitor.check(10.3) is None


@pytest.mark.parametrize('changes',[dict(startup_disabled=False,explicit_enable_count=1),
    dict(explicit_enable_count=0),dict(explicit_enable_count=True),dict(explicit_enable_count=-1),
    dict(explicit_enable_count=1.5),dict(startup_disabled='true',explicit_enable_count=1)])
def test_running_without_valid_disabled_start_and_explicit_enable_is_rejected(changes):
    monitor=module().StallMonitor(started=10.)
    values=status(status='RUNNING',enabled=1,speed_command=.2);values.update(changes)
    assert monitor.observe(values,10.2)=='startup_not_disabled'


def test_restart_ignores_previous_process_witness_without_refreshing_status():
    monitor=module().StallMonitor(started=11.,startup_timeout=1.,expected_instance='child-two')
    assert monitor.observe(status(status='RUNNING',enabled=1,explicit_enable_count=1),11.2) is None
    assert monitor.last_status is None
    assert monitor.check(12.01)=='startup_status_timeout'
    assert monitor.observe(status(startup_instance='child-two'),12.1) is None
    assert monitor.last_status==12.1


def test_autoenable_after_first_disabled_status_is_also_rejected():
    monitor=module().StallMonitor(started=10.)
    assert monitor.observe(status(),10.1) is None
    assert monitor.observe(status(status='RUNNING',enabled=1,speed_command=.2),10.2)=='startup_not_disabled'


def test_current_disabled_state_does_not_replace_a_disabled_birth_witness():
    monitor=module().StallMonitor(started=10.)
    assert monitor.observe(status(startup_disabled=False),10.1)=='startup_not_disabled'


def test_previous_process_status_does_not_keep_current_process_alive():
    monitor=module().StallMonitor(started=10.,expected_instance='child-two',status_timeout=.5)
    assert monitor.observe(status(startup_instance='child-two'),10.1) is None
    assert monitor.observe(status(),10.55) is None
    assert monitor.last_status==10.1
    assert monitor.check(10.61)=='status_stalled'


def test_worker_deadline_detects_native_stall_even_with_fresh_status():
    monitor=module().StallMonitor(started=10.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert monitor.observe(status(),10.1) is None
    assert monitor.observe(status(status='RUNNING',enabled=1,explicit_enable_count=1,worker_busy=True,worker_elapsed_s=.51),10.2)=='native_worker_stalled'


def test_each_started_child_must_report_disabled_and_missing_status_is_bounded():
    cls=module().StallMonitor
    monitor=cls(started=10.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert monitor.observe(dict(status='RUNNING',worker_busy=False,worker_elapsed_s=0.),10.1)=='startup_not_disabled'
    restarted=cls(started=11.,stall_timeout=.5,status_timeout=.4,startup_timeout=2.)
    assert restarted.observe(status(),11.1) is None
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
