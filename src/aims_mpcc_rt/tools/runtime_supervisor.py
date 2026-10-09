#!/usr/bin/env python3
"""Bound native process shutdown; observe status only and never enable drive.

The child starts in its own process group. A stalled solve or status stream
causes SIGTERM followed by bounded SIGKILL. Optional restarts launch a fresh
disabled child and require a new explicit enable request from the operator.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


class StallMonitor:
    def __init__(self, *, started, stall_timeout=.5, status_timeout=.5, startup_timeout=15.):
        self.started=started
        self.stall_timeout=stall_timeout
        self.status_timeout=status_timeout
        self.startup_timeout=startup_timeout
        self.last_status=None
        self.reason=None

    def observe(self, values, now):
        if self.reason:return self.reason
        if self.last_status is None and (values.get('status') not in ('WAITING','READY','FAULT')
                or values.get('speed_command',0.)!=0.):
            self.reason='startup_not_disabled'
        elif 'worker_busy' not in values or 'worker_elapsed_s' not in values:
            self.reason='worker_status_missing'
        elif values['worker_busy'] and (not math.isfinite(values['worker_elapsed_s'])
                                      or values['worker_elapsed_s']>self.stall_timeout):
            self.reason='native_worker_stalled'
        self.last_status=now
        return self.reason

    def check(self, now):
        if self.reason:return self.reason
        if self.last_status is None:
            return 'startup_status_timeout' if now-self.started>self.startup_timeout else None
        return 'status_stalled' if now-self.last_status>self.status_timeout else None


def enable_subreaper():
    # ros2 run adds a launcher process. Reparent its native descendants here so
    # a launcher's early exit cannot leave a wedged solve or orphan zombie.
    import ctypes
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(36,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),'cannot enable child subreaper')


def terminate_child(child, *, grace_s=.25, kill_s=.5):
    """Bound shutdown and reap the complete isolated child process group."""
    begin=time.monotonic();signals=[]
    def alive():
        child.poll()
        # Popen owns its direct child's wait; reap only adopted descendants.
        if child.returncode is not None:
            while True:
                try:pid,_=os.waitpid(-child.pid,os.WNOHANG)
                except ChildProcessError:break
                if not pid:break
        try:os.killpg(child.pid,0);return True
        except ProcessLookupError:return False
    def send(number,name):
        try:os.killpg(child.pid,number);signals.append(name)
        except ProcessLookupError:pass
    def wait_group(timeout):
        deadline=time.monotonic()+timeout
        while alive() and time.monotonic()<deadline:time.sleep(.005)
        return not alive()
    if alive():
        send(signal.SIGTERM,'SIGTERM')
        if not wait_group(grace_s):
            send(signal.SIGKILL,'SIGKILL')
            if not wait_group(kill_s):raise subprocess.TimeoutExpired(child.args,grace_s+kill_s)
    child.wait(timeout=kill_s)
    return dict(signals=signals,shutdown_s=time.monotonic()-begin,returncode=child.returncode)


def run(args):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray
    enable_subreaper()
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('mpcc_runtime_supervisor')
    shutdown=False;child=None;monitor=None;events=[];restarts=0;exit_code=0
    def save():
        if args.report:
            path=Path(args.report);path.parent.mkdir(parents=True,exist_ok=True)
            temporary=path.with_name(path.name+'.tmp')
            temporary.write_text(json.dumps(dict(events=events,restarts=restarts,exit_code=exit_code,
                    scope='process lifecycle only; no drive publication or automatic enable'),indent=2)+'\n')
            temporary.replace(path)
    def stop(_signum,_frame):
        nonlocal shutdown
        shutdown=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def observe(message):
        for status in message.status:
            if status.name=='aims_mpcc' and monitor is not None:
                try:values={kv.key:json.loads(kv.value) for kv in status.values}
                except (ValueError,TypeError):monitor.reason='malformed_worker_status';return
                reason=monitor.observe(values,time.monotonic())
                if reason:events.append(dict(event='watchdog',time=time.monotonic(),reason=reason))
    subscription=node.create_subscription(DiagnosticArray,args.status_topic,observe,10)
    try:
        while not shutdown:
            # Drain samples belonging to the previous incarnation before creating
            # a new monitor. No enable service or command publisher exists here.
            monitor=None
            for _ in range(5):rclpy.spin_once(node,timeout_sec=0.)
            child=subprocess.Popen(args.command,start_new_session=True)
            monitor=StallMonitor(started=time.monotonic(),stall_timeout=args.stall_timeout,
                                 status_timeout=args.status_timeout,startup_timeout=args.startup_timeout)
            events.append(dict(event='started',time=monitor.started,pid=child.pid,restart=restarts))
            print(json.dumps(events[-1]),flush=True);save()
            reason=None
            while not shutdown and child.poll() is None:
                rclpy.spin_once(node,timeout_sec=.01)
                reason=monitor.check(time.monotonic())
                if reason:break
            stopped=terminate_child(child,grace_s=args.term_grace,kill_s=args.kill_grace)
            events.append(dict(event='stopped',time=time.monotonic(),reason=reason or 'shutdown' if shutdown else reason or 'child_exit',**stopped))
            print(json.dumps(events[-1]),flush=True);save()
            if shutdown:exit_code=0;break
            exit_code=70 if reason else child.returncode
            if reason and reason!='startup_not_disabled' and restarts<args.restart_limit:
                restarts+=1;continue
            break
    finally:
        if child is not None and child.poll() is None:terminate_child(child,grace_s=args.term_grace,kill_s=args.kill_grace)
        save()
        node.destroy_node();rclpy.shutdown()
    return exit_code


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--status-topic',default='/mpcc_rt_shadow/mpcc/status')
    parser.add_argument('--stall-timeout',type=float,default=.5)
    parser.add_argument('--status-timeout',type=float,default=.5)
    parser.add_argument('--startup-timeout',type=float,default=15.)
    parser.add_argument('--term-grace',type=float,default=.25)
    parser.add_argument('--kill-grace',type=float,default=.5)
    parser.add_argument('--restart-limit',type=int,default=0)
    parser.add_argument('--report')
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if args.command and args.command[0]=='--':args.command.pop(0)
    if not args.command:parser.error('a native controller command must follow --')
    if any(not math.isfinite(v) or v<=0 for v in (args.stall_timeout,args.status_timeout,args.startup_timeout,args.term_grace,args.kill_grace)):
        parser.error('watchdog and shutdown bounds must be positive finite seconds')
    if args.restart_limit<0:parser.error('restart-limit must be nonnegative')
    raise SystemExit(run(args))
