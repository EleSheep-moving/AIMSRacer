#!/usr/bin/env python3
"""Observe a TEST_ACCESS blocked-worker fixture and bound its process shutdown.

The hook blocks immediately before Core.solve; it does not create or diagnose
an internal acados deadlock. It exists only in test_runtime_lifecycle.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time


def assess(commands,events):
    blocked=[event for event in events if event['status'].get('test_worker_blocked')
             and event['status'].get('worker_busy')]
    begin=blocked[0]['time'] if blocked else float('inf')
    positive=[row for row in commands if row['time']<begin and row['speed']>0.]
    output=[row for row in commands if row['time']>=begin]
    gaps=[after['time']-before['time'] for before,after in zip(output,output[1:])]
    status_gaps=[after['time']-before['time'] for before,after in zip(blocked,blocked[1:])]
    recovery=[e for e in blocked if e['status'].get('status')=='RECOVERING']
    ready=[e for e in blocked if e['status'].get('status')=='READY' and e['status'].get('enabled')==0]
    zeros=[row for row in output if row['speed']==0.]
    checks=dict(
        positive_before_worker_block=bool(positive and begin-positive[-1]['time']<=.15),
        actual_worker_block_observed=bool(blocked),
        output_remains_responsive_while_blocked=bool(len(output)>=20 and gaps and max(gaps)<=.1
            and blocked[-1]['time']-output[-1]['time']<=.1),
        status_remains_responsive_while_blocked=bool(len(blocked)>=5 and status_gaps and max(status_gaps)<=.25),
        input_feedback_stays_fresh=bool(blocked and all(0.<=e['status'].get(key,float('inf'))<=.1
            for e in blocked for key in ('source_age_s','receipt_age_s','authority_age_s','applied_age_s'))),
        ttl_recovery_disabled_ready=bool(recovery and ready and recovery[0]['time']<ready[0]['time']),
        original_source_ttl_exhausted=bool(recovery and all(
            'expired' in e['status'].get('reason','').lower()
            and isinstance(e['status'].get('plan_source_age_s'),(float,int))
            and e['status']['plan_source_age_s']>=e['status'].get('plan_ttl',float('inf'))
            for e in recovery)),
        sustained_zero_without_reenable=bool(ready and zeros and commands[-1]['time']-zeros[0]['time']>=.2
            and all(row['speed']==0. for row in commands if row['time']>=ready[0]['time'])
            and all(e['status'].get('enabled')==0 for e in events if e['time']>=ready[0]['time'])),
        no_other_fault=all(e['status'].get('status')!='FAULT' for e in events))
    return dict(checks=checks,blocked_command_samples=len(output),blocked_status_samples=len(blocked),
        first_blocked_s=begin if blocked else None,first_recovering_s=recovery[0]['time'] if recovery else None,
        first_disabled_ready_s=ready[0]['time'] if ready else None,
        first_recovery_source_age_s=recovery[0]['status'].get('plan_source_age_s') if recovery else None,
        plan_ttl_s=recovery[0]['status'].get('plan_ttl') if recovery else None,
        maximum_blocked_publish_gap_s=max(gaps) if gaps else None,
        maximum_blocked_status_gap_s=max(status_gaps) if status_gaps else None)


def run(args):
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not 200<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=232:
        raise SystemExit('requires private ROS_DOMAIN_ID in [200,232], ROS_LOCALHOST_ONLY=1')
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticArray
    root=Path(__file__).resolve().parents[3]
    source=root/'src/aims_mpcc_rt/tools/runtime_supervisor.py'
    spec=importlib.util.spec_from_file_location('stall_supervisor',source)
    supervisor=importlib.util.module_from_spec(spec);spec.loader.exec_module(supervisor)
    supervisor.enable_subreaper()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[]);node=rclpy.create_node('worker_stall_external_observer')
    commands=[];events=[]
    def drive(message):commands.append(dict(time=time.monotonic(),speed=message.drive.speed,
        acceleration=message.drive.acceleration,steering=message.drive.steering_angle))
    def status(message):
        for item in message.status:
            if item.name=='aims_mpcc':events.append(dict(time=time.monotonic(),status={kv.key:json.loads(kv.value) for kv in item.values}))
    node.create_subscription(AckermannDriveStamped,'/mpcc_rt_test/drive',drive,10)
    node.create_subscription(DiagnosticArray,'/mpcc_rt_test/mpcc/status',status,10)
    command=[str(args.executable.resolve()),str(args.bundle.resolve()),'stall',str(out/'controller')]
    report=dict(scope=__doc__,command=command,domain=os.environ['ROS_DOMAIN_ID'],
        binary_sha256=hashlib.sha256(args.executable.read_bytes()).hexdigest(),
        termination_policy='wait for TTL recovery/disabled output, then SIGTERM grace 0.5 s and bounded SIGKILL; production default watchdog unchanged')
    child=None
    try:
        with (out/'fixture.log').open('w') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+args.timeout
            ready_since=None
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.005)
                if child.poll() is not None:raise RuntimeError('stall fixture exited before external lifecycle proof')
                latest=events[-1]['status'] if events else {}
                if latest.get('status')=='READY' and latest.get('enabled')==0 and latest.get('test_worker_blocked'):
                    ready_since=ready_since or time.monotonic()
                    if time.monotonic()-ready_since>=.3:break
            else:raise RuntimeError('blocked worker did not recover to disabled READY within bound')
            report.update(assess(commands,events))
            original={topic:node.count_publishers(topic) for topic in (
                '/drive','/ackermann_cmd','/commands/motor/speed','/commands/servo/position','/commands/motor/current','/commands/motor/duty_cycle')}
            report['original_driving_publishers']=original
            report['checks']['original_driving_publishers_zero']=not any(original.values())
            report['shutdown']=supervisor.terminate_child(child,grace_s=.5,kill_s=.5)
            report['checks']['blocked_join_requires_bounded_sigkill']=report['shutdown']['signals']==['SIGTERM','SIGKILL'] and report['shutdown']['shutdown_s']<=1.1
            report['overall_pass']=all(report['checks'].values())
    except Exception as exc:report.update(overall_pass=False,error=str(exc))
    finally:
        if child is not None and child.poll() is None:report['cleanup']=supervisor.terminate_child(child,grace_s=.5,kill_s=.5)
        report['returncode']=child.returncode if child else None
        (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        (out/'status.json').write_text(json.dumps(events,indent=2)+'\n')
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2))
    return 0 if report['overall_pass'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=6.)
    raise SystemExit(run(parser.parse_args()))
