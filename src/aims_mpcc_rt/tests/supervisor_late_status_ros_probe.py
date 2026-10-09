#!/usr/bin/env python3
"""Observe a real native child's first supervisor sample only after explicit enable.

The relay models late DDS discovery deterministically. The native controller,
enable service, drive/status timers and supervisor are real; plant inputs and
the relay are synthetic and private. No vehicle driving topics are published.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

TOOLS=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
from protocol_probe import Probe,PREFIX,topic_remap_arguments
from ament_index_python.packages import get_package_prefix
import rclpy
from diagnostic_msgs.msg import DiagnosticArray


def run(args):
    assert 200<=int(os.environ.get('ROS_DOMAIN_ID','0'))<=232
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    bundle=args.bundle.resolve();rclpy.init(args=[]);probe=Probe(bundle)
    relay_topic=PREFIX+'/late_supervisor_status'
    relay=probe.create_publisher(DiagnosticArray,relay_topic,10)
    state=dict(open=False,first=None,held_samples=0)
    def forward(message):
        if not state['open']:
            state['held_samples']+=1
            return
        if state['first'] is None:
            state['first']=dict(time=time.monotonic(),status=probe.status.copy())
        relay.publish(message)
    subscription=probe.create_subscription(DiagnosticArray,PREFIX+'/mpcc/status',forward,10)
    native=['ros2','run','aims_mpcc_rt','mpcc_rt_node','--ros-args',
        '-p','artifact_directory:='+str(bundle),'-p','simulation:=true','-p','shadow:=false',
        '-p','repeat_laps:=true','-p','log_directory:='+str(out/'controller'),
        *topic_remap_arguments(PREFIX)]
    command=[sys.executable,str(args.supervisor),'--status-topic',relay_topic,
        '--report',str(out/'supervisor.json'),'--',*native]
    binary=Path(get_package_prefix('aims_mpcc_rt'))/'lib/aims_mpcc_rt/mpcc_rt_node'
    report=dict(scope=__doc__,command=command,domain=int(os.environ['ROS_DOMAIN_ID']),checks=[],
        controller_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        supervisor_sha256=hashlib.sha256(args.supervisor.read_bytes()).hexdigest())
    def check(name,condition,**values):
        report['checks'].append(dict(name=name,passed=bool(condition),**values))
        assert condition,name+': '+json.dumps(values)
    log=(out/'controller.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        check('native_ready_disabled',probe.spin_until(lambda:probe.status.get('status')=='READY'
            and probe.status.get('enabled')==0 and probe.status.get('worker_ready') is True,15.),status=probe.status)
        probe.settle(.15)
        reply=probe.enable()
        check('operator_explicit_enable_accepted',reply['success'],reply=reply)
        check('real_positive_running_before_supervisor_discovery',probe.spin_until(lambda:probe.status.get('status')=='RUNNING'
            and len(probe.commands)>=2 and all(row[1]>0 for row in probe.commands[-2:]),6.),status=probe.status)
        state['open']=True;begin=time.monotonic();before=len(probe.commands)
        probe.settle(1.)
        check('first_forwarded_status_is_running',state['first'] is not None
            and state['first']['status']['status']=='RUNNING',first=state['first'],held_samples=state['held_samples'])
        check('late_discovery_keeps_native_running',process.poll() is None
            and probe.status.get('status')=='RUNNING' and len(probe.commands[before:])>=40
            and all(row[1]>0 for row in probe.commands[before:]),status=probe.status,
            received_command_samples=len(probe.commands[before:]),duration_s=time.monotonic()-begin)
        supervisor=json.loads((out/'supervisor.json').read_text())
        verified=[event for event in supervisor['events'] if event['event']=='startup_verified']
        check('supervisor_persisted_startup_proof',len(verified)==1 and verified[0]['first_status']=='RUNNING'
            and verified[0]['startup_disabled']==1 and verified[0]['explicit_enable_count']==1,
            supervisor=supervisor)
        report['original_driving_publishers']={topic:probe.count_publishers(topic) for topic in
            ('/drive','/ackermann_cmd','/commands/motor/speed','/commands/servo/position','/commands/motor/current','/commands/motor/duty_cycle')}
        check('original_driving_topics_unused',not any(report['original_driving_publishers'].values()))
        report['overall_pass']=True
    except Exception as exc:
        report.update(overall_pass=False,error=str(exc),final_status=probe.status)
    finally:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=2.)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1.)
        report['supervisor_exit_code']=process.returncode
        log.close();probe.destroy_node();rclpy.shutdown()
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(overall_pass=report['overall_pass'],
        failed_checks=[item['name'] for item in report['checks'] if not item['passed']],output=str(out))))
    return 0 if report['overall_pass'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--supervisor',type=Path,default=TOOLS/'runtime_supervisor.py')
    raise SystemExit(run(parser.parse_args()))
