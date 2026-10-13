#!/usr/bin/env python3
"""Profile a controller beside raw FAST-LIO/EKF/NDT replay, never a real driver.

The selector receives synthetic RC permission; recorded motor telemetry is replayed
with its original source stamps. These are scheduling/data-flow measurements, NOT
causal calibration of the generated requests against the recorded car response.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time


def quantiles(values):
    import numpy as np
    values=[float(value) for value in values if value is not None]
    if not values:return dict(count=0)
    return dict(count=len(values),p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)),max=max(values),
                over_20_ms=sum(value>20. for value in values))


def command_metrics(rows,mode):
    active=[r for r in rows if (r.get('phase')=='RUNNING' and r.get('event')=='command'
                               if mode=='pp' else r.get('state') not in ('IDLE','STOPPING','FAULT','COMPLETE'))]
    if mode=='pp':
        callbacks=[float(r['callback_s'])*1000. for r in active if r.get('callback_s')]
        gaps=[float(r['publish_gap_s'])*1000. for r in active if r.get('publish_gap_s')]
    else:
        callbacks=[float(r['callback_ms']) for r in active if r.get('callback_ms')]
        gaps=[float(r['timer_interval_ms']) for r in active if r.get('timer_interval_ms')]
    return len(active),quantiles(callbacks),quantiles(gaps)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('bag','map','seed','bundle','output'):p.add_argument('--'+name,required=True,type=Path)
    p.add_argument('--mode',choices=('pp','calibration'),default='pp')
    p.add_argument('--seconds',type=float,default=60.)
    p.add_argument('--domain',type=int,default=223)
    p.add_argument('--timing-rearms',type=int,default=0,
                   help='Explicit virtual service re-arms after faults, for aggregate timing only; never field operation')
    args=p.parse_args()
    if not 1<=args.domain<=230 or args.seconds<=15:p.error('Use isolated domain 1..230 and more than 15 seconds')
    if not 0<=args.timing_rearms<=100:p.error('timing-rearms must be between zero and 100')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    import rclpy
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticArray
    from crsf_receiver_msg.msg import CRSFChannels16
    from std_srvs.srv import SetBool
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    root=Path(__file__).resolve().parents[2]
    logs=[];processes=[]
    def spawn(command,name):
        log=(args.output/(name+'.log')).open('w');logs.append(log)
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc);return proc
    def binary(package,name):return str(Path(get_package_prefix(package))/'lib'/package/name)

    reference=json.loads((args.bundle/'reference.json').read_text())
    ref=args.output/'profiling-reference';ref.mkdir()
    data=(args.bundle/'reference.json').read_bytes();(ref/'reference.json').write_bytes(data)
    # A synthetic all-lap interval exercises association; it is deliberately NOT
    # an operator-approved straight interval and must never become a field input.
    (ref/'calibration_regions.json').write_text(json.dumps(dict(schema_version=1,
        reference_sha256=hashlib.sha256(data).hexdigest(),regions=[dict(
            id='PROFILING_ONLY_NOT_A_FIELD_STRAIGHT',kind='straight',start_s=0.,
            end_s=reference['length']-1e-6)])))

    db=sqlite3.connect('file:'+str(next(args.bag.glob('*.db3')))+'?mode=ro',uri=True)
    row=db.execute("select id,type from topics where name='/sensors/core'").fetchone()
    if row is None:raise RuntimeError('Recorded /sensors/core is required')
    core_type=get_message(row[1])
    records=iter(db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(row[0],)))
    pending=next(records,None)
    rclpy.init();node=rclpy.create_node('controller_replay_profile',parameter_overrides=[Parameter('use_sim_time',value=True)])
    rc=node.create_publisher(CRSFChannels16,'/rc/channels',1)
    telemetry=node.create_publisher(core_type,'/sensors/core',1)
    channels=CRSFChannels16()
    for i in range(1,17):setattr(channels,'ch'+str(i),992)
    channels.ch5=1810;channels.ch6=172;channels.ch7=1810;channels.ch10=1810
    channels.ch8=1810 if args.mode=='calibration' else 172
    statuses=[];localization={};health_received=status_received=0.
    def status_update(msg):
        nonlocal status_received
        statuses.extend({kv.key:kv.value for kv in item.values} for item in msg.status)
        status_received=time.monotonic()
    def health_update(msg):
        nonlocal health_received
        for item in msg.status:
            if item.name=='aims_racer_system/localization':
                localization.clear();localization.update({kv.key:kv.value for kv in item.values})
                health_received=time.monotonic()
    node.create_subscription(DiagnosticArray,'/localization/status',health_update,10)
    node.create_subscription(DiagnosticArray,'/'+('pp' if args.mode=='pp' else 'calib')+'/status',
        status_update,10)
    service=node.create_client(SetBool,'/'+('pp' if args.mode=='pp' else 'calib')+'/enable')
    until=time.monotonic()+2.
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    if node.count_publishers('/vesc/command'):
        raise RuntimeError('An existing graph owns commands in this supposedly isolated domain')
    replay=controller=selector=None;enabled=False;future=None;attempts=0;telemetry_count=0;enable_responses=[]
    rearms=0;last_successful_enable=0.
    try:
        selector=spawn([binary('ackermann_mux','joystick_control_v2'),'--ros-args','-p',
            'channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10','-p','use_sim_time:=true'],'selector')
        cfg=str(args.bundle/'input_config.yaml')
        controller_cmd=[binary('aims_pp','pp_node') if args.mode=='pp' else binary('aims_calibration','longitudinal_calibration'),
            '--ros-args','-p','use_sim_time:=true','-p','vehicle_config:='+cfg,'-p','reference_directory:='+str(ref),
            '-p','log_directory:='+str(args.output/'controller')]
        if args.mode=='pp':controller_cmd+=['-p','repeat_laps:=true','-p','speed_limit:=1.0']
        else:controller_cmd+=['-p','steering_source:=pp','-p','stage:=speed_hold','-p',
            'pp_config:='+str(Path(get_package_share_directory('aims_pp'))/'config/pp.yaml')]
        controller=spawn(controller_cmd,'controller')
        replay=spawn([sys.executable,str(root/'verification/localization/fastlio_ndt_replay.py'),
            str(args.bag),str(args.output/'localization'),'--map',str(args.map),'--seed',str(args.seed),
            '--max-seconds',str(args.seconds),'--audit-executable',
            str(root/'build/aims_racer_system/tf_authority_audit')],'replay')
        deadline=time.monotonic()+args.seconds+35.;last_attempt=last_rc=0.
        while time.monotonic()<deadline and replay.poll() is None:
            if controller.poll() is not None or selector.poll() is not None:raise RuntimeError('Controller/selector exited; inspect logs')
            now=time.monotonic();ros=node.get_clock().now().nanoseconds
            if now-last_rc>=.01:rc.publish(channels);last_rc=now
            # Only the newest recorded sample at this replay instant is sent.
            latest=None
            while pending is not None and pending[0]<=ros:
                latest=pending;pending=next(records,None)
            if latest is not None:
                telemetry.publish(deserialize_message(latest[1],core_type));telemetry_count+=1
            if future is not None and future.done():
                response=future.result()
                enabled=bool(response and response.success)
                if enabled:last_successful_enable=now
                enable_responses.append(dict(ros_ns=ros,success=enabled,message=response.message if response else 'No response'))
                future=None
            ready=(localization.get('ready')=='true' and localization.get('map_valid')=='true'
                   and now-health_received<.3 and telemetry_count>0)
            if enabled and statuses and rearms<args.timing_rearms and status_received>last_successful_enable:
                phase=statuses[-1].get('phase',statuses[-1].get('state'))
                if phase=='FAULT':enabled=False;rearms+=1
            if not enabled and future is None and now-last_attempt>=1. and service.service_is_ready() and statuses and ready:
                req=SetBool.Request();req.data=True;future=service.call_async(req);last_attempt=now;attempts+=1
            rclpy.spin_once(node,timeout_sec=.002)
        if replay.poll() is None:raise RuntimeError('Replay exceeded its deadline')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in reversed(processes):
            try:proc.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2.)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown();db.close()
    runtime=list((args.output/'controller').rglob('runtime.csv'))
    if len(runtime)!=1:raise RuntimeError('Expected exactly one runtime log')
    rows=list(csv.DictReader(runtime[0].open()))
    active_count,callbacks,gaps=command_metrics(rows,args.mode)
    summary=dict(mode=args.mode,domain=args.domain,scope='Recorded feedback, virtual authority; timing only, no causal response acceptance',
        real_driver_started=False,timing_rearm_limit=args.timing_rearms,timing_rearms=rearms,
        enable_requests=attempts,enable_accepted=any(r['success'] for r in enable_responses),
        last_enable_success=enabled,enable_responses=enable_responses,telemetry_count=telemetry_count,
        active_rows=active_count,callback_ms=callbacks,command_gap_ms=gaps,
        last_status=statuses[-1] if statuses else {},runtime_csv=str(runtime[0]),
        localization_summary=str(args.output/'localization/summary.json'))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    if not active_count:raise RuntimeError('No active controller rows; timing measurement is not qualified')


if __name__=='__main__':main()
