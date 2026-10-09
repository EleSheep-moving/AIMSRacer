#!/usr/bin/env python3
"""Isolated real selector/converter route with an independent lagged plant.

The controller's odometry comes from the synthetic plant; an estimator replay
may run alongside it for CPU load, but is not its feedback or ground truth.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import sys
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import SetBool
from aims_mpcc.integration import Plant,child_commands,topic_remap_arguments,DRIVING_TOPICS
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath


def run(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    bundle=Path(args.bundle).resolve();config=load_config(bundle/'input_config.yaml')
    artifact=json.loads((bundle/'manifest.json').read_text())
    if args.implementation=='legacy' and artifact['dt']!=.1:raise ValueError('legacy runtime uses 0.1 s stages')
    if args.implementation=='legacy' and args.backend=='acados' and not args.legacy_artifact:
        raise ValueError('legacy acados requires --legacy-artifact prepared for that backend')
    path=ReferencePath.load(bundle/'input_reference')
    remaps=topic_remap_arguments(args.prefix)
    init_args=['--ros-args',*remaps]
    if args.implementation=='legacy':
        init_args+=['-p',f'path_directory:={bundle}/input_reference','-p',f'vehicle_config:={bundle}/input_config.yaml',
          '-p','simulation:=true','-p',f'backend:={args.backend}','-p',f'solve_frequency:={args.frequency}',
          '-p',f'horizon:={artifact["horizon"]}',
          '-p',f'solver_timeout:={args.budget}','-p','handover_delay:=0.02','-p','plan_ttl:=0.8',
          '-p',f'log_directory:={out}/controller']
        if args.legacy_artifact:init_args+=['-p',f'artifact_directory:={args.legacy_artifact}']
    rclpy.init(args=init_args)
    plant=Plant(path,config,speed_tau=args.speed_tau,steer_tau=args.steer_tau,odom_delay=args.odom_delay)
    executor=SingleThreadedExecutor();executor.add_node(plant)
    status={'phase':'WAITING','reason':''}
    def observe(message):
        if message.status:
            s=message.status[0];phase,_,reason=s.message.partition(': ')
            values={k.key:k.value for k in s.values}
            status.update(phase=json.loads(values['status']) if 'status' in values else phase,
                          reason=json.loads(values['reason']) if 'reason' in values else reason,values=values)
    subscription=plant.create_subscription(DiagnosticArray,args.prefix+'/mpcc/status',observe,10)
    client=plant.create_client(SetBool,args.prefix+'/mpcc/enable')
    commands=child_commands(args.vesc,remaps)
    command=['ros2','run','aims_mpcc_rt','mpcc_rt_node','--ros-args',
      '-p',f'artifact_directory:={bundle}','-p',f'vehicle_config:={bundle}/input_config.yaml',
      '-p',f'path_directory:={bundle}/input_reference','-p','simulation:=true',
      '-p','shadow:=false','-p',f'repeat_laps:={str(args.repeat_laps).lower()}',
      '-p',f'solve_frequency:={args.frequency}',
      '-p',f'solver_timeout:={args.budget}','-p',f'log_directory:={out}/controller',*remaps]
    legacy=None;callbacks=[];publications=[];activations={};replies={};attempts={}
    if args.implementation=='acados_cpp':commands.append(('controller',command))
    processes=[];handles=[];samples=[];started=None;injected=None;measurement_end=None
    last_progress=None;unwrapped_progress=0.
    result={'scope':'independent synthetic feedback through real RC selector and VESC converter; isolated topics',
      'arguments':vars(args),'commands':commands,'bundle_manifest':json.loads((bundle/'native_manifest.json').read_text())}
    try:
        for name,command in commands:
            handle=(out/(name+'.log')).open('w');handles.append(handle)
            processes.append(subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True))
        if args.implementation=='legacy':
            from aims_mpcc.integration import ShadowMPCCNode
            from rclpy.publisher import Publisher
            original_publish=Publisher.publish
            def publish(self,message):
                begin=time.monotonic();answer=original_publish(self,message)
                if self.topic_name==args.prefix+'/drive':publications.append([begin,time.monotonic()])
                return answer
            Publisher.publish=publish
            class RepeatedLegacy(ShadowMPCCNode):
                def tick(self):
                    if args.repeat_laps:self.supervisor.lap_goal=self.supervisor.progress+self.path.length
                    begin=time.monotonic()
                    try:return super().tick()
                    finally:
                        callbacks.append([begin,time.monotonic()])
                        a=getattr(self,'last_activation_timing',None)
                        if a and a.get('first_proposal_published_at') is not None:activations[a['submitted_at']]=dict(a)
                        r=self.request_timing
                        if r and r.get('submitted_at') is not None:replies[r['submitted_at']]=dict(r)
            legacy=RepeatedLegacy();executor.add_node(legacy)
            original_submit=legacy.worker.submit;original_poll=legacy.worker.poll
            def submit(request):
                answer=original_submit(request)
                if answer:attempts[request['submitted_at']]=dict(submitted=request['submitted_at'],events=[],failed=False)
                return answer
            def poll(now):
                pending=legacy.worker.pending;answer=original_poll(now)
                if answer:
                    key=answer.get('submitted_at',pending)
                    if key in attempts:
                        row=attempts[key];row['events'].append(dict(kind=answer['kind'],received=now))
                        if answer['kind'] in ('skipped','error','restarting'):
                            row['failed']=True;row['terminal']=now
                        elif answer['kind']=='result':
                            row['terminal']=now;row['failed']|=not answer.get('success',False) or answer.get('discarded',False)
                return answer
            legacy.worker.submit=submit;legacy.worker.poll=poll
            original_accept=legacy.supervisor.accept;original_activate=legacy.supervisor.activate
            def accept(candidate,now):
                answer=original_accept(candidate,now);key=candidate['submitted_at']
                if key in attempts:
                    attempts[key]['events'].append(dict(kind='candidate_accept',accepted=answer,received=now))
                    attempts[key]['failed']|=not answer
                return answer
            def activate(now,*positional,**keywords):
                candidate=legacy.supervisor.pending_plan
                key=candidate['submitted_at'] if candidate else None
                answer=original_activate(now,*positional,**keywords)
                if key in attempts:
                    attempts[key]['events'].append(dict(kind='activation',accepted=answer,received=now,
                        reason=legacy.supervisor.reason))
                    attempts[key]['failed']|=not answer
                return answer
            legacy.supervisor.accept=accept;legacy.supervisor.activate=activate
        deadline=time.monotonic()+20.;future=None
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.005)
            if any(p.poll() is not None for p in processes):raise RuntimeError('route process exited during startup')
            if not future and client.service_is_ready() and plant.count_publishers(args.prefix+'/ackermann_cmd')==1:
                request=SetBool.Request();request.data=True;future=client.call_async(request)
            if future and future.done():
                response=future.result()
                if response.success:break
                future=None
        else:raise RuntimeError('enable did not become ready: '+status['reason'])
        original={t:plant.count_publishers(t) for t in DRIVING_TOPICS}
        if any(original.values()):raise RuntimeError('unexpected unprefixed driving publisher in isolated domain')
        result['original_driving_publishers']=original
        started=time.monotonic();last_sample=started;end=started+args.seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.003);now=time.monotonic()
            if any(p.poll() is not None for p in processes):raise RuntimeError('route process exited during execution')
            if now-last_sample>=.02:
                last_sample=now;theta,error=path.project([plant.x,plant.y]);ref=path.at(theta)
                if last_progress is not None:unwrapped_progress+=(theta-last_progress+path.length/2)%path.length-path.length/2
                last_progress=theta
                heading=math.atan2(math.sin(plant.yaw-ref['yaw']),math.cos(plant.yaw-ref['yaw']))
                samples.append([now-started,plant.x,plant.y,plant.yaw,plant.speed,error,heading,plant.speed_target,plant.steer_target,unwrapped_progress])
            if args.scenario!='nominal' and injected is None and now-started>=args.inject_after:
                injected=now
                if args.scenario=='authority':plant.autonomy=False
                elif args.scenario=='odometry':plant.publish_odom=False
                elif args.scenario=='clock':plant.clock_offset=-.2
                elif args.scenario=='disable':
                    request=SetBool.Request();request.data=False;client.call_async(request)
            if status['phase']=='FAULT':
                if args.scenario=='nominal':raise RuntimeError('nominal controller fault: '+status['reason'])
                if injected and now-injected>.5:
                    if abs(plant.speed_target)>1e-9:raise RuntimeError('fault did not stop converter speed command')
                    result['fault_detection_s']=now-injected-.5
                    break
            if args.scenario=='nominal' and args.repeat_laps and now-started>1. and status['phase']!='RUNNING':
                raise RuntimeError('nominal controller left RUNNING: '+status['phase']+' '+status['reason'])
            if args.scenario=='disable' and injected and now-injected>1. and plant.speed<.05 and abs(plant.speed_target)<1e-9:break
            if not args.repeat_laps and status['phase']=='COMPLETE':break
        measurement_end=time.monotonic();result['status']=status;result['elapsed_s']=measurement_end-started
        result['measurement_start']=started;result['measurement_end']=measurement_end
        data=np.asarray(samples)
        if not len(data):raise RuntimeError('no plant samples')
        result.update(contour_p95_m=float(np.percentile(np.abs(data[:,5]),95)),
          contour_rms_m=float(np.sqrt(np.mean(data[:,5]**2))),contour_max_m=float(np.max(np.abs(data[:,5]))),
          heading_p95_rad=float(np.percentile(np.abs(data[:,6]),95)),
          speed_p50_mps=float(np.median(data[data[:,0]>=min(3.,args.seconds/2),4])),
          final_speed_mps=float(data[-1,4]),laps=abs(float(unwrapped_progress))/path.length,
          maximum_command_speed_mps=float(np.max(data[:,7])),maximum_command_steering_rad=float(np.max(np.abs(data[:,8]))))
        result['tracking_pass']=bool(result['contour_max_m']<=.25 and result['speed_p50_mps']>=.8*config.cruise_speed and
          result['maximum_command_speed_mps']<=config.max_speed+1e-6 and result['maximum_command_steering_rad']<=config.steer_limit+1e-6)
        if args.repeat_laps and args.scenario=='nominal':
            tail=data[data[:,0]>=max(3.,result['elapsed_s']-3.),4]
            result['sustained_tracking_pass']=bool(result['elapsed_s']>=args.seconds-.05 and len(tail) and np.median(tail)>=.8*config.cruise_speed)
            result['tracking_pass']=result['tracking_pass'] and result['sustained_tracking_pass']
        if args.scenario!='nominal':result['fault_pass']=bool(injected and abs(plant.speed_target)<1e-9)
        result['finish_pass']=bool(args.repeat_laps or (status['phase']=='COMPLETE' and plant.speed<.05 and abs(plant.speed_target)<1e-9))
        result['overall_pass']=(result['tracking_pass'] and result['finish_pass']) if args.scenario=='nominal' else result['fault_pass']
    except Exception as exc:
        measurement_end=time.monotonic();result['overall_pass']=False;result['error']=str(exc)
        result['measurement_start']=started;result['measurement_end']=measurement_end
    finally:
        if legacy is not None:
            # Stop submitting and receive the last in-flight terminal outcome.
            # Publication during this drain is outside the measurement window.
            legacy.worker.submit=lambda request:False
            deadline=time.monotonic()+max(1.,4*args.budget)
            while any('terminal' not in row for row in attempts.values()) and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.005)
        # Only stop the process groups created by this harness.
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for h in handles:h.close()
        if legacy is not None:
            executor.remove_node(legacy);legacy.destroy_node();Publisher.publish=original_publish
        executor.remove_node(plant);plant.destroy_node();rclpy.shutdown()
        with (out/'trajectory.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['time','x','y','yaw','speed','contour','heading','speed_command','steering_command','progress']);writer.writerows(samples)
        timing=out/'controller/runtime.csv'
        if timing.exists():result['timing']=timing_report(timing,args.budget,started,measurement_end)
        elif args.implementation=='legacy':
            solved=[]
            for file in (out/'controller').glob('solver-*.jsonl'):
                solved += [r for line in file.read_text().splitlines() if (r:=json.loads(line)).get('kind')=='solve']
            def statistics(values):return {k:float(np.percentile(values,q)) if values else None for k,q in [('p50',50),('p95',95),('p99',99),('max',100)]}
            measured=[r for r in attempts.values() if started is not None and started<=r['submitted']<=measurement_end]
            for row in measured:
                row['unfinished']='terminal' not in row
                row['failed']|=row['unfinished']
            consecutive=maximum=0
            for row in measured:
                consecutive=consecutive+1 if row['failed'] else 0;maximum=max(maximum,consecutive)
            measured_pub=[r for r in publications if started is not None and started<=r[0]<=measurement_end]
            measured_callbacks=[r for r in callbacks if started is not None and started<=r[0]<=measurement_end]
            (out/'request-outcomes.json').write_text(json.dumps(measured,indent=2)+'\n')
            result['timing']=dict(requests=len(measured),failures=sum(r['failed'] for r in measured),
              max_consecutive_failures=maximum,unfinished=sum(r['unfinished'] for r in measured),
              log_integrity_pass=not any(r['unfinished'] for r in measured),submission_records=len(attempts),
              measurement_s=measurement_end-started if started is not None else 0,publications=len(measured_pub),
              complete=statistics([r['terminal']-r['submitted'] for r in measured if 'terminal' in r]),
              core=statistics([r['solve_time_s'] for r in solved]),
              publish_callback=statistics([b-a for a,b in measured_callbacks]),
              publish_gap=statistics([b[0]-a[0] for a,b in zip(measured_pub,measured_pub[1:])]),
              request_to_first_publish=statistics([r['request_to_first_proposal_s'] for r in activations.values()]),
              accepted_activations=len(activations),received_replies=len(replies),
              late=sum(r.get('terminal',measurement_end)-r['submitted']>args.budget for r in measured),
              scope='every submission/timeout/restart/acceptance/activation; first publication accepted activations only')
        (out/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['overall_pass'] else 1


def timing_report(file,budget,started=None,ended=None):
    with Path(file).open() as f:rows=list(csv.DictReader(f))
    all_submissions=[r for r in rows if r['event']=='submission']
    all_requests=[r for r in rows if r['event']=='request']
    summaries=[r for r in rows if r['event']=='summary']
    submitted=[r for r in all_submissions if started is None or started<=float(r['submitted'])<=ended]
    selected={r['sequence'] for r in submitted}
    requests=[r for r in all_requests if r['sequence'] in selected]
    pub=[r for r in rows if r['event']=='publish' and (started is None or started<=float(r['steady_s'])<=ended)]
    activation_rows=[r for r in rows if r['event']=='activation' and r['sequence'] in selected]
    sequence_total=int(summaries[0]['sequence']) if len(summaries)==1 else -1
    all_sequence=[int(r['sequence']) for r in all_submissions]
    all_results=[int(r['sequence']) for r in all_requests]
    integrity=bool(sequence_total>=0 and int(float(summaries[0]['violation']))==0 and
        sorted(all_sequence)==list(range(1,sequence_total+1)) and sorted(all_results)==sorted(all_sequence))
    def stats(rows,key):
        values=[float(r[key]) for r in rows]
        return {label:float(np.percentile(values,q)) if values else None for label,q in [('p50',50),('p95',95),('p99',99),('max',100)]}
    first={}
    for p in pub:first.setdefault(p['sequence'],float(p['steady_s']))
    activations=[first[r['sequence']]-float(r['submitted']) for r in requests if r['sequence'] in first]
    rejected={r['sequence'] for r in activation_rows if int(r['accepted'])==0}
    consecutive=maximum=0
    for request in requests:
        consecutive=consecutive+1 if int(request['accepted'])==0 or request['sequence'] in rejected else 0
        maximum=max(maximum,consecutive)
    return dict(requests=len(submitted),received_results=len(requests),log_integrity_pass=integrity,
      unfinished=len(submitted)-len(requests),failures=sum(int(r['accepted'])==0 for r in requests)+len(submitted)-len(requests),
      max_consecutive_failures=maximum,publications=len(pub),
      measurement_s=ended-started if started is not None else None,
      activation_attempts=len(activation_rows),accepted_activations=sum(int(r['accepted'])==1 for r in activation_rows),
      handover_rejected=sum(int(r['accepted'])==0 for r in activation_rows),
      handover=stats(activation_rows,'complete_s'),reanchor=stats(activation_rows,'validation_s'),
      native_failed=sum(int(r['status'])!=0 for r in requests),late=sum(float(r['complete_s'])>budget for r in requests),
      complete=stats(requests,'complete_s'),core=stats(requests,'core_s'),native=stats(requests,'native_s'),
      preparation=stats(requests,'preparation_s'),validation=stats(requests,'validation_s'),
      source_age=stats(requests,'observation_age_s'),publish_callback=stats(pub,'complete_s'),publish_gap=stats(pub,'publish_gap_s'),
      request_to_first_publish={k:float(np.percentile(activations,q)) if activations else None for k,q in [('p50',50),('p95',95),('p99',99),('max',100)]})


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--bundle',required=True);parser.add_argument('--vesc',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--frequency',type=float,default=20.);parser.add_argument('--budget',type=float,default=.05)
    parser.add_argument('--prefix',default='/mpcc_rt_shadow');parser.add_argument('--speed-tau',type=float,default=.2)
    parser.add_argument('--steer-tau',type=float,default=.15);parser.add_argument('--odom-delay',type=float,default=0.)
    parser.add_argument('--scenario',choices=['nominal','authority','odometry','clock','disable'],default='nominal')
    parser.add_argument('--inject-after',type=float,default=4.)
    parser.add_argument('--implementation',choices=['acados_cpp','legacy'],default='acados_cpp')
    parser.add_argument('--backend',choices=['ipopt','qp','acados'],default='ipopt')
    parser.add_argument('--legacy-artifact',default='',help='separately prepared legacy acados artifact')
    parser.add_argument('--single-lap',action='store_false',dest='repeat_laps',default=True)
    raise SystemExit(run(parser.parse_args()))
