#!/usr/bin/env python3
"""Isolated real selector/converter route with an independent lagged plant.

The controller's odometry comes from the synthetic plant; an estimator replay
may run alongside it for CPU load, but is not its feedback or ground truth.
"""
import argparse
import csv
import hashlib
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
          '-p',f'solver_timeout:={args.budget}','-p',f'handover_delay:={args.handover_delay}','-p',f'plan_ttl:={.8*artifact["horizon"]*artifact["dt"]}',
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
            if status['phase']=='FAULT':fault_events.append((time.monotonic(),dict(status)))
    subscription=plant.create_subscription(DiagnosticArray,args.prefix+'/mpcc/status',observe,10)
    client=plant.create_client(SetBool,args.prefix+'/mpcc/enable')
    commands=child_commands(args.vesc,remaps)
    command=['ros2','run','aims_mpcc_rt','mpcc_rt_node','--ros-args',
      '-p',f'artifact_directory:={bundle}','-p',f'vehicle_config:={bundle}/input_config.yaml',
      '-p',f'path_directory:={bundle}/input_reference','-p','simulation:=true',
      '-p',f'repeat_laps:={str(args.repeat_laps).lower()}',
      '-p',f'solve_frequency:={args.frequency}','-p',f'horizon:={artifact["horizon"]}',
      '-p',f'plan_ttl:={.8*artifact["horizon"]*artifact["dt"]}',
      '-p',f'solver_timeout:={args.budget}','-p',f'handover_delay:={args.handover_delay}','-p',f'log_directory:={out}/controller',*remaps]
    legacy=None;callbacks=[];publications=[];activations={};replies={};attempts={}
    fault_events=[];positive_before=False;actionability=None;fault_seen=None;zero_seen=None
    if args.implementation=='acados_cpp':
        command=[sys.executable,str(Path(__file__).with_name('runtime_supervisor.py')),
                 '--status-topic',args.prefix+'/mpcc/status','--report',str(out/'supervisor.json'),'--',*command]
        commands.append(('controller',command))
    processes=[];handles=[];samples=[];started=None;injected=None;measurement_end=None
    last_progress=None;unwrapped_progress=0.
    result={'scope':'independent synthetic feedback through real RC selector and VESC converter; isolated topics',
      'arguments':vars(args),'commands':commands,'bundle_manifest':json.loads((bundle/'native_manifest.json').read_text())}
    if args.implementation=='acados_cpp':
        from ament_index_python.packages import get_package_prefix
        binary=Path(get_package_prefix('aims_mpcc_rt'))/'lib/aims_mpcc_rt/mpcc_rt_node'
        result['controller_binary']=str(binary)
        result['controller_binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
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
                        record_worker_reply(attempts[key],answer,now)
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
        deadline=time.monotonic()+20.;future=None;disabled_seen=None
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.005)
            if any(p.poll() is not None for p in processes):raise RuntimeError('route process exited during startup')
            if status['phase'] in ('WAITING','READY') and disabled_seen is None:disabled_seen=time.monotonic()
            native_ready=(args.implementation!='acados_cpp' or (disabled_seen is not None and
                time.monotonic()-disabled_seen>=.2 and status.get('values',{}).get('worker_ready')=='true'))
            if not future and native_ready and client.service_is_ready() and plant.count_publishers(args.prefix+'/ackermann_cmd')==1:
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
                samples.append([now-started,plant.x,plant.y,plant.yaw,plant.speed,error,heading,plant.speed_target,plant.steer_target,unwrapped_progress,plant.steering,
                    (plant.speed_target-plant.speed)/plant.speed_tau])
            if args.scenario!='nominal' and injected is None and now-started>=args.inject_after:
                positive_before=bool(status['phase']=='RUNNING' and plant.commands and
                    plant.commands[-1][1]>0. and now-plant.commands[-1][0]<=.1)
                if not positive_before:raise RuntimeError('fault injection requires recent positive RUNNING output')
                injected=now;actionability=plant.last_odom+.1 if args.scenario=='odometry' else now
                if args.scenario=='authority':plant.autonomy=False
                elif args.scenario=='odometry':plant.publish_odom=False
                elif args.scenario=='clock':plant.clock_offset=-.2
                elif args.scenario=='disable':
                    request=SetBool.Request();request.data=False;client.call_async(request)
            if status['phase']=='FAULT':
                if args.scenario=='nominal':raise RuntimeError('nominal controller fault: '+status['reason'])
                if injected is None:raise RuntimeError('controller faulted before the injected scenario')
                fault_seen=min((json.loads(event.get('values',{}).get('fault_steady_s',str(t)))
                    for t,event in fault_events if t>=injected),default=None)
                zero_seen=min((t for t,speed,_ in plant.commands if t>=injected and abs(speed)<1e-9),default=None)
                if zero_seen is not None and now-zero_seen>=.3:
                    after=[speed for t,speed,_ in plant.commands if t>=zero_seen]
                    if not after or any(abs(speed)>1e-9 for speed in after):raise RuntimeError('fault output did not remain zero')
                    break
            if args.scenario=='nominal' and args.repeat_laps and now-started>1. and status['phase']!='RUNNING':
                raise RuntimeError('nominal controller left RUNNING: '+status['phase']+' '+status['reason'])
            if args.scenario=='disable' and operator_stop_pass(positive_before=positive_before,injected_s=injected,
                phase=status['phase'],reason=status['reason'],speed=plant.speed,target_speed=plant.speed_target):break
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
        result['independent_plant_envelope']=plant_envelope_report(data[:,4],data[:,10],data[:,11],config)
        result['tracking_pass']=bool(result['contour_max_m']<=.25 and result['speed_p50_mps']>=.8*config.cruise_speed and
          result['maximum_command_speed_mps']<=config.max_speed+1e-6 and result['maximum_command_steering_rad']<=config.steer_limit+1e-6)
        if args.repeat_laps and args.scenario=='nominal':
            tail=data[data[:,0]>=max(3.,result['elapsed_s']-3.),4]
            result['sustained_tracking_pass']=bool(result['elapsed_s']>=args.seconds-.05 and len(tail) and np.median(tail)>=.8*config.cruise_speed)
            result['tracking_pass']=result['tracking_pass'] and result['sustained_tracking_pass']
        if args.scenario!='nominal':
            if args.scenario=='disable':
                result['fault_pass']=operator_stop_pass(positive_before=positive_before,injected_s=injected,
                    phase=status['phase'],reason=status['reason'],speed=plant.speed,target_speed=plant.speed_target)
            else:
                expected={'authority':'withdrawn','odometry':'freshness' if args.implementation=='acados_cpp' else 'expired',
                          'clock':'odometry' if args.implementation=='acados_cpp' else 'timestamp'}[args.scenario]
                result['fault_pass']=fault_pass(positive_before=positive_before,phase=status['phase'],reason=status['reason'],
                    expected_reason=expected,actionability_s=actionability or 0.,fault_s=fault_seen,zero_s=zero_seen,
                    zero_speed=plant.speed_target)
                result['fault_evidence']=dict(positive_before=positive_before,injected_s=injected,actionability_s=actionability,
                    fault_s=fault_seen,zero_s=zero_seen,expected_reason=expected,response_bound_s=.1,
                    fault_detection_s=fault_seen-actionability if fault_seen is not None else None,
                    zero_response_s=zero_seen-actionability if zero_seen is not None else None)
        result['finish_pass']=bool(args.repeat_laps or (status['phase']=='COMPLETE' and plant.speed<.05 and abs(plant.speed_target)<1e-9))
        result['overall_pass']=(result['tracking_pass'] and result['finish_pass']) if args.scenario=='nominal' else result['fault_pass']
    except Exception as exc:
        measurement_end=time.monotonic();result['overall_pass']=False;result['error']=str(exc)
        result['measurement_start']=started;result['measurement_end']=measurement_end
        result['elapsed_s']=measurement_end-started if started is not None else None
        if samples:
            data=np.asarray(samples)
            result['independent_plant_envelope']=plant_envelope_report(data[:,4],data[:,10],data[:,11],config)
            result['contour_max_m']=float(np.max(np.abs(data[:,5])))
            result['final_speed_mps']=float(data[-1,4])
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
            writer=csv.writer(f);writer.writerow(['time','x','y','yaw','speed','contour','heading','speed_command','steering_command','progress','actual_steering','physical_acceleration']);writer.writerows(samples)
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
                row['disposition']='inflight' if row['unfinished'] else 'cutoff' if row['terminal']>measurement_end else 'rejected' if row['failed'] else 'accepted'
            consecutive=maximum=0
            for row in measured:
                consecutive=consecutive+1 if row['disposition']=='rejected' else 0;maximum=max(maximum,consecutive)
            measured_pub=[r for r in publications if started is not None and started<=r[0]<=measurement_end]
            measured_callbacks=[r for r in callbacks if started is not None and started<=r[0]<=measurement_end]
            measured_results=[r for r in measured if r['disposition'] not in ('cutoff','inflight')]
            measured_activations=[r for key,r in activations.items() if started is not None and
                started<=key<=measurement_end and r['first_proposal_published_at']<=measurement_end]
            solved=[r for r in solved if started is not None and started<=r.get('submitted_at',r.get('request',{}).get('submitted_at',-1))<=measurement_end]
            (out/'request-outcomes.json').write_text(json.dumps(measured,indent=2)+'\n')
            result['timing']=dict(requests=len(measured),accounted=len(measured),failures=sum(r['disposition']=='rejected' for r in measured),
              inflight=sum(r['disposition']=='inflight' for r in measured),cutoff=sum(r['disposition']=='cutoff' for r in measured),
              rejected=sum(r['disposition']=='rejected' for r in measured),
              max_consecutive_failures=maximum,unfinished=sum(r['unfinished'] for r in measured),
              log_integrity_pass=not any(r['unfinished'] for r in measured),submission_records=len(attempts),
              measurement_s=measurement_end-started if started is not None else 0,publications=len(measured_pub),
              complete=statistics([r['terminal']-r['submitted'] for r in measured_results]),
              core=statistics([r['solve_time_s'] for r in solved]),
              publish_callback=statistics([b-a for a,b in measured_callbacks]),
              publish_gap=statistics([b[0]-a[0] for a,b in zip(measured_pub,measured_pub[1:])]),
              request_to_first_publish=statistics([r['request_to_first_proposal_s'] for r in measured_activations]),
              accepted_activations=len(measured_activations),received_replies=len(replies),
              late=sum(r['terminal']-r['submitted']>args.budget for r in measured_results),
              scope='every submission/timeout/restart/acceptance/activation; first publication accepted activations only')
        (out/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['overall_pass'] else 1


def plant_envelope_report(speed,steering,acceleration,config):
    speed=np.asarray(speed,float);steering=np.asarray(steering,float);acceleration=np.asarray(acceleration,float)
    accel_axis=config.longitudinal_envelope_accel or config.accel_limit
    brake_axis=config.longitudinal_envelope_brake or config.brake_limit
    # This independent bicycle plant has yaw_rate=v*tan(delta)/wheelbase. It
    # contains no identified understeer law, tyre forces, or real vehicle data.
    lateral=speed**2*np.tan(steering)/config.wheelbase
    axes=np.where(acceleration>=0,accel_axis,brake_axis)
    utilization=(acceleration/axes)**2+(lateral/config.lateral_accel_limit)**2
    return dict(scope='independent synthetic lagged bicycle plant; not an identified real vehicle',
        model_utilization_max=float(np.max(utilization)),model_utilization_p95=float(np.percentile(utilization,95)),
        violation_samples=int(np.sum(utilization>1.+1e-6)),samples=len(utilization),
        physical_acceleration_max_mps2=float(np.max(acceleration)),physical_braking_max_mps2=float(max(0.,np.max(-acceleration))),
        lateral_acceleration_abs_max_mps2=float(np.max(np.abs(lateral))),
        acceptance_gate=False,limitation='motor speed floor and plant speed lag can exceed the ideal model longitudinal bound')


def record_worker_reply(row,reply,now):
    row['events'].append(dict(kind=reply['kind'],received=now))
    if reply['kind']=='skipped':
        # A deadline notification leaves the native solve running. Its eventual
        # delivery or bounded worker restart is the terminal accounting event.
        row['failed']=True;row['deadline_notified']=now
    elif reply['kind'] in ('error','restarting'):
        row['failed']=True;row['terminal']=now
    elif reply['kind']=='result':
        row['terminal']=now
        row['failed']|=not reply.get('success',False) or reply.get('discarded',False)


def operator_stop_pass(*,positive_before,injected_s,phase,reason,speed,target_speed):
    return bool(positive_before and injected_s is not None and phase=='READY'
                and 'stop' in reason.lower() and speed<.05 and abs(target_speed)<1e-9)


def fault_pass(*, positive_before, phase, reason, expected_reason,
               actionability_s, fault_s, zero_s, zero_speed):
    """A stop qualifies only after demonstrated driving and the named event."""
    return bool(positive_before and phase == 'FAULT'
                and expected_reason.lower() in reason.lower() and zero_speed == 0.
                and fault_s is not None and zero_s is not None
                and 0. <= fault_s-actionability_s <= .1
                and 0. <= zero_s-actionability_s <= .1)


def timing_report(file,budget,started=None,ended=None):
    with Path(file).open() as f:rows=list(csv.DictReader(f))
    all_submissions=[r for r in rows if r['event']=='submission']
    all_requests=[r for r in rows if r['event']=='request']
    summaries=[r for r in rows if r['event']=='summary']
    submitted=[r for r in all_submissions if started is None or started<=float(r['submitted'])<=ended]
    skipped=[r for r in rows if r['event']=='skipped_pending' and
             (started is None or started<=float(r['steady_s'])<=ended)]
    selected={r['sequence'] for r in submitted}
    requests=[r for r in all_requests if r['sequence'] in selected]
    # Completion means delivery to the runtime decision callback, not merely a
    # native solve finishing. Drain results remain accounted as cutoff outcomes.
    measured_results=[r for r in requests if ended is None or float(r['steady_s'])<=ended]
    pub=[r for r in rows if r['event']=='publish' and (started is None or started<=float(r['steady_s'])<=ended)]
    activation_rows=[r for r in rows if r['event']=='activation' and r['sequence'] in selected
                     and (ended is None or float(r['steady_s'])<=ended)]
    sequence_total=int(summaries[0]['sequence']) if len(summaries)==1 else -1
    all_sequence=[int(r['sequence']) for r in all_submissions]
    all_results=[int(r['sequence']) for r in all_requests]
    integrity=bool(sequence_total>=0 and int(float(summaries[0]['violation']))==0 and
        sorted(all_sequence)==list(range(1,sequence_total+1)) and sorted(all_results)==sorted(all_sequence))
    def stats(records,key):
        values=[float(r[key]) for r in records if r.get(key) not in (None,'')]
        return {label:float(np.percentile(values,q)) if values else None for label,q in [('p50',50),('p95',95),('p99',99),('max',100)]}
    by_sequence={r['sequence']:r for r in requests}
    outcomes=[]
    for submission in submitted:
        reply=by_sequence.get(submission['sequence'])
        disposition=('inflight' if reply is None else 'cutoff' if ended is not None and float(reply['steady_s'])>ended
                     else 'rejected' if int(reply['accepted'])==0 else 'accepted')
        outcomes.append(dict(sequence=int(submission['sequence']),submitted=float(submission['submitted']),
                             disposition=disposition,terminal_disposition=reply.get('disposition','') if reply else None,
                             delivered_at_s=float(reply['steady_s']) if reply else None,
                             complete_s=float(reply['complete_s']) if reply else None,
                             reason=reply.get('reason','') if reply else None))
    counts={key:sum(r['disposition']==key for r in outcomes) for key in ('inflight','cutoff','rejected','accepted')}
    first={}
    for p in pub:
        if int(p['sequence'])>0:first.setdefault(p['sequence'],float(p['steady_s']))
    published=[r for r in measured_results if r['sequence'] in first]
    latencies=[first[r['sequence']]-float(r['submitted']) for r in published]
    rejected={r['sequence'] for r in activation_rows if int(r['accepted'])==0}
    consecutive=maximum=0
    for outcome in outcomes:
        failed=outcome['disposition']=='rejected' or str(outcome['sequence']) in rejected
        consecutive=consecutive+1 if failed else 0;maximum=max(maximum,consecutive)
    return dict(requests=len(submitted),eligible_slots=len(submitted)+len(skipped),skipped_pending_slots=len(skipped),
      received_results=len(requests),log_integrity_pass=integrity,
      accounted=len(outcomes),outcomes=outcomes,**counts,unfinished=counts['inflight'],failures=counts['rejected'],
      max_consecutive_failures=maximum,publications=len(pub),
      measurement_s=ended-started if started is not None else None,
      activation_attempts=len(activation_rows),accepted_activations=sum(int(r['accepted'])==1 for r in activation_rows),
      handover_rejected=sum(int(r['accepted'])==0 for r in activation_rows),
      handover=stats(activation_rows,'complete_s'),reanchor=stats(activation_rows,'validation_s'),
      native_failed=sum(int(r['status'])!=0 for r in measured_results),late=sum(float(r['complete_s'])>budget+1e-12 for r in measured_results),
      complete=stats(measured_results,'complete_s'),core=stats(measured_results,'core_s'),native=stats(measured_results,'native_s'),
      preparation=stats(measured_results,'preparation_s'),validation=stats(measured_results,'validation_s'),
      compute=stats(measured_results,'compute_s'),delivery_wait=stats(measured_results,'delivery_wait_s'),
      published_complete=stats(published,'complete_s'),
      source_age=stats(measured_results,'observation_age_s'),publish_callback=stats(pub,'complete_s'),publish_gap=stats(pub,'publish_gap_s'),
      request_to_first_publish={k:float(np.percentile(latencies,q)) if latencies else None for k,q in [('p50',50),('p95',95),('p99',99),('max',100)]})


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--bundle',required=True);parser.add_argument('--vesc',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--frequency',type=float,default=20.);parser.add_argument('--budget',type=float,default=.05)
    parser.add_argument('--handover-delay',type=float,default=.02,help='forecast lead; default preserves the delivered 20 ms profile')
    parser.add_argument('--prefix',default='/mpcc_rt_test');parser.add_argument('--speed-tau',type=float,default=.2)
    parser.add_argument('--steer-tau',type=float,default=.15);parser.add_argument('--odom-delay',type=float,default=0.)
    parser.add_argument('--scenario',choices=['nominal','authority','odometry','clock','disable'],default='nominal')
    parser.add_argument('--inject-after',type=float,default=4.)
    parser.add_argument('--implementation',choices=['acados_cpp','legacy'],default='acados_cpp')
    parser.add_argument('--backend',choices=['ipopt','qp','acados'],default='ipopt')
    parser.add_argument('--legacy-artifact',default='',help='separately prepared legacy acados artifact')
    parser.add_argument('--single-lap',action='store_false',dest='repeat_laps',default=True)
    raise SystemExit(run(parser.parse_args()))
