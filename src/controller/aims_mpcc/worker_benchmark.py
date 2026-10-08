"""No-drive async worker measurement; compilation is a separate offline mode."""
import argparse
from contextlib import redirect_stdout
import io
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np
from .benchmark import summarize as direct_summary
from .solver_diagnostics import json_safe


def _distribution(values):
    return dict(observations=len(values),median_s=float(np.median(values)) if values else None,
                p95_s=float(np.percentile(values,95)) if values else None,
                p99_s=float(np.percentile(values,99)) if values else None,
                max_s=max(values) if values else None)


def summarize(rows,budget_s=.05):
    summary=direct_summary(rows,budget_s)
    summary.update(full_request_max_s=max((r['full_request_s'] for r in rows),default=None),
        full_request_median_s=float(np.median([r['full_request_s'] for r in rows])) if rows else None,
        late_replies=sum(r.get('discarded',False) and not r.get('censored',False) for r in rows),
        censored_requests=sum(r.get('censored',False) for r in rows),
        numerically_accepted_candidates=sum(bool(r.get('validation',{}).get('accepted')) for r in rows if r.get('validation')),
        executable_candidates=sum(r['success'] and bool(r.get('validation',{}).get('accepted')) and
                                  not r.get('discarded',False) and not r.get('censored',False)
                                  for r in rows if r.get('validation')))
    names=sorted({name for r in rows for name in r.get('components',{})})
    summary['components']={name:_distribution([r['components'][name] for r in rows if name in r.get('components',{})])
                           for name in names}
    return summary


def nominal_request(path,config,horizon,now,elapsed,theta=0.):
    point=path.at(theta)
    steering=float(np.arctan(config.wheelbase*(1+config.understeer_coefficient*config.cruise_speed**2)*point['curvature']))
    state=dict(x=point['x'],y=point['y'],yaw=point['yaw'],speed=config.cruise_speed,steering=steering)
    previous=dict(acceleration=0.,steering=steering,steering_rate=0.)
    return dict(generation=1,state=state,previous=previous,speed_refs=[config.cruise_speed]*(horizon+1),
                elapsed=elapsed,stamp=now,source_stamp=now,submitted_at=now,
                handover_command=dict(previous,speed=config.cruise_speed),
                map_alignment=(0.,0.,0.) if path.frame_id=='map' else None)


def _reply_row(result,active,delivered_at,config,path,deadline):
    from .validation import validate_candidate
    validation_start=time.monotonic()
    validation=validate_candidate(result,config,path)
    completed=time.monotonic();diagnostics=result.get('diagnostics',{})
    components=dict(active['components'],caller_validation_s=completed-validation_start)
    for source,target in (('preparation_time_s','preparation_s'),('optimizer_time_s','optimizer_s'),
                          ('diagnostics_time_s','diagnostics_s')):
        if diagnostics.get(source) is not None:components[target]=float(diagnostics[source])
    for source,target in (('solve_time_s','worker_solve_s'),('worker_validation_time_s','worker_validation_s')):
        if result.get(source) is not None:components[target]=float(result[source])
    if result.get('worker_started_at') is not None:
        components['queue_s']=max(0.,result['worker_started_at']-active['submitted_at'])
    if result.get('worker_finished_at') is not None:
        components['delivery_s']=max(0.,delivered_at-result['worker_finished_at'])
        components['worker_total_s']=max(0.,result['worker_finished_at']-result['worker_started_at'])
    discarded=bool(result.get('discarded') or delivered_at>active['submitted_at']+deadline)
    return dict(success=bool(result.get('success')),status=result.get('status'),discarded=discarded,
        worker_discarded=bool(result.get('discarded')),skip_notified=bool(result.get('skip_notified')),
        delivered=True,censored=False,submitted_at=active['submitted_at'],delivered_at=delivered_at,
        completed_at=completed,full_request_s=completed-active['started_at'],
        delivery_s=delivered_at-active['submitted_at'],components=components,validation=validation,
        worker_validation=result.get('validation'),iterations=result.get('iterations'),
        constraint_violation=result.get('constraint_violation'),artifact_fingerprint=diagnostics.get('artifact_fingerprint'),
        native_core=diagnostics.get('native_core'),reference_progress=active['reference_progress'],execution_authorized=False)


def run_case(reference,config,backend,horizon,samples=100,duration=None,period=.05,deadline=.25,
             budget=.05,artifact_directory=None,startup_timeout=180.,log_directory=None,poll_period=.001,sample_lap=False):
    """Collect every submitted request; drain replies after the sample/time limit."""
    if samples is not None and (type(samples) is not int or samples<1):raise ValueError('samples must be a positive integer')
    for name,value in dict(period=period,deadline=deadline,budget=budget,startup_timeout=startup_timeout,poll_period=poll_period).items():
        if not math.isfinite(value) or value<=0:raise ValueError(f'{name} must be positive and finite')
    if duration is not None and (not math.isfinite(duration) or duration<=0):raise ValueError('duration must be positive and finite')
    if samples is None and duration is None:raise ValueError('samples or duration required')
    from .path import ReferencePath
    from .worker import AsyncSolver
    path=ReferencePath.load(reference);path.validate_config(config)
    worker=AsyncSolver(reference,config,horizon,deadline,log_directory,backend,artifact_directory)
    rows=[];events=[];blocked=0;submitted=0;active=None;started=time.monotonic()
    report=dict(backend=backend,horizon=horizon,dt=.1,scenario='synthetic-reference-following-states' if sample_lap else 'fixed-nominal-reference-state',
        sample_lap=bool(sample_lap),progress_mapping='forward Euler: v/norm(spline derivative)*elapsed' if sample_lap else 'fixed',
        config=asdict(config),samples_limit=samples,duration_limit_s=duration,request_period_s=period,
        result_deadline_s=deadline,budget_s=budget,caller_poll_period_s=poll_period,
        includes_ipc=True,includes_caller_validation=True,includes_solver_construction=False,
        production_node_timer_included=False,supervisor_handover_included=False,
        online_compilation_allowed=False,execution_authorized=False,requests=rows,events=events,status='completed')
    try:
        while not worker.ready:
            now=time.monotonic();event=worker.poll(now)
            if event is not None:
                events.append(event)
                if event.get('kind') in ('error','restarting'):raise RuntimeError(event.get('error',event.get('reason')))
            if now-started>startup_timeout:raise TimeoutError('Worker initialization timeout; prepare artifacts offline first')
            if not worker.ready:time.sleep(poll_period)
        report['startup_s']=time.monotonic()-started
        collection_start=time.monotonic();next_due=collection_start;last_submission=None;theta=0.
        while True:
            now=time.monotonic();event=worker.poll(now);received=time.monotonic()
            if event is not None:
                if event.get('kind')=='result' and active is not None:
                    rows.append(_reply_row(event,active,received,config,path,deadline));active=None
                else:events.append(event)
                if event.get('kind') in ('error','restarting'):
                    if active is not None:
                        rows.append(dict(success=False,status=event['kind'],discarded=True,delivered=False,censored=True,
                            full_request_s=received-active['started_at'],components=active['components'],validation=None,
                            submitted_at=active['submitted_at'],execution_authorized=False));active=None
                    report.update(status='worker_error',error=event.get('error',event.get('reason')));break
            now=time.monotonic()
            limit=(samples is not None and submitted>=samples) or (duration is not None and now-collection_start>=duration)
            if limit and active is None:break
            if not limit and now>=next_due:
                due_count=1+int((now-next_due)/period);next_due+=due_count*period
                if active is not None:blocked+=due_count
                else:
                    blocked+=due_count-1;prep=time.monotonic()
                    elapsed=period if last_submission is None else prep-last_submission
                    if sample_lap and last_submission is not None:
                        theta=(theta+config.cruise_speed*elapsed/np.linalg.norm(path.curve.numpy(theta,1)))%path.length
                    request=nominal_request(path,config,horizon,prep,elapsed,theta)
                    submit_started=time.monotonic();request['submitted_at']=submit_started
                    accepted=worker.submit(request);submit_end=time.monotonic()
                    if accepted:
                        submitted+=1;last_submission=submit_started
                        active=dict(started_at=prep,submitted_at=submit_started,reference_progress=theta,
                            components=dict(request_preparation_s=submit_started-prep,submission_s=submit_end-submit_started))
                    else:
                        events.append(dict(kind='submission_rejected',stamp=submit_end));blocked+=1
            time.sleep(poll_period)
        report['collection_wall_time_s']=time.monotonic()-collection_start
    except (RuntimeError,TimeoutError) as exc:
        report.update(status='startup_failed' if not worker.ready else 'worker_error',error=str(exc))
    finally:
        worker.close();worker.process.join(timeout=1.)
    report.update(submitted_requests=submitted,blocked_submission_opportunities=blocked,summary=summarize(rows,budget))
    return json_safe(report)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',required=True);parser.add_argument('--vehicle-config',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--backend','--backends',nargs='+',choices=['ipopt','acados','qp'],default=['ipopt','acados','qp'])
    parser.add_argument('--horizon','--horizons',nargs='+',type=int,choices=[10,15,20],default=[10,15,20])
    parser.add_argument('--samples',type=int);parser.add_argument('--seconds','--duration',type=float)
    parser.add_argument('--period',type=float,default=.05);parser.add_argument('--deadline',type=float,default=.25)
    parser.add_argument('--budget',type=float,default=.05);parser.add_argument('--startup-timeout',type=float,default=180.)
    parser.add_argument('--poll-period',type=float,default=.001);parser.add_argument('--artifact-directory')
    parser.add_argument('--log-directory');parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--sample-lap',action='store_true',help='Generate nominal moving reference states; no recorded vehicle motion or closed loop')
    args=parser.parse_args(argv)
    from .io import load_config
    config=load_config(args.vehicle_config);cases=[]
    samples=args.samples if args.samples is not None or args.seconds is not None else 100
    for backend in args.backend:
        for horizon in args.horizon:
            if args.prepare_only:
                from .prepare_solver import main as prepare_main
                call=[args.reference,'--vehicle-config',args.vehicle_config,'--backend',backend,'--horizon',str(horizon)]
                if args.artifact_directory:call+=['--artifact-directory',args.artifact_directory]
                print(json.dumps(dict(kind='preparing',backend=backend,horizon=horizon)),flush=True)
                start=time.monotonic();captured=io.StringIO()
                with redirect_stdout(captured):
                    try:prepare_main(call);case=dict(backend=backend,horizon=horizon,status='prepared')
                    except Exception as exc:case=dict(backend=backend,horizon=horizon,status='prepare_failed',error=str(exc))
                for line in captured.getvalue().splitlines():
                    try:record=json.loads(line)
                    except ValueError:continue
                    if isinstance(record,dict) and 'status' in record:case['warmup']=record
                case.update(preparation_s=time.monotonic()-start,execution_authorized=False)
            else:
                case=run_case(args.reference,config,backend,horizon,samples,args.seconds,args.period,args.deadline,args.budget,
                              args.artifact_directory,args.startup_timeout,args.log_directory,args.poll_period,args.sample_lap)
            cases.append(case);print(json.dumps({k:case[k] for k in ('backend','horizon','status','summary') if k in case}),flush=True)
    root=Path(__file__).parent
    report=dict(schema_version=1,layer='offline-preparation' if args.prepare_only else 'full-async-worker',
        hardware_validated=False,execution_authorized=False,production_node_timer_included=False,
        supervisor_handover_included=False,offline_compilation_allowed=args.prepare_only,online_compilation_allowed=False,
        platform=dict(system=platform.system(),machine=platform.machine(),python=platform.python_version()),
        reference_sha256=hashlib.sha256((Path(args.reference)/'path.csv').read_bytes()).hexdigest(),
        reference_metadata_sha256=hashlib.sha256((Path(args.reference)/'metadata.json').read_bytes()).hexdigest(),
        vehicle_config_sha256=hashlib.sha256(Path(args.vehicle_config).read_bytes()).hexdigest(),
        source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
            ('worker_benchmark.py','worker.py','validation.py','envelope.py','solver.py','backends.py',
             'acados_backend.py','qp_backend.py','backend_models.py','rollout_native.py','kernels/rollout.c')},
        component_note='queue includes submission/dispatch; delivery component starts after worker completion; full request includes request preparation through caller validation',cases=cases)
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(json_safe(report),allow_nan=False,indent=2)+'\n')
    return int(any(c['status'] not in ('completed','prepared') for c in cases))


if __name__=='__main__':raise SystemExit(main())
