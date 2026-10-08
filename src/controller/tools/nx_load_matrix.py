#!/usr/bin/env python3
"""Run no-drive worker measurements with recorded estimation load on Orin NX.

Source the ROS and localization overlays before invoking this tool. Solver
artifacts must already exist. All spawned load processes belong to this run.
"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def enable_subreaper():
    # Adopt this driver's detached descendants if a replay parent exits early.
    if ctypes.CDLL(None,use_errno=True).prctl(36,1,0,0,0)!=0:
        raise OSError(ctypes.get_errno(),'Cannot own orphaned replay children')


def _owned_children_from_status():
    """NX kernels can omit task/children; follow PPid only from this driver."""
    relations={};own_pid=os.getpid();own_seen=False
    try:
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():continue
            try:status=(entry/'status').read_text()
            except (FileNotFoundError,ProcessLookupError):continue
            parent=int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
            pid=int(entry.name);own_seen=own_seen or pid==own_pid
            relations.setdefault(parent,[]).append(pid)
        if not own_seen:raise OSError('Driver status is unavailable')
    except (OSError,ValueError,StopIteration) as exc:
        raise RuntimeError('Cannot enumerate owned load processes from /proc') from exc
    result=[];seen={own_pid};pending=[own_pid]
    while pending:
        for child in relations.get(pending.pop(),[]):
            if child in seen:continue
            seen.add(child);result.append(child);pending.append(child)
    return result


def owned_children():
    result=[];seen={os.getpid()}
    def visit(pid):
        tasks=Path('/proc')/str(pid)/'task'
        try:threads=list(tasks.iterdir())
        except FileNotFoundError:
            if pid==os.getpid():raise
            return  # descendant exited during enumeration
        for thread in threads:
            try:values=(thread/'children').read_text().split()
            except FileNotFoundError:
                if thread.exists():raise  # CONFIG_PROC_CHILDREN is unavailable
                continue  # thread exited during enumeration
            for value in values:
                child=int(value)
                if child in seen:continue
                seen.add(child);result.append(child);visit(child)
    try:visit(os.getpid())
    except (OSError,ValueError):return _owned_children_from_status()
    return result


def cleanup(processes,grace=3.):
    """Terminate and reap this driver's children, including adopted orphans."""
    for sig,delay in ((signal.SIGINT,grace),(signal.SIGTERM,.5),(signal.SIGKILL,.5)):
        until=time.monotonic()+delay;signalled=set()
        while True:
            # Descendants can appear or be adopted during any escalation phase.
            for pid in owned_children():
                if pid in signalled:continue
                try:os.kill(pid,sig)
                except ProcessLookupError:pass
                signalled.add(pid)
            for process in processes:process.poll()
            # Popen children are reaped first so their return codes stay intact.
            protected={process.pid for process in processes if process.poll() is None}
            for pid in owned_children():
                if pid in protected:continue
                try:os.waitpid(pid,os.WNOHANG)
                except ChildProcessError:pass
            if not owned_children():return
            if time.monotonic()>=until:break
            time.sleep(.02)
    if owned_children():raise RuntimeError('Owned load processes did not shut down')


def read_events(filename):
    rows=[]
    if not filename.exists():return rows
    for line in filename.read_text().splitlines():
        try:rows.append(json.loads(line))
        except ValueError:pass  # last buffered line can be incomplete while live
    return rows


def load_coverage(rows,events,ended_monotonic=None):
    """Report actual overlap; an idle tail cannot qualify as shared load."""
    timings=[event['received'] for event in events if event.get('kind')=='native_timing']
    if not rows or not timings:return dict(valid=False,reason='No requests or native registrations')
    def completed(row):
        return row.get('completed_at',row['submitted_at']+row['full_request_s'])
    begin,end=rows[0]['submitted_at'],completed(rows[-1])
    first,last=min(timings),max(timings)
    overlapping=[row for row in rows if first<=row['submitted_at'] and completed(row)<=last]
    interval=max(0.,min(last,end)-max(first,begin))
    registrations=sum(begin<=stamp<=end for stamp in timings)
    within=sorted(stamp for stamp in timings if begin<=stamp<=end)
    boundaries=[begin]+within+[end]
    max_gap=max(right-left for left,right in zip(boundaries,boundaries[1:]))
    committed=sum(event.get('kind')=='anchor' and
        event.get('values',{}).get('anchor_committed')=='true' for event in events)
    # One-second callback gaps cover normal 1 Hz registration cadence. The
    # process itself must remain alive for the complete measurement interval.
    valid=(first<=begin+1. and last>=end-1. and registrations>=max(1,int(end-begin))
           and max_gap<=1. and committed>0 and (ended_monotonic is None or ended_monotonic>=end))
    return dict(valid=bool(valid),measurement_start=begin,measurement_end=end,
        native_start=first,native_end=last,native_updates_in_window=registrations,
        committed_anchor_events=committed,shared_interval_s=interval,
        max_native_gap_s=max_gap,native_gap_limit_s=1.,
        fully_overlapping_requests=len(overlapping),total_requests=len(rows),
        reason='' if valid else 'Incomplete replay or registration coverage')


def replay_errors(entry):
    """Graph/init audit completion is separate from native computation coverage."""
    errors=[];code=entry.get('replay_exit')
    # Python SIGINT is -2; a wrapper may report 128+SIGINT. Either is expected
    # only for our requested shutdown, with a complete successful summary below.
    if code!=0 and not (entry.get('intentional_replay_shutdown') and code in (-signal.SIGINT,128+signal.SIGINT)):
        errors.append(f'Unexpected replay exit: {code}')
    if entry.get('replay_drain_timeout'):errors.append('Replay did not finish within its bounded drain')
    if entry.get('replay_shutdown_timeout'):errors.append('Replay shutdown timed out')
    summary=entry.get('replay_summary')
    if not isinstance(summary,dict):
        errors.append('Missing final replay summary');return errors
    counts=summary.get('counts',{});authorities=summary.get('tf_authorities',{})
    if counts.get('ekf',0)<=0 or counts.get('body_cloud',0)<=0:
        errors.append('Replay produced no complete local state')
    if (authorities.get('odom/base_link')!=1 or authorities.get('map/odom')!=1 or
            any(count!=1 for count in authorities.values())):
        errors.append('Replay TF authority audit failed')
    if summary.get('initializer_cli_exit')!=0:errors.append('Replay initialization CLI did not succeed')
    if summary.get('accepted',0)<=0:errors.append('Replay established no trusted global anchor')
    return errors


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reference',required=True)
    p.add_argument('--vehicle-config',required=True)
    p.add_argument('--artifact-directory',required=True)
    p.add_argument('--backend',choices=['acados','qp','ipopt'],default='acados')
    p.add_argument('--horizon',type=int,choices=[10,15,20],default=10)
    p.add_argument('--seconds',type=float,default=300.)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--conditions',nargs='+',choices=['idle','shared','stress'],
                   default=['idle','shared','stress'])
    p.add_argument('--replay-runner',required=True)
    p.add_argument('--bag',required=True)
    p.add_argument('--map',required=True)
    p.add_argument('--seed',required=True)
    p.add_argument('--domain',type=int,default=202)
    p.add_argument('--sample-lap',action='store_true')
    args=p.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds or args.repeats<1:
        p.error('Positive seconds and repeat count required')
    enable_subreaper()
    args.output.mkdir(parents=True,exist_ok=False)
    record=dict(arguments=vars(args).copy(),started_wall=time.time(),
                driving_publishers_started=False,results=[])
    record['arguments']['output']=str(args.output)
    for key in ('vehicle_config','seed','replay_runner'):
        record[key+'_sha256']=hashlib.sha256(Path(getattr(args,key)).read_bytes()).hexdigest()
    record['map_sha256']=hashlib.sha256(Path(args.map).read_bytes()).hexdigest()
    (args.output/'matrix.json').write_text(json.dumps(record,indent=2)+'\n')
    # Two independent workers each occupy approximately 60% of one CPU core.
    stress_code='''import math,time
while True:
 begin=time.monotonic()
 while time.monotonic()-begin<.03:
  value=sum(math.sqrt(i) for i in range(1,300))
 time.sleep(max(0.,.05-(time.monotonic()-begin)))
'''
    for condition in args.conditions:
        for repeat in range(args.repeats):
            directory=args.output/f'{condition}-{repeat+1}'
            directory.mkdir()
            processes=[];logs=[];entry=dict(condition=condition,repeat=repeat+1,
                started_wall=time.time(),started_monotonic=time.monotonic(),status='running')
            def spawn(command,name,env=None):
                log=(directory/(name+'.log')).open('w');logs.append(log)
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                         start_new_session=True,env=env)
                processes.append(process);return process
            try:
                spawn(['tegrastats','--interval','1000'],'tegrastats')
                if condition=='stress':
                    for i in range(2):spawn([sys.executable,'-c',stress_code],f'stress-{i}')
                    entry['stress_workers']=2;entry['stress_duty']=.6
                replay=None
                if condition in ('shared','stress'):
                    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
                    # The controller pins its BLAS/OpenMP workers separately.
                    # Leave estimator concurrency to its installed parameters.
                    env.pop('OMP_NUM_THREADS',None)
                    entry['estimator_omp_environment']=None
                    replay=spawn(['/usr/bin/python3',args.replay_runner,args.bag,
                        str(directory/'replay'),'--map',args.map,'--seed',args.seed,
                        '--use-initializer-cli','--max-seconds',str(args.seconds+60.)],
                        'replay',env)
                    # Wait for actual native registration, then verify the
                    # complete measurement window after collection.
                    until=time.monotonic()+60.
                    while time.monotonic()<until:
                        if replay.poll() is not None:
                            raise RuntimeError('Estimation replay exited before measurement')
                        if any(event.get('kind')=='native_timing' for event in
                               read_events(directory/'replay/events.jsonl')):break
                        time.sleep(.2)
                    else:raise RuntimeError('No native registration before measurement')
                command=[sys.executable,'-m','aims_mpcc.worker_benchmark',
                    '--reference',args.reference,'--vehicle-config',args.vehicle_config,
                    '--backend',args.backend,'--horizon',str(args.horizon),
                    '--seconds',str(args.seconds),'--period','.05',
                    '--artifact-directory',args.artifact_directory,
                    '--log-directory',str(directory/'worker'),
                    '--output',str(directory/'benchmark.json')]
                if args.sample_lap:command.append('--sample-lap')
                entry['benchmark_command']=command
                benchmark=spawn(command,'benchmark')
                while benchmark.poll() is None:
                    if replay is not None and replay.poll() is not None:
                        # Benchmark still drains its current reply; report the
                        # exact unshared tail rather than silently dropping it.
                        entry.setdefault('replay_ended_wall',time.time())
                        entry.setdefault('replay_ended_monotonic',time.monotonic())
                    time.sleep(.2)
                entry['benchmark_exit']=benchmark.returncode
                if replay is not None:
                    drain_started=time.monotonic();entry['replay_drain_limit_s']=65.
                    if replay.poll() is None:
                        # The runner writes its summary on natural max-seconds
                        # completion. SIGINT would skip that final audit. This
                        # drain is excluded from the benchmark request window.
                        try:replay.wait(timeout=65.)
                        except subprocess.TimeoutExpired:
                            entry['replay_drain_timeout']=True
                            if replay.poll() is None:
                                try:
                                    os.killpg(replay.pid,signal.SIGINT)
                                    entry['intentional_replay_shutdown']=True
                                except ProcessLookupError:pass
                            try:replay.wait(timeout=15.)
                            except subprocess.TimeoutExpired:entry['replay_shutdown_timeout']=True
                    entry['replay_exit']=replay.poll()
                    entry['replay_drain_s']=time.monotonic()-drain_started
                    entry['replay_completed_normally']=bool(entry['replay_exit']==0 and not entry.get('replay_drain_timeout'))
                entry['status']='recorded'
            except Exception as exc:
                entry.update(status='error',error=repr(exc))
            finally:
                try:cleanup(processes)
                except Exception as exc:
                    entry.update(status='cleanup_failed',cleanup_error=repr(exc))
                for log in logs:log.close()
                try:
                    if (directory/'benchmark.json').exists():
                        case=json.loads((directory/'benchmark.json').read_text())['cases'][0]
                        entry['benchmark_summary']=case['summary']
                        if condition in ('shared','stress'):
                            entry['load_coverage']=load_coverage(case['requests'],
                                read_events(directory/'replay/events.jsonl'),entry.get('replay_ended_monotonic'))
                            if not entry['load_coverage']['valid'] and entry['status']=='recorded':
                                entry['status']='invalid_load_window'
                    if (directory/'replay/summary.json').exists():
                        entry['replay_summary']=json.loads((directory/'replay/summary.json').read_text())
                    if condition in ('shared','stress'):
                        entry['replay_errors']=replay_errors(entry)
                        if entry['replay_errors'] and entry['status']=='recorded':entry['status']='invalid_replay'
                except (OSError,ValueError,KeyError,TypeError,IndexError) as exc:
                    entry['artifact_error']=repr(exc)
                    if entry['status']=='recorded':entry['status']='artifact_failed'
                if condition in ('shared','stress'):
                    entry['load_qualified']=bool(entry['status']=='recorded' and entry.get('benchmark_exit')==0
                        and entry.get('load_coverage',{}).get('valid') and not entry.get('replay_errors'))
                entry['ended_wall']=time.time()
                record['results'].append(entry)
                (args.output/'matrix.json').write_text(json.dumps(record,indent=2)+'\n')
            print(json.dumps(entry),flush=True)
            if entry['status']!='recorded' or entry.get('benchmark_exit')!=0:
                raise SystemExit('Matrix stopped; inspect the retained failed case')


if __name__=='__main__':main()
