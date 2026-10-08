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


def owned_children():
    result=[]
    def visit(pid):
        try:values=(Path('/proc')/str(pid)/'task'/str(pid)/'children').read_text().split()
        except OSError:return
        for value in values:
            child=int(value);result.append(child);visit(child)
    visit(os.getpid())
    return result


def cleanup(processes,grace=3.):
    """Terminate and reap this driver's children, including adopted orphans."""
    for sig,delay in ((signal.SIGINT,grace),(signal.SIGTERM,.5),(signal.SIGKILL,.5)):
        for pid in owned_children():
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        until=time.monotonic()+delay
        while time.monotonic()<until:
            for process in processes:process.poll()
            # Popen children are reaped first so their return codes stay intact.
            protected={process.pid for process in processes if process.poll() is None}
            for pid in owned_children():
                if pid in protected:continue
                try:os.waitpid(pid,os.WNOHANG)
                except ChildProcessError:pass
            if not owned_children():return
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
                    if replay.poll() is None:
                        os.killpg(replay.pid,signal.SIGINT)
                        entry['intentional_replay_shutdown']=True
                    try:replay.wait(timeout=15.)
                    except subprocess.TimeoutExpired:
                        entry['replay_shutdown_timeout']=True
                    entry['replay_exit']=replay.poll()
                entry['status']='recorded'
            except Exception as exc:
                entry.update(status='error',error=repr(exc))
            finally:
                cleanup(processes)
                for log in logs:log.close()
                if (directory/'benchmark.json').exists():
                    case=json.loads((directory/'benchmark.json').read_text())['cases'][0]
                    entry['benchmark_summary']=case['summary']
                    if condition in ('shared','stress'):
                        entry['load_coverage']=load_coverage(case['requests'],
                            read_events(directory/'replay/events.jsonl'),entry.get('replay_ended_monotonic'))
                        if not entry['load_coverage']['valid']:entry['status']='invalid_load_window'
                if (directory/'replay/summary.json').exists():
                    entry['replay_summary']=json.loads((directory/'replay/summary.json').read_text())
                entry['ended_wall']=time.time()
                record['results'].append(entry)
                (args.output/'matrix.json').write_text(json.dumps(record,indent=2)+'\n')
            print(json.dumps(entry),flush=True)
            if entry['status']!='recorded' or entry.get('benchmark_exit')!=0:
                raise SystemExit('Matrix stopped; inspect the retained failed case')


if __name__=='__main__':main()
