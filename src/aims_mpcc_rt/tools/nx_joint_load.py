#!/usr/bin/env python3
"""Measure isolated controller command routing with genuine estimation CPU load.

Sensor replay is CPU load only. Controller feedback is an independent plant;
this experiment does not establish real vehicle tracking or localization quality.
Source the ROS, vehicle, and localization overlays before running.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(root, bundle, inputs=()):
    files=list((root/'src/controller/aims_mpcc').rglob('*.py'))
    files+=list((root/'src/aims_mpcc_rt').rglob('*.cpp'))+list((root/'src/aims_mpcc_rt').rglob('*.hpp'))
    files+=list((root/'src/aims_mpcc_rt/tools').glob('*.py'))
    files+=[p for p in bundle.rglob('*') if p.is_file() and p.suffix in ('.c','.h','.so','.json','.yaml','.csv')]
    manifest=json.loads((bundle/'native_manifest.json').read_text())
    files += [Path(name) for name in manifest['dependencies']]
    files += [root/'src/controller/tools/nx_load_matrix.py',root/'src/aims_mpcc_rt/tools/nx_joint_load.py']
    files += [p.resolve() for p in inputs if p is not None and p.is_file()]
    for directory in inputs:
        if directory is not None and directory.is_dir():
            files += [p for p in directory.rglob('*') if p.is_file() and p.suffix in ('.so','.json','.c','.h')]
    from ament_index_python.packages import get_package_prefix
    for package, executable in [('aims_mpcc_rt','mpcc_rt_node'),('ackermann_mux','joystick_control_v2'),('vesc_ackermann','ackermann_to_vesc_node')]:
        files.append(Path(get_package_prefix(package))/'lib'/package/executable)
    return {str(p):digest(p) for p in sorted(set(files))}


def timing_pass(timing, frequency, budget):
    """20/40 Hz gates include rejected attempts and publication scheduling."""
    count=timing.get('requests',0)
    complete=timing.get('complete',{});pub=timing.get('publish_gap',{})
    failures=timing.get('failures',count)+timing.get('handover_rejected',0)
    skipped=timing.get('skipped_pending_slots',0)
    scheduled=timing.get('eligible_slots',count+skipped)
    return bool(count and complete.get('p95') is not None and
        complete['p95']<=.8*budget and complete['p99']<=budget and
        timing.get('late',count)/count<=.001 and failures/count<=.001 and
        timing.get('inflight',timing.get('unfinished',0))==0 and timing.get('accounted',count)==count and
        timing.get('max_consecutive_failures',count)<3 and timing.get('log_integrity_pass',False) and
        scheduled==count+skipped and timing.get('measurement_s',0)>0 and
        scheduled>=.99*frequency*timing['measurement_s']-1 and
        timing.get('publications',0)>=.99*50*timing['measurement_s']-1 and
        pub.get('p99') is not None and pub['p99']<=.03 and pub['max']<=.06)


def run(args):
    root=args.root.resolve();bundle=args.bundle.resolve();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(root/'src/controller/tools'))
    from nx_load_matrix import enable_subreaper,cleanup,read_events,load_coverage,replay_errors
    enable_subreaper()
    env=dict(os.environ)
    env['ROS_DOMAIN_ID']=str(args.domain);env['ROS_LOCALHOST_ONLY']='1'
    env['PYTHONPATH']=str(root/'src/controller')+os.pathsep+env.get('PYTHONPATH','')
    env['OMP_NUM_THREADS']='1';env['OPENBLAS_NUM_THREADS']='1'
    evidence=dict(arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        source_commit=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),
        source_before=snapshot(root,bundle,[args.vesc,args.replay_runner,args.seed,args.legacy_artifact]),environment={k:env.get(k) for k in (
          'ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','ACADOS_SOURCE_DIR','LD_LIBRARY_PATH')},
        scope='synthetic closed loop and estimation CPU load; excludes physical driving qualification')
    hardware={}
    for name,command in [('uname',['uname','-a']),('power',['nvpmodel','-q']),('clocks',['jetson_clocks','--show'])]:
        try:hardware[name]=subprocess.check_output(command,stderr=subprocess.STDOUT,text=True,timeout=5)
        except (OSError,subprocess.SubprocessError) as error:hardware[name]=str(error)
    evidence['hardware']=hardware
    (out/'manifest.json').write_text(json.dumps(evidence,indent=2)+'\n')
    processes=[];handles=[];replay=None
    def spawn(command,name,environment):
        handle=(out/(name+'.log')).open('w');handles.append(handle)
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
        processes.append(process);return process
    try:
        if shutil.which('tegrastats'):spawn(['tegrastats','--interval','1000'],'tegrastats',env)
        if args.shared:
            load_env=dict(env);load_env.pop('OMP_NUM_THREADS',None)
            command=['/usr/bin/python3',str(args.replay_runner),str(args.bag),str(out/'replay'),
                '--map',str(args.map),'--seed',str(args.seed),'--use-initializer-cli',
                '--max-seconds',str(args.seconds+60.)]
            evidence['load_command']=command
            replay=spawn(command,'replay',load_env)
            deadline=time.monotonic()+60.
            while time.monotonic()<deadline:
                if replay.poll() is not None:raise RuntimeError('replay exited before trusted anchor')
                events=read_events(out/'replay/events.jsonl')
                if any(e.get('kind')=='native_timing' for e in events) and any(
                    e.get('kind')=='anchor' and e.get('values',{}).get('anchor_committed')=='true' for e in events):break
                time.sleep(.2)
            else:raise RuntimeError('no native registration and trusted anchor within 60 seconds')
        command=[sys.executable,str(root/'src/aims_mpcc_rt/tools/acceptance.py'),
            '--bundle',str(bundle),'--vesc',str(args.vesc),'--output',str(out/'controller'),
            '--seconds',str(args.seconds),'--frequency',str(args.frequency),'--budget',str(args.budget),
            '--implementation',args.implementation,'--backend',args.backend,'--handover-delay',str(args.handover_delay)]
        if args.legacy_artifact:command+=['--legacy-artifact',str(args.legacy_artifact)]
        evidence['controller_command']=command
        evidence['started']=time.monotonic();controller=spawn(command,'controller',env)
        while controller.poll() is None:
            if time.monotonic()-evidence['started']>args.seconds+45.:
                raise RuntimeError('controller exceeded bounded experiment duration')
            if replay is not None and replay.poll() is not None:
                evidence.setdefault('replay_ended_monotonic',time.monotonic())
                raise RuntimeError('estimation load ended during controller measurement')
            time.sleep(.02)
        evidence['ended']=time.monotonic();evidence['controller_exit']=controller.returncode
        if replay is not None:
            # Allow the replay's bounded normal shutdown to produce its TF audit.
            until=time.monotonic()+90.
            while replay.poll() is None and time.monotonic()<until:time.sleep(.2)
            evidence['replay_drain_timeout']=replay.poll() is None
            evidence['replay_exit']=replay.poll()
            summary=out/'replay/summary.json'
            if summary.exists():evidence['replay_summary']=json.loads(summary.read_text())
            evidence['replay_errors']=replay_errors(evidence)
        report=out/'controller/report.json'
        if report.exists():
            evidence['controller_report']=json.loads(report.read_text())
            if args.shared:
                boundaries=evidence['controller_report']
                window=[dict(submitted_at=boundaries.get('measurement_start') or evidence['started'],
                    completed_at=boundaries.get('measurement_end') or evidence['ended'],full_request_s=0)]
                evidence['load_coverage']=load_coverage(window,read_events(out/'replay/events.jsonl'),evidence.get('replay_ended_monotonic'))
    except Exception as error:evidence['error']=repr(error)
    finally:
        try:cleanup(processes)
        except Exception as error:evidence['cleanup_error']=repr(error)
        for handle in handles:handle.close()
        evidence['source_after']=snapshot(root,bundle,[args.vesc,args.replay_runner,args.seed,args.legacy_artifact])
        evidence['source_changed']=evidence['source_before']!=evidence['source_after']
        report=evidence.get('controller_report',{})
        evidence['timing_pass']=timing_pass(report.get('timing',{}),args.frequency,args.budget)
        evidence['qualified']=bool(not evidence.get('error') and not evidence.get('cleanup_error') and
            not evidence['source_changed'] and evidence.get('controller_exit')==0 and report.get('overall_pass') and
            evidence['timing_pass'] and (not args.shared or (evidence.get('load_coverage',{}).get('valid') and not evidence.get('replay_errors'))))
        (out/'manifest.json').write_text(json.dumps(evidence,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:evidence.get(k) for k in ('qualified','timing_pass','controller_exit','error','cleanup_error','load_coverage')},indent=2))
    return 0 if evidence['qualified'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','bundle','output','vesc'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--seconds',type=float,default=180.);p.add_argument('--frequency',type=float,default=20.)
    p.add_argument('--budget',type=float,default=.05);p.add_argument('--domain',type=int,default=224)
    p.add_argument('--handover-delay',type=float,default=.02)
    p.add_argument('--implementation',choices=['acados_cpp','legacy'],default='acados_cpp')
    p.add_argument('--backend',choices=['ipopt','qp','acados'],default='ipopt')
    p.add_argument('--legacy-artifact',type=Path)
    p.add_argument('--shared',action='store_true')
    for name in ('replay-runner','bag','map','seed'):p.add_argument('--'+name,type=Path)
    args=p.parse_args()
    if not 0<args.seconds<=300. or not 0<args.frequency<=50. or not 0<args.budget or not 0<args.handover_delay<=.1:
        p.error('seconds must be in (0,300], frequency in (0,50], positive budget, handover-delay in (0,.1]')
    if args.shared and any(getattr(args,k) is None for k in ('replay_runner','bag','map','seed')):
        p.error('shared requires replay-runner, bag, map and seed')
    raise SystemExit(run(args))
