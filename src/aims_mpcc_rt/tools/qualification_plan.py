#!/usr/bin/env python3
"""Prepare qualification commands; execution requires an explicit frozen manifest.

Run --prepare first. After the final build/bundles are stable, run --freeze,
then --execute --group protocol/acceptance/matched/nx. Every group runs locally
in the sourced target overlay; generate a separate plan on the NX host.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def fingerprint(paths):
    return {str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in sorted(set(paths))}


def verify_frozen(expected):
    changed=[name for name,digest in expected.items() if not Path(name).is_file()
             or hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest]
    if changed:raise RuntimeError('Frozen source/binary/bundle changed: '+', '.join(changed[:8]))


def build_plan(root,bundles,out,vesc,*,load=None):
    tools=root/'src/aims_mpcc_rt/tools'
    groups=dict(protocol=[],acceptance=[],matched=[],nx=[])
    for lead in (0.,.02,.05,.1):
        for frequency in (10.,20.,40.):
            name=f'protocol-lead{int(lead*1000)}-hz{int(frequency)}'
            command=[sys.executable,str(tools/'protocol_probe.py'),'--scenario','lifecycle',
                '--bundle',str(bundles/'qual-v2-circle-v05-n10'),'--output',str(out/name),
                '--handover-delay',str(lead),'--frequency',str(frequency),'--seconds','6']
            groups['protocol'].append(dict(name=name,command=command,timeout_s=60,
                environment=dict(ROS_DOMAIN_ID='225',ROS_LOCALHOST_ONLY='1')))
    for speed in ('v05','v10'):
        name='default20-lead20-'+speed
        command=[sys.executable,str(tools/'acceptance.py'),'--bundle',str(bundles/f'qual-v2-circle-{speed}-n10'),
            '--vesc',str(vesc),'--output',str(out/name),'--seconds','30','--frequency','20',
            '--handover-delay','.02','--prefix','/mpcc_qualification']
        groups['acceptance'].append(dict(name=name,command=command,timeout_s=90,
            environment=dict(ROS_DOMAIN_ID='231',ROS_LOCALHOST_ONLY='1')))
    comparison=dict(cases=[dict(name=shape+'-'+speed,
        v1_bundle=str(bundles/f'qual-v1-{shape}-{speed}-n10'),
        v2_bundle=str(bundles/f'qual-v2-{shape}-{speed}-n10'))
        for shape in ('circle','route') for speed in ('v05','v10')])
    groups['matched'].append(dict(name='matched90',timeout_s=3660,environment=dict(ROS_LOCALHOST_ONLY='1'),
        command=[sys.executable,str(tools/'compare_output_profiles.py'),'--root',str(root),
          '--manifest',str(out/'compare_cases.json'),'--vesc',str(vesc),'--output',str(out/'matched90'),
          '--seconds','90','--domain','210','--lag-multipliers','.75','1','1.25']))
    if load:
        for repeat in range(1,4):
            name=f'nx-shared180-{repeat}'
            command=[sys.executable,str(tools/'nx_joint_load.py'),'--root',str(root),
                '--bundle',str(bundles/'qual-v2-circle-v10-n10'),'--vesc',str(vesc),
                '--output',str(out/name),'--seconds','180','--frequency','20','--budget','.05',
                '--handover-delay','.02','--domain','224','--shared']
            for key in ('replay_runner','bag','map','seed'):command+=['--'+key.replace('_','-'),str(load[key])]
            groups['nx'].append(dict(name=name,command=command,timeout_s=360,environment=dict(ROS_LOCALHOST_ONLY='1')))
    return dict(root=str(root),bundle_directory=str(bundles),output=str(out),vesc=str(vesc),
        load={k:str(v) for k,v in (load or {}).items()},groups=groups,comparison=comparison,
        scope='private synthetic feedback, matched profiles and genuine estimation CPU load; no vehicle qualification',
        replay_policy='single bounded replay, confirmed NX bag duration 410.41990888 s; measured load coverage remains mandatory')


def freeze_inputs(plan):
    from ament_index_python.packages import get_package_prefix
    root=Path(plan['root']);bundles=Path(plan['bundle_directory'])
    paths=[p for directory in (root/'src/aims_mpcc_rt',root/'src/controller') for p in directory.rglob('*')
           if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.yaml')]
    for directory in bundles.glob('qual-*-n10'):
        paths += [p for p in directory.rglob('*') if p.is_file() and p.suffix in ('.py','.c','.h','.json','.yaml','.csv','.so')]
        manifest=directory/'native_manifest.json'
        if manifest.exists():paths += [Path(p) for p in json.loads(manifest.read_text())['dependencies']]
    from nx_joint_load import installed_executables
    for package,binary in installed_executables(bool(plan.get('load'))):
        paths.append(Path(get_package_prefix(package))/'lib'/package/binary)
    paths.append(Path(plan['vesc']))
    for key,name in plan.get('load',{}).items():
        path=Path(name)
        paths.append(path/'metadata.yaml' if key=='bag' else path)
    return fingerprint(paths)


def run_case(command,environment,log,timeout,root,*,cleanup_grace=3.):
    sys.path.insert(0,str(root/'src/controller/tools'))
    from nx_load_matrix import enable_subreaper,cleanup
    enable_subreaper()
    process=subprocess.Popen(command,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        try:return dict(returncode=process.wait(timeout=timeout))
        except subprocess.TimeoutExpired:return dict(returncode=None,error='bounded case timeout')
    finally:cleanup([process],grace=cleanup_grace)


def assess_case(group,job,root,result):
    qualified=result.get('returncode')==0
    if group=='acceptance':
        sys.path.insert(0,str(root/'src/aims_mpcc_rt/tools'))
        from nx_joint_load import timing_pass
        output=Path(job['command'][job['command'].index('--output')+1])
        file=output/'report.json'
        report=json.loads(file.read_text()) if file.exists() else {}
        tracking=bool(report.get('overall_pass'))
        timing=timing_pass(report.get('timing',{}),20.,.05)
        return {**result,'tracking_pass':tracking,'timing_pass':timing,'qualified':qualified and tracking and timing}
    return {**result,'qualified':qualified}


def execute(plan,frozen,group):
    out=Path(plan['output']);results=[]
    env=dict(os.environ)
    env.pop('AIMS_MPCC_CAPTURE_REQUEST',None);env.pop('AIMS_MPCC_CAPTURE_TAKEOVER',None)
    for job in plan['groups'][group]:
        verify_frozen(frozen)
        child_env={**env,**job['environment']}
        begin=time.monotonic()
        with (out/(job['name']+'.log')).open('w') as log:
            result=dict(name=job['name'],**run_case(job['command'],child_env,log,job['timeout_s'],Path(plan['root'])))
        result=assess_case(group,job,Path(plan['root']),result)
        try:verify_frozen(frozen)
        except RuntimeError as exc:result.update(qualified=False,error=str(exc),source_changed=True)
        result['elapsed_s']=time.monotonic()-begin;results.append(result)
        (out/(group+'-results.json')).write_text(json.dumps(dict(cases=results),indent=2)+'\n')
        print(json.dumps(result),flush=True)
        if result.get('source_changed'):break
    return 0 if results and all(r['qualified'] for r in results) else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--bundle-directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--vesc',type=Path,required=True)
    parser.add_argument('--group',choices=['protocol','acceptance','matched','nx'],default='protocol')
    action=parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--prepare',action='store_true');action.add_argument('--freeze',action='store_true');action.add_argument('--execute',action='store_true')
    for name in ('replay-runner','bag','map','seed'):parser.add_argument('--'+name,type=Path)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    load=None
    if any(getattr(args,key) for key in ('replay_runner','bag','map','seed')):
        if not all(getattr(args,key) for key in ('replay_runner','bag','map','seed')):parser.error('load needs replay-runner, bag, map and seed together')
        load={key:getattr(args,key).resolve() for key in ('replay_runner','bag','map','seed')}
    plan=build_plan(args.root.resolve(),args.bundle_directory.resolve(),out,args.vesc.resolve(),load=load)
    if args.prepare:
        (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        (out/'compare_cases.json').write_text(json.dumps(plan['comparison'],indent=2)+'\n')
    elif args.freeze:
        (out/'frozen.json').write_text(json.dumps(dict(files=freeze_inputs(plan)),indent=2)+'\n')
    else:
        saved=json.loads((out/'plan.json').read_text())
        if plan!=saved:parser.error('execution arguments differ from prepared plan; prepare a new output directory')
        raise SystemExit(execute(saved,json.loads((out/'frozen.json').read_text())['files'],args.group))
