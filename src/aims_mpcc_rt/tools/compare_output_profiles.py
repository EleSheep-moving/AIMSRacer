#!/usr/bin/env python3
"""Matched independent-plant comparison of explicit command profiles.

Recorded estimator data is not plant feedback. Every subprocess command and
failure remains in the evidence; an incomplete baseline is not a passing ratio.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

def compare_reports(baseline,candidate):
    if not baseline.get('overall_pass',False):
        return dict(status='unavailable',relative_pass=False,reason='baseline did not pass')
    if not candidate.get('overall_pass',False):
        return dict(status='available',relative_pass=False,reason='candidate did not pass')
    contour_limit=baseline['contour_p95_m']*1.1+.01
    heading_limit=baseline['heading_p95_rad']*1.1+math.radians(.5)
    passed=candidate['contour_p95_m']<=contour_limit and candidate['heading_p95_rad']<=heading_limit
    return dict(status='available',relative_pass=passed,contour_limit_m=contour_limit,
                heading_limit_rad=heading_limit)

def validate_pair(first,second):
    a=json.loads((first/'config.json').read_text());b=json.loads((second/'config.json').read_text())
    if a.get('command_profile','legacy_bounded_v1')!='legacy_bounded_v1' or b.get('command_profile')!='rate_bounded_v2':
        raise ValueError('comparison requires explicit v1 and v2 bundles')
    for c in (a,b):
        c.pop('command_profile',None)
        c['steering_acceleration_scale']=c.get('steering_acceleration_scale') or c['steer_acceleration']
    if a!=b:raise ValueError('comparison vehicle/weight/solver configurations differ beyond command profile')
    for name in ('reference.json',):
        if (first/name).read_bytes()!=(second/name).read_bytes():raise ValueError('comparison references differ')
    ma=json.loads((first/'manifest.json').read_text());mb=json.loads((second/'manifest.json').read_text())
    if (ma['horizon'],ma['dt'])!=(mb['horizon'],mb['dt']):raise ValueError('comparison prediction meshes differ')

def output_metrics(directory):
    answer={};file=directory/'trajectory.csv'
    if file.exists():
        rows=list(csv.DictReader(file.open()));previous=0.;reversals=0
        for a,b in zip(rows,rows[1:]):
            delta=float(b['steering_command'])-float(a['steering_command'])
            if abs(delta)>.001:
                if previous and delta*previous<0.:reversals+=1
                previous=delta
        answer['steering_direction_reversals']=reversals
    file=directory/'controller/runtime.csv'
    if file.exists():
        rows=[r for r in csv.DictReader(file.open()) if r['event']=='publish' and int(r['sequence'])>0]
        for desired,actual,name in [('requested_acceleration','emitted_acceleration','acceleration_command_error'),
                                    ('requested_steering_rate','emitted_steering_rate','steering_rate_command_error')]:
            values=[abs(float(r[desired])-float(r[actual])) for r in rows if r.get(desired) and r.get(actual)]
            if values:answer[name]=dict(mean=sum(values)/len(values),maximum=max(values),samples=len(values))
        if any(r.get('limiter_active') for r in rows):
            answer['limiter_active_samples']=sum(float(r.get('limiter_active') or 0.)!=0. for r in rows)
    return answer

def run(args):
    root=args.root.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    cases=json.loads(args.manifest.read_text())['cases'];results=[]
    manifest=dict(cases=cases,commands=[],scope='actual private controller/selector/converter with independent lagged plant')
    manifest['source_hashes']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
      for p in list((root/'src/aims_mpcc_rt').rglob('*.cpp'))+list((root/'src/aims_mpcc_rt').rglob('*.hpp'))}
    env=dict(os.environ,ROS_LOCALHOST_ONLY='1');tool=root/'src/aims_mpcc_rt/tools/acceptance.py'
    for index,case in enumerate(cases):
        pair={v:Path(case[v+'_bundle']).resolve() for v in ('v1','v2')};validate_pair(pair['v1'],pair['v2'])
        for lag in args.lag_multipliers:
            reports={}
            for version,bundle in pair.items():
                directory=out/(case['name']+'-'+version+'-lag'+str(lag).replace('.','p'))
                command=[sys.executable,str(tool),'--bundle',str(bundle),'--vesc',str(args.vesc.resolve()),
                  '--output',str(directory),'--seconds',str(args.seconds),'--frequency','20','--budget','.05',
                  '--handover-delay','.02','--speed-tau',str(.2*lag),'--steer-tau',str(.15*lag)]
                env['ROS_DOMAIN_ID']=str(args.domain+index)
                manifest['commands'].append(dict(case=case['name'],lag=lag,version=version,command=command,domain=env['ROS_DOMAIN_ID']))
                (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
                with (out/(directory.name+'.log')).open('w') as log:
                    completed=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,
                                             timeout=args.seconds+60.)
                file=directory/'report.json'
                report=json.loads(file.read_text()) if file.exists() else dict(overall_pass=False,error='report absent')
                reports[version]=report
                results.append(dict(case=case['name'],lag=lag,version=version,returncode=completed.returncode,
                                    directory=str(directory),report=report,output_metrics=output_metrics(directory)))
            comparison=compare_reports(reports['v1'],reports['v2'])
            for row in results[-2:]:row['comparison']=comparison
            (out/'summary.json').write_text(json.dumps(dict(runs=results),indent=2)+'\n')
    return all(row['report'].get('overall_pass',False) and row['comparison']['relative_pass'] for row in results)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--vesc',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90.);parser.add_argument('--domain',type=int,default=210)
    parser.add_argument('--lag-multipliers',type=float,nargs='+',default=[.75,1.,1.25])
    options=parser.parse_args()
    if options.seconds<=0 or any(v<=0 for v in options.lag_multipliers):parser.error('positive duration and lag required')
    raise SystemExit(0 if run(options) else 1)
