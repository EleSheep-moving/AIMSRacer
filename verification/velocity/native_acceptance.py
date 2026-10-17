#!/usr/bin/env python3
"""Native/Python analytic-truth parity under original delayed receipt ordering."""
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np
from observer import Observer
from synthetic import scenario
from compare import compare

def run(executable,output):
    output.mkdir(parents=True,exist_ok=False);report={}
    for name,f,kind in [('sine05',.5,'sine'),('sine1',1.,'sine'),('sine2',2.,'sine'),('sine3',3.,'sine'),('turn',1.,'turn'),('rest',1.,'rest'),('bias',1.,'bias')]:
        _,_,_,events=scenario(f,kind);o=Observer();rows=[];commands=[]
        for _,_,event,t,a,b in events:
            if event=='imu':accepted=o.add_imu(t,a,b);values=[*a,*b]
            else:accepted=o.add_lio(t,a,b);values=[*a,*b]
            e=o.latest();commands.append(event.upper()+' '+str(t)+' '+' '.join(format(float(v),'.17g') for v in values)+'\n')
            rows.append([t,int(accepted),e['stamp_ns'],int(e['valid']),e['status'],e['anchor_stamp_ns'],e['vx'],e['variance'],*e['position_world'],*e['velocity_world'],*e['effective_bias'],e['rejected_lio'],o.history_size,e['lio_reinitializations']])
        import csv
        with (output/f'{name}-python.csv').open('w') as file:
            w=csv.writer(file);w.writerow('input_source_ns accepted stamp_ns valid status anchor_stamp_ns vx variance px py pz vwx vwy vwz bix biy biz rejected_lio history_size lio_reinitializations'.split());w.writerows(rows)
        source=output/f'{name}.txt';source.write_text(''.join(commands))
        with (output/f'{name}-native.csv').open('w') as file:subprocess.run([str(executable),str(source)],stdout=file,check=True)
        report[name]=compare(output/f'{name}-python.csv',output/f'{name}-native.csv')
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v['max_absolute_difference']['vx'] for k,v in report.items()}))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.executable,a.output)
