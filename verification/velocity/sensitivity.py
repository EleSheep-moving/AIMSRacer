#!/usr/bin/env python3
"""Fixed-config timestamp/geometry sensitivity; differences are not truth errors."""
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np

def main(a):
    a.output.mkdir(parents=True,exist_ok=False)
    commands={'lio_minus20ms':['--lio-shift-ns','-20000000'],'lio_plus20ms':['--lio-shift-ns','20000000'],
              'lever_y_minus2cm':['--lever-y','-.02'],'lever_y_plus2cm':['--lever-y','.02'],
              'lever_z_minus2cm':['--lever-z','.01'],'lever_z_plus2cm':['--lever-z','.05'],
              'mount_roll_minus1deg':['--mount-roll-deg','-1'],'mount_roll_plus1deg':['--mount-roll-deg','1']}
    baseline=np.genfromtxt(a.baseline,delimiter=',',names=True,dtype=None,encoding='utf8');mask=(baseline['event']=='IMU')&(baseline['valid']==1)&(baseline['accepted']==1)
    origin=1792002365474876930;tt=(baseline['stamp_ns']-origin)*1e-9;mask&=(tt>=41.926416916)&(tt<=220.431558514);base=baseline[mask];report={}
    for name,args in commands.items():
        output=a.output/f'{name}.csv'
        with output.open('w') as f:subprocess.run([str(a.executable),str(a.events),*args],stdout=f,check=True)
        data=np.genfromtxt(output,delimiter=',',names=True,dtype=None,encoding='utf8');data=data[mask]
        assert np.array_equal(base['stamp_ns'],data['stamp_ns']);ok=(data['valid']==1)&(data['accepted']==1);delta=data['vx'][ok]-base['vx'][ok]
        report[name]={'difference_rms_mps':float(np.sqrt(np.mean(delta**2))),'difference_abs_p95_mps':float(np.percentile(abs(delta),95)),'difference_abs_max_mps':float(max(abs(delta))),'valid_n':int(sum(ok)),'nominal_n':len(base),'coverage':float(np.mean(ok)),'lio_reinitializations':int(data['lio_reinitializations'][-1]),'note':'difference vs nominal, not independent accuracy'}
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--executable',type=Path,required=True);p.add_argument('--events',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
