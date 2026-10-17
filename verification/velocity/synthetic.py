#!/usr/bin/env python3
"""Fixed-seed physical synthetic truth. Selection never reads field/wheel arrays."""
import argparse
import json
from pathlib import Path
import hashlib
import time
import numpy as np
from scipy.spatial.transform import Rotation
from observer import Observer, Config


def scenario(frequency=1., kind='sine', duration=10., noise=True):
    t=np.arange(0,duration+.0001,.005);ns=10_000_000_000+np.rint(t*1e9).astype(np.int64)
    w=2*np.pi*frequency
    if kind=='turn':
        yaw=.8*t;r=.8
        position=np.c_[3*np.sin(yaw)/r+.3*(np.cos(yaw)-1)/r,
                       3*(1-np.cos(yaw))/r+.3*np.sin(yaw)/r,.025*(1-np.cos(2*t))]
        velocity=np.c_[3*np.cos(yaw)-.3*np.sin(yaw),3*np.sin(yaw)+.3*np.cos(yaw),.05*np.sin(2*t)]
        acceleration=np.c_[-r*(3*np.sin(yaw)+.3*np.cos(yaw)),r*(3*np.cos(yaw)-.3*np.sin(yaw)),.1*np.cos(2*t)]
        roll=.05*np.sin(1.3*t);pitch=.03*np.sin(.9*t)
        rd=.065*np.cos(1.3*t);pd=.027*np.cos(.9*t);yd=np.full(len(t),r)
    elif kind=='rest':
        position=np.zeros((len(t),3));velocity=position.copy();acceleration=position.copy()
        yaw=roll=pitch=rd=pd=yd=np.zeros(len(t))
    else:
        position=np.c_[3*t+(1-np.cos(w*t))/w,.2*(1-np.cos(.7*t))/.7,np.zeros(len(t))]
        velocity=np.c_[3+np.sin(w*t),.2*np.sin(.7*t),np.zeros(len(t))]
        acceleration=np.c_[w*np.cos(w*t),.14*np.cos(.7*t),np.zeros(len(t))]
        yaw=roll=pitch=rd=pd=yd=np.zeros(len(t))
    rotation=Rotation.from_euler('xyz',np.c_[roll,pitch,yaw]);q=rotation.as_quat()
    gyro=np.c_[rd-yd*np.sin(pitch),pd*np.cos(roll)+yd*np.sin(roll)*np.cos(pitch),
               -pd*np.sin(roll)+yd*np.cos(roll)*np.cos(pitch)]
    force=rotation.inv().apply(acceleration+np.array([0,0,9.80665]))
    truth=(rotation.inv().apply(velocity)-np.cross(gyro,np.array([.3,0,.03])))[:,0]
    if kind=='bias':force[t>=2]+=np.array([.05,-.03,.02])
    rng=np.random.default_rng(91345)
    if noise:
        force+=rng.normal(size=force.shape)*np.array([.045,.013,.063]);gyro+=rng.normal(0,.002,gyro.shape)
    events=[]
    for i,stamp in enumerate(ns):
        events.append((int(stamp),1,'imu',int(stamp),force[i]/9.80665,gyro[i]))
        if i%20==0:
            lag=[.04,.09,.15][(i//20)%3]
            observed=position[i]+(rng.normal(0,.01,3) if noise else 0)
            events.append((int(stamp+round(lag*1e9)),0,'lio',int(stamp),observed,q[i]))
    events.sort(key=lambda e:e[:2])
    return t,ns,truth,events


def run(config,case):
    t,ns,truth,events=case;o=Observer(config);rows=[]
    for _,_,kind,stamp,x,y in events:
        if kind=='imu':
            assert o.add_imu(stamp,x,y)
            e=o.latest()
            if e['valid']:rows.append((stamp,e['vx']))
        else:assert o.add_lio(stamp,x,y)
    stamps=np.array([s for s,v in rows]);idx=np.searchsorted(ns,stamps)
    assert np.array_equal(ns[idx],stamps)
    m=t[idx]>=3.;tt=t[idx][m];values=np.array([v for s,v in rows])[m];target=truth[idx][m]
    return tt,values,target,o


def sweep(output):
    frequencies=[.5,1.,2.,3.]
    cases=[scenario(f) for f in frequencies]+[scenario(kind='turn'),scenario(kind='rest'),scenario(kind='bias')]
    records=[]
    for sigma in [.01,.03,.10]:
        for psd in [1e-6,1e-5,1e-4]:
            config=Config(position_std=sigma,bias_psd=psd);results=[]
            for i,case in enumerate(cases):
                t,y,truth,o=run(config,case)
                row=dict(kind='sine' if i<4 else ['turn','rest','bias'][i-4],rmse=float(np.sqrt(np.mean((y-truth)**2))),n=len(y))
                if i<4:
                    f=frequencies[i];A=np.c_[np.sin(2*np.pi*f*t),np.cos(2*np.pi*f*t),np.ones(len(t))]
                    b=np.linalg.lstsq(A,y,rcond=None)[0]
                    row.update(frequency_hz=f,gain=float(np.hypot(b[0],b[1])),lag_ms=float(-np.arctan2(b[1],b[0])/(2*np.pi*f)*1000))
                results.append(row)
            passed=all(r['rmse']<=.1 and (r['kind']!='sine' or (abs(r['gain']-1)<=.1 and abs(r['lag_ms'])<=15)) for r in results)
            entry=dict(position_std=sigma,bias_psd=psd,passed=passed,mean_rmse=float(np.mean([r['rmse'] for r in results])),cases=results)
            records.append(entry);print(json.dumps({k:v for k,v in entry.items() if k!='cases'}),flush=True)
    eligible=[r for r in records if r['passed']]
    if not eligible:raise RuntimeError('no candidate meets synthetic acceptance; do not port')
    selected=min(eligible,key=lambda r:(r['mean_rmse'],-r['position_std'],r['bias_psd']))
    report=dict(records=records,selected=selected,truth='analytic 3D IMU-origin motion; rear velocity exact rigid-body transfer',
                noise='fixed seed 91345; force sigma [.045,.013,.063]m/s2, gyro .002rad/s, LIO position .01m',
                timing='200Hz IMU,10Hz LIO,receipt lags cycling40/90/150ms; first3s excluded',
                method='gain/phase sin-cos regression; RMSE against unshifted truth; no wheel or field selection',
                source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('observer.py')]})
    (output/'synthetic.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'frozen-config.json').write_text(json.dumps({k:selected[k] for k in ['position_std','bias_psd']},indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);sweep(a.output)
