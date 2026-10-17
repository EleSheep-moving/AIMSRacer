#!/usr/bin/env python3
"""Plot actual dual-reference replay VY and recorded rear LIO/navigation baselines."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_reference import interpolate

STATIC=(13.065745826,41.425777542)
RUN=(41.926416916,220.431558514)
COLORS=dict(own='#0072B2',lio='#555555',ekf='#D55E00',cg='#009E73')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(values):
    return dict(n=len(values),mean=float(np.mean(values)),std=float(np.std(values)),
                abs_p95=float(np.percentile(abs(values),95)),abs_max=float(np.max(abs(values))))


def line(ax,t,y,bounds,label,color,gap,style='-',marker=None):
    indices=np.flatnonzero((t>=bounds[0])&(t<=bounds[1]))
    for i,part in enumerate(np.split(indices,np.flatnonzero(np.diff(t[indices])>gap)+1)):
        if len(part):ax.plot(t[part],y[part],style,color=color,lw=1.25,alpha=.88,
                            marker=marker,markersize=2.5,label=label if i==0 else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.input;out=args.output
    if out.exists():raise ValueError('output must be a new directory')
    meta=json.loads((root/'raw-meta.json').read_text())
    replay=json.loads((root/'replay-full/summary.json').read_text())
    archived=json.loads((root/'waveform-comparison.json').read_text())
    assert sha(root/'raw.npz')==meta['npz_sha256']
    assert sha(root/'replay-full/outputs.csv')==replay['output_sha256']
    assert archived['provenance']['replay_csv_sha256']==replay['output_sha256']
    assert replay['input_source']==meta['source']
    origin=meta['origin_ns'];own={point:[] for point in ['rear_axle','center_of_mass']}
    with (root/'replay-full/outputs.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['kind']=='velocity':own[row['reference']].append((int(row['original_source_ns']),float(row['y'])))
    data={}
    for point,rows in own.items():
        ns=np.asarray([r[0] for r in rows],np.int64);values=np.asarray([r[1] for r in rows])
        assert np.all(np.diff(ns)>0) and np.isfinite(values).all()
        data[point]=((ns-origin)*1e-9,values)
    with np.load(root/'raw.npz') as raw:
        for name,key in [('lio','rear_axle__lio_odom'),('ekf','odometry__filtered')]:
            data[name]=((raw[key+'_ts'][:,1]-origin)*1e-9,raw[key][:,1].copy())
    metrics=dict(source=meta['source'],independent_truth=False,units='m/s',source_origin_ns=origin,
        static_window_s=STATIC,running_window_s=RUN,detail_window_s=[100,106],
        no_additional_smoothing=True,no_time_shift=True,statistics={},
        actual_executable_sha256=replay['implementation']['actual_executable_sha256'],
        provenance={str(p):sha(p) for p in [root/'raw.npz',root/'raw-meta.json',root/'replay-full/outputs.csv',root/'replay-full/summary.json',Path(__file__),Path(__file__).with_name('compare_reference.py')]})
    for name,(t,y) in data.items():
        metrics['statistics'][name]={label:summary(y[(t>=a)&(t<=b)]) for label,(a,b) in [('static',STATIC),('running',RUN)]}
    grid=np.arange(8800,43800)*.005;lags=np.arange(-160,161)*.005
    estimates={};masks=[]
    for name in ['rear_axle','ekf']:
        estimates[name],good=interpolate(*data[name],grid,.03 if name=='rear_axle' else .04);masks.append(good)
    targets=[]
    for lag in lags:
        values,good=interpolate(*data['lio'],grid-lag,.25);targets.append(values);masks.append(good)
    common=np.logical_and.reduce(masks);targets=np.asarray(targets)
    assert common.sum()==35000
    scores={};alignment={}
    for name,old_name in [('rear_axle','own_observer'),('ekf','robot_localization')]:
        scores[name]=np.sqrt(np.mean((estimates[name][None,common]-targets[:,common])**2,axis=1))
        best=int(np.argmin(scores[name]));old=archived['comparisons']['vy']['estimators'][old_name]
        assert abs(scores[name][160]-old['zero_shift_rmse'])<1e-12
        assert abs(lags[best]*1000-old['best_lag_ms'])<1e-9
        alignment[name]=dict(n=int(common.sum()),zero_shift_rmse=float(scores[name][160]),
                             best_lag_ms=float(lags[best]*1000),best_rmse=float(scores[name][best]))
    metrics['rear_lio_waveform_comparison']=alignment
    out.mkdir(parents=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(15,8.5))
    for ax,bounds,title in [(axes[0,0],RUN,'A  Rear axle: full running window'),
                            (axes[0,1],(100,106),'B  Rear axle: fixed 100–106 s detail')]:
        line(ax,*data['rear_axle'],bounds,'Our observer',COLORS['own'],.03)
        line(ax,*data['ekf'],bounds,'Navigation EKF',COLORS['ekf'],.04,'-.')
        line(ax,*data['lio'],bounds,'LIO (shared-sensor baseline)',COLORS['lio'],.25,'--',marker='o' if ax is axes[0,1] else None)
        ax.set_title(title,loc='left');ax.legend(fontsize=8);ax.set_xlim(bounds)
    a,b=RUN;values=np.concatenate([y[(t>=a)&(t<=b)] for t,y in data.values()]);padding=np.ptp(values)*.08
    for ax in [axes[0,0],axes[0,1],axes[1,0]]:ax.set_ylim(min(0,values.min())-padding,max(0,values.max())+padding)
    ax=axes[1,0]
    line(ax,*data['rear_axle'],RUN,'Rear axle',COLORS['own'],.03)
    line(ax,*data['center_of_mass'],RUN,'Center of mass (nominal geometry)',COLORS['cg'],.03,'--')
    ax.set_title('C  Same velocity state, different reference points',loc='left');ax.legend(fontsize=8);ax.set_xlim(RUN)
    ax=axes[1,1]
    line(ax,*data['rear_axle'],STATIC,'Rear axle',COLORS['own'],.03)
    line(ax,*data['center_of_mass'],STATIC,'Center of mass',COLORS['cg'],.03,'--')
    ax.set_title('D  Short static window (independent y scale)',loc='left');ax.legend(fontsize=8);ax.set_xlim(STATIC)
    for ax in axes.flat:
        ax.axhline(0,color='#666666',lw=.6);ax.grid(alpha=.2);ax.set_ylabel('VY (m/s)');ax.set_xlabel('Source time from bag origin (s)')
    fig.suptitle('VY quality — current estimator replay of 2026-10-15 field recording',fontsize=14,y=.98)
    fig.text(.06,.025,'No extra smoothing or curve shifts. Missing support is broken. LIO/EKF share sensors; no independent Vicon truth yet.',fontsize=10)
    fig.tight_layout(rect=[0,.055,1,.95]);fig.savefig(out/'vy-comparison.png',dpi=165);fig.savefig(out/'vy-comparison.svg');plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4.7))
    for name,label,color,style in [('rear_axle','Our rear VY',COLORS['own'],'-'),('ekf','Navigation VY',COLORS['ekf'],'-.')]:
        ax.plot(lags*1000,scores[name],style,color=color,label=label)
        best=int(np.argmin(scores[name]));ax.scatter([lags[best]*1000],[scores[name][best]],color=color,zorder=5)
        ax.annotate(f"{alignment[name]['best_lag_ms']:.0f} ms; {scores[name][best]:.4f} m/s",
                    (lags[best]*1000,scores[name][best]),xytext=(-12,28) if name=='rear_axle' else (14,-2),
                    ha='right' if name=='rear_axle' else 'left',textcoords='offset points',color=color)
    ax.axvline(0,color='#666666',lw=.8);ax.set_xlabel('Waveform shift (ms): positive = estimate trails LIO');ax.set_ylabel('RMSE vs rear LIO (m/s)')
    ax.set_title('Alignment diagnostic: uncorrected RMSE, same 35,000 source-time samples',loc='left');ax.grid(alpha=.2);ax.legend();ax.set_ylim(bottom=0)
    fig.text(.08,.02,'Diagnostic only: LIO is not independent truth; lower aligned RMSE does not establish vehicle accuracy.',fontsize=9)
    fig.tight_layout(rect=[0,.055,1,1]);fig.savefig(out/'vy-alignment.png',dpi=165);plt.close(fig)
    metrics['artifact_sha256']={p.name:sha(p) for p in out.iterdir()}
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(json.dumps(dict(statistics=metrics['statistics'],comparison=alignment),indent=2))


if __name__=='__main__':main()
