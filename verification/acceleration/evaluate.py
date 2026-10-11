"""Offline evaluation only: future wheel references never enter the observer.

All float times are seconds relative to an integer origin. Wheel acceleration
is a centered/zero-phase proxy, not independent truth. Holding emitted estimates
uses receipt time; reference interpolation uses acquisition/header source time.
"""
from __future__ import annotations
from dataclasses import asdict
import csv
import json
from pathlib import Path
import numpy as np
from scipy.signal import butter, sosfiltfilt, savgol_filter

FS = 200.


def received_hold(times, values, grid, max_age):
    times, values, grid = np.asarray(times), np.asarray(values), np.asarray(grid)
    indices = np.searchsorted(times, grid, side='right')-1
    clipped = np.clip(indices, 0, len(times)-1)
    output = values[clipped].astype(float, copy=True)
    output[(indices<0) | (grid-times[clipped]>max_age+1e-12)] = np.nan
    return output


def gap_interp(times, values, grid, max_gap):
    """Linear offline reference interpolation, no interpolation across outages."""
    times, values, grid = np.asarray(times), np.asarray(values), np.asarray(grid)
    if len(times)<2:
        return np.full(len(grid), np.nan)
    hi = np.searchsorted(times, grid, side='left')
    exact = (hi<len(times)) & (times[np.clip(hi,0,len(times)-1)] == grid)
    lo, right = np.clip(hi-1,0,len(times)-1), np.clip(hi,0,len(times)-1)
    output = np.interp(grid, times, values)
    valid = exact | ((hi>0)&(hi<len(times))&(times[right]-times[lo]<=max_gap+1e-12))
    output[~valid] = np.nan
    return output


def blocks(mask, minimum=1):
    boundaries = np.diff(np.r_[False, mask, False].astype(int))
    return [(a,b) for a,b in zip(np.flatnonzero(boundaries==1),
                                 np.flatnonzero(boundaries==-1)) if b-a>=minimum]


def lowpass(values):
    output = np.full_like(values, np.nan)
    for a,b in blocks(np.isfinite(values), 30):
        output[a:b] = sosfiltfilt(butter(2,3.,fs=FS,output='sos'),values[a:b])
    return output


def reference(run):
    if hasattr(run, '_evaluation_reference'):
        return run._evaluation_reference
    elapsed = lambda t: (t-run.origin_ns)*1e-9
    grid = np.arange(np.ceil(elapsed(run.imu_receipt_ns[0])*FS)/FS,
                     elapsed(run.imu_receipt_ns[-1]), 1/FS)
    wheel_t = elapsed(run.wheel_source_ns)
    wheel = gap_interp(wheel_t,run.wheel_speed,grid,.060)
    derivative = np.full(len(grid),np.nan)
    for a,b in blocks(np.isfinite(wheel),60):
        derivative[a:b] = savgol_filter(wheel[a:b],33,3,deriv=1,delta=1/FS)
    derivative = lowpass(derivative)
    gyro_norm = gap_interp(elapsed(run.imu_source_ns),np.linalg.norm(run.gyro,axis=1),grid,.030)
    yaw_rate = gap_interp(elapsed(run.imu_source_ns),run.gyro[:,2],grid,.030)
    lio_speed = gap_interp(elapsed(run.lio_source_ns),run.lio_speed,grid,.250)
    active = (grid>=run.start_s)&(grid<=run.end_s)
    stationary = ((grid<run.start_s-1)|(grid>run.end_s+1)) & \
        (abs(wheel)<.03)&(gyro_norm<.08)&(lio_speed<.05)
    kept = np.zeros(len(grid),dtype=bool)
    for a,b in blocks(stationary,int(FS)): kept[a:b] = True
    data = dict(grid=grid,wheel=wheel,proxy=derivative,active=active,stationary=kept,
                straight=abs(yaw_rate)<.2, wheel_t=wheel_t)
    run._evaluation_reference = data
    return data


def rms(values):
    valid = np.asarray(values)[np.isfinite(values)]
    return float(np.sqrt(np.mean(valid**2))) if len(valid) else float('nan')


def onset_lags(grid, proxy, estimated, active):
    result = {}
    for label, sign in [('acceleration',1),('braking',-1)]:
        candidates = [(a,b) for a,b in blocks(np.isfinite(proxy)&(sign*proxy>.25),int(.10*FS))
                      if active[a] and a>0 and np.isfinite(proxy[a-1]) and sign*proxy[a-1]<=.25]
        # Find crossings before clipping the search window. A stream already
        # above threshold at its start (or after a gap) has no known onset.
        measured_blocks=blocks(np.isfinite(estimated)&(sign*estimated>.25),int(.05*FS))
        measured_crossings=[grid[a] for a,b in measured_blocks
                            if a>0 and np.isfinite(estimated[a-1])
                            and sign*estimated[a-1]<=.25]
        previous = -np.inf
        used=set()
        lags, misses = [], 0
        for a,b in candidates:
            onset = grid[a]
            if onset-previous<.75: continue
            previous = onset
            matches = [(i,t) for i,t in enumerate(measured_crossings)
                       if i not in used and abs(t-onset)<=.5+1e-12]
            if matches:
                # Closest sustained crossing to the reference, including negative lag.
                index,crossing = min(matches,key=lambda item:abs(item[1]-onset))
                used.add(index)
                lags.append((crossing-onset)*1000)
            else: misses += 1
        result[label+'_onset_count'] = len(lags)+misses
        result[label+'_onset_missed'] = misses
        result[label+'_onset_lag_median_ms'] = float(np.median(lags)) if lags else float('nan')
        result[label+'_onset_lag_p95_ms'] = float(np.percentile(lags,95)) if lags else float('nan')
    return result


def evaluate_run(run, estimates):
    ref=reference(run)
    grid,active=ref['grid'],ref['active']
    v=received_hold(estimates[:,0],estimates[:,1],grid,.020)
    a=received_hold(estimates[:,0],estimates[:,2],grid,.020)
    good=active&np.isfinite(a)&np.isfinite(v)&np.isfinite(ref['proxy'])
    stationary=ref['stationary']&np.isfinite(a)
    static_residual=[]
    for left,right in blocks(stationary,int(FS)):
        static_residual.extend(a[left:right]-np.mean(a[left:right]))
    m={'coverage_fraction':float(good.sum()/active.sum()),
       'stationary_seconds':float(stationary.sum()/FS),
       'stationary_sigma_mps2':rms(static_residual),
       'stationary_mean_mps2':float(np.mean(a[stationary])) if stationary.any() else float('nan'),
       'active_hf_rms_mps2':rms((a-lowpass(a))[good]),
       'proxy_acceleration_rmse_mps2':rms((a-ref['proxy'])[good])}
    for label,mask in [('straight',ref['straight']),('curved',~ref['straight']),
                       ('acceleration',ref['proxy']>.25),('braking',ref['proxy']<-.25)]:
        keep=good&mask
        m[label+'_proxy_rmse_mps2']=rms((a-ref['proxy'])[keep])
        m[label+'_seconds']=float(keep.sum()/FS)
    for horizon in [.1,.2]:
        future=gap_interp(ref['wheel_t'],run.wheel_speed,grid+horizon,.060)
        keep=good&np.isfinite(future)
        m[f'forecast_{int(horizon*1000)}ms_rmse_mps']=rms((v+a*horizon-future)[keep])
        m[f'forecast_{int(horizon*1000)}ms_samples']=int(keep.sum())
    m.update(onset_lags(grid,ref['proxy'],a,active))
    return m


def save_final(runs,selected,output,replay):
    from replay import HELD_OUT
    rows=[]
    arrays=output/'estimates'
    arrays.mkdir(exist_ok=True)
    for run in runs:
        ref=reference(run)
        for name,config in selected.items():
            estimates,counters,costs=replay(run,config,measure_time=True)
            metrics=evaluate_run(run,estimates)
            all_costs=np.array([c[3] for c in costs])
            timing={}
            for kind in ['speed','force','gravity','delayed']:
                samples=[c[3] for c in costs if c[1] and (c[2] if kind=='delayed' else c[0]==kind)]
                timing.update({f'{kind}_processing_{p}_ms':float(np.percentile(samples,v))
                               if samples else float('nan') for p,v in [('p50',50),('p95',95),('max',100)]})
            span=(run.events[-1][0]-run.events[0][0])*1e-9
            rows.append(dict(run=run.name,candidate=name,
                             role='held_out' if run.name in HELD_OUT else 'training',**metrics,
                             processing_p50_ms=float(np.percentile(all_costs,50)),
                             processing_p95_ms=float(np.percentile(all_costs,95)),
                             processing_max_ms=float(max(all_costs)),
                             processing_ms_per_recorded_second=float(all_costs.sum()/span),**timing,
                             **{'count_'+key:value for key,value in counters.items()}))
            assert counters['numerical_failure']==0 and counters['invalid']==0
            np.savez_compressed(arrays/(run.name+'-'+name+'.npz'),origin_ns=np.int64(run.origin_ns),
                                estimates=estimates,grid=ref['grid'],wheel=ref['wheel'],
                                proxy=ref['proxy'],active=ref['active'],stationary=ref['stationary'],
                                gravity_time=(run.lio_source_ns-run.origin_ns)*1e-9,
                                gravity=run.gravity,force_time=(run.imu_source_ns-run.origin_ns)*1e-9,
                                force=run.force,force_valid=run.force_valid,
                                config=json.dumps(asdict(config)))
            print('final',run.name,name,'forecast200',round(metrics['forecast_200ms_rmse_mps'],4),flush=True)
    with (output/'metrics.csv').open('w') as file:
        writer=csv.DictWriter(file,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    (output/'metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
    plot_results(rows,output)


def plot_results(rows,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from replay import HELD_OUT
    labels={'wheel':'Wheel KF','wheel_imu':'Wheel + IMU','wheel_imu_lio':'Wheel + IMU + delayed LIO'}
    colors={'wheel':'#2878b5','wheel_imu':'#ed8a23','wheel_imu_lio':'#399664'}
    plt.rcParams.update({'font.size':12,'axes.grid':True,'grid.alpha':.25})
    for run in HELD_OUT:
        fig,axes=plt.subplots(3,1,figsize=(16,9),sharex=True,constrained_layout=True)
        first=True
        for name in labels:
            data=np.load(output/'estimates'/(run+'-'+name+'.npz'))
            est=data['estimates'];ref_time=data['grid'];active=data['active']
            if first:
                axes[0].plot(ref_time,data['wheel'],color='.5',label='Wheel source-time speed')
                axes[1].plot(ref_time,data['proxy'],color='.5',lw=2,label='Offline wheel derivative proxy')
                axes[2].plot(data['gravity_time'],data['gravity'],color='.5',label='LIO gravity at scan source time')
                first=False
            axes[0].plot(est[:,0],est[:,1],color=colors[name],label=labels[name],alpha=.8)
            axes[1].plot(est[:,0],est[:,2],color=colors[name],label=labels[name],alpha=.8)
            if name!='wheel':axes[2].plot(est[:,0],est[:,4],color=colors[name],label=labels[name]+' effective offset')
        axes[0].set_ylabel('Velocity [m/s]');axes[1].set_ylabel('Acceleration [m/s²]')
        axes[2].set_ylabel('Offset / gravity [m/s²]');axes[2].set_xlabel('Receipt elapsed time [s]')
        for ax in axes: ax.legend(loc='upper right',fontsize=9)
        axes[0].set_xlim(ref_time[active][0]-1,ref_time[active][-1]+2)
        fig.suptitle(run+' — held-out causal estimates (wheel proxy is not ground truth)',fontsize=15)
        fig.savefig(output/(run+'.png'),dpi=120);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(16,9),constrained_layout=True)
    metrics=[('stationary_sigma_mps2','Stationary acceleration noise [m/s²]'),
             ('active_hf_rms_mps2','Acceleration >3 Hz residual proxy [m/s²]'),
             ('forecast_200ms_rmse_mps','200 ms velocity forecast RMSE [m/s]')]
    for ax,(metric,title) in zip(axes,metrics):
        for i,name in enumerate(labels):
            group=[r for r in rows if r['candidate']==name]
            ax.scatter(np.full(6,i)-.08,[r[metric] for r in group if r['role']=='training'],
                       color=colors[name],marker='o',alpha=.5,label='Training' if i==0 else None)
            ax.scatter(np.full(2,i)+.08,[r[metric] for r in group if r['role']=='held_out'],
                       color=colors[name],marker='x',s=90,label='Held out' if i==0 else None)
        ax.set_xticks(range(3),['Wheel','+ IMU','+ delayed LIO']);ax.set_title(title,fontsize=12)
        ax.legend()
    fig.suptitle('Acceleration observer comparison — six training runs, two held-out runs',fontsize=17)
    fig.savefig(output/'comparison.png',dpi=120);plt.close(fig)


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Regenerate plots from existing frozen results; no retuning.')
    parser.add_argument('--evidence',type=Path,required=True)
    args=parser.parse_args()
    plot_results(json.loads((args.evidence/'metrics.json').read_text()),args.evidence)
