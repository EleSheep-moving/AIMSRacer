#!/usr/bin/env python3
"""Auditable six-axis audit and receipt-causal native replay; never loads EKF arrays."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import numpy as np
from scipy import signal
from scipy.spatial.transform import Rotation, Slerp
from decimal import Decimal

G=9.80665
RUN=(41.926416916,220.431558514)
STATIC=(13.065745826,41.425777542)
LEVER=np.array([.30,0.,.03])


def receipt_order(rows):
    return sorted(rows,key=lambda row:row[0])  # stable ties; no global source-time sort


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def summary(x):
    a=np.asarray(x)
    return {'count':len(a),'mean':np.mean(a,axis=0).tolist(),'std':np.std(a,axis=0).tolist(),
            'p50':np.percentile(a,50,axis=0).tolist(),'p95':np.percentile(a,95,axis=0).tolist(),
            'max':np.max(a,axis=0).tolist()}


def oadev(values,dt,ms=None):
    """Overlapping Allan deviation of rate/acceleration samples, not integrated phase.

    For cluster size m: sqrt(mean((mean[k+m:k+2m]-mean[k:k+m])**2)/2).
    Overlapping pairs are correlated; pair count is support, not independent DOF.
    """
    x=np.asarray(values,float)
    if x.ndim==1:x=x[:,None]
    if dt<=0 or len(x)<3 or not np.all(np.isfinite(x)):raise ValueError('invalid Allan input')
    if ms is None:ms=np.unique(np.maximum(1,np.geomspace(1,max(1,len(x)//10),20).astype(int)))
    c=np.vstack([np.zeros((1,x.shape[1])),np.cumsum(x,axis=0)])
    rows=[]
    for m in ms:
        m=int(m)
        if m<1 or 2*m>len(x):continue
        delta=(c[2*m:]-2*c[m:-m]+c[:-2*m])/m
        rows.append({'tau_s':m*dt,'cluster_samples':m,'pairs':len(delta),
                     'deviation':np.sqrt(np.mean(delta**2,axis=0)/2).tolist()})
    return rows


def fit_known_poses(measured,expected):
    """Diagonal scale model measured = scale*known_specific_force + bias."""
    x,y=np.asarray(measured,float),np.asarray(expected,float)
    if x.shape!=y.shape or x.ndim!=2 or x.shape[1]!=3 or len(x)<6:raise ValueError('need >=6 known poses')
    scales=[];bias=[];res=[]
    for axis in range(3):
        A=np.column_stack([y[:,axis],np.ones(len(y))])
        if np.linalg.matrix_rank(A)<2 or np.ptp(y[:,axis])<G:raise ValueError('poses do not excite all axes')
        b=np.linalg.lstsq(A,x[:,axis],rcond=None)[0]
        if b[0]<=0:raise ValueError('invalid scale')
        scales.append(float(b[0]));bias.append(float(b[1]));res.append(float(np.sqrt(np.mean((A@b-x[:,axis])**2))))
    return {'scale':scales,'bias':bias,'residual_rms':res,'model':'measured = scale * expected + bias'}


def segment_welch(t,values,nperseg=512):
    t,x=np.asarray(t),np.asarray(values)
    dt=np.median(np.diff(t));cuts=np.flatnonzero((np.diff(t)>1.5*dt)|(np.diff(t)<=0))+1
    powers=[];weights=[];used=0
    for ids in np.split(np.arange(len(t)),cuts):
        if len(ids)<nperseg:continue
        # Only local jitter is interpolated; a gap is never crossed.
        u=np.arange(t[ids[0]],t[ids[-1]]+dt/2,dt)
        y=np.column_stack([np.interp(u,t[ids],x[ids,j]) for j in range(x.shape[1])])
        f,p=signal.welch(y,fs=1/dt,nperseg=nperseg,noverlap=nperseg//2,axis=0,detrend='constant')
        w=1+(len(y)-nperseg)//(nperseg//2);powers.append(p);weights.append(w);used+=len(ids)
    if not powers:raise ValueError('no contiguous Welch segment long enough')
    p=np.average(powers,axis=0,weights=weights)
    return {'frequency_hz':f.tolist(),'power_density':p.tolist(),'segments':len(powers),'samples':used,
            'dt_s':float(dt),'nperseg':nperseg,'welch_windows':int(sum(weights)),
            'method':'Hann 50% overlap, constant detrend, local regular interpolation, split gap >1.5 median dt'}


def load_reference(path,meta):
    required=['clock_scale','clock_offset_ns','clock_uncertainty_ns','rotation_xyzw','translation_m','window_s','polyorder','body_origin_in_marker_m','body_orientation_in_marker_xyzw']
    if any(k not in meta for k in required):raise ValueError('reference requires explicit clock, rigid transform, differentiation metadata')
    if meta['clock_scale']<=0 or meta['clock_uncertainty_ns']<0:raise ValueError('invalid clock')
    a=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf8')
    if len(a)<7:raise ValueError('too few reference samples')
    ns=a['source_ns'].astype(np.int64);rec=a['receive_ns'].astype(np.int64)
    # Subtract integer origin before affine float conversion to retain ns precision.
    origin=int(ns[0]);t=(ns-origin)*1e-9*meta['clock_scale']
    if np.any(np.diff(t)<=0):raise ValueError('reference source must increase')
    dt=float(np.median(np.diff(t)))
    if np.max(np.diff(t))>1.5*dt:raise ValueError('split reference gaps before differentiating')
    rotation=Rotation.from_quat(meta['rotation_xyzw']);offset=np.asarray(meta['translation_m'])
    marker_q=Rotation.from_quat(np.column_stack([a[k] for k in ['qx','qy','qz','qw']]))
    marker_p=np.column_stack([a[k] for k in ['px','py','pz']])
    body_p=marker_p+marker_q.apply(meta['body_origin_in_marker_m'])
    p=rotation.apply(body_p)+offset
    q=rotation*marker_q*Rotation.from_quat(meta['body_orientation_in_marker_xyzw'])
    n=max(meta['polyorder']+2,int(round(meta['window_s']/dt)));n+=1-n%2
    if n>len(t):raise ValueError('reference shorter than differentiation window')
    uniform=np.arange(int(np.floor((t[-1]-t[0])/dt+1e-6))+1)*dt+t[0];uniform=np.minimum(uniform,t[-1])
    p=np.column_stack([np.interp(uniform,t,p[:,j]) for j in range(3)])
    acceleration=signal.savgol_filter(p,n,meta['polyorder'],deriv=2,delta=dt,axis=0)
    q_uniform=Slerp(t,q)(uniform)
    valid=np.ones(len(uniform),dtype=bool);valid[:n//2]=False;valid[-(n//2):]=False
    covnames=[f'cov_{i}{j}' for i in range(6) for j in range(6)]
    present=[k in a.dtype.names for k in covnames]
    if any(present) and not all(present):raise ValueError('reference pose covariance must contain all 36 entries')
    cov=np.column_stack([a[k] for k in covnames]).reshape(-1,6,6) if all(present) else np.zeros((len(a),6,6))
    if not np.all(np.isfinite(cov)):raise ValueError('nonfinite reference covariance')
    mapped_origin=int((Decimal(str(meta['clock_scale']))*Decimal(origin)+Decimal(str(meta['clock_offset_ns']))).to_integral_value())
    return {'source_origin_mapped_ns':mapped_origin,'original_source_ns':ns,'original_receive_ns':rec,
            'mapped_source_ns':mapped_origin+np.rint(t*1e9).astype(np.int64),
            'uniform_mapped_source_ns':mapped_origin+np.rint(uniform*1e9).astype(np.int64),
            'original_pose_covariance':cov,'pose_covariance_unknown':not all(present),'valid':valid,'acceleration_body':q_uniform.inv().apply(acceleration),'body_orientation_xyzw':q_uniform.as_quat(),
            'time_s':uniform,'acceleration_world':acceleration,'receive_ns':rec,'pose_rotation':q,
            'offline_centered_future_samples':True,'effective_window_s':n*dt,
            'clock_uncertainty_ns':meta['clock_uncertainty_ns']}


def trailing_derivative(t,w,window=.025):
    """Historical trailing least-squares slope; sample-centre support age is exposed."""
    out=np.zeros_like(w);age=np.zeros(len(t));left=0
    for i in range(len(t)):
        while t[i]-t[left]>window+1e-9:left+=1
        ids=np.arange(left,i+1)
        if len(ids)<3:continue
        local=t[ids]-t[i];centered=local-local.mean();den=centered@centered
        if den<1e-10:continue
        out[i]=centered@w[ids]/den;age[i]=-local.mean()
    return out,age


def load_input(p):
    meta=json.loads((p/'metadata.json').read_text());path=p/'yaw-input.npz'
    if sha(path)!=meta['npz_sha256']:raise ValueError('NPZ hash mismatch')
    z=np.load(path)
    # Explicit allowlist: EKF and command streams are never read.
    a={k:z[k] for k in ['raw','raw_ts','imu','imu_ts','lio','lio_ts']}
    source,ri,pi=np.intersect1d(a['raw_ts'][:,1],a['imu_ts'][:,1],return_indices=True)
    if len(source)!=len(a['imu']):raise ValueError('rear IMU source stamps not uniquely raw matched')
    bias_rows=a['raw'][ri,3:]-a['imu'][pi,3:]
    bias=np.median(bias_rows,axis=0)
    if np.max(np.abs(bias_rows-bias))>1e-10:raise ValueError('gyro correction is not a constant matched bias')
    return a,meta,source,ri,pi,bias


def corrected_events(z):
    if np.intersect1d(z['corrected_ts'][:,0],z['lio_ts'][:,0]).size:
        raise ValueError('cross-topic receipt tie: extraction must retain sqlite id before replay')
    return receipt_order([(int(r),int(st),'I',i) for i,(r,st) in enumerate(z['corrected_ts'])]+
                         [(int(r),int(st),'L',i) for i,(r,st) in enumerate(z['lio_ts'])])


def audit(p,out,trace=None,trace_args=None,exact_input=None):
    a,meta,s,ri,pi,bias=load_input(p);origin=int(meta['origin_ns']);t=(s-origin)*1e-9
    raw=a['raw'][ri].copy();raw[:,:3]*=G;rear=a['imu'][pi];w=raw[:,3:]-bias
    running=(t>=RUN[0])&(t<=RUN[1]);stationary=(t>=STATIC[0])&(t<=STATIC[1])
    dt=np.diff(s)*1e-9
    alpha,alpha_age=trailing_derivative(t,w)
    tang=np.cross(alpha,LEVER);cent=np.cross(w,np.cross(w,LEVER));pred=raw[:,:3]-tang-cent
    # Reconstruct the historical trailing 25 ms regression and report residual.
    reconstructed_residual=rear[:,:3]-pred
    metric={'source':str(p),'origin_ns':origin,'windows_source_relative_s':{'stationary':STATIC,'running':RUN},
            'hashes':{str(x):sha(x) for x in [p/'yaw-input.npz',p/'metadata.json',p/'status.json',Path(__file__)]},
            'runtime':{'python':platform.python_version(),'numpy':np.__version__},
            'implementation_hashes':{str(x):sha(x) for x in [Path('src/aims_racer_system/src/body_acceleration.cpp'),Path('src/aims_racer_system/include/aims_racer_system/body_acceleration.hpp'),Path('src/aims_racer_system/tests/body_acceleration_trace.cpp')] if x.exists()},
            'matching':{'raw_count':len(a['raw']),'rear_count':len(rear),'exact_matches':len(s),
                        'raw_without_rear':len(a['raw'])-len(s),'gyro_bias_raw_minus_processed_rad_s':bias.tolist(),
                        'gyro_bias_residual_absmax':float(np.max(np.abs(a['raw'][ri,3:]-rear[:,3:]-bias)))},
            'limitations':['No independent acceleration truth; LIO shares IMU.',
                          ('Exact source extraction includes covariance; raw/corrected zero matrices indicate unknown precision.' if exact_input else 'Extract omits covariance arrays.')+' Sensor temperature, saturation flags and calibration metadata unavailable.',
                          'Running standard deviation and PSD contain dynamics and vibration, not isolated sensor noise.',
                          'Single stationary pose cannot identify accelerometer tilt, bias and scale separately.'],
            'static':{'raw_si':summary(raw[stationary]),'processed_si':summary(rear[stationary]),
                      'raw_force_norm':summary(np.linalg.norm(raw[stationary,:3],axis=1)),
                      'tangential':summary(tang[stationary]),'centripetal':summary(cent[stationary])},
            'running':{'raw_si':summary(raw[running]),'processed_si':summary(rear[running]),
                       'tangential':summary(tang[running]),'centripetal':summary(cent[running])},
            'historical_reconstruction_residual':summary(reconstructed_residual[2:]),'angular_derivative_sample_center_age_ms':summary(alpha_age[running]*1000),'timing':{}}
    for name in ['raw','imu','lio']:
        ts=a[name+'_ts'];d=np.diff(ts[:,1])*1e-9
        metric['timing'][name]={'source_gap_s':summary(d),'receipt_minus_source_ms':summary((ts[:,0]-ts[:,1])*1e-6),
                                'nonincreasing_source':int(np.sum(d<=0)),'gaps_gt_1p5_median':int(np.sum(d>1.5*np.median(d)))}
    metric['static']['allan']={'duration_s':float(t[stationary][-1]-t[stationary][0]),
        'method':'overlapping cluster means; nominal tau = cluster samples times median source period; source jitter not time-weighted; no long term bias-instability fit',
        'source_period_s':summary(np.diff(t[stationary])),
        'raw':oadev(raw[stationary],float(np.median(dt)))}
    psd_raw=segment_welch(t[running],raw[running]);psd_rear=segment_welch(t[running],rear[running])
    (out/'psd.json').write_text(json.dumps({'raw':psd_raw,'processed':psd_rear},indent=2)+'\n')
    metric['psd']={k:v for k,v in psd_raw.items() if k not in ['frequency_hz','power_density']}
    # Receipt ordered merged events. Corrected gyro becomes available at rear receipt, a conservative
    # proxy for its missing corrected-raw receipt stream. Never synthesize pre-readiness IMU rows.
    events=[(int(r),int(st),'I',i) for i,(r,st) in enumerate(a['imu_ts'][pi])]
    events += [(int(r),int(st),'L',i) for i,(r,st) in enumerate(a['lio_ts'])]
    events=receipt_order(events)
    corrected=None
    if exact_input:
        ep=exact_input/'imu-quality-input.npz';em=json.loads((exact_input/'metadata.json').read_text())
        if sha(ep)!=em['npz_sha256']:raise ValueError('exact source hash mismatch')
        ez=np.load(ep);corrected=ez['corrected']
        if not np.array_equal(ez['corrected_ts'][:,1],s):raise ValueError('exact corrected source mismatch')
        if np.max(np.abs(corrected[:,:3]-a['raw'][ri,:3]))>1e-12 or np.max(np.abs(corrected[:,3:]-w))>1e-12:raise ValueError('corrected mismatch')
        events=corrected_events(ez)
        for name in ['raw','corrected','rear','lio']:
            ts=ez[name+'_ts'];metric['timing'][name+'_exact_receipt_age_ms']=summary((ts[:,0]-ts[:,1])*1e-6)
        metric['exact_extraction']={'metadata':em,'metadata_sha256':sha(exact_input/'metadata.json'),
            'rear_minus_corrected_receipt_ms':summary((ez['rear_ts'][:,0]-ez['corrected_ts'][:,0])*1e-6),
            'rear_accel_unavailable_rows':int(np.sum(ez['rear_cov'][:,0]<0)),
            'corrected_accel_cov_all_zero_rows':int(np.sum(np.all(ez['corrected_cov'][:,:9]==0,axis=1))),
            'corrected_gyro_cov_all_zero_rows':int(np.sum(np.all(ez['corrected_cov'][:,9:18]==0,axis=1))),
            'cross_topic_receipt_ties':int(np.intersect1d(ez['corrected_ts'][:,0],ez['lio_ts'][:,0]).size)}
    with (out/'events.tsv').open('w') as f:
        for receipt,stamp,kind,i in events:
            if kind=='I':v=(list(corrected[i]) if corrected is not None else list(a['raw'][ri[i],:3])+list(w[i]));line='IMU '+str(stamp)+' '+' '.join(format(float(x),'.17g') for x in v)
            else:
                q=a['lio'][i,5:9];line='LIO '+str(stamp)+' '+' '.join(format(float(x),'.17g') for x in [q[3],q[0],q[1],q[2]])
            f.write(f'{receipt}\t{line}\n')
    metric['replay_input']={'events':len(events),'imu':len(s),'lio':len(a['lio']),
        'global_source_reversals':sum(events[i][1]<events[i-1][1] for i in range(1,len(events))),
        'ordering':'stable receipt ascending; source timestamp preserved as integer ns',
        'gyro_receipt_proxy':('none: exact corrected IMU receipt' if corrected is not None else 'rear IMU receipt; corrected raw receipt unavailable; conservative extra pipeline latency'),
        'gyro_bias_readiness_source_s':float(t[0]),'events_sha256':sha(out/'events.tsv')}
    trace_rows=None
    if trace:
        payload='\n'.join(x.split('\t',1)[1] for x in (out/'events.tsv').read_text().splitlines())+'\n'
        command=[str(trace)]+(trace_args or [])
        proc=subprocess.run(command,input=payload,text=True,capture_output=True,check=True)
        (out/'trace.csv').write_text(proc.stdout);(out/'trace.stderr').write_text(proc.stderr)
        trace_rows=list(csv.DictReader(proc.stdout.splitlines()))
        imu_rows=[r for r in trace_rows if r['event']=='IMU']
        valid=[r for r in imu_rows if r['accepted']=='1' and r['valid']=='1']
        sts={}
        for r in imu_rows:sts[r['status']]=sts.get(r['status'],0)+1
        metric['native']={'command':command,'binary_sha256':sha(trace),'trace_sha256':sha(out/'trace.csv'),
                          'imu_rows':len(imu_rows),'valid_rows':len(valid),'status_counts':sts}
        for label,window in [('static',STATIC),('running',RUN)]:
            rows=[r for r in valid if window[0]<=(int(r['stamp_ns'])-origin)*1e-9<=window[1]]
            if rows:metric['native'][label]={k:summary([float(r[k]) for r in rows]) for k in ['ax','jerk','raw_ax','raw_ay','raw_az','variance','measurement_variance','alpha_window_span_s','alpha_sample_center_age_s']}
        # Gravity sensitivity: compare propagated snapshot with the last received source-before-IMU
        # LIO orientation. This is a consistency comparison, not known gravity error.
        anchors={int(st):q for st,q in zip(a['lio_ts'][:,1],a['lio'][:,5:9])}
        ages=[];delta=[]
        for r in valid:
            st=int(r['stamp_ns']);anchor=int(r['anchor_stamp_ns'])
            if not RUN[0]<=(st-origin)*1e-9<=RUN[1] or anchor not in anchors:continue
            q=anchors[anchor];held=Rotation.from_quat(q).inv().apply([0.,0.,-G])
            propagated=np.array([float(r[k]) for k in ['gravity_x','gravity_y','gravity_z']])
            ages.append((st-anchor)*1e-6);delta.append(np.linalg.norm((held-propagated)[:2]))
        if ages:metric['gravity_consistency']={'anchor_age_ms':summary(ages),'held_vs_propagated_horizontal_mps2':summary(delta),
            'meaning':'shared-IMU model consistency, not measured gravity estimation error'}
    plots(out,t,raw,rear,tang,cent,stationary,psd_raw,psd_rear,trace_rows,origin)
    metric['output_files']={p.name:sha(p) for p in out.iterdir() if p.is_file()}
    (out/'metrics.json').write_text(json.dumps(metric,indent=2,allow_nan=False)+'\n')
    return metric


def plots(out,t,raw,rear,tang,cent,stationary,psd_raw,psd_rear,trace_rows,origin):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    av=oadev(raw[stationary],float(np.median(np.diff(t))))
    fig,ax=plt.subplots(1,2,figsize=(12,4))
    for j in range(6):ax[j//3].loglog([r['tau_s'] for r in av],[r['deviation'][j] for r in av],label=['ax','ay','az','wx','wy','wz'][j])
    for a in ax:a.grid();a.legend();a.set_xlabel('tau s')
    ax[0].set_ylabel('m/s²');ax[1].set_ylabel('rad/s');fig.suptitle('28 s stationary overlapping Allan deviation: no long-term bias fit');fig.tight_layout();fig.savefig(out/'short-static-allan.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(13,9))
    for j in range(6):
        a=ax.flat[j];a.plot(t[stationary],raw[stationary,j],label='raw SI',lw=.6);a.plot(t[stationary],rear[stationary,j],label='processed rear',lw=.6,alpha=.7)
        a.set_ylabel(['ax m/s²','ay m/s²','az m/s²','wx rad/s','wy rad/s','wz rad/s'][j]);a.grid();a.set_xlabel('source relative s')
    ax.flat[0].legend();fig.suptitle('Stationary matched samples: processing can amplify angular derivative noise');fig.tight_layout();fig.savefig(out/'static-six-axis.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(13,9))
    for j in range(6):
        a=ax.flat[j]
        for name,p in [('raw SI',psd_raw),('processed rear',psd_rear)]:a.semilogy(p['frequency_hz'],np.asarray(p['power_density'])[:,j],label=name)
        a.set_ylabel(['ax','ay','az','wx','wy','wz'][j]+' PSD');a.set_xlabel('Hz');a.grid()
    ax.flat[0].legend();fig.suptitle('RUNNING Welch PSD: motion + vibration + noise; no independent truth');fig.tight_layout();fig.savefig(out/'running-psd.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for j in range(3):
        ax[j].plot(t[stationary],tang[stationary,j],label='alpha × lever',lw=.6);ax[j].plot(t[stationary],cent[stationary,j],label='omega × (omega × lever)',lw=.6)
        ax[j].set_ylabel('xyz'[j]+' m/s²');ax[j].grid()
    ax[0].legend();ax[-1].set_xlabel('source relative s');fig.tight_layout();fig.savefig(out/'lever-decomposition.png',dpi=150);plt.close(fig)
    if trace_rows:
        rr=[r for r in trace_rows if r['event']=='IMU' and r['accepted']=='1' and r['valid']=='1'];tt=np.array([(int(r['stamp_ns'])-origin)*1e-9 for r in rr])
        fig,ax=plt.subplots(3,1,figsize=(13,9),sharex=True)
        for key,label in [('raw_ax','reconstructed ax (unfiltered)'),('ax','estimated ax (filtered)')]:ax[0].plot(tt,[float(r[key]) for r in rr],label=label,lw=.6)
        for key,label in [('force_x','mounted specific force x'),('gravity_x','gravity x'),('tangential_x','tangential correction x'),('centripetal_x','centripetal correction x')]:ax[1].plot(tt,[float(r[key]) for r in rr],label=label,lw=.6)
        ax[2].plot(tt,[float(r['variance']) for r in rr],label='longitudinal variance');
        for aa in ax:aa.legend();aa.grid()
        ax[0].set_ylabel('ax m/s²');ax[1].set_ylabel('components m/s²');ax[2].set_ylabel('(m/s²)²');ax[2].set_xlabel('source relative s');fig.tight_layout();fig.savefig(out/'causal-native-stages.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--trace',type=Path);p.add_argument('--trace-arg',action='append',default=[]);p.add_argument('--exact-input',type=Path)
    p.add_argument('--static-input',type=Path,help='future long static NPZ time_s + values(N,6), or CSV time_s,ax,ay,az,wx,wy,wz in SI')
    p.add_argument('--calibration',type=Path,help='JSON measured and expected arrays from known poses')
    p.add_argument('--reference-csv',type=Path);p.add_argument('--reference-metadata',type=Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    if args.input:audit(args.input,args.output,args.trace,args.trace_arg,args.exact_input)
    if args.static_input:
        if args.static_input.suffix=='.npz':z=np.load(args.static_input);t=z['time_s'];v=z['values']
        else:
            z=np.genfromtxt(args.static_input,delimiter=',',names=True);t=z['time_s'];v=np.column_stack([z[k] for k in ['ax','ay','az','wx','wy','wz']])
        dt=np.median(np.diff(t))
        if np.any(np.diff(t)<=0) or np.max(np.abs(np.diff(t)-dt))>.1*dt:raise ValueError('Allan needs contiguous near-uniform static input; split gaps first')
        (args.output/'long-static-allan.json').write_text(json.dumps({'source_sha256':sha(args.static_input),'duration_s':float(t[-1]-t[0]),'rows':oadev(v,float(dt))},indent=2)+'\n')
    if args.calibration:
        d=json.loads(args.calibration.read_text());(args.output/'calibration.json').write_text(json.dumps(fit_known_poses(d['measured'],d['expected']),indent=2)+'\n')
    if args.reference_csv:
        if not args.reference_metadata:p.error('--reference-csv requires --reference-metadata')
        r=load_reference(args.reference_csv,json.loads(args.reference_metadata.read_text()));np.savez_compressed(args.output/'offline-reference.npz',**{k:r[k] for k in ['time_s','acceleration_world','acceleration_body','body_orientation_xyzw','valid','original_source_ns','original_receive_ns','mapped_source_ns','uniform_mapped_source_ns','original_pose_covariance']})
        (args.output/'reference-provenance.json').write_text(json.dumps({'csv_sha256':sha(args.reference_csv),'metadata_sha256':sha(args.reference_metadata),'offline_centered_future_samples':True,'effective_window_s':r['effective_window_s'],'clock_uncertainty_ns':r['clock_uncertainty_ns'],'source_origin_mapped_ns':r['source_origin_mapped_ns'],'pose_covariance_unknown':r['pose_covariance_unknown'],'transform_and_differentiation':json.loads(args.reference_metadata.read_text())},indent=2)+'\n')

if __name__=='__main__':main()
