"""Offline wheel/current identification. No ROS or other motion sensors.

Numerical/plot dependencies are deliberately confined to this module. The
query runtime in wheel_current_model requires only the standard library.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import numpy as np
from scipy.optimize import isotonic_regression
from scipy.integrate import trapezoid

RUNS = ('233659-B-full', '235350-B-resume-from-2mps',
        '000720-B-resume-from-2p5mps', '001857-B-resume-from-3p5mps',
        '013446-C-descending-groups', '015017-C-resume-minus60A-odom30cm')
CHANNELS = {'wheel': ['vx'],
            'vesc': ['iq','id','motor_current','input_current','voltage','duty','erpm','fault'],
            'final': ['mode','speed','current','steering'],
            'request': ['mode','speed','current','steering']}
THRESHOLD = 50.


def file_hash(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def write_csv(path,rows):
    if not rows:return
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader();writer.writerows(rows)


def read_archive(path):
    """Load four channels only; do not even materialize other NPZ arrays."""
    path=Path(path);meta=json.loads(path.with_suffix('.json').read_text())
    if file_hash(path)!=meta['npz_sha256']:raise ValueError('archive hash mismatch: '+path.name)
    series={};quality=[]
    with np.load(path,allow_pickle=False) as archive:
        for key,names in CHANNELS.items():
            columns=meta['columns'][key]
            if key in meta.get('errors',{}):raise ValueError('selected channel decode errors: '+key)
            values=np.asarray(archive[key][:,[columns.index(n) for n in names]],dtype=float)
            stamps=archive[key+'_ts']
            if stamps.shape!=(len(values),2) or not len(values):raise ValueError('invalid timestamp shape: '+key)
            if not np.isfinite(values).all() or not np.isfinite(stamps).all():raise ValueError('nonfinite selected channel: '+key)
            if np.any(np.diff(stamps[:,1])<=0):raise ValueError('non-increasing source timestamps: '+key)
            ts=stamps[:,1]*1e-9
            series[key]=(ts,values)
            quality.append(dict(run=path.stem,channel=key,samples=len(values),
                source_interval_p50_ms=float(np.median(np.diff(stamps[:,1]))*1e-6),
                source_interval_max_ms=float(np.max(np.diff(stamps[:,1]))*1e-6)))
    return series,quality


def _exit_time(series,key,start,target):
    t,x=series[key];ids=np.flatnonzero((t>=start-1e-5)&(t<=start+9.))
    matches=(x[:,0]==1)&(np.abs(x[:,2]-target)<1e-4)
    active=ids[matches[ids]]
    if not len(active):return None
    exits=ids[(ids>active[0])&~matches[ids]]
    return float(t[exits[0]]) if len(exits) else None


def _sustained(mask,ids):
    for j in range(len(ids)-2):
        if mask[ids[j]] and np.sum(mask[ids[j:j+3]])>=2:return int(ids[j])
    return None


def _speed_fit(t,v):
    if len(t)<3 or t[-1]<=t[0]:return {}
    x=t-t[0];slope,intercept=np.polyfit(x,v,1)
    residual=v-(slope*x+intercept);ss=float(np.sum((v-v.mean())**2))
    half=x<=x[-1]/2;slopes=[]
    for mask in (half,~half):
        slopes.append(float(np.polyfit(x[mask],v[mask],1)[0]) if mask.sum()>=3 else None)
    return dict(wheel_samples=len(t),duration_s=float(x[-1]),
        speed_start_mps=float(v[0]),speed_end_mps=float(v[-1]),
        speed_min_mps=float(v.min()),speed_max_mps=float(v.max()),
        speed_mps=float(trapezoid(v,x)/x[-1]),acceleration_mps2=float(slope),
        endpoint_acceleration_mps2=float((v[-1]-v[0])/x[-1]),
        r2=float(1-np.sum(residual**2)/ss) if ss>0 else None,
        residual_rmse_mps=float(np.sqrt(np.mean(residual**2))),
        first_half_acceleration_mps2=slopes[0],second_half_acceleration_mps2=slopes[1],
        max_sample_gap_ms=float(np.max(np.diff(t))*1000))


def reconstruct_trial(event,series):
    row={key:event.get(key) for key in ('run','trial_id','plan_index','stage_letter','current',
        'start_target_speed','end_target_speed','success','reason')}
    row['trial_key']=f"{row['run']}:{row['trial_id']}"
    start=event.get('first_current_request_stamp');ack=event.get('first_current_ack_stamp')
    if start is None or ack is None:return dict(row,status='missing_command_boundary')
    target=event['current'];exit_time=event.get('exit_final_stamp')
    if exit_time is None:exit_time=_exit_time(series,'final',start,target)
    row.update(first_current_request_stamp=start,first_current_ack_stamp=ack,exit_final_stamp=exit_time)
    if exit_time is None:return dict(row,status='missing_final_exit')
    t,vesc=series['vesc'];iq=vesc[:,0];wt,w=series['wheel']
    pre=(t>=start-.25)&(t<=start-.03)
    before=float(np.median(iq[pre])) if pre.any() else float(target)
    core=np.flatnonzero((t>=ack+.02)&(t<=exit_time-.01))
    if len(core)<3:core=np.flatnonzero((t>=ack)&(t<=exit_time))
    if not len(core):return dict(row,status='no_current_samples')
    level=float(np.median(iq[core]));mad=float(np.median(np.abs(iq[core]-level)))
    tol=max(1.,3*1.4826*mad,.1*abs(level-before));band=np.abs(iq-level)<=tol
    row.update(plateau_iq_a=level,plateau_tolerance_a=tol,pre_iq_a=before)
    left=_sustained(band,np.flatnonzero((t>=start)&(t<=exit_time)))
    if left is None:return dict(row,status='no_sustained_plateau')
    departure=_sustained(~band,np.flatnonzero((np.arange(len(t))>left)&(t>=exit_time-.02)&(t<=exit_time+.25)))
    if departure is None:return dict(row,status='right_censored_no_observed_departure')
    right=departure-1
    if right<=left:return dict(row,status='insufficient_plateau_samples')
    st,et=float(t[left]),float(t[right]);mask=(wt>=st-1e-6)&(wt<=et+1e-6)
    stats=_speed_fit(wt[mask],w[mask,0])
    if not stats:return dict(row,status='insufficient_wheel_samples')
    direction=np.sign(level-before) or np.sign(target-before) or 1
    peak=left+int(np.argmax(direction*iq[left:right+1]))
    row.update(stats,status='reconstructed',start_stamp=st,end_stamp=et,
        iq_samples=right-left+1,iq_mean_a=float(np.mean(iq[left:right+1])),
        iq_std_a=float(np.std(iq[left:right+1])),iq_target_error_a=float(np.mean(iq[left:right+1])-target),
        voltage_mean_v=float(np.mean(vesc[left:right+1,4])),
        fault_samples=int(np.sum(vesc[left:right+1,7]!=0)),
        plateau_in_band_fraction=float(np.mean(band[left:right+1])),
        literal_extreme_stamp=float(t[peak]),departure_stamp=float(t[departure]),
        departure_sample_interval_ms=float((t[departure]-t[right])*1000))
    sensitivity=[]
    for dl,dr in [(-1,0),(1,0),(0,-1),(0,1)]:
        lo,hi=max(0,left+dl),min(len(t)-1,right+dr)
        m=(wt>=t[lo]-1e-6)&(wt<=t[hi]+1e-6);fit=_speed_fit(wt[m],w[m,0])
        if fit:sensitivity.append(fit['acceleration_mps2'])
    row['boundary_acceleration_min']=float(min(sensitivity)) if sensitivity else None
    row['boundary_acceleration_max']=float(max(sensitivity)) if sensitivity else None
    return row


def _longest_contiguous(ids):
    parts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    parts=[p for p in parts if len(p)>=3]
    return max(parts,key=len) if parts else np.array([],dtype=int)


def training_records(row,series,baseline):
    if row['status']!='reconstructed' or row['fault_samples']:return []
    wt,w=series['wheel'];it,current=series['vesc'];v=w[:,0]
    mask=(wt>=row['start_stamp']-1e-6)&(wt<=row['end_stamp']+1e-6)
    zero=np.flatnonzero(mask&(v<=0))
    if len(zero):mask&=np.arange(len(wt))<zero[0]
    low,high=baseline['speed_domain_mps'];mask&=(v>=low)&(v<=high)
    high_pulse=row['stage_letter']=='B' and row['iq_mean_a']>=THRESHOLD
    pieces=[]
    if high_pulse:
        ids=_longest_contiguous(np.flatnonzero(mask))
        if len(ids):pieces.append((None,ids))
    else:
        for center in np.arange(low,high+.01,.5):
            ids=_longest_contiguous(np.flatnonzero(mask&(v>=center-.5)&(v<=center+.5)))
            if len(ids):pieces.append((float(center),ids))
    result=[]
    seen=set()
    for center,ids in pieces:
        # Edge speed bins can select identical samples: do not duplicate them.
        key=(int(ids[0]),int(ids[-1]))
        if key in seen:continue
        seen.add(key);t=wt[ids];stats=_speed_fit(t,v[ids]);iq=np.interp(t,it,current[:,0])
        record={k:row[k] for k in ('run','trial_id','trial_key','stage_letter','current','start_target_speed','success','reason')}
        record.update(stats,start_stamp=float(t[0]),end_stamp=float(t[-1]),
            iq_mean_a=float(np.mean(iq)),speed_bin_center_mps=center,
            response_window='domain_plateau' if high_pulse else 'local_speed_window')
        result.append(record)
    for record in result:record['weight']=1./len(result)
    return result


def _baseline(baseline,speed):
    return np.polynomial.polynomial.polyval(speed,baseline['coefficients_ascending'])


def _group_points(points,step):
    grouped={}
    for x,y,weight in points:
        group=round(x/step)
        grouped.setdefault(group,[]).append((x,y,weight))
    result=[]
    for group in sorted(grouped):
        a=np.array(grouped[group]);weight=a[:,2]
        result.append((float(np.average(a[:,0],weights=weight)),float(np.average(a[:,1],weights=weight)),float(weight.sum())))
    return result


def _isotonic(points,lower=-np.inf,upper=np.inf):
    if not points:return [],[]
    x,y,weights=np.array(points).T
    order=np.argsort(x);x,y,weights=x[order],y[order],weights[order]
    fitted=isotonic_regression(y,weights=weights).x
    return x.tolist(),np.clip(fitted,lower,upper).tolist()


def fit_model(records,baseline):
    """Anchored weighted isotonic least squares, then linear interpolation.

    Current samples in 5 A bins and normalized low-current samples in u bins
    of width 0.125 are compressed by weighted means before isotonic fitting.
    The shared 50 A anchor is interpolated from a pooled >=40 A boundary fit;
    the >=50 A high branch is then fitted subject to that fixed lower anchor.
    """
    coefficients=list(baseline['coefficients_ascending'])
    if not 1<=len(coefficients)<=4:raise ValueError('baseline must be at most cubic')
    baseline=dict(baseline,coefficients_ascending=coefficients+[0.]*(4-len(coefficients)))
    high=[r for r in records if r['stage_letter']=='B' and r['iq_mean_a']>=THRESHOLD]
    if len(high)<2 or len(set(r['iq_mean_a']for r in high))<2:raise ValueError('insufficient high-current B coverage')
    boundary=[(r['iq_mean_a'],r['acceleration_mps2'],r['weight']) for r in records if r['stage_letter']=='B' and r['iq_mean_a']>=40.]
    bx,by=_isotonic(_group_points(boundary,5.),lower=0.)
    if not bx or bx[0]>THRESHOLD or bx[-1]<THRESHOLD:
        raise ValueError('B observations must bracket the shared 50 A anchor')
    # The threshold is an explicitly fitted shared anchor, not an observed sample.
    a50=max(0.,float(np.interp(THRESHOLD,bx,by)))
    hx,hy=_isotonic(_group_points([(r['iq_mean_a'],r['acceleration_mps2'],r['weight'])for r in high],5.),lower=a50)
    if hx[0]<=THRESHOLD+1e-10:hx,hy=hx[1:],hy[1:]
    hx=[THRESHOLD]+hx;hy=[a50]+hy
    if len(hx)<2:raise ValueError('insufficient high-current knots')
    # A grouped estimate is constant up to that group's actual observed extreme.
    # This extends the last bin value, never support beyond recorded raw current.
    high_max=max(r['iq_mean_a']for r in high)
    if high_max>hx[-1]+1e-10:hx.append(float(high_max));hy.append(hy[-1])
    rows=[]
    low_records=[r for r in records if r['iq_mean_a']<THRESHOLD]
    for speed in np.arange(baseline['speed_domain_mps'][0],baseline['speed_domain_mps'][1]+.01,.5):
        points=[]
        for r in low_records:
            nearest=.5*math.floor(2*r['speed_mps']+.5)
            if abs(nearest-speed)>.01:continue
            base=float(_baseline(baseline,r['speed_mps']))
            u=(r['iq_mean_a']-base)/(THRESHOLD-base)
            if u<1:points.append((u,r['acceleration_mps2'],r['weight']))
        if not points:continue
        negative=_group_points([p for p in points if p[0]<-1e-9],.125)
        positive=_group_points([p for p in points if 1e-9<p[0]<1.],.125)
        nx,ny=_isotonic(negative,upper=0.)
        px,py=_isotonic(positive,lower=0.,upper=a50)
        u=nx+[0.]+px+[1.];a=ny+[0.]+py+[a50]
        # Observed extrema remain supported even after bin averaging.
        min_u=min((p[0]for p in points),default=0.)
        if nx and min_u<nx[0]-1e-10:u.insert(0,float(min_u));a.insert(0,ny[0])
        supports=[[float(min_u),0.]] if nx else [[0.,0.]]
        if px:supports[-1][1]=1.
        rows.append(dict(speed_mps=float(speed),u=u,acceleration_mps2=a,support_intervals=supports,
                         training_records=len(points),training_weight=float(sum(p[2]for p in points))))
    vmin=max(baseline['speed_domain_mps'][0],.5*math.floor(2*min(r.get('speed_min_mps',r['speed_mps'])for r in high)))
    vmax=min(baseline['speed_domain_mps'][1],.5*math.ceil(2*max(r.get('speed_max_mps',r['speed_mps'])for r in high)))
    return dict(schema_version=1,kind='wheel_current_model',current_signal='avg_iq',outcome='mean_wheel_acceleration',
        threshold_iq_a=THRESHOLD,baseline=baseline,rows=rows,
        high=dict(iq_a=hx,acceleration_mps2=hy,support_intervals=[[THRESHOLD,float(high_max)]],
                  speed_domain_mps=[float(vmin),float(vmax)]),
        provenance={},validation={},fit=dict(method='anchored weighted isotonic least squares with piecewise linear interpolation',
            speed_grid_mps=.5,high_current_bin_a=5.,low_u_bin=.125,
            boundary_anchor_method='nonnegative pooled B current >=40 A; interpolate isotonic binned response at 50 A',
            within_bin_endpoint_policy='nearest isotonic bin value to recorded extremes; no support beyond observed current range',
            shared_50a_acceleration_mps2=a50,training_records=len(records),
            trial_weight_total=float(sum(r['weight']for r in records))))


def load_dataset(root):
    root=Path(root);audits=[];records=[];quality=[];sources=[];raw={}
    baseline_path=root/'evidence/2026-10-14/233659-B-full/params/baseline_current.json'
    base=json.loads(baseline_path.read_text())
    if base['current_signal']!='avg_iq' or base['planning_model']['kind']!='cubic_polynomial':raise ValueError('unsupported frozen A baseline')
    baseline=dict(coefficients_ascending=base['planning_model']['coefficients'],speed_domain_mps=base['planning_model']['speed_domain_mps'])
    sources.append(dict(role='baseline',path=str(baseline_path),sha256=file_hash(baseline_path)))
    for run in RUNS:
        candidates=[p for p in (root/'evidence').glob('2026-*/*/*/trials.json') if p.parent.parent.name==run]
        if len(candidates)>1:candidates=[p for p in candidates if p.parent.name!='calibration-result-snapshot']
        if len(candidates)!=1:raise ValueError('ambiguous source trial record: '+run)
        trial_path=candidates[0];payload=json.loads(trial_path.read_text())
        archive=root/'raw-extracts'/f'{run}.npz';series,q=read_archive(archive);raw[run]=series;quality.extend(q)
        sources.extend([dict(role='archive',path=str(archive),sha256=file_hash(archive)),
                        dict(role='archive_metadata',path=str(archive.with_suffix('.json')),sha256=file_hash(archive.with_suffix('.json'))),
                        dict(role='source_trials',path=str(trial_path),sha256=file_hash(trial_path))])
        for event in payload['trials']:
            event=dict(event,run=run,stage_letter='B' if '-B-'in run else 'C')
            audit=reconstruct_trial(event,series);training=training_records(audit,series,baseline)
            audit['training_records']=len(training)
            audit['training_status']='included' if training else 'no_supported_forward_window'
            audits.append(audit);records.extend(training)
    if len(audits)!=336:raise ValueError('formal attempt count mismatch')
    return audits,records,baseline,dict(sources=sources,channels=list(CHANNELS),timestamp='message header source timestamp; no fitted shift',
        excluded_inputs=['imu','lio','ekf','observer'],formal_runs=list(RUNS)),quality,raw


def _metrics(predictions):
    valid=[p for p in predictions if p['valid']]
    total=sum(p['weight']for p in predictions);covered=sum(p['weight']for p in valid)
    result=dict(attempted_records=len(predictions),predicted_records=len(valid),
                trial_weight_total=float(total),predicted_trial_weight=float(covered),
                weighted_coverage=float(covered/total) if total else 0.)
    if valid:
        error=np.array([p['predicted_acceleration_mps2']-p['observed_acceleration_mps2']for p in valid])
        weight=np.array([p['weight']for p in valid])
        result.update(rmse_mps2=float(np.sqrt(np.average(error**2,weights=weight))),
            mae_mps2=float(np.average(np.abs(error),weights=weight)),
            bias_mps2=float(np.average(error,weights=weight)),
            max_abs_error_mps2=float(np.max(np.abs(error))))
    else:result.update(rmse_mps2=None,mae_mps2=None,bias_mps2=None,max_abs_error_mps2=None)
    return result


def _prediction(record,model,**extra):
    result=model.predict_acceleration(record['iq_mean_a'],record['speed_mps']) if model else None
    return dict(trial_key=record['trial_key'],run=record['run'],trial_id=record['trial_id'],
        stage=record['stage_letter'],region=('B_high'if record['iq_mean_a']>=THRESHOLD else 'B_low')if record['stage_letter']=='B'else'C',
        iq_a=record['iq_mean_a'],wheel_speed_mps=record['speed_mps'],weight=record['weight'],
        observed_acceleration_mps2=record['acceleration_mps2'],
        predicted_acceleration_mps2=result.value if result else None,
        valid=result.valid if result else False,reason=result.reason if result else 'insufficient_fold_coverage',**extra)


def cross_validate(records,baseline):
    from .wheel_current_model import WheelCurrentModel
    predictions=[];summaries={}
    for scheme in ('start_speed','current_level'):
        if scheme=='start_speed':groups=[str(r['start_target_speed'])for r in records]
        else:
            groups=[]
            for r in records:
                level=r['current']-float(_baseline(baseline,r['start_target_speed'])) if r['stage_letter']=='B' else r['current']
                groups.append(r['stage_letter']+':'+str(5*round(level/5)))
        folds=[]
        for group in sorted(set(groups)):
            train=[r for r,g in zip(records,groups)if g!=group]
            test=[r for r,g in zip(records,groups)if g==group]
            try:model=WheelCurrentModel.from_dict(fit_model(train,baseline));error=None
            except ValueError as exc:model=None;error=str(exc)
            fold=[_prediction(r,model,scheme=scheme,heldout_group=group)for r in test]
            predictions.extend(fold);folds.append(dict(heldout_group=group,training_records=len(train),
                test_trial_keys=sorted(set(r['trial_key']for r in test)),fit_error=error,**_metrics(fold)))
        selected=[p for p in predictions if p['scheme']==scheme]
        summaries[scheme]=dict(overall=_metrics(selected),regions={region:_metrics([p for p in selected if p['region']==region])for region in ('B_low','B_high','C')},folds=folds)
    return predictions,summaries


def verify_windows(audits,raw,legacy_path=None):
    """Independent OLS via lstsq; check direct native current/sample bounds."""
    differences=[];current_differences=[];counts=0
    for row in audits:
        if row['status']!='reconstructed':continue
        series=raw[row['run']];t,w=series['wheel'];it,iq=series['vesc']
        mask=(t>=row['start_stamp']-1e-6)&(t<=row['end_stamp']+1e-6)
        x=t[mask]-t[mask][0]
        independent=np.linalg.lstsq(np.column_stack([x,np.ones(len(x))]),w[mask,0],rcond=None)[0][0]
        im=(it>=row['start_stamp'])&(it<=row['end_stamp'])
        differences.append(abs(float(independent)-row['acceleration_mps2']))
        current_differences.append(abs(float(np.mean(iq[im,0]))-row['iq_mean_a']))
        assert len(x)==row['wheel_samples'] and int(im.sum())==row['iq_samples']
        counts+=1
    assert not differences or max(differences)<1e-7
    assert not current_differences or max(current_differences)<1e-10
    result=dict(independent_ols_windows=counts,max_acceleration_difference_mps2=max(differences,default=0.),
                max_current_difference_a=max(current_differences,default=0.))
    if legacy_path is not None and Path(legacy_path).exists():
        legacy={(r['run'],r['trial_id']):r for r in json.loads(Path(legacy_path).read_text())}
        changes=[];matched=0
        for r in audits:
            old=legacy.get((r['run'],r['trial_id']))
            if old is None or r['status']!=old['status']:
                changes.append(dict(run=r['run'],trial_id=r['trial_id'],reason='status differs'));continue
            if r['status']=='reconstructed':
                if abs(r['start_stamp']-old['start_stamp'])>1e-6 or abs(r['end_stamp']-old['end_stamp'])>1e-6:
                    changes.append(dict(run=r['run'],trial_id=r['trial_id'],reason='native boundaries differ'))
                else:matched+=1
        result.update(legacy_windows_matched=matched,legacy_boundary_changes=changes)
    return result


def verify_runtime(model):
    valid_count=0;inverse_count=0;max_error=0.;supported=0;attempted=0
    lower=min((interval[0]for row in model.to_dict()['rows']for interval in row['support_intervals']),default=0)
    baseline=model.to_dict()['baseline']
    for speed in np.linspace(*baseline['speed_domain_mps'],17):
        base=float(_baseline(baseline,speed));iq_min=base+lower*(THRESHOLD-base)
        currents=sorted(set(np.linspace(iq_min,model.to_dict()['high']['iq_a'][-1],111).tolist()+[base,50.-1e-7,50.,50.+1e-7]))
        previous=None
        for iq in currents:
            attempted+=1;forward=model.predict_acceleration(float(iq),float(speed))
            if not forward.valid:continue
            supported+=1
            if previous is not None:assert forward.value>=previous-1e-9
            previous=forward.value;valid_count+=1
            inverse=model.current_for_acceleration(forward.value,float(speed))
            assert inverse.valid,(iq,speed,forward,inverse)
            repeated=model.predict_acceleration(inverse.value,float(speed));assert repeated.valid
            error=abs(repeated.value-forward.value);assert error<1e-7
            inverse_count+=1;max_error=max(max_error,error)
        inverse=model.current_for_acceleration(0.,float(speed))
        assert inverse.valid and abs(inverse.value-base)<1e-10
        if model.predict_acceleration(50.-1e-7,float(speed)).valid:
            assert abs(model.predict_acceleration(50.-1e-7,float(speed)).value-model.predict_acceleration(50.,float(speed)).value)<1e-5
    return dict(monotone_supported_queries=valid_count,roundtrips=inverse_count,max_roundtrip_error_mps2=max_error,
                grid_attempted=attempted,grid_supported=supported)


def plot_results(output,records,model,raw,audits,predictions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'axes.spines.top':False,'axes.spines.right':False,'font.size':10})
    fig,axes=plt.subplots(1,2,figsize=(14,5.5),layout='constrained')
    for stage,ax in zip(('B','C'),axes):
        rows=[r for r in records if r['stage_letter']==stage]
        scatter=ax.scatter([r['iq_mean_a']for r in rows],[r['acceleration_mps2']for r in rows],
            c=[r['speed_mps']for r in rows],cmap='viridis',vmin=1,vmax=5,s=13,alpha=.5)
        fig.colorbar(scatter,ax=ax,label='Measured mean wheel speed (m/s)')
        for speed in [1.,2.,3.,4.,5.]:
            current=np.linspace(0 if stage=='B' else -80,99 if stage=='B' else 15,450)
            values=[model.predict_acceleration(float(i),speed)for i in current]
            a=np.array([r.value if r.valid else np.nan for r in values])
            ax.plot(current,a,lw=1.7,label=f'{speed:g} m/s')
        if stage=='B':ax.axvline(50,color='gray',ls=':',lw=1)
        ax.set(title=f'Stage {stage}: fitted monotone wheel response',xlabel='Measured Iq (A)',ylabel='Mean wheel acceleration (m/s²)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(output/'response-curves.png',dpi=170);plt.close(fig)
    speed=np.linspace(1,5,81);current=np.linspace(-80,100,181)
    surface=np.full((len(speed),len(current)),np.nan)
    for j,v in enumerate(speed):
        for k,i in enumerate(current):
            result=model.predict_acceleration(float(i),float(v))
            if result.valid:surface[j,k]=result.value
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    coverage=axes[0].imshow(np.isfinite(surface),origin='lower',aspect='auto',extent=[-80,100,1,5],cmap='Greys',vmin=0,vmax=1)
    axes[0].set(title='Query support: black=supported, white=unsupported')
    heat=axes[1].imshow(surface,origin='lower',aspect='auto',extent=[-80,100,1,5],cmap='coolwarm')
    fig.colorbar(heat,ax=axes[1],label='Mean wheel acceleration (m/s²)');axes[1].set(title='Monotone response (unsupported cells blank)')
    for ax in axes:ax.set(xlabel='Iq (A)',ylabel='Wheel speed (m/s)');ax.axvline(50,color='orange',ls='--',lw=1)
    fig.savefig(output/'coverage.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for scheme,ax in zip(('start_speed','current_level'),axes):
        for region in ('B_low','B_high','C'):
            rows=[p for p in predictions if p['scheme']==scheme and p['region']==region and p['valid']]
            ax.scatter([p['observed_acceleration_mps2']for p in rows],[p['predicted_acceleration_mps2']for p in rows],s=12,alpha=.5,label=region)
        lo,hi=ax.get_xlim();ax.plot([lo,hi],[lo,hi],'k--',lw=1)
        ax.set(title=f'Grouped holdout: {scheme}',xlabel='Observed wheel acceleration (m/s²)',ylabel='Predicted wheel acceleration (m/s²)');ax.grid(alpha=.2);ax.legend()
    fig.savefig(output/'heldout-predictions.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for pair,(run,num)in zip(axes,[('233659-B-full',19),('015017-C-resume-minus60A-odom30cm',80)]):
        r=next(r for r in audits if r['run']==run and r['trial_id']==num)
        t,iq=raw[run]['vesc'];wt,w=raw[run]['wheel'];origin=r['start_stamp'];end=r['end_stamp']
        a=(t>=origin-.06)&(t<=end+.10);b=(wt>=origin-.06)&(wt<=end+.10)
        pair[0].plot(t[a]-origin,iq[a,0],'.-');pair[1].plot(wt[b]-origin,w[b,0],'.-')
        for ax in pair:ax.axvspan(0,end-origin,color='green',alpha=.12);ax.set_xlabel('Time from current plateau onset (s)');ax.grid(alpha=.2)
        pair[0].set(title=f'{run}, trial {num}',ylabel='Measured Iq (A)')
        pair[1].set(title=f"Wheel slope {r['acceleration_mps2']:.2f} m/s², R² {r['r2']:.4f}",ylabel='Wheel speed (m/s)')
    fig.savefig(output/'wheel-only-windows.png',dpi=170);plt.close(fig)


def run_fit(root,output):
    from .wheel_current_model import WheelCurrentModel
    root=Path(root).resolve();output=Path(output).resolve()
    if output==root or any(output.is_relative_to(root/name)for name in ('raw-extracts','evidence')):
        raise ValueError('output must not overwrite source directories')
    if output.exists() and any(output.iterdir()):raise ValueError('output directory is not empty')
    audits,records,baseline,provenance,quality,raw=load_dataset(root)
    model_payload=fit_model(records,baseline)
    predictions,validation=cross_validate(records,baseline)
    verification=verify_windows(audits,raw,root/'measured-current-windows/windows.json')
    model_payload['provenance']=dict(provenance,fit_source_sha256=file_hash(__file__))
    model_payload['validation']=validation
    model=WheelCurrentModel.from_dict(model_payload);verification.update(verify_runtime(model))
    weights={}
    for r in records:weights[r['trial_key']]=weights.get(r['trial_key'],0.)+r['weight']
    assert all(abs(w-1.)<1e-9 for w in weights.values())
    verification['normalized_trial_weights']=len(weights)
    # Recheck every input before publishing derived results.
    assert all(file_hash(source['path'])==source['sha256']for source in provenance['sources'])
    train_predictions=[_prediction(r,model)for r in records]
    summary=dict(formal_attempts=len(audits),reconstructed_windows=sum(r['status']=='reconstructed'for r in audits),
        attempts_with_training_data=len(weights),training_records=len(records),
        training_regions={region:_metrics([p for p in train_predictions if p['region']==region])for region in ('B_low','B_high','C')},
        grouped_validation=validation,verification=verification,
        scope='offline mean wheel acceleration calibration, no other motion sensors, no controller changes',
        nominal_speed_domain_mps=baseline['speed_domain_mps'],high_iq_domain_a=model_payload['high']['support_intervals'],
        assumptions=['Low branch uses u=(Iq-Iq0(v))/(50-Iq0(v)); rows interpolate at common u.',
          'Anchored isotonic least squares imposes current monotonicity; violations remain visible as residuals.',
          '50 A is a candidate engineering split, not an established physical transition.',
          'Support is interpolation within recorded local current ranges and 0.5 m/s speed bins, not an accuracy guarantee.',
          'B >=50 A uses supported plateau-average response; other intervals use native speed-local windows.',
          'The response horizon varies by interval; this is not a causal instantaneous plant or validated control law.',
          'CV RMSE is conditional on supported test inputs; weighted coverage is reported alongside errors.',
          'Frozen A speed-holding curve remains the zero-acceleration anchor; no zero-speed/reverse extrapolation.'])
    output.mkdir(parents=True,exist_ok=True)
    write_json(output/'model.json',model.to_dict());write_json(output/'summary.json',summary)
    write_json(output/'verification.json',verification);write_json(output/'audit.json',audits)
    write_csv(output/'audit.csv',audits);write_csv(output/'training-intervals.csv',records)
    write_csv(output/'source-quality.csv',quality);write_csv(output/'heldout-predictions.csv',predictions)
    plot_results(output,records,model,raw,audits,predictions)
    _write_report(output,summary,model)
    hashes={p.name:file_hash(p)for p in sorted(output.iterdir())if p.is_file()}
    write_json(output/'artifact-sha256.json',hashes)
    return dict(output=str(output),formal_attempts=len(audits),reconstructed_windows=summary['reconstructed_windows'],
        training_records=len(records),attempts_with_training_data=len(weights),
        validation={scheme:value['regions']for scheme,value in validation.items()},verification=verification)


def _write_report(output,summary,model):
    lines=['# 轮速—IQ 离线标定模型','',
        '只使用轮速、VESC 实测 avg_iq 和命令信号。IMU、EKF、LIO、滑移估计均不参与模型或筛选。',
        '这是区间平均轮速加速度模型，尚未接入车辆在线控制。','',
        f"正式尝试 **{summary['formal_attempts']}** 次，重建电流平台 **{summary['reconstructed_windows']}** 次；"
        f"**{summary['attempts_with_training_data']}** 次试验形成 **{summary['training_records']}** 个支持域内训练区间。"
        '同一试验的重叠区间总权重为 1，不视为独立实验。','',
        '正向加速 IQ≥50 A 合并不同速度；其余加速、减速、零电流滑行保留轮速维度。'
        '低段以冻结的 basic current 为零加速度锚点，50 A 处连续衔接。',
        '拟合采用加权单调最小二乘：高段电流按 5 A 分组，低段归一坐标 u 按 0.125 分组，速度档间隔 0.5 m/s。'
        '共享 50 A 锚点要求两侧观测夹住边界；各组在本组实际观测极值处沿用该组拟合值，未把支持域延伸到实测电流之外。',
        '单调回归约束只表示交付模型的形状，不能消除录包中的非恒定响应；原始观测、失败标签和拟合残差完整保留。','',
        '## 留组预测结果','',
        '误差仅对有支持的测试输入计算；覆盖率必须同时阅读。未设置精度通过线，也未将当前误差当作实车验收。','',
        '| 留组方法 | 区域 | RMSE (m/s²) | MAE (m/s²) | 偏差 (m/s²) | 试验权重覆盖率 |',
        '|---|---|---:|---:|---:|---:|']
    fmt=lambda x:'—'if x is None else f'{x:.3f}'
    for scheme,data in summary['grouped_validation'].items():
        for region,m in data['regions'].items():
            lines.append(f"| {scheme} | {region} | {fmt(m['rmse_mps2'])} | {fmt(m['mae_mps2'])} | {fmt(m['bias_mps2'])} | {m['weighted_coverage']:.1%} |")
    lines+=['','## 查询示例','',
        'Python: `model = WheelCurrentModel.load("model.json")`，随后调用 '
        '`model.predict_acceleration(iq_a, wheel_speed_mps)` 或 '
        '`model.current_for_acceleration(accel_mps2, wheel_speed_mps)`。','',
        '| 查询 | 返回 |','|---|---|']
    for text,result in [('3 m/s、80 A → 加速度',model.predict_acceleration(80.,3.)),
                        ('3 m/s、−30 A → 加速度',model.predict_acceleration(-30.,3.)),
                        ('3 m/s、+5 m/s² → IQ',model.current_for_acceleration(5.,3.)),
                        ('3 m/s、−3 m/s² → IQ',model.current_for_acceleration(-3.,3.)),
                        ('3 m/s、零加速度 → IQ',model.current_for_acceleration(0.,3.))]:
        lines.append(f"| {text} | {result.value:.4f} |"if result.valid else f'| {text} | 不支持：{result.reason} |')
    lines+=['','反向平坦区选择最接近 basic current 的解；返回 IQ 是目标实测平台电流，不是已发布到 VESC 的命令。'
        '超出数据支持区域返回 valid=false，不外推、不静默限幅。','',
        '## 文件','',
        '- `model.json`：标准库即可加载的模型、来源哈希、支持域与验证结果。',
        '- `audit.csv/json`：全部正式尝试、原失败原因、平台边界及轮速拟合质量。',
        '- `training-intervals.csv`：实际训练区间、权重与原试验关联。',
        '- `heldout-predictions.csv`：逐区间留组预测与不支持原因。',
        '- `verification.json`：原生窗口独立复算、旧窗口核对与正反向检查。',
        '- `response-curves.png`、`coverage.png`、`heldout-predictions.png`、`wheel-only-windows.png`：模型与证据图。','',
        '![模型曲线](response-curves.png)','', '![支持域](coverage.png)','',
        '## 方法与限制','']+['- '+x for x in summary['assumptions']]
    lines+=['','当前 basic current 为冻结的 1～5 m/s Stage A 三次模型；输入文件、窗口规则及拟合来源在模型中留存。'
        '本次没有将旧研究的 IMU 一致性要求沿用为训练门槛，也没有改变采集程序、转弯速度或控制器。','']
    (output/'README.md').write_text('\n'.join(lines))
