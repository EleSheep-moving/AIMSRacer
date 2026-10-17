#!/usr/bin/env python3
"""Input audit and frozen-config receipt-order replay. No wheel arrays loaded."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from observer import Observer, Config


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def stats(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return dict(n=len(x),mean=float(x.mean()),std=float(x.std()),p50=float(np.percentile(x,50)),
                abs_p95=float(np.percentile(abs(x),95)),abs_max=float(max(abs(x)))) if len(x) else dict(n=0)


def evaluate(folder,out,config):
    meta=json.loads((folder/'metadata.json').read_text());assert sha(folder/'input.npz')==meta['npz_sha256']
    with np.load(folder/'input.npz') as archive:z={k:archive[k] for k in archive.files}
    origin=int(z['raw_ts'][0,1]);audit={}
    for name in ['raw','imu','lio']:
        ts=z[name+'_ts'];v=z[name];d=np.diff(ts[:,1])*1e-9
        audit[name]=dict(n=len(v),nonfinite_rows=int(sum(~np.isfinite(v).all(axis=1))),
                         source_nonincreasing=int(sum(d<=0)),source_gap_s=stats(d),
                         receipt_minus_source_ms=stats((ts[:,0]-ts[:,1])*1e-6),
                         covariance_all_zero_rows=int(sum(np.all(z[name+'_cov']==0,axis=1))))
    audit['lio']['quaternion_norm_error_max']=float(max(abs(np.linalg.norm(z['lio'][:,3:7],axis=1)-1)))
    o=Observer(config);records=[];payload=[];rejects={'imu':0,'lio':0}
    for receipt,row_id,kind,index in z['receipt_order']:
        if kind==0:continue
        name='imu' if kind==1 else 'lio';v=z[name][index];stamp=int(z[name+'_ts'][index,1])
        if name=='imu':accepted=o.add_imu(stamp,v[:3],v[3:6]);values=v[:6]
        else:accepted=o.add_lio(stamp,v[:3],v[3:7]);values=v[:7]
        rejects[name]+=not accepted
        payload.append(name.upper()+' '+str(stamp)+' '+' '.join(format(float(x),'.17g') for x in values)+'\n')
        e=o.latest();records.append([name,receipt,stamp,int(accepted),e['stamp_ns'],int(e['valid']),e['status'],e['anchor_stamp_ns'],
                                    e['vx'],e['variance'],*e['position_world'],*e['velocity_world'],*e['effective_bias'],
                                    e['rejected_lio'],o.history_size,e['lio_reinitializations']])
    headers=['event','receipt_ns','input_source_ns','accepted','stamp_ns','valid','status','anchor_stamp_ns','vx','variance',
             'px','py','pz','vwx','vwy','vwz','bix','biy','biz','rejected_lio','history_size','lio_reinitializations']
    with (out/'trace.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(headers);writer.writerows(records)
    (out/'events.txt').write_text(''.join(payload))
    rows=np.array(records,dtype=object);mask=(rows[:,0]=='imu')&(rows[:,3]==1)&(rows[:,5]==1)
    valid=rows[mask];tt=(valid[:,4].astype(np.int64)-origin)*1e-9
    values=valid[:,8].astype(float);bias=valid[:,16:19].astype(float)
    statuses,counts=np.unique(rows[rows[:,0]=='imu',6],return_counts=True)
    metrics=dict(source=meta['source'],input_sha256=meta['npz_sha256'],trace_sha256=sha(out/'trace.csv'),
                 implementation_sha256=sha(Path(__file__).with_name('observer.py')),evaluation_sha256=sha(Path(__file__)),origin_ns=origin,source_audit=audit,input_rejected=rejects,events=len(records),valid_imu_n=len(valid),
                 status_counts={str(k):int(n) for k,n in zip(statuses,counts)},vx=stats(values),
                 final_effective_bias_I=bias[-1].tolist() if len(bias) else None,
                 max_history_size=int(max(rows[:,20])),last_rejected_lio=int(rows[-1,19]),lio_reinitializations=int(rows[-1,21]),
                 independent_truth=False,config={'position_std':config.position_std,'bias_psd':config.bias_psd})
    if '022604_' in meta['source']:
        # Keep the established MPCC source origin, independent of extraction start.
        base=1792002365474876930;tt=(valid[:,4].astype(np.int64)-base)*1e-9
        metrics['mpcc_origin_ns']=base
        metrics['stationary_vx']=stats(values[(tt>=13.065745826)&(tt<=41.425777542)])
        metrics['running_vx']=stats(values[(tt>=41.926416916)&(tt<=220.431558514)])
    (out/'summary.json').write_text(json.dumps(metrics,indent=2)+'\n')
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--frozen',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);config=Config(**json.loads(a.frozen.read_text()));all_metrics={}
    for name in ['B','C','mpcc']:
        out=a.output/name;out.mkdir();metrics=evaluate(a.inputs/name,out,config);all_metrics[name]=metrics
        print(json.dumps({'name':name,'valid_imu_n':metrics['valid_imu_n'],'vx':metrics['vx'],'max_history':metrics['max_history_size'],'rejected_lio':metrics['last_rejected_lio']}),flush=True)
    (a.output/'summary.json').write_text(json.dumps(all_metrics,indent=2)+'\n')
