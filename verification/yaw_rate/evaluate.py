#!/usr/bin/env python3
"""Native gyro-only yaw observer audit. References are offline comparisons, never inputs."""
import argparse
import io
import json
import hashlib
import subprocess
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def interpolate(t,x,g,max_gap=.025):
    t=np.asarray(t);g=np.asarray(g);y=np.interp(g,t,x)
    right=np.searchsorted(t,g,side='left');left=np.maximum(0,right-1);right=np.minimum(len(t)-1,right)
    exact=t[right]==g
    y[(g<t[0])|(g>t[-1])|((t[right]-t[left]>max_gap)&~exact)]=np.nan
    return y


def residual(x):
    """105 ms nominal SG residual; contiguous finite data only; trim 10 edge samples."""
    x=np.asarray(x);r=np.full(x.shape,np.nan);ids=np.flatnonzero(np.isfinite(x))
    for part in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
        if len(part)<21:continue
        good=part[10:-10]
        r[good]=x[good]-savgol_filter(x[part],21,3)[10:-10]
    return r


def stats(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    if not len(x):return {'n':0}
    return {'n':len(x),'mean':float(x.mean()),'std_ddof0':float(x.std()),'absolute_p95':float(np.percentile(abs(x),95))}


def trace(binary,stamps,gyro,q=None):
    payload=''.join(f'{int(t)} '+ ' '.join(format(float(v),'.17g') for v in w)+'\n' for t,w in zip(stamps,gyro))
    result=subprocess.run([str(binary)]+([] if q is None else [str(q)]),input=payload,text=True,capture_output=True,check=True)
    z=np.genfromtxt(io.StringIO(result.stdout),delimiter=',',names=True,dtype=None,encoding='utf8')
    assert len(z)==len(stamps) and np.array_equal(z['stamp_ns'],stamps)
    return z,result.stdout


def synthetic(binary,out):
    t=np.arange(0,12,.005);ns=1_000_000_000+np.rint(t*1e9).astype(np.int64)
    frequencies=[.5,1,2,3,5,10,14,23,40]
    ys=[np.sin(2*np.pi*f*t) for f in frequencies]+[np.random.default_rng(731).normal(0,.03,len(t))]
    stamps=np.concatenate([ns+j*13_000_000_000 for j in range(len(ys))]);values=np.concatenate(ys)
    gyro=np.c_[np.zeros(len(values)),np.zeros(len(values)),values];records=[]
    for q in [.01,.1,1,2,3,4,6,10,100,1000,10000]:
        z,_=trace(binary,stamps,gyro,q);response=[]
        for j,f in enumerate(frequencies):
            sl=slice(j*len(t),(j+1)*len(t));m=t>=3;w=2*np.pi*f
            A=np.c_[np.sin(w*t[m]),np.cos(w*t[m]),np.ones(m.sum())]
            b=np.linalg.lstsq(A,z['rate'][sl][m],rcond=None)[0]
            # alpha truth = w*cos(w*t); fit directly to retain derivative gain/phase.
            c=np.linalg.lstsq(A,z['alpha'][sl][m],rcond=None)[0]
            response.append({'frequency_hz':f,'rate_gain':float(np.hypot(*b[:2])),
                'rate_lag_ms':float(-np.arctan2(b[1],b[0])/w*1000),
                'alpha_gain':float(np.hypot(*c[:2])/w),
                'alpha_lag_ms':float(np.arctan2(c[0],c[1])/w*1000)})
        r={'q':q,'R':.0001,'response':response,'white_noise_in_std':float(ys[-1][t>=3].std()),
           'white_noise_out_std':float(z['rate'][-len(t):][t>=3].std()),
           'passes_rate_target':all(abs(v['rate_gain']-1)<=.1 and abs(v['rate_lag_ms'])<=15 for v in response if v['frequency_hz']<=3)}
        records.append(r)
    selected=min(r['q'] for r in records if r['passes_rate_target'])
    result={'records':records,'selected_q':selected,'selection':'smallest grid q meeting .5/1/2/3 Hz gain +/-10%, filter lag +/-15ms',
            'method':'12s 200Hz sine, discard first3s, sin/cos least squares; seeded sigma.03 white noise; filter only',
            'binary_sha256':sha(binary)}
    (out/'synthetic-response.json').write_text(json.dumps(result,indent=2)+'\n');return result


def replay(binary,input_dir,out,label,origin,window,static=None,comparison=None):
    path=input_dir/'imu-quality-input.npz';meta=json.loads((input_dir/'metadata.json').read_text())
    assert sha(path)==meta['npz_sha256'];z=np.load(path);ts=z['corrected_ts'];v=z['corrected']
    assert np.all(np.diff(ts[:,0])>0) and np.all(np.diff(ts[:,1])>0)
    rows,raw_csv=trace(binary,ts[:,1],v[:,3:6]);(out/f'{label}-trace.csv').write_text(raw_csv)
    valid=(rows['accepted']==1)&(rows['valid']==1)
    assert np.all(np.isfinite(rows['rate'][valid])) and np.all(np.isfinite(rows['alpha'][valid]))
    t=(ts[:,1]-origin)*1e-9;grid=np.arange(window[0],window[1],.005)
    cg=interpolate(t,v[:,5],grid);fg=interpolate(t[valid],rows['rate'][valid],grid)
    rc,rf=residual(cg),residual(fg);mask=np.isfinite(rc)&np.isfinite(rf)
    offsets=np.arange(-.050,.0501,.001);fits=[]
    # One common mask across every candidate shift; no gain/offset fitted.
    shifted=np.array([interpolate(t,v[:,5],grid-lag) for lag in offsets])
    common=np.isfinite(fg)&np.all(np.isfinite(shifted),axis=0)
    for lag,y in zip(offsets,shifted):
        fits.append({'lag_ms':int(round(lag*1000)),'rmse':float(np.sqrt(np.mean((fg[common]-y[common])**2)))})
    result={'label':label,'source':meta['source'],'input_sha256':sha(path),'input_n':len(ts),'valid_n':int(valid.sum()),
            'source_gap_ms':stats(np.diff(ts[:,1])*1e-6),'publication_boundary':'accepted valid native IMU events; no ROS receipt-age gate',
            'display_window_s':window,'hf_corrected_gyro':stats(rc[mask]),'hf_new_rate':stats(rf[mask]),
            'alignment_best':min(fits,key=lambda r:r['rmse']),'alignment_samples':int(common.sum()),
            'alignment_definition':'new(t) versus corrected gyro(t-lag); no gain/offset; common samples, source-time fit, not hardware delay',
            'trace_sha256':sha(out/f'{label}-trace.csv'),'core_call_p99_us':float(np.percentile(rows['cost_us'],99))}
    if static:
        m=(t>=static[0])&(t<=static[1])&valid
        result['static_window_s']=static;result['static_corrected']=stats(v[m,5]);result['static_output']=stats(rows['rate'][m])
    ekf=None
    if comparison:
        comp=np.load(comparison/'yaw-input.npz');cm=json.loads((comparison/'metadata.json').read_text())
        assert sha(comparison/'yaw-input.npz')==cm['npz_sha256']
        et=(comp['ekf_ts'][:,1]-origin)*1e-9;ev=comp['ekf'][:,2];ekf=(et,ev)
        result['comparison_only_ekf_sha256']=sha(comparison/'yaw-input.npz')
    return result,(t,v,rows,valid,z,ekf)


def plots(data,out,window):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    t,v,r,valid,z,ekf=data
    grid=np.arange(*window,.005);cg=interpolate(t,v[:,5],grid)
    trend=np.full_like(cg,np.nan);ids=np.flatnonzero(np.isfinite(cg))
    for p in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
        if len(p)>=21:trend[p]=savgol_filter(cg[p],21,3)
    slope=np.gradient(trend,.005);ok=np.isfinite(slope)&(grid>window[0]+1)&(grid<window[1]-1)
    center=float(grid[np.nanargmax(np.where(ok,abs(slope),np.nan))]);zoom=(center-.6,center+.8)
    fig,axes=plt.subplots(3,1,figsize=(13.5,10));fig.subplots_adjust(left=.085,right=.98,top=.84,bottom=.17,hspace=.60)
    for ax,(lo,hi),title in zip(axes[:2],[window,zoom],['MPCC RUNNING 全程','最快变化段附近（由同一 SG 趋势定位）']):
        m=(t>=lo)&(t<=hi);mf=m&valid
        # Corrected and original raw source stamps match exactly for this input.
        rawid=np.searchsorted(z['raw_ts'][:,1],r['stamp_ns'][m]);assert np.array_equal(z['raw_ts'][rawid,1],r['stamp_ns'][m])
        ax.plot(t[m],z['raw'][rawid,5],color='#999999',lw=.8,alpha=.55,label='原始 gyro z')
        ax.plot(t[m],v[m,5],color='#D77D13',lw=.8,alpha=.65,label='去零偏 gyro z')
        if ekf:
            et,ev=ekf;me=(et>=lo)&(et<=hi);ax.plot(et[me],ev[me],color='#8A6BA0',lw=1.2,ls='--',label='导航 EKF（仅对照）')
        ax.plot(t[mf],r['rate'][mf],color='#245F87',lw=1.3,label='新 r 估计')
        ax.set(xlim=(lo,hi),ylabel='r / (rad/s)',xlabel='IMU 源时间，相对录包起点 / s')
        ax.set_title(title,loc='left',fontsize=12,fontweight='bold',pad=12);ax.grid(color='#E4E7EB')
    ax=axes[2];lo,hi=zoom;m=(t>=lo)&(t<=hi)&valid
    dt=np.diff(r['stamp_ns'])*1e-9;d=np.r_[np.nan,np.diff(v[:,5])/dt];d[np.r_[True,(dt<.003)|(dt>.007)]]=np.nan
    ax.plot(t[m],d[m],color='#999999',alpha=.55,lw=.8,label='相邻 gyro 差分（仅对照）')
    ax.plot(t[m],r['alpha'][m],color='#245F87',lw=1.5,label='新 dr/dt 估计')
    ax.set(xlim=zoom,ylabel='dr/dt / (rad/s²)',xlabel='IMU 源时间，相对录包起点 / s')
    ax.set_title('同一时段的角加速度',loc='left',fontweight='bold',fontsize=12,pad=12);ax.grid(color='#E4E7EB');ax.legend(loc='upper right',fontsize=10)
    h,l=axes[0].get_legend_handles_labels();fig.legend(h,l,loc='upper left',bbox_to_anchor=(.085,.915),ncol=4,frameon=False,fontsize=10)
    fig.suptitle('最新 MPCC：独立 gyro 主导 yaw rate / 角加速度',x=.085,y=.985,ha='left',fontsize=19,fontweight='bold')
    fig.text(.085,.94,'2026-10-15 022604 · 原接收顺序离线重放 · 运动学 [r, dr/dt] · 未使用 EKF／轮速／电流作为输入',fontsize=10.5,color='#444444')
    fig.text(.085,.105,'所有曲线保持源时间，无人工平移或额外绘图平滑。r = 车体 omega_z，不是一般三维姿态下的欧拉 yaw 导数。',fontsize=10.5,color='#444444')
    fig.text(.085,.065,'角加速度差分仅用 3–7 ms 间隔；图中各曲线均非独立真值。新估计仍继承上游 gyro 零偏残余。',fontsize=10.5,color='#444444')
    for ext in ['png','pdf']:fig.savefig(out/f'yaw-rate-comparison.{ext}',dpi=180,facecolor='white')
    plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--mpcc-input',type=Path,required=True)
    p.add_argument('--comparison',type=Path);p.add_argument('--bc-root',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    synth=synthetic(a.binary,a.output)
    assert synth['selected_q']==4.,'Reassess the default if the independent sweep changes'
    # Verify executable default equals selected q at every numeric output (timing excluded).
    probe=np.arange(100)*5_000_000+1_000_000_000;gyro=np.c_[np.zeros(100),np.zeros(100),np.sin(np.arange(100))]
    d,_=trace(a.binary,probe,gyro);s,_=trace(a.binary,probe,gyro,4.)
    for key in d.dtype.names:
        if key!='cost_us':assert np.array_equal(d[key],s[key]),'binary default differs from selected q'
    metrics,data=replay(a.binary,a.mpcc_input,a.output,'mpcc',1792002365474876930,(41.926416916,220.431558514),(13.065745826,41.425777542),a.comparison)
    results=[metrics];plots(data,a.output,metrics['display_window_s'])
    if a.bc_root:
        for run in ['233659-B-full','015017-C-resume-minus60A-odom30cm']:
            src=a.bc_root/run;z=np.load(src/'imu-quality-input.npz');origin=int(z['raw_ts'][0,1])
            start=(int(z['corrected_ts'][0,1])-origin)*1e-9+1.;end=(int(z['corrected_ts'][-1,1])-origin)*1e-9-1.
            r,_=replay(a.binary,src,a.output,run,origin,(start,end));results.append(r)
    summary={'synthetic':synth,'runs':results,'binary_sha256':sha(a.binary),'renderer_sha256':sha(__file__),
             'independent_truth':False,'observer_inputs':['corrected gyro only'],'model_covariance_calibrated':False,
             'hf_method':'200Hz local interpolation <=25ms gaps, SG21 order3 residual, trim10 points per finite segment; same finite support; includes true fast movement and vibration'}
    (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(results,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
