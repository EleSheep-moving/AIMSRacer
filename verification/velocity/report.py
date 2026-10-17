#!/usr/bin/env python3
"""Export standalone source-backed engineering figures; no parameter fitting.

Wheel input is optional and used ONLY in a separately labelled comparison.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from evaluate import stats
from scipy.signal import welch
from scipy.integrate import trapezoid

BLUE='#2463A6';GOLD='#B47F14';PINK='#B34F79';GRAY='#555B65';OLIVE='#6D782B'

def main(a):
    a.output.mkdir(parents=True,exist_ok=False)
    with np.load(a.inputs/'mpcc/input.npz') as z:d={k:z[k] for k in z.files}
    z=np.genfromtxt(a.trace,delimiter=',',names=True,dtype=None,encoding='utf8');mask=(z['event']=='IMU')&(z['accepted']==1)&(z['valid']==1);all_trace=z;z=z[mask]
    origin=1792002365474876930;t=(z['stamp_ns']-origin)*1e-9;vx=z['vx'];lt=(d['lio_ts'][:,1]-origin)*1e-9
    it=(d['imu_ts'][:,1]-origin)*1e-9
    # Convert the raw LIO IMU-origin body velocity to the same approximate rear origin.
    lio_vx=d['lio'][:,7]-.03*np.interp(lt,it,d['imu'][:,4])
    held_index=np.searchsorted(d['lio_ts'][:,0],d['imu_ts'][:,0],side='right')-1
    held=lio_vx[np.maximum(held_index,0)];held[held_index<0]=np.nan
    receipt_hold=np.interp(z['stamp_ns'],d['imu_ts'][:,1],held)
    moving=(t>=41.926416916)&(t<=220.431558514);static=(t>=13.065745826)&(t<=41.425777542)
    summary={'reference':'rear axle; shared approximate lever [0.30,0,0.03]m; rotation identity',
        'baseline_definition':{'lio_source':'raw IMU-origin twist converted to rear at LIO source time; offline comparator, not independent observation',
        'lio_hold':'last LIO twist available by original receipt time, no future pose interpolation',
        'ax_integral':'previous accepted rear ax integral, stationary initial vx=0; no corrections after initialization'},
        'running_vx':stats(vx[moving]),'stationary_vx':stats(vx[static]),'uncalibrated_effective_covariance':True}
    for label,dt in [('whole',None),('RUNNING',(41.926416916,220.431558514))]:
        result={}
        for key in ['raw','imu','lio']:
            ts=d[key+'_ts'];tm=(ts[:,1]-origin)*1e-9;pick=np.ones(len(tm),bool) if dt is None else (tm>=dt[0])&(tm<=dt[1]);source=ts[pick,1];gap=np.diff(source)*1e-9
            result[key]={'n':int(pick.sum()),'receipt_minus_source_ms':stats((ts[pick,0]-source)*1e-6),'source_interval_ms':stats(gap*1e3),'gaps_above_30ms':int(sum(gap>.03))}
        summary['input_'+label]=result
    static_imu=(it>=13.065745826)&(it<=41.425777542);st=d['imu'][static_imu]
    sq=(lt>=13.065745826)&(lt<=41.425777542);rpy=Rotation.from_quat(d['lio'][sq,3:7]).as_euler('xyz')
    summary['stationary_inputs']={'specific_force_mean_si':(st[:,:3]*9.80665).mean(axis=0).tolist(),'specific_force_std_si':(st[:,:3]*9.80665).std(axis=0).tolist(),'gyro_std_rad_s':st[:,3:6].std(axis=0).tolist(),'lio_rpy_std_deg':np.rad2deg(rpy.std(axis=0)).tolist(),'warning':'mean force includes gravity, mounting and attitude errors; not hardware accelerometer bias'}
    if 'ix' in all_trace.dtype.names:
        rows=all_trace[(all_trace['event']=='LIO')&(all_trace['accepted']==1)&(all_trace['valid']==1)]
        tm=(rows['input_source_ns']-origin)*1e-9;rows=rows[(tm>=41.926416916)&(tm<=220.431558514)]
        summary['accepted_position_innovation_norm_m']=stats(np.sqrt(rows['ix']**2+rows['iy']**2+rows['iz']**2))
        summary['position_consistency_definition']='pre-update innovation at accepted historical LIO source time; shared information, not independent accuracy'
    spectra={}
    for label,lo,hi in [('stationary',13.065745826,41.425777542),('RUNNING',41.926416916,220.431558514)]:
        tt=np.arange(lo,hi,.005);force=np.column_stack([np.interp(tt,it,d['imu'][:,i])*9.80665 for i in range(3)])
        frequency,power=welch(force,fs=200.,nperseg=1024,noverlap=512,axis=0,detrend='constant')
        band=(frequency>=20)&(frequency<=80)
        spectra[label]={'20_80Hz_force_rms_si':np.sqrt(trapezoid(power[band],frequency[band],axis=0)).tolist(),'duration_s':hi-lo}
        np.savetxt(a.output/f'{label}-force-PSD.csv',np.column_stack([frequency,power]),delimiter=',',header='frequency_Hz,Pxx_x,Pxx_y,Pxx_z')
    summary['spectral_audit']=dict(method='source-time resample200Hz; Welch1024,Hann,50%overlap; offline audit only, not observer filtering',segments=spectra,interpretation='motion-band energy includes vibration and real motion; short static segment does not identify Allan bias instability')
    summary['lio_pose_step_3d_m']=stats(np.linalg.norm(np.diff(d['lio'][:,:3],axis=0),axis=1))
    summary['lio_all_covariance_zero']=bool(np.all(d['lio_cov']==0));summary['lio_angular_twist_all_zero']=bool(np.all(d['lio'][:,10:13]==0))
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'figure.facecolor':'white'})
    fig,axs=plt.subplots(3,1,figsize=(13,10),layout='constrained')
    axs[0].plot(t[moving],vx[moving],color=BLUE,lw=1.1,label='Independent rear Vx (IMU + delayed LIO position)')
    sm=(lt>=t[moving][0])&(lt<=t[moving][-1]);axs[0].plot(lt[sm],lio_vx[sm],color=GRAY,ls='--',lw=.8,alpha=.8,label='LIO source-time velocity (comparison only)')
    axs[0].set(title='MPCC: complete RUNNING interval',ylabel='Rear Vx [m/s]');axs[0].legend(loc='lower left')
    zoom=(t>=72)&(t<=79);q=(lt>=72)&(lt<=79)
    axs[1].plot(t[zoom],vx[zoom],color=BLUE,label='Independent Vx');axs[1].step(t[zoom],receipt_hold[zoom],where='post',color=GOLD,ls='--',label='LIO velocity held at receipt time');axs[1].plot(lt[q],lio_vx[q],'.',color=GRAY,ms=4,label='LIO at source time')
    axs[1].set(title='Timing comparison: source time versus arrival-time hold',ylabel='Rear Vx [m/s]');axs[1].legend()
    for k,c,label in [('bix',BLUE,'effective x'),('biy',GOLD,'effective y'),('biz',PINK,'effective z')]:axs[2].plot(t[moving],z[k][moving],color=c,label=label)
    axs[2].set(title='Slow effective acceleration residuals (not identified hardware bias)',ylabel='Residual [m/s²]')
    axs[2].legend()
    for ax in axs:ax.set_xlabel('IMU source time relative to established MPCC origin [s]')
    for ax in [axs[0],axs[2]]:ax.set_xticks([41.93,60,90,120,150,180,220.43])
    fig.savefig(a.output/'mpcc-vx.png',dpi=170);plt.close(fig)
    if a.integral:
        with np.load(a.integral) as archive:old={k:archive[k] for k in archive.files}
        ot=old['source_relative_s'];ok=old['running_valid']
        fig,axs=plt.subplots(2,1,figsize=(13,7),sharex=True,layout='constrained')
        axs[0].plot(t[moving],vx[moving],color=BLUE,label='Independent rear Vx');axs[0].plot(ot[ok],old['integral_vx'][ok],color=GOLD,ls='--',label='Previous pure rear ax integral')
        axs[0].set(ylabel='Speed [m/s]',title='MPCC: position constraints bound integral accumulation');axs[0].legend()
        axs[1].plot(t[moving],vx[moving],color=BLUE,label='Independent rear Vx');axs[1].plot(ot[ok],old['wheel_vx'][ok],color=GRAY,ls='--',label='Wheel speed: diagnostic comparison only')
        axs[1].set(xlabel='Source time relative to MPCC origin [s]',ylabel='Speed [m/s]',title='Wheel trace is absent from estimation, initialization and tuning');axs[1].legend()
        fig.savefig(a.output/'mpcc-baselines.png',dpi=170);plt.close(fig)
        summary['comparison_integral_input_sha256']=hashlib.sha256(a.integral.read_bytes()).hexdigest()
    fig,axs=plt.subplots(2,1,figsize=(13,6),sharex=True,layout='constrained')
    for ax,key,c in zip(axs,['imu','lio'],[BLUE,GOLD]):
        ts=d[key+'_ts'];tm=(ts[:,1]-origin)*1e-9;ok=(tm>=41.926416916)&(tm<=220.431558514);age=(ts[:,0]-ts[:,1])*1e-6
        ax.plot(tm[ok],age[ok],color=c,lw=.7);ax.set(ylabel='Receipt − source [ms]',title=f'{key}: observed arrival age during RUNNING')
    axs[-1].set_xlabel('Sensor source time relative to MPCC origin [s]');fig.savefig(a.output/'mpcc-input-age.png',dpi=170);plt.close(fig)
    summary['sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [a.trace,a.inputs/'mpcc/input.npz',Path(__file__)]}
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--trace',type=Path,required=True);p.add_argument('--integral',type=Path);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
