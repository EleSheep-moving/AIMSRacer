#!/usr/bin/env python3
"""Native matched-XY observer replay; measured occupancy, not grip or slip truth."""
from pathlib import Path
import argparse,hashlib,io,json,subprocess
import numpy as np
G=9.80665

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stats(a):
 a=np.asarray(a);return {'n':len(a),'mean':float(a.mean()),'std_ddof0':float(a.std()),'abs_p95':float(np.percentile(abs(a),95)),'abs_max':float(abs(a).max())}
def run(binary,payload,lateral=True):
 proc=subprocess.run([str(binary)]+(['--lateral'] if lateral else []),input=payload,text=True,capture_output=True,check=True)
 return np.genfromtxt(io.StringIO(proc.stdout),delimiter=',',names=True,dtype=None,encoding='utf8'),proc.stdout

def extract_events(input_dir):
 meta=json.loads((input_dir/'metadata.json').read_text());p=input_dir/'imu-quality-input.npz';assert sha(p)==meta['npz_sha256']
 with np.load(p) as a:z={k:a[k] for k in ['raw','raw_ts','corrected','corrected_ts','lio','lio_ts']}
 for k in ['corrected_ts','lio_ts']:assert np.all(np.diff(z[k][:,1])>0)
 assert not np.intersect1d(z['corrected_ts'][:,0],z['lio_ts'][:,0]).size
 events=[(int(r),int(s),'IMU',i) for i,(r,s) in enumerate(z['corrected_ts'])]+[(int(r),int(s),'LIO',i) for i,(r,s) in enumerate(z['lio_ts'])]
 events.sort(key=lambda e:e[0]);lines=[]
 for rec,stamp,kind,i in events:
  v=z['corrected'][i] if kind=='IMU' else z['lio'][i,[8,5,6,7]]
  lines.append(f'{rec}\t{kind} {stamp} '+' '.join(format(float(x),'.17g') for x in v)+'\n')
 return z,meta,lines

def audit(binary,input_dir,out,name,legacy=None,mpcc=False):
 z,meta,lines=extract_events(input_dir);(out/f'{name}-events.tsv').write_text(''.join(lines))
 payload=''.join(s.split('\t',1)[1] for s in lines);a,csv=run(binary,payload);(out/f'{name}-trace.csv').write_text(csv)
 base,_=run(binary,payload,False)
 equality={k:bool(np.array_equal(a[k],base[k])) for k in ['event','accepted','stamp_ns','valid','status','ax','jerk','variance']}
 assert all(equality.values()),equality
 result={'name':name,'original_bag':meta['source'],'input_sha256':sha(input_dir/'imu-quality-input.npz'),'events':len(a),'ax_identical_to_lateral_disabled':equality,
  'trace_sha256':sha(out/f'{name}-trace.csv'),'new_binary_sha256':sha(binary)}
 if legacy:
  old,_=run(legacy,payload,False)
  for k in ['event','accepted','stamp_ns','valid','status']:assert np.array_equal(a[k],old[k])
  result['old_accepted_binary_sha256']=sha(legacy)
  result['max_ax_difference_from_accepted_binary']=float(np.max(abs(a['ax']-old['ax'])))
  assert result['max_ax_difference_from_accepted_binary']<1e-12
 v=a[(a['event']=='IMU')&(a['accepted']==1)&(a['valid']==1)]
 assert np.all(v['lateral_valid']==1) and np.all(np.isfinite(v['ay']))
 assert np.array_equal(v['variance'],v['lateral_variance'])
 origin=1792002365474876930 if mpcc else int(z['raw_ts'][0,1]);t=(v['stamp_ns']-origin)*1e-9
 windows={'static':(13.065745826,41.425777542),'running':(41.926416916,220.431558514)} if mpcc else {'all_valid':(float(t.min()),float(t.max()))}
 result['valid_imu_n']=len(v);result['origin_ns']=origin;result['windows']={}
 for name_,(lo,hi) in windows.items():
  m=(t>=lo)&(t<=hi);r={'window_s':[lo,hi],'n':int(m.sum())}
  for k in ['force_y','gravity_y','tangential_y','centripetal_y','raw_ay','ay']:r[k]=stats(v[k][m])
  r['lateral_g']=stats(v['ay'][m]/G);r['longitudinal_g']=stats(v['ax'][m]/G);result['windows'][name_]=r
 result['interpretation']='native source-time valid outputs only; no independent truth; isotropic effective covariance; XY correlation unidentified'
 return result,(t,v)

def synthetic(binary,out):
 results=[];t=np.arange(0,10,.005);ns=10_000_000_000+np.rint(t*1e9).astype(np.int64)
 for f in [.5,1,2,3]:
  w=2*np.pi*f
  for kind in ['lateral_translation','yaw_pivot_about_rear']:
   rate=np.zeros(len(t)) if kind=='lateral_translation' else np.sin(w*t)
   alpha=np.zeros(len(t)) if kind=='lateral_translation' else w*np.cos(w*t)
   yaw=np.zeros(len(t)) if kind=='lateral_translation' else (1-np.cos(w*t))/w
   truth=np.sin(w*t) if kind=='lateral_translation' else np.zeros(len(t))
   lines=[]
   for i,(stamp,r,al,y,ay) in enumerate(zip(ns,rate,alpha,yaw,truth)):
    if i%20==0:lines.append(f'LIO {stamp} {np.cos(y/2):.17g} 0 0 {np.sin(y/2):.17g}\n')
    force=[-r*r*.3/G,(ay+al*.3)/G,1.]
    lines.append(f'IMU {stamp} '+ ' '.join(format(float(v),'.17g') for v in [*force,0.,0.,r])+'\n')
   a,_=run(binary,''.join(lines));a=a[a['event']=='IMU'];m=(t>=2)&(a['valid']==1)
   record={'kind':kind,'frequency_hz':f,'n':int(m.sum()),'true_rear_ay_amplitude':1. if kind=='lateral_translation' else 0.,
    'raw_reconstruction_error_rms':float(np.sqrt(np.mean((a['raw_ay'][m]-truth[m])**2))),
    'filtered_error_rms':float(np.sqrt(np.mean((a['ay'][m]-truth[m])**2))),
    'filtered_error_abs_p95_g':float(np.percentile(abs(a['ay'][m]-truth[m]),95)/G)}
   if kind=='lateral_translation':
    A=np.c_[np.sin(w*t[m]),np.cos(w*t[m]),np.ones(m.sum())];b=np.linalg.lstsq(A,a['ay'][m],rcond=None)[0]
    record.update(gain=float(np.hypot(*b[:2])),equivalent_filter_lag_ms=float(-np.arctan2(b[1],b[0])/w*1000))
    assert abs(record['gain']-1)<.1 and abs(record['equivalent_filter_lag_ms'])<20
   results.append(record)
 (out/'synthetic-physics.json').write_text(json.dumps(results,indent=2)+'\n');return results

def plots(out,t,z):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.font_manager import FontProperties
 font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
 plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
 m=(t>=41.926416916)&(t<=220.431558514);x=z['ay'][m]/G;y=z['ax'][m]/G
 fig=plt.figure(figsize=(13.5,9.3));gs=fig.add_gridspec(2,2,left=.085,right=.97,bottom=.18,top=.84,hspace=.55,wspace=.3)
 ax=fig.add_subplot(gs[:,0]);h=ax.hexbin(x,y,gridsize=60,mincnt=1,bins='log',cmap='Blues',linewidths=0)
 ax.set(xlabel='横向 G：ay / g0（左为正）',ylabel='纵向 G：ax / g0（加速为正）',aspect='equal')
 ax.set_title('RUNNING 加速度采样分布',loc='left',fontsize=12,fontweight='bold',pad=12)
 ax.axhline(0,color='#555',lw=.7,ls='--');ax.axvline(0,color='#555',lw=.7,ls='--')
 fig.colorbar(h,ax=ax,pad=.03,shrink=.7,label='样本数 / 格（对数色阶）')
 for row,(lo,hi),title in [(0,(20.,22.),'静止局部'),(1,(52.7,54.3),'快速换向局部')]:
  a=fig.add_subplot(gs[row,1]);mask=(t>=lo)&(t<=hi)
  a.plot(t[mask],z['force_y'][mask]/G,color='#9AA7B2',alpha=.75,lw=.9,label='IMU 比力 y')
  a.plot(t[mask],z['raw_ay'][mask]/G,color='#D47B13',alpha=.75,lw=.9,label='重力＋杆臂补偿后')
  a.plot(t[mask],z['ay'][mask]/G,color='#245F87',lw=1.5,label='最终横向 G')
  a.axhline(0,color='#555',lw=.7,ls='--');a.set(xlim=(lo,hi),xlabel='IMU 源时间，相对录包起点 / s',ylabel='ay / g0')
  a.set_title(title,loc='left',fontsize=12,fontweight='bold',pad=12);a.grid(color='#E4E7EB')
  if row==0:a.legend(loc='upper right',fontsize=9)
 fig.suptitle('后轴横向 G 与 G-G 观测：最新 MPCC',x=.085,y=.97,ha='left',fontsize=19,fontweight='bold')
 fig.text(.085,.91,'同一 IMU 源时间、相同 XY 滤波增益 · 2026-10-15 022604 MPCC · 无轮速／导航 EKF／电流输入',fontsize=11,color='#444')
 fig.text(.085,.11,'参考点：后轴 base_link；g0=9.80665 m/s²。左图是采样占用分布，未拟合附着极限或摩擦圆。',fontsize=10.5,color='#444')
 fig.text(.085,.07,'右侧子图纵轴范围不同；曲线无人工移位、去均值或额外绘图滤波。补偿前后物理定义不同，差值不是精度误差。',fontsize=10.5,color='#444')
 for ext in ['png','pdf']:fig.savefig(out/f'lateral-g-and-gg.{ext}',dpi=180,facecolor='white')
 plt.close(fig)
 fig,axes=plt.subplots(2,1,figsize=(13,7.5));fig.subplots_adjust(left=.09,right=.98,top=.85,bottom=.20,hspace=.5)
 mask=(t>=52.7)&(t<=54.3)
 for k,label,color in [('force_y','IMU 比力 y','#7991A2'),('gravity_y','重力项 +g_y','#968140'),('tangential_y','切向项 -[α×l]y','#C87519'),('centripetal_y','向心项 -[ω×(ω×l)]y','#86718D')]:
  sign=-1 if k in ['tangential_y','centripetal_y'] else 1
  axes[0].plot(t[mask],sign*z[k][mask]/G,label=label,color=color,lw=1)
 axes[0].legend(ncol=2,fontsize=9);axes[0].set_ylabel('各项 / g0')
 axes[1].plot(t[mask],z['raw_ay'][mask]/G,color='#C87519',lw=1,label='补偿后，未滤波')
 axes[1].plot(t[mask],z['ay'][mask]/G,color='#245F87',lw=1.6,label='最终横向 G');axes[1].legend(fontsize=10);axes[1].set_ylabel('ay / g0')
 for ax in axes:ax.set(xlim=(52.7,54.3),xlabel='IMU 源时间，相对录包起点 / s');ax.grid(color='#E4E7EB')
 fig.suptitle('快速换向：横向重力与杆臂补偿分项',x=.09,y=.97,ha='left',fontweight='bold',fontsize=17)
 fig.text(.09,.10,'总重建 = 比力 + 重力 - 切向项 - 向心项；采用尾随 25ms 三轴 gyro 回归。',fontsize=11,color='#444')
 fig.text(.09,.055,'源时间标签一致不代表各项相位相同；动态补偿误差需要合成物理试验和后续 Vicon 独立核验。',fontsize=10.5,color='#444')
 fig.savefig(out/'lateral-compensation.png',dpi=180,facecolor='white');plt.close(fig)

def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--mpcc-input',type=Path,required=True)
 p.add_argument('--legacy-binary',type=Path);p.add_argument('--bc-root',type=Path);p.add_argument('--output',type=Path,required=True)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 syn=synthetic(a.binary,a.output);r,(t,z)=audit(a.binary,a.mpcc_input,a.output,'mpcc',a.legacy_binary,True);results=[r];plots(a.output,t,z)
 if a.bc_root:
  for run_ in ['233659-B-full','015017-C-resume-minus60A-odom30cm']:
   r,_=audit(a.binary,a.bc_root/run_,a.output,run_);results.append(r)
 result={'runs':results,'synthetic':syn,'renderer_sha256':sha(__file__),'g0':G,'independent_truth':False,'reference_point':'rear axle base_link','covariance_xy_correlation':'unidentified; diagonal effective model only'}
 (a.output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({'runs':results,'synthetic':syn},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
