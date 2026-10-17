"""Offline source-time FastLIO comparison; fixed rigid alignment only.

Mount quaternion R_BI maps IMU-coordinate vectors into body coordinates.
No command publisher or online estimator is present in this module.
"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .analyze import load_poses
from .reference import (build_reference, error_statistics, map_time,
                        sample_reference, vector, waveform_offset)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read(path):
    if not Path(path).exists():
        return []
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def _provenance(folder, config):
    folder = Path(folder)
    metadata = json.loads((folder/'metadata.json').read_text()) if (folder/'metadata.json').exists() else {}
    for name, expected in metadata.get('data_sha256', {}).items():
        path = folder/name
        if path.parent != folder or _sha(path) != expected:
            raise ValueError('input hash mismatch: '+name)
    return dict(input_files={p.name: _sha(p) for p in folder.iterdir() if p.is_file() and p.suffix in ('.csv','.json','.jsonl')},
                config_sha256=hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
                programs={p.name:_sha(p) for p in Path(__file__).parent.glob('*.py')},
                extraction_metadata=metadata)


def _qualification(config, metadata):
    warnings=[]
    if not config['clock'].get('verified'): warnings.append('clock_mapping_unverified')
    if not config.get('body_orientation_measured'): warnings.append('body_orientation_not_measured')
    if not config['references']['rear_axle'].get('measured'): warnings.append('rear_origin_not_measured')
    if config['vicon'].get('tracking_semantics') not in ('csv_valid_flag','bridge_publishes_valid_only'):
        warnings.append('tracking_semantics_unverified')
    if not config['vicon'].get('bridge_timestamp_semantics'): warnings.append('source_timestamp_semantics_unknown')
    if config['vicon'].get('prediction_or_retiming_enabled') is not False: warnings.append('reference_retiming_unknown_or_enabled')
    if metadata.get('synthetic'): warnings.append('synthetic_data')
    return dict(qualified=not warnings, warnings=warnings, clock_uncertainty_ns=float(config['clock']['uncertainty_ns']))


def _pose_geometry(poses, config):
    ns=map_time(poses['source_ns'], config['clock'])
    unit=config['vicon'].get('position_unit')
    if unit not in ('m','mm'): raise ValueError('explicit Vicon position unit required')
    p=np.asarray(poses['position'],float)*(1 if unit=='m' else .001)
    q=np.asarray(poses['quaternion'],float)
    good=np.asarray(poses['valid'],bool)&np.isfinite(p).all(axis=1)&np.isfinite(q).all(axis=1)&(abs(np.linalg.norm(q,axis=1)-1)<1e-3)
    safe=q.copy(); safe[~good]=[0,0,0,1]
    marker=Rotation.from_quat(safe)
    body=marker*Rotation.from_quat(vector(config.get('body_orientation_in_marker_xyzw'),4,'body orientation'))
    lever=vector(config['references']['rear_axle'].get('origin_in_marker_m'),3,'rear origin')
    return ns,p+marker.apply(lever),body,good


def _strict_ns(rows, label):
    ns=np.array([int(r['source_ns']) for r in rows],np.int64)
    if len(ns)>1 and np.any(np.diff(ns)<=0): raise ValueError(label+' source duplicate/backward; split clock epochs')
    if len(ns) and np.any(ns<=0): raise ValueError(label+' missing source timestamp')
    return ns


def _interpolate(ns, values, target, good, gap):
    """Linear interpolation strictly inside contiguous valid sample pairs."""
    ns=np.asarray(ns,np.int64); target=np.asarray(target,np.int64)
    values=np.asarray(values,float)
    out=np.full((len(target),)+values.shape[1:],np.nan)
    if len(ns)<2: return out,np.zeros(len(target),bool)
    idx=np.clip(np.searchsorted(ns,target,side='right')-1,0,len(ns)-2)
    valid=(target>=ns[0])&(target<=ns[-1])&good[idx]&good[idx+1]&((ns[idx+1]-ns[idx])*1e-9<=gap)
    # Exact valid samples do not need a valid neighbor.
    exact=np.searchsorted(ns,target); exists=exact<len(ns)
    exact=np.minimum(exact,len(ns)-1); exact_ok=exists&(ns[exact]==target)&good[exact]
    valid |= exact_ok
    w=((target-ns[idx])/(ns[idx+1]-ns[idx])).reshape((-1,)+(1,)*(values.ndim-1))
    out[valid]=(values[idx]+w*(values[idx+1]-values[idx]))[valid]
    out[exact_ok]=values[exact[exact_ok]]
    return out,valid


def _sample_pose(ns,p,rot,good,target,gap):
    pos,valid=_interpolate(ns,p,target,good,gap)
    quat=np.tile([0.,0.,0.,1.],(len(target),1))
    if len(ns)>=2:
        idx=np.clip(np.searchsorted(ns,target,side='right')-1,0,len(ns)-2)
        for i in np.flatnonzero(valid):
            if ns[idx[i]]==target[i]: quat[i]=rot[idx[i]].as_quat()
            elif target[i]==ns[-1]: quat[i]=rot[-1].as_quat()
            else:
                # Relative seconds preserve source timestamps at nanosecond scale.
                quat[i]=Slerp([0.,(ns[idx[i]+1]-ns[idx[i]])*1e-9],rot[idx[i]:idx[i]+2])([(target[i]-ns[idx[i]])*1e-9]).as_quat()[0]
    return pos,Rotation.from_quat(quat),valid


def _alignment(settings, ns, p, rot, truth_p, truth_rot, matched, origin, vicon, pose_gap):
    spec=settings.get('world_alignment',{})
    q,t=spec.get('rotation_xyzw'),spec.get('translation_m')
    if (q is None)!=(t is None): raise ValueError('alignment rotation and translation both required')
    if q is not None:
        r=Rotation.from_quat(vector(q,4,'alignment rotation')); trans=vector(t,3,'alignment translation')
        return r,trans,dict(method='explicit_fixed_transform',rotation_xyzw=r.as_quat().tolist(),translation_m=trans.tolist())
    window=spec.get('static_window_s')
    if not isinstance(window,(list,tuple)) or len(window)!=2 or not np.isfinite(window).all() or window[1]<=window[0]:
        raise ValueError('fixed alignment or designated static window required; whole trajectory fitting forbidden')
    start=origin+int(round(window[0]*1e9));end=origin+int(round(window[1]*1e9))
    def support(source,valid,gap,label):
        # Inspect original samples, including the samples bracketing both
        # boundaries. Merely having three samples somewhere inside a window
        # does not establish complete coverage of that window.
        if len(source)<3 or source[0]>start or source[-1]<end:
            raise ValueError('static alignment window lacks full '+label+' coverage')
        lo=max(0,int(np.searchsorted(source,start,side='right'))-1)
        hi=min(len(source)-1,int(np.searchsorted(source,end)))
        ids=np.arange(lo,hi+1)
        if len(ids)<3 or not valid[ids].all() or np.any(np.diff(source[ids])*1e-9>gap):
            raise ValueError('static alignment window has '+label+' gap/occlusion/invalid pose')
        return ids
    ids=support(ns,matched,float(settings.get('max_gap_s',.25)),'LIO')
    vn,vp,vr,vg=vicon
    raw_ids=support(vn,vg,pose_gap,'raw Vicon')
    max_translation=float(spec.get('max_static_translation_m',.02));max_rotation=float(spec.get('max_static_rotation_rad',.03))
    if not np.isfinite([max_translation,max_rotation]).all() or min(max_translation,max_rotation)<=0:
        raise ValueError('invalid static motion thresholds')
    # The raw Vicon stream is intentionally NOT downsampled to LIO query
    # times: resolved motion between LIO samples can alias to a false stop.
    for position,rotation,indices in [(p,rot,ids),(vp,vr,raw_ids)]:
        if np.max(np.linalg.norm(position[indices]-position[indices[0]],axis=1))>max_translation or np.max((rotation[indices[0]].inv()*rotation[indices]).magnitude())>max_rotation:
            raise ValueError('motion in designated static alignment window')
    # Only pairs actually inside the designated window determine the fixed
    # transform; bracketing samples above serve coverage qualification only.
    fit_ids=ids[(ns[ids]>=start)&(ns[ids]<=end)]
    if len(fit_ids)<3:raise ValueError('static alignment window has too few interior LIO samples')
    r=(truth_rot[fit_ids]*rot[fit_ids].inv()).mean(); trans=np.mean(truth_p[fit_ids]-r.apply(p[fit_ids]),axis=0)
    return r,trans,dict(method='designated_static_window',static_window_s=list(window),samples=len(fit_ids),
        static_window_validation=dict(full_window_coverage=True,raw_vicon_samples_including_boundary_brackets=len(raw_ids),
            lio_samples_including_boundary_brackets=len(ids),all_raw_vicon_samples_stationary=True,
            lio_gap_limit_s=float(settings.get('max_gap_s',.25)),vicon_gap_limit_s=float(pose_gap)),
        rotation_xyzw=r.as_quat().tolist(),translation_m=trans.tolist())



def _drift(t,error):
    if len(t)<3 or np.ptp(t)<=0:return dict(n=len(t))
    x=t-np.mean(t)
    return dict(n=len(t),linear_growth_m_per_s=float(x@(error-np.mean(error))/(x@x)),
                first_error_m=float(error[0]),last_error_m=float(error[-1]),end_minus_start_m=float(error[-1]-error[0]),
                interpretation='unshifted error magnitude slope; no trajectory fit removed')


def analyze(folder:Path, config:dict, output:Path)->dict:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder,output=Path(folder),Path(output)
    if output.exists():raise ValueError('output directory must not already exist')
    provenance=_provenance(folder,config); poses=load_poses(folder/'poses.csv')
    vn,vp,vr,vg=_pose_geometry(poses,config); origin=int(vn[0]);settings=config['fastlio']
    gap=float(settings.get('max_gap_s',.25)); pose_gap=float(config['differentiation']['max_gap_s'])
    if not np.isfinite(gap) or gap<=0:raise ValueError('invalid FastLIO gap')
    mount=Rotation.from_quat(vector(settings.get('imu_mount_body_xyzw'),4,'IMU mount'))
    lever=vector(settings.get('rear_to_imu_body_m'),3,'rear to IMU')
    refs=build_reference(poses,config,float(config['differentiation']['primary_window_s']))['rear_axle']
    rows=_read(folder/'odometry.csv'); gyro=_read(folder/'gyro.csv')
    gn=_strict_ns(gyro,'gyro') if gyro else np.array([],np.int64)
    gv=mount.apply(np.array([[float(r[k]) for k in ('wx','wy','wz')] for r in gyro])) if gyro else np.empty((0,3))
    result=dict(physical_acceptance=False,source_time_errors_without_curve_shift=True,full_trajectory_alignment=False,
                reference_qualification=_qualification(config,provenance['extraction_metadata']),provenance=provenance,
                reference_velocity_uses_centered_offline_derivatives=True,streams={},alignment={})
    if any(s.get('pose_reference')=='imu' for s in settings.get('streams',[])) and not settings.get('geometry_measured'):
        result['reference_qualification']['warnings'].append('imu_geometry_not_measured')
        result['reference_qualification']['qualified']=False
    plotted=[];aligned=[]
    for spec in settings.get('streams',[]):
        name=spec['name']; current=[r for r in rows if r['stream'] in (name,spec.get('topic'))]
        if not current:
            result['streams'][name]=dict(available=False,reason='no_odometry_samples');continue
        ns=_strict_ns(current,name)
        p=np.array([[float(r[k]) for k in ('px','py','pz')] for r in current]);q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in current])
        finite=np.isfinite(p).all(axis=1)&np.isfinite(q).all(axis=1)&(abs(np.linalg.norm(q,axis=1)-1)<1e-3)
        safe=q.copy();safe[~finite]=[0,0,0,1]; rot=Rotation.from_quat(safe)
        raw=spec['pose_reference']=='imu'
        if spec['pose_reference'] not in ('imu','rear_axle'):raise ValueError('unknown FastLIO pose reference')
        if raw:
            rot=rot*mount.inv(); p=p-rot.apply(lever)
        tp,tr,matched=_sample_pose(vn,vp,vr,vg,ns,pose_gap);matched &= finite
        # Determine once from the first available configured stream, then use
        # that transform unchanged for every other stream and evaluation time.
        if not result['alignment']:
            ar,at,description=_alignment(settings,ns,p,rot,tp,tr,matched,origin,(vn,vp,vr,vg),pose_gap)
            description['determined_from_stream']=name
            result['alignment']=description
        else:
            ar=Rotation.from_quat(result['alignment']['rotation_xyzw'])
            at=np.asarray(result['alignment']['translation_m'])
        p=ar.apply(p)+at; rot=ar*rot
        pe=np.linalg.norm(p-tp,axis=1);ae=(tr.inv()*rot).magnitude();t=(ns-origin)*1e-9
        entry=dict(available=True,published_samples=len(ns),matched_samples=int(matched.sum()),
            position_error_m=error_statistics(pe[matched]),orientation_error_rad=error_statistics(ae[matched]),
            position_axis_errors_m={k:error_statistics((p-tp)[matched,i]) for i,k in enumerate('xyz')},
            drift_growth=_drift(t[matched],pe[matched]),source_gaps_above_threshold=int(np.sum(np.diff(ns)*1e-9>gap)),
            velocity={},pose_reference='rear_axle',angular_velocity_valid=bool(spec.get('angular_velocity_valid',False)),
            frame_ids=sorted(set(r['frame_id'] for r in current)),child_frame_ids=sorted(set(r['child_frame_id'] for r in current)))
        velocity=np.array([[float(r[k]) for k in ('vx','vy','vz')] for r in current]);velocity_good=np.isfinite(velocity).all(axis=1)
        frame=spec.get('velocity_frame')
        if frame not in ('body','world'): raise ValueError('explicit FastLIO velocity_frame required')
        if raw or frame=='world':velocity_good &= finite
        if frame=='world':velocity=rot.inv().apply(ar.apply(velocity))
        elif raw:velocity=mount.apply(velocity)
        if raw:
            omega,ok=_interpolate(gn,gv,ns,np.isfinite(gv).all(axis=1),float(settings.get('gyro_max_gap_s',.04)))
            velocity_good &= ok
            velocity=velocity-np.cross(omega,lever)
            if not gyro:entry['velocity']=dict(available=False,reason='missing_synced_corrected_gyro_for_lever_rate')
        target=sample_reference(refs,ns)
        if not entry['velocity']:
            entry['velocity']=dict(available=bool(velocity_good.any()),reason=None if velocity_good.any() else 'no_synchronized_gyro_or_finite_velocity_support')
            for i,k in enumerate(('vx','vy')):
                good=velocity_good&np.isfinite(target[:,i]);entry['velocity'][k]=error_statistics((velocity[:,i]-target[:,i])[good])
                # Diagnostic only. Primary errors above remain at original times.
                vv=velocity[:,i].copy();vv[~velocity_good]=np.nan
                entry['velocity'][k]['waveform_offset']=waveform_offset(ns,vv,refs,i,config['comparison'],config['clock']['uncertainty_ns'])
        if spec.get('angular_velocity_valid',False):
            omega=np.array([[float(r[k]) for k in ('wx','wy','wz')] for r in current])
            angular_frame=spec.get('angular_velocity_frame','body')
            if angular_frame=='world':omega=rot.inv().apply(ar.apply(omega))
            elif angular_frame=='body':
                if raw:omega=mount.apply(omega)
            else:raise ValueError('invalid angular velocity frame')
            # Sanitized identity quaternions exist only to make vectorized
            # Rotation construction safe; they never qualify measurements.
            angular_good=finite&np.isfinite(omega).all(axis=1)
            good=angular_good&np.isfinite(target[:,4])
            entry['yaw_rate']=error_statistics((omega[:,2]-target[:,4])[good])
            angular_values=omega[:,2].copy();angular_values[~angular_good]=np.nan
            entry['yaw_rate']['waveform_offset']=waveform_offset(ns,angular_values,refs,4,config['comparison'],config['clock']['uncertainty_ns'])
        else:entry['yaw_rate']=dict(n=0,reason='angular_velocity_not_declared_valid')
        for i in np.flatnonzero(matched):
            aligned.append([int(ns[i]),name,*p[i],*tp[i],float(pe[i]),float(ae[i])])
        plotted.append((name,t,np.where(matched,pe,np.nan),np.where(matched,ae,np.nan)))
        result['streams'][name]=entry
    output.mkdir(parents=True)
    with (output/'aligned.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['source_ns','stream','px','py','pz','vicon_px','vicon_py','vicon_pz','position_error_m','orientation_error_rad']);writer.writerows(aligned)
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    for name,t,pe,ae in plotted:
        axes[0].plot(t,pe,label=name);axes[1].plot(t,ae,label=name)
    for ax,label in zip(axes,['Position error / m','Orientation geodesic error / rad']):
        ax.set_ylabel(label);ax.grid(True)
        if plotted:ax.legend()
    axes[-1].set_xlabel('Seconds from first mapped Vicon pose');fig.tight_layout();fig.savefig(output/'fastlio.png',dpi=140);plt.close(fig)
    (output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# FastLIO 动捕对照','', '按源时间匹配后轴位姿，主要误差不做曲线平移；仅使用显式固定外参或指定静止窗口。没有全轨迹配准。',
           '原始 IMU 位姿先应用安装旋转，再扣除后轴至 IMU 杆臂；原始速度必须有同步的修正陀螺仪以扣除角速度叉乘杆臂。缺失时不假设为零。',
           '速度参考为离线中心窗口求导；波形偏移只作诊断，不等于物理延迟。',
           '标定／时间资格：`'+json.dumps(result['reference_qualification'],ensure_ascii=False)+'`','',
           '| 流 | 匹配数 | 位置 RMSE m | 姿态 RMSE rad | vx RMSE | vy RMSE |','| --- | ---: | ---: | ---: | ---: | ---: |']
    for name,s in result['streams'].items():
        if not s.get('available'):lines.append(f'| {name} | 无输入 | — | — | — | — |');continue
        def metric(d):return f"{d['rmse']:.6g}" if d.get('n') else '无支持'
        lines.append(f"| {name} | {s['matched_samples']} | {metric(s['position_error_m'])} | {metric(s['orientation_error_rad'])} | {metric(s['velocity'].get('vx',{}))} | {metric(s['velocity'].get('vy',{}))} |")
    lines+=['','漂移增长、缺测、波形偏移和输入／配置／程序哈希详见 [metrics.json](metrics.json)。本报告不自动判定实车验收通过。','','![FastLIO](fastlio.png)','']
    (output/'report.zh-CN.md').write_text('\n'.join(lines))
    return result
