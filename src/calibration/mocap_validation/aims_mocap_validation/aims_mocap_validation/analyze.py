"""Five-state source-time comparison. No estimate is used to construct the reference."""
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from .reference import QUANTITIES, build_reference, error_statistics, map_time, sample_reference, waveform_offset


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_poses(path):
    with Path(path).open() as stream: rows = list(csv.DictReader(stream))
    required = ['source_ns','receive_ns','px','py','pz','qx','qy','qz','qw','valid']
    if not rows or any(k not in rows[0] for k in required):
        raise ValueError('pose CSV requires '+','.join(required))
    flags = [str(row['valid']).lower() for row in rows]
    if any(flag not in ('true','false','0','1') for flag in flags): raise ValueError('invalid tracking valid flag')
    return dict(source_ns=np.asarray([int(row['source_ns']) for row in rows],np.int64),
        receive_ns=np.asarray([int(row['receive_ns']) for row in rows],np.int64),
        position=np.asarray([[float(row[k]) for k in ['px','py','pz']] for row in rows]),
        quaternion=np.asarray([[float(row[k]) for k in ['qx','qy','qz','qw']] for row in rows]),
        valid=np.asarray([flag in ('true','1') for flag in flags]))


def age_statistics(age_ms):
    x = np.asarray(age_ms, float)
    return dict(n=len(x),mean=float(np.mean(x)),p50=float(np.median(x)),
                p95=float(np.percentile(x,95)),max=float(np.max(x)),negative=int(np.sum(x<0)))


REGIMES = ('static', 'straight', 'turn', 's_turn')
REGIME_DEFAULTS = dict(static_speed_max_mps=.05,static_yaw_rate_max_rad_s=.03,
    static_acceleration_max_mps2=.2,moving_speed_min_mps=.2,
    straight_yaw_rate_max_rad_s=.05,turn_yaw_rate_min_rad_s=.1,
    s_turn_min_lobe_s=.2,s_turn_max_transition_s=.5,s_turn_max_pair_span_s=4.,
    min_samples=20,min_supported_span_s=.2,max_gap_s=.04)


def classify_regimes(segments, options=None):
    """Describe primary rear-reference motion without reading any estimate.

    S-turn means two sustained opposite-sign yaw lobes with a bounded
    transition and pair span, within contiguous moving Vicon support. These
    descriptive defaults are not physical acceptance limits. Categorical
    labels are supported only between a run's first and last grid samples.
    """
    settings=dict(REGIME_DEFAULTS,**(options or {}))
    for key in REGIME_DEFAULTS:
        if not np.isfinite(settings[key]) or settings[key]<=0:
            raise ValueError('invalid state segmentation '+key)
    if int(settings['min_samples'])!=settings['min_samples'] or settings['min_samples']<2:
        raise ValueError('state segmentation min_samples must be an integer >=2')
    if not (settings['static_speed_max_mps']<settings['moving_speed_min_mps'] and
            settings['static_yaw_rate_max_rad_s']<=settings['straight_yaw_rate_max_rad_s']<settings['turn_yaw_rate_min_rad_s']):
        raise ValueError('state segmentation thresholds overlap')
    runs=[];unclassified=0;unavailable=0;valid_count=0
    for segment in segments:
        ns=np.asarray(segment['ns'],np.int64);values=np.asarray(segment['values'],float)
        if len(ns)>1 and np.any(np.diff(ns)<=0):raise ValueError('regime reference timestamps must increase')
        valid=np.asarray(segment['valid'],bool)&np.isfinite(values).all(axis=1)
        unavailable+=int((~valid).sum());valid_count+=int(valid.sum())
        ids=np.flatnonzero(valid)
        breaks=(np.diff(ids)!=1)|((np.diff(ns[ids])*1e-9)>settings['max_gap_s'])
        for part in np.split(ids,np.flatnonzero(breaks)+1):
            if not len(part):continue
            t=ns[part];v=values[part];speed=np.linalg.norm(v[:,:2],axis=1);yaw=v[:,4]
            moving=speed>=settings['moving_speed_min_mps']
            labels=np.full(len(part),'unclassified',dtype=object)
            static=(speed<=settings['static_speed_max_mps'])&(abs(yaw)<=settings['static_yaw_rate_max_rad_s'])&(np.linalg.norm(v[:,2:4],axis=1)<=settings['static_acceleration_max_mps2'])
            labels[static]='static'
            labels[moving&(abs(yaw)<=settings['straight_yaw_rate_max_rad_s'])]='straight'
            turn=moving&(abs(yaw)>=settings['turn_yaw_rate_min_rad_s'])
            labels[turn]='turn'
            signs=np.where(turn,np.sign(yaw),0)
            lobes=[]
            for lobe in np.split(np.arange(len(part)),np.flatnonzero(np.diff(signs)!=0)+1):
                if len(lobe) and signs[lobe[0]] and (t[lobe[-1]]-t[lobe[0]])*1e-9+1e-12>=settings['s_turn_min_lobe_s']:
                    lobes.append(lobe)
            for left,right in zip(lobes,lobes[1:]):
                transition=(t[right[0]]-t[left[-1]])*1e-9
                span=(t[right[-1]]-t[left[0]])*1e-9
                if signs[left[0]]!=signs[right[0]] and transition<=settings['s_turn_max_transition_s'] and span<=settings['s_turn_max_pair_span_s'] and moving[left[0]:right[-1]+1].all():
                    labels[left[0]:right[-1]+1]='s_turn'
            unclassified+=int(np.sum(labels=='unclassified'))
            for run in np.split(np.arange(len(part)),np.flatnonzero(labels[1:]!=labels[:-1])+1):
                if len(run) and labels[run[0]] in REGIMES:
                    runs.append(dict(regime=str(labels[run[0]]),start_ns=int(t[run[0]]),end_ns=int(t[run[-1]]),samples=len(run)))
    support={}
    for name in REGIMES:
        selected=[run for run in runs if run['regime']==name]
        samples=sum(run['samples'] for run in selected)
        span=sum((run['end_ns']-run['start_ns'])*1e-9 for run in selected)
        status='missing' if not samples else 'insufficient_support' if samples<settings['min_samples'] or span<settings['min_supported_span_s'] else 'available'
        support[name]=dict(status=status,samples=samples,contiguous_runs=len(selected),supported_span_s=float(span))
    return dict(source='primary_rear_axle_vicon_reference_only',thresholds_are_acceptance_limits=False,
        settings=settings,runs=runs,support=support,unclassified_samples=unclassified,
        unavailable_reference_grid_samples=unavailable,valid_reference_grid_samples=valid_count,
        label_sampling='closed first-to-last grid-sample intervals within each contiguous same-label run; no extrapolation across transitions or gaps',
        s_turn_definition='two sustained opposite-sign moving yaw lobes, bounded transition and pair span; full pair has contiguous moving reference support')


def _regime_ids(classification,ns,regime):
    ids=np.full(len(ns),-1,dtype=int)
    for i,run in enumerate(classification['runs']):
        if run['regime']==regime:ids[(ns>=run['start_ns'])&(ns<=run['end_ns'])]=i
    return ids


def _evaluation_mask(part,ns,origin,classification):
    mask=np.ones(len(ns),bool);t=(ns-origin)*1e-9
    if part.get('start_s') is not None:mask &= t>=float(part['start_s'])
    if part.get('end_s') is not None:mask &= t<float(part['end_s'])
    if part.get('regime'):mask &= _regime_ids(classification,ns,part['regime'])>=0
    return mask


def analyze(folder, config, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    folder, output = Path(folder), Path(output)
    if output.exists(): raise ValueError('output directory must not already exist')
    metadata = json.loads((folder/'metadata.json').read_text()) if (folder/'metadata.json').exists() else {}
    for name, expected in metadata.get('data_sha256',{}).items():
        if sha(folder/name) != expected: raise ValueError('input hash mismatch: '+name)
    poses = load_poses(folder/'poses.csv')
    mapped = map_time(poses['source_ns'],config['clock']); origin = int(mapped[0])
    windows = list(map(float,config['differentiation']['windows_s']))
    primary = float(config['differentiation']['primary_window_s'])
    estimate_gap=float(config['comparison']['estimate_max_gap_s'])
    if not np.isfinite(estimate_gap) or estimate_gap<=0:raise ValueError('invalid estimate gap threshold')
    if primary not in windows: raise ValueError('primary derivative window must occur in windows_s')
    references = {str(window):build_reference(poses,config,window) for window in windows}
    groups, rejected = defaultdict(list), defaultdict(int)
    stream_gaps={item['name']:float(item.get('max_gap_s',estimate_gap)) for item in config.get('streams',[])}
    if any(not np.isfinite(gap) or gap<=0 for gap in stream_gaps.values()):raise ValueError('invalid stream gap threshold')
    with (folder/'estimates.csv').open() as stream:
        for row in csv.DictReader(stream):
            point, name = row['reference'], row['quantity']
            if point not in config['references'] or name not in QUANTITIES:
                raise ValueError('unknown estimate reference/quantity')
            expected_frame = config['references'][point].get('frame_id','base_link' if point=='rear_axle' else 'center_of_mass')
            key = (row['stream'],point,name)
            if row['frame_id'] != expected_frame: rejected['wrong_frame'] += 1; continue
            value = float(row['value'])
            if not np.isfinite(value): rejected['nonfinite'] += 1; continue
            item = (int(row['source_ns']),int(row['receive_ns']),value)
            if groups[key] and item[0] <= groups[key][-1][0]:
                raise ValueError('estimate source duplicate/backward: split clock epochs before analysis')
            groups[key].append(item)
    if not groups: raise ValueError('no finite correctly framed estimates')
    segmentation_options=dict(max_gap_s=float(config['differentiation']['max_gap_s']))
    segmentation_options.update(config.get('state_segmentation',{}))
    classification=classify_regimes(references[str(primary)]['rear_axle'],segmentation_options)
    evaluation = [dict(part) for part in (config.get('evaluation_windows') or [dict(name='whole',start_s=None,end_s=None,role='evaluation')])]
    for regime in REGIMES:
        name='regime_'+regime
        if any(part['name']==name for part in evaluation):raise ValueError('evaluation window uses reserved regime name: '+name)
        evaluation.append(dict(name=name,regime=regime,role='descriptive_reference_regime'))
    result = dict(synthetic=bool(metadata.get('synthetic',False)), independent_sensor_reference='Vicon pose differentiation',
        physical_acceptance=False, source_time_errors_without_curve_shift=True,
        derivative_uses_future_samples=True, common_error_support_across_derivative_windows=True,
        reference_origin_mapped_ns=origin, primary_window_s=primary, evaluation_windows=evaluation,
        regime_classification=classification,
        reference_qualification={}, comparisons={}, rejected_estimates=dict(rejected),
        source_counts=dict(vicon_pose=len(mapped),tracking_valid=int(np.sum(poses['valid']))),
        reference_support={window:{point:[dict(window_samples=s['window_samples'],dt_s=s['dt_s'],
            sample_span_s=s['sample_span_s'],effective_window_s=s['effective_window_s'],
            valid_samples=int(np.sum(s['valid'])),grid_samples=len(s['ns'])) for s in segments]
            for point,segments in points.items()} for window,points in references.items()},
        vicon_receive_minus_mapped_source_ms=age_statistics((poses['receive_ns']-mapped)*1e-6),
        provenance=dict(input_files={p.name:sha(p) for p in folder.iterdir() if p.is_file() and p.suffix in ('.csv','.json','.jsonl')},
                        programs={p.name:sha(p) for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('test_')},
                        config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
                        extraction_metadata=metadata))
    for point,item in config['references'].items():
        warnings = []
        if not config['clock'].get('verified'): warnings.append('clock_mapping_unverified')
        if not config.get('body_orientation_measured'): warnings.append('body_orientation_not_measured')
        if not item.get('measured'): warnings.append('reference_point_not_measured')
        if config['vicon'].get('tracking_semantics') not in ('bridge_publishes_valid_only','csv_valid_flag'):
            warnings.append('tracking_validity_semantics_unverified')
        if not config['vicon'].get('bridge_timestamp_semantics'): warnings.append('bridge_timestamp_semantics_unknown')
        if config['vicon'].get('prediction_or_retiming_enabled') is not False:
            warnings.append('reference_prediction_or_retiming_enabled_or_unknown')
        if result['synthetic']: warnings.append('synthetic_data')
        result['reference_qualification'][point] = dict(qualified=not warnings,warnings=warnings,
            clock_uncertainty_ns=config['clock']['uncertainty_ns'])
    output.mkdir(parents=True)
    with (output/'regime_labels.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['regime','start_source_ns','end_source_ns','primary_reference_samples'])
        for run in classification['runs']:writer.writerow([run['regime'],run['start_ns'],run['end_ns'],run['samples']])
    arrays = {}
    for window,points in references.items():
        for point,segments in points.items():
            for i,s in enumerate(segments):
                for key in ['ns','values','valid']:
                    arrays[f'w{window}_{point}_segment{i}_{key}'] = s[key]
    np.savez_compressed(output/'reference.npz',**arrays)
    figures = {point:plt.subplots(5,1,figsize=(14,12),sharex=True) for point in config['references']}
    for point,(fig,axes) in figures.items():
        for segment in references[str(primary)][point]:
            for j,ax in enumerate(axes):
                ax.plot((segment['ns']-origin)*1e-9,segment['values'][:,j],color='black',lw=1.1,
                        label='Vicon centered reference' if segment is references[str(primary)][point][0] else None)
    available = set()
    with (output/'aligned.csv').open('w') as stream:
        writer = csv.writer(stream);writer.writerow(['source_ns','receive_ns','stream','reference','quantity','estimate','vicon_reference','error'])
        for (name,point,quantity),items in groups.items():
            column = QUANTITIES.index(quantity)
            current_gap=stream_gaps.get(name,estimate_gap)
            ns = np.asarray([v[0] for v in items],np.int64);receive = np.asarray([v[1] for v in items],np.int64)
            values = np.asarray([v[2] for v in items]); t = (ns-origin)*1e-9
            targets = {window:sample_reference(points[point],ns)[:,column] for window,points in references.items()}
            common = np.logical_and.reduce([np.isfinite(v) for v in targets.values()])
            if common.any() and name.startswith('own_'): available.add((point,quantity))
            for i in np.flatnonzero(common):
                target = targets[str(primary)][i]
                writer.writerow([int(ns[i]),int(receive[i]),name,point,quantity,values[i],target,values[i]-target])
            for part in evaluation:
                selection = _evaluation_mask(part,ns,origin,classification)
                good = common&selection
                entry = dict(published_samples=int(selection.sum()),matched_samples=int(good.sum()),
                    matched_fraction_of_published=float(good.sum()/selection.sum()) if selection.any() else 0.,
                    windows={window:error_statistics(values[good]-target[good]) for window,target in targets.items()})
                if selection.any(): entry['receive_minus_source_ms'] = age_statistics((receive[selection]-ns[selection])*1e-6)
                # Periods belong to ORIGINAL consecutive stream samples and
                # one contiguous regime run, never to disjoint selected runs.
                adjacent=selection[:-1]&selection[1:]
                if part.get('regime'):
                    run_ids=_regime_ids(classification,ns,part['regime'])
                    adjacent &= run_ids[:-1]==run_ids[1:]
                    entry['regime_support_status']=classification['support'][part['regime']]['status']
                periods=np.diff(ns)[adjacent]*1e-6
                entry['source_gaps_above_threshold']=0
                if len(periods):
                    entry['source_period_ms']=age_statistics(periods)
                    entry['source_gaps_above_threshold']=int(np.sum(periods>current_gap*1000))
                if good.sum()>=3 and np.ptp(t[good])>0:
                    error=values[good]-targets[str(primary)][good]
                    centered=t[good]-np.mean(t[good])
                    entry['error_slope_per_s']=float(centered@(error-np.mean(error))/(centered@centered))
                primary_segments = references[str(primary)][point]
                reference_ns = np.concatenate([s['ns'][s['valid']] for s in primary_segments])
                ref_mask = _evaluation_mask(part,reference_ns,origin,classification)
                reference_ns = reference_ns[ref_mask]
                indices = np.clip(np.searchsorted(ns,reference_ns,side='right')-1,0,max(0,len(ns)-2))
                covered = np.zeros(len(reference_ns),bool)
                if len(ns) >= 2:
                    covered = (reference_ns>=ns[0])&(reference_ns<=ns[-1])&((ns[indices+1]-ns[indices])*1e-9<=current_gap)
                entry['reference_grid_coverage'] = dict(covered=int(covered.sum()),total=len(covered),fraction=float(np.mean(covered)) if len(covered) else 0.)
                if part.get('regime'):
                    entry['waveform_offset']=dict(identifiable=False,reason='not_computed_for_categorical_regime_mask',
                        interpretation='categorical intervals can be disjoint; no lag fit across selected regimes')
                else:
                    entry['waveform_offset'] = waveform_offset(ns[selection],values[selection],primary_segments,column,
                        config['comparison'],config['clock']['uncertainty_ns'])
                result['comparisons']['/'.join([part['name'],name,point,quantity])] = entry
            stride = max(1,len(ns)//5000)
            parts=np.split(np.arange(len(ns)),np.flatnonzero(np.diff(ns)*1e-9>current_gap)+1)
            color=None
            for i,part in enumerate(parts):
                ids=part[::stride]
                line,=figures[point][1][column].plot(t[ids],values[ids],alpha=.75,lw=.8,color=color,
                    marker='.' if len(ids)==1 else None,label=name if i==0 else None)
                color=line.get_color()
    result['complete_required_outputs'] = len(available)==10
    # Secondary comparison on exactly the same reference grid for all available
    # estimators of a quantity. Native-sample errors above are retained separately.
    result['common_grid_comparisons']={}
    for point in config['references']:
        primary_segments=references[str(primary)][point]
        grid=np.concatenate([s['ns'][s['valid']] for s in primary_segments])
        truth=sample_reference(primary_segments,grid)
        for column,quantity in enumerate(QUANTITIES):
            common=np.logical_and.reduce([np.isfinite(sample_reference(points[point],grid)[:,column]) for points in references.values()])
            estimates={}
            for (name,ref,q),items in groups.items():
                if (ref,q)!=(point,quantity) or len(items)<2:continue
                ns=np.asarray([item[0] for item in items],np.int64);values=np.asarray([item[2] for item in items])
                ids=np.clip(np.searchsorted(ns,grid,side='right')-1,0,len(ns)-2)
                supported=(grid>=ns[0])&(grid<=ns[-1])&((ns[ids+1]-ns[ids])*1e-9<=stream_gaps.get(name,estimate_gap))
                common &= supported
                estimates[name]=np.interp((grid-ns[0])*1e-9,(ns-ns[0])*1e-9,values)
            if not estimates:continue
            for part in evaluation:
                good=common&_evaluation_mask(part,grid,origin,classification)
                result['common_grid_comparisons']['/'.join([part['name'],point,quantity])]=dict(n=int(good.sum()),
                    interpretation='same reference grid/support; interpolation does not increase measurement bandwidth',
                    streams={name:error_statistics(values[good]-truth[good,column]) for name,values in estimates.items()})
                if part.get('regime'):
                    result['common_grid_comparisons']['/'.join([part['name'],point,quantity])]['regime_support_status']=classification['support'][part['regime']]['status']
    status_path = folder/'status.jsonl';status_counts=defaultdict(lambda:defaultdict(int))
    if status_path.exists():
        for line in status_path.read_text().splitlines():
            row=json.loads(line)
            for key,value in row.get('fields',{}).items():
                if key.endswith(('_valid','_reason')) or key=='quality_degraded': status_counts[row['stream']][key+'='+value]+=1
    result['status_message_counts']={name:dict(counts) for name,counts in status_counts.items()}
    for point,(fig,axes) in figures.items():
        for quantity,ax in zip(QUANTITIES,axes):
            ax.set_ylabel(quantity+(' rad/s' if quantity=='yaw_rate' else ' m/s' if quantity in ('vx','vy') else ' m/s²'))
            ax.grid(True);ax.legend(loc='upper right',fontsize=7)
        axes[-1].set_xlabel('Seconds from first mapped Vicon pose')
        fig.suptitle(point+'; unshifted estimates; centered offline reference; plot-only thinning')
        fig.tight_layout();fig.savefig(output/(point+'.png'),dpi=150);plt.close(fig)
    (output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# Vicon 五量对照报告','',
        '主要误差按原源时间计算，未平移估计曲线。参考使用中心窗口及未来动捕样本，不进入在线估计器。',
        f"完整后轴／质心五量支持：{result['complete_required_outputs']}；合成数据：{result['synthetic']}。本报告不自动判定实车验收通过。",'',
        '参考标定／同步状态：`'+json.dumps(result['reference_qualification'],ensure_ascii=False)+'`','',
        '| 窗口／流／参考点／量 | n | bias | RMSE | MAE | P95绝对误差 | 参考覆盖率 | 最佳波形偏移ms |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for key,entry in result['comparisons'].items():
        stats=entry['windows'][str(primary)];lag=entry['waveform_offset'];offset=lag.get('best_waveform_offset_ms') if lag.get('identifiable') else '不可辨识'
        if stats['n']:
            lines.append(f"| {key} | {stats['n']} | {stats['bias']:.5g} | {stats['rmse']:.5g} | {stats['mae']:.5g} | {stats['abs_p95']:.5g} | {entry['reference_grid_coverage']['fraction']:.1%} | {offset} |")
        else: lines.append(f'| {key} | 0 | — | — | — | — | 0 | 无共同支持 |')
    lines += ['', '## Vicon 参考工况分层', '',
        '标签仅由主窗口的后轴 Vicon 速度、惯性加速度和横摆角速度生成，所有估计器及质心共用相同时间标签；阈值是可配置的描述规则，不是验收限值。',
        '静止同时满足低平面速度、低横摆角速度和低平面加速度；直行需达到移动速度且横摆率低；转弯需达到移动速度且横摆率高。中间范围保留为未分类。',
        'S 弯需在连续有效且持续移动的参考内出现两个持续足够长、符号相反的横摆率波瓣，过渡时间和整对波瓣跨度均受限；中间过渡纳入该 S 弯。缺测或遮挡会切断候选，不跨缺口拼接。',
        '分类只在同标签连续参考段首末网格时刻之间有效，边界不外推。各工况的支持不足／缺失显式列出，仍保留可用样本的描述统计。源断流只统计同一工况连续段内相邻原发布样本。工况分层不计算波形偏移，避免跨不连续工况拟合时移；整段和手动窗口保留原偏移诊断。',
        '实际分类配置：`'+json.dumps(classification['settings'],ensure_ascii=False)+'`', '',
        '| 工况 | 支持状态 | 主参考样本 | 连续段数 | 段内跨度合计 s |', '| --- | --- | ---: | ---: | ---: |']
    for regime,support in classification['support'].items():
        lines.append(f"| {regime} | {support['status']} | {support['samples']} | {support['contiguous_runs']} | {support['supported_span_s']:.5g} |")
    lines += ['', f"未分类有效参考样本：{classification['unclassified_samples']}；主参考无效网格样本：{classification['unavailable_reference_grid_samples']}。标签段见 [regime_labels.csv](regime_labels.csv)。", '', '## 相同参考网格的基线比较', '',
        '各量所有可用估计器共用相同参考时刻及缺口支持；插值不增加 LIO 等低频观测的独立带宽。上表保留各流原发布时刻评分。',
        '', '| 窗口／参考点／量 | 共同 n | 各流 RMSE |', '| --- | ---: | --- |']
    for key,entry in result['common_grid_comparisons'].items():
        values='；'.join(f"{name}: {stats['rmse']:.5g}" for name,stats in entry['streams'].items() if stats['n'])
        lines.append(f"| {key} | {entry['n']} | {values or '无共同支持'} |")
    lines += ['', '## 求导窗口敏感性', '', '| 流／量 | 各窗口 RMSE（相同样本支持） |', '| --- | --- |']
    for key,entry in result['comparisons'].items():
        values='；'.join(f"{window}s: {stats['rmse']:.5g}" for window,stats in entry['windows'].items() if stats['n'])
        lines.append(f'| {key} | {values or "无共同支持"} |')
    lines += ['', '全部求导窗口共用误差支持；窗口敏感性、接收减源时间年龄、状态计数及来源哈希见 [metrics.json](metrics.json)。',
        '波形偏移在共同动态支持上最大化去均值归一化相关；去均值只用于形状／偏移诊断，主要误差及偏移处的 RMSE 均保留原值。它不是时钟校准，也不是独立的物理／滤波延迟。静止／弱激励、低相关、搜索边界或周期多解不接受为可辨识偏移。',
        '动捕导数依赖窗口、跟踪质量和几何；未测外参、未知同步或合成数据均在上方标记。状态统计为消息计数，不冒充时间占比。',
        '图仅在显示时抽稀，指标使用全部匹配点。主要误差不按估计器 valid 标志筛掉已发布的差样本；源断流通过覆盖率和状态另外呈现。','',
        '![rear axle](rear_axle.png)','![center of mass](center_of_mass.png)','']
    (output/'report.zh-CN.md').write_text('\n'.join(lines))
    return result
