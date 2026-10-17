"""Read-only event response analysis using causal finite differences.

Every differentiated Vicon observation records its complete pose support.
Intervals describe whole-system observed threshold response, not a servo or
motor's isolated mechanical delay. No publishing interfaces are imported.
"""
import csv
import json
from pathlib import Path

import numpy as np

from .analyze import load_poses
from .fastlio import _pose_geometry, _provenance, _qualification, _read, _strict_ns

FIELDS=dict(speed='target_speed_mps',current='target_current',brake='target_brake_current',duty='target_duty',steering='steering_angle')
DEFAULTS=dict(baseline_s=.4,response_s=1.,min_hold_s=.1,confirmation_s=.02,min_confirm_samples=3,
              noise_multiplier=4.,max_gap_s=.04)
STEPS=dict(speed=.1,current=5.,brake=5.,duty=.02,steering=.03)
FLOORS=dict(erpm=100.,vx=.05,ax=.3,yaw_rate=.05,ay=.3)
MODES={'0':'speed','1':'current','2':'duty','3':'brake','speed':'speed','current':'current','duty':'duty','brake':'brake','brake_current':'brake'}


def _signals(poses,config,gap):
    ns,p,body,valid=_pose_geometry(poses,config)
    values={k:[] for k in ('vx','ax','yaw_rate','ay')}
    velocity=np.full((len(ns),3),np.nan)
    for i in range(1,len(ns)):
        dt=(ns[i]-ns[i-1])*1e-9
        if not (valid[i-1:i+1].all() and dt<=gap):continue
        velocity[i]=(p[i]-p[i-1])/dt
        vb=body[i].inv().apply(velocity[i])
        omega=(body[i-1].inv()*body[i]).as_rotvec()/dt
        values['vx'].append((int(ns[i]),float(vb[0]),int(ns[i-1]),int(ns[i])))
        values['yaw_rate'].append((int(ns[i]),float(omega[2]),int(ns[i-1]),int(ns[i])))
        if i>=2 and np.isfinite(velocity[i-1]).all():
            # Differentiate WORLD velocity, then rotate inertial acceleration.
            # Differentiating body velocity alone would omit omega cross v.
            midpoint_dt=(ns[i]-ns[i-2])*.5e-9
            acceleration=body[i].inv().apply((velocity[i]-velocity[i-1])/midpoint_dt)
            for name,j in [('ax',0),('ay',1)]:
                values[name].append((int(ns[i]),float(acceleration[j]),int(ns[i-2]),int(ns[i])))
    return values,ns,valid


def _coverage(ns,start,end,gap):
    if len(ns)<2:return False
    # Include samples bracketing the requested interval, so a gap crossing a
    # window boundary cannot disappear by selecting only interior timestamps.
    a=max(0,int(np.searchsorted(ns,start,side='right'))-1)
    b=min(len(ns)-1,int(np.searchsorted(ns,end)))
    return bool(ns[a]<=start and ns[b]>=end and np.all(np.diff(ns[a:b+1])*1e-9<=gap))


def _response(signal,start,end,settings,floor,clock_uncertainty_ns):
    baseline=int(round(settings['baseline_s']*1e9));gap=settings['max_gap_s']
    if not signal:return dict(detected=False,reason='missing_signal')
    ns=np.array([s[0] for s in signal],np.int64);v=np.array([s[1] for s in signal]);support=np.array([[s[2],s[3]] for s in signal],np.int64)
    before=(ns>=start-baseline)&(ns<start)&(support[:,1]<start)&np.isfinite(v)
    if before.sum()<3 or not _coverage(ns,start-baseline,start,gap):
        return dict(detected=False,reason='insufficient_response_baseline',baseline_samples=int(before.sum()))
    median=float(np.median(v[before]));noise=float(1.4826*np.median(abs(v[before]-median)))
    # Only causal measurements ending after the command participate. Their
    # support may begin before it, and that uncertainty stays in the interval.
    after=(ns>=start)&(ns<=end)&np.isfinite(v)
    ids=np.flatnonzero(after)
    if not _coverage(ns,start,end,gap):return dict(detected=False,reason='signal_gap',baseline_median=median,baseline_mad_sigma=noise)
    def detect(multiplier):
        threshold=max(floor,settings['noise_multiplier']*noise)*multiplier
        run=[];first=None;confirmed=None
        for i in ids:
            crossed=abs(v[i]-median)>=threshold
            same_direction=not run or np.sign(v[i]-median)==np.sign(v[run[0]]-median)
            contiguous=not run or (ns[i]-ns[run[-1]])*1e-9<=gap
            if not crossed:run=[];continue
            if not same_direction or not contiguous:run=[]
            run.append(i)
            if len(run)>=settings['min_confirm_samples'] and (ns[i]-ns[run[0]])*1e-9+1e-12>=settings['confirmation_s']:
                first=run[0];confirmed=i;break
        answer=dict(detected=first is not None,threshold=float(threshold))
        if first is None:answer['reason']='insufficient_response_excitation';return answer
        # For directly sampled telemetry, previous source observation brackets
        # the threshold crossing; derivatives retain all contributing poses.
        low=int(support[first,0]); high=int(support[first,1])
        interval=[(low-start)*1e-6,(high-start)*1e-6]
        answer.update(response_interval_ms=interval,first_crossing_source_ns=int(ns[first]),
            support_source_ns=[low,high],confirmation_source_ns=int(ns[confirmed]),
            confirmation_latency_ms=float((ns[confirmed]-start)*1e-6),confirmation_samples=len(run),
            clock_expanded_response_interval_ms=[interval[0]-clock_uncertainty_ns*1e-6,interval[1]+clock_uncertainty_ns*1e-6],
            sign=int(np.sign(v[first]-median)))
        return answer
    primary=detect(1.)
    primary.update(baseline_samples=int(before.sum()),baseline_median=median,baseline_mad_sigma=noise,
        threshold_rule='max(response_floor, noise_multiplier * 1.4826 * baseline_MAD)',
        derivative_uses_future_samples=False,threshold_sensitivity={str(m):detect(m) for m in (.5,1.,2.)},
        uncertainty='pose/telemetry source support plus clock mapping bound; threshold dependence reported separately')
    return primary


def _candidates(rows,ns,commands,steps,settings,origin):
    changes=[]
    for i in range(1,len(ns)):
        channels=[k for j,k in enumerate(FIELDS) if abs(commands[i,j]-commands[i-1,j])>=steps[k]]
        if channels:changes.append(dict(index=i,channels=channels,selection='automatic'))
    manual=settings.get('event_windows')
    if manual is None or manual==[]:return changes,changes
    result=[]
    for item in manual:
        channel=item['channel']
        if channel not in FIELDS:raise ValueError('unknown actuator event channel')
        start=int(item['source_ns']) if item.get('source_ns') is not None else origin+int(round(float(item['start_s'])*1e9))
        i=int(np.searchsorted(ns,start))
        if i<=0 or i>=len(ns):raise ValueError('manual event needs source-time command baseline and command sample')
        # Manual selection cannot invent a command at a nonexistent source stamp.
        if ns[i]!=start:raise ValueError('manual event start must match a command source timestamp')
        end=int(item['end_ns']) if item.get('end_ns') is not None else origin+int(round(float(item.get('end_s',(start-origin)*1e-9+settings['response_s']))*1e9))
        if end<=start:raise ValueError('manual event end must follow start')
        # Manual selection chooses an event; it cannot erase other command
        # axes that changed at the same source stamp. Keep selected axis first.
        simultaneous=next((change['channels'] for change in changes if change['index']==i),[])
        channels=[channel]+[name for name in simultaneous if name!=channel]
        result.append(dict(index=i,channels=channels,selection='manual',end_ns=end))
    return result,changes


def analyze(folder:Path, config:dict, output:Path)->dict:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder,output=Path(folder),Path(output)
    if output.exists():raise ValueError('output directory must not already exist')
    provenance=_provenance(folder,config);settings=dict(DEFAULTS,**config.get('actuator',{}))
    for key in DEFAULTS:
        value=float(settings[key])
        if not np.isfinite(value) or value<=0:raise ValueError('invalid actuator '+key)
    settings['min_confirm_samples']=int(settings['min_confirm_samples'])
    if settings['min_confirm_samples']<2:raise ValueError('sustained response requires at least two samples')
    steps=dict(STEPS,**settings.get('step_thresholds',{}));floors=dict(FLOORS,**settings.get('response_floors',{}))
    if any(not np.isfinite(v) or v<=0 for v in [*steps.values(),*floors.values()]):raise ValueError('positive finite actuator thresholds required')
    poses=load_poses(folder/'poses.csv');signals,vn,vg=_signals(poses,config,settings['max_gap_s']);origin=int(vn[0])
    rows=_read(folder/'commands.csv')
    if not rows:raise ValueError('no command samples')
    ns=_strict_ns(rows,'command');command=np.array([[float(r.get(f,'nan')) for f in FIELDS.values()] for r in rows])
    if not np.isfinite(command).all():raise ValueError('nonfinite command values')
    modes=[MODES.get(str(r['mode']).lower(),'unknown') for r in rows]
    telemetry=_read(folder/'telemetry.csv');tn=_strict_ns(telemetry,'telemetry') if telemetry else np.array([],np.int64)
    signals['erpm']=[]
    previous_finite=None
    for i,r in enumerate(telemetry):
        value=float(r['erpm'])
        if np.isfinite(value):
            signals['erpm'].append((int(tn[i]),value,int(tn[i] if previous_finite is None else previous_finite),int(tn[i])))
            previous_finite=int(tn[i])
    events,changes=_candidates(rows,ns,command,steps,settings,origin)
    result=dict(physical_acceptance=False,pure_servo_delay_identified=False,source_time_analysis=True,
        derivative_uses_future_samples=False,reference_qualification=_qualification(config,provenance['extraction_metadata']),
        provenance=provenance,settings=settings,events=[],
        interpretation='command source to whole-vehicle/ERPM threshold response; includes transport, actuator and vehicle dynamics',
        inertial_acceleration='differentiate world target-point velocity first, then rotate to body; no centered smoothing')
    for number,event in enumerate(events):
        i=event['index'];start=int(ns[i]);end=int(event.get('end_ns',start+round(settings['response_s']*1e9)))
        baseline_start=start-int(round(settings['baseline_s']*1e9));min_hold=start+int(round(settings['min_hold_s']*1e9))
        prior=(ns>=baseline_start)&(ns<start);during=(ns>=start)&(ns<=end)
        reasons=[];channels=event['channels'];channel=channels[0];j=list(FIELDS).index(channel)
        delta=float(command[i,j]-command[i-1,j])
        if len(channels)>1:reasons.append('overlapping_commands')
        if modes[i]!=modes[i-1]:reasons.append('mode_change')
        if modes[i]=='unknown':reasons.append('unknown_control_mode')
        if channel!='steering' and modes[i]!=channel:reasons.append('inactive_mode_target_change')
        if abs(delta)<steps[channel]:reasons.append('insufficient_command_excitation')
        if prior.sum()<3 or not _coverage(ns,baseline_start,start,settings['max_gap_s']):reasons.append('insufficient_command_baseline')
        elif np.any(np.ptp(command[prior],axis=0)>=np.array(list(steps.values()))):reasons.append('unstable_command_baseline')
        # Adjacent step detection alone misses a slow ramp made of many small
        # increments. Nonselected axes must remain stable over the combined
        # baseline+response window; the selected target must hold its plateau
        # throughout the scored response window.
        combined=prior|during
        nonselected=[k for k in FIELDS if k!=channel]
        if combined.any() and any(np.ptp(command[combined,list(FIELDS).index(k)])>=steps[k] for k in nonselected):
            reasons.append('overlapping_commands')
        if during.any() and np.ptp(command[during,j])>=steps[channel]:
            reasons.append('unstable_command_plateau')
        if not _coverage(ns,start,end,settings['max_gap_s']):reasons.append('command_gap_or_short_recording')
        later=[c for c in changes if ns[c['index']]>start and ns[c['index']]<=end]
        if later:
            reasons.append('overlapping_commands')
            if ns[later[0]['index']]<min_hold:reasons.append('insufficient_command_hold')
        hold=(ns>=start)&(ns<=min_hold)
        if not _coverage(ns,start,min_hold,settings['max_gap_s']) or np.ptp(command[hold,j])>=steps[channel]:
            reasons.append('insufficient_command_hold')
        if not _coverage(vn,baseline_start,end,settings['max_gap_s']) or not vg[(vn>=baseline_start)&(vn<=end)].all():reasons.append('vicon_occlusion_or_gap')
        if any(modes[k]!=modes[i] for k in np.flatnonzero(prior|during)):reasons.append('mode_change')
        # Reject nonselected mode targets and simultaneous control axes.
        active=[k for k in FIELDS if k not in (modes[i],'steering')]
        if any(abs(command[i,list(FIELDS).index(k)])>=steps[k] for k in active):reasons.append('inactive_mode_target_nonzero')
        responses={name:_response(signal,start,end,settings,floors[name],0. if name=='erpm' else float(config['clock']['uncertainty_ns'])) for name,signal in signals.items()}
        result['events'].append(dict(id=number,channel=channel,channels=channels,selection=event['selection'],source_ns=start,end_ns=end,
            source_s=(start-origin)*1e-9,mode=modes[i],command_before=float(command[i-1,j]),command_after=float(command[i,j]),step=delta,
            valid=not reasons,invalid_reasons=list(dict.fromkeys(reasons)),responses=responses,
            interpretation='responses are diagnostic only when event invalid; no pure servo/motor delay attribution'))
    output.mkdir(parents=True)
    with (output/'response_signals.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['source_ns','quantity','value','support_start_ns','support_end_ns'])
        for name,signal in signals.items():
            for n,value,low,high in signal:w.writerow([n,name,value,low,high])
    with (output/'events.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['event_id','source_ns','channel','valid','invalid_reasons','quantity','detected','response_interval_low_ms','response_interval_high_ms','confirmation_latency_ms'])
        for event in result['events']:
            for name,response in event['responses'].items():
                interval=response.get('response_interval_ms',[None,None])
                w.writerow([event['id'],event['source_ns'],event['channel'],event['valid'],';'.join(event['invalid_reasons']),name,response['detected'],*interval,response.get('confirmation_latency_ms')])
    fig,axes=plt.subplots(6,1,figsize=(13,13),sharex=True)
    for j,name in enumerate(FIELDS):axes[0].plot((ns-origin)*1e-9,command[:,j],label=name)
    for ax,(name,signal) in zip(axes[1:],signals.items()):
        if signal:ax.plot([(s[0]-origin)*1e-9 for s in signal],[s[1] for s in signal],lw=.9)
        ax.set_ylabel(name)
    for ax in axes:
        for event in result['events']:ax.axvline(event['source_s'],color='red' if not event['valid'] else 'green',alpha=.5)
        ax.grid(True)
    axes[0].legend(ncol=5);axes[-1].set_xlabel('Seconds from first mapped Vicon pose')
    fig.tight_layout();fig.savefig(output/'actuator.png',dpi=140);plt.close(fig)
    (output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# 执行器整车响应区间','',
        '使用命令源时间和 ERPM 源时间；Vicon 按显式时钟映射。位置直接做后向有限差分，不使用中心平滑，不使用未来样本提前响应。',
        '先将标记点转到实测后轴，再对世界速度求导并旋转到车体系，避免把车体系速度导数误作惯性加速度。',
        '阈值为基线中位数附近 max(响应下限, 4×MAD 标准差估计)，实际噪声倍数见配置；持续样本数和持续时间须同时满足。',
        '区间下端至上端覆盖响应值所依赖的原始观测；确认时刻单列，时钟扩展区间和 0.5／1／2 倍阈值敏感性见 JSON。',
        '没有实际舵角反馈，因此这些量包含命令传输、执行器和车体动力学，只能解释为整车响应，不能分离纯舵机机械延迟。',
        '参考资格：`'+json.dumps(result['reference_qualification'],ensure_ascii=False)+'`','',
        '| 事件／通道 | 有效 | 量 | 响应区间 ms | 确认 ms | 原因 |','| --- | --- | --- | --- | --- | --- |']
    for event in result['events']:
        for name,response in event['responses'].items():
            lines.append(f"| {event['id']}/{event['channel']} | {event['valid']} | {name} | {response.get('response_interval_ms','未检出')} | {response.get('confirmation_latency_ms','—')} | {';'.join(event['invalid_reasons']) or response.get('reason','')} |")
    lines+=['','无效事件仍保留诊断值，不能据此作延迟归因；未检出仅说明本阈值／观测支持下激励不足。',
            '输入、配置和程序哈希见 [metrics.json](metrics.json)，各求导支持见 [response_signals.csv](response_signals.csv)。本报告不自动判定实车验收通过。','','![响应](actuator.png)','']
    (output/'report.zh-CN.md').write_text('\n'.join(lines))
    return result
