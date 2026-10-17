"""Pure start eligibility and recorded-result acceptance, shared by node and CLI."""
import hashlib
import json
import math

AGE_NAMES = ('wheel_source','telemetry_source','final_source','rc_source','authority_source',
             'wheel_receipt','telemetry_receipt','final_receipt','rc_receipt','authority_receipt')


def bounded(value, low, high):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and low<=value<=high


def check_start(data):
    reasons=[]
    def require(ok, reason):
        if not ok: reasons.append(reason)
    require(data.get('state')=='IDLE' and data.get('enabled') is False and data.get('enable_count')==0,
            'acquisition_already_started')
    require(not data.get('logging_error','missing') and data.get('logging_dropped')==0,'logging_unavailable')
    freshness=data.get('freshness')
    if not bounded(freshness,0.001,.2): freshness=0.
    for name in AGE_NAMES:
        require(bounded(data.get('ages',{}).get(name),0.,freshness),name+'_stale_or_future')
    require(bounded(data.get('wheel_speed'),-.1,.1),'vehicle_must_be_stationary')
    require(data.get('telemetry_valid') is True,'invalid_motor_telemetry')
    require(data.get('current_signal')=='avg_iq','q_axis_current_required')
    require(data.get('authority') is True,'calibration_authority_required')
    require(data.get('rc_valid') is True,'rc_locked_or_wrong_mode')
    require(data.get('final_zero') is True,'final_zero_speed_required')
    producers=data.get('publishers',{})
    require(producers==dict(calibration=1,final=1,drive=0),'exclusive_command_producers_required')
    require(data.get('driver_startup_disabled') is True,'driver_startup_must_be_disabled')
    require(bounded(data.get('driver_parameter_age'),0.,2.),'driver_parameter_unverified')
    require(not data.get('artifact_error','missing'),'artifact_changed')
    if data.get('pp') is True:
        require(data.get('localization_reason')=='','localization_unavailable')
        require(bounded(data.get('cross_track'),-.25,.25),'reference_distance_exceeded')
        require(bounded(data.get('heading_error'),-.35,.35),'reference_heading_exceeded')
    if data.get('require_acceleration') is True:
        require(data.get('acceleration_valid') is True,'acceleration_unavailable')
    return reasons


def summarize_results(payload):
    if not isinstance(payload,dict): payload={}
    if type(payload.get('schema_version')) is int and payload.get('schema_version')==3: return summarize_full_step_results(payload)
    if type(payload.get('schema_version')) is int and payload.get('schema_version')==2: return summarize_current_results(payload)
    if 'schema_version' in payload and (type(payload['schema_version']) is not int or payload['schema_version']!=1):
        return dict(valid=False,planned=0,records=0,accepted=0,rejected=0,missing=[],errors=['unsupported_schema_version'],trials=[])
    config=payload.get('configuration',{});metadata=payload.get('metadata',{})
    if not isinstance(config,dict): config={}
    if not isinstance(metadata,dict): metadata={}
    errors=[];plan=[]
    if metadata.get('evidence_valid') is not True: errors.append('metadata_evidence_invalid')
    digest=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    if metadata.get('configuration_sha256')!=digest: errors.append('configuration_hash_mismatch')
    metadata_valid=not errors
    stage=config.get('stage')
    targets=config.get('speeds' if stage=='speed_hold' else 'currents',[])
    repeats=config.get('repeats',0)
    if stage not in ('speed_hold','accel_interval','decel_current') or not isinstance(targets,list) or not targets or type(repeats) is not int or not 1<=repeats<=10000:
        errors.append('invalid_plan')
    else: plan=[v for v in targets for _ in range(repeats)]
    trials=payload.get('trials',[])
    if not isinstance(trials,list): trials=[];errors.append('invalid_trials')
    counts={};accepted=0;details=[]
    for row in trials:
        if not isinstance(row,dict): errors.append('invalid_trial');continue
        index=row.get('plan_index'); target=row.get('requested_target')
        valid_index=type(index) is int and 0<=index<len(plan)
        target_ok=valid_index and bounded(target,-100.,100.) and bounded(plan[index],-100.,100.) and math.isclose(target,plan[index],rel_tol=0.,abs_tol=1e-9)
        valid=metadata_valid and row.get('success') is True and row.get('evidence_valid') is True and row.get('stage')==stage and target_ok
        if valid and stage=='speed_hold':
            duration=row.get('duration_s'); minimum=config.get('measurement_seconds')
            valid=bounded(minimum,1e-9,1e6) and bounded(duration,minimum-1e-9,1e6)
        if valid_index: counts[index]=counts.get(index,0)+1
        if valid: accepted+=1
        else: errors.append('invalid_trial_'+str(index))
        details.append(dict(plan_index=index,target=target,valid=valid,reason=row.get('reason',''),
            duration_s=row.get('duration_s'),segment_count=row.get('segment_count')))
    if any(count!=1 for count in counts.values()): errors.append('duplicate_plan_index')
    missing=[i for i in range(len(plan)) if i not in counts]
    if missing: errors.append('incomplete_plan')
    return dict(valid=bool(plan) and not errors,planned=len(plan),records=len(trials),accepted=accepted,
        rejected=len(trials)-accepted,missing=missing,errors=errors,trials=details,
        current_signal=metadata.get('current_signal','legacy_current_motor'),
        legacy_current_signal=metadata.get('current_signal')!='avg_iq')


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def _same_number(first,second):
    return bounded(first,-1e12,1e12) and bounded(second,-1e12,1e12) and math.isclose(first,second,rel_tol=0.,abs_tol=1e-8)


def _tracking_failure_is_supported(row,item,settings,freshness):
    """Recompute the gate from captured feedback instead of trusting a flag."""
    evidence=row.get('tracking_evidence',{});observations=row.get('tracking_observations',[])
    if not isinstance(evidence,dict) or evidence.get('current_signal')!='avg_iq' or not isinstance(observations,list): return False
    ack=row.get('first_current_ack_stamp');request=row.get('first_current_request_stamp')
    if not bounded(request,1e-12,1e12) or not bounded(ack,request,1e12): return False
    start=evidence.get('window_start_stamp');end=evidence.get('window_end_stamp')
    if not bounded(start,0.,1e12) or not bounded(end,start,1e12):return False
    time_tolerance=max(1e-8,2*math.ulp(end))
    if start<ack+settings.settle_seconds-time_tolerance:return False
    if end-start+time_tolerance<settings.window_seconds or not _same_number(evidence.get('window_duration_s'),end-start): return False
    expected=dict(target_current=item.current,tolerance=max(settings.abs_tolerance,settings.rel_tolerance*abs(item.current)),
        settle_seconds=settings.settle_seconds,window_seconds=settings.window_seconds,
        min_samples=settings.min_samples,failure_fraction=settings.failure_fraction)
    if any(not _same_number(evidence.get(key),value) for key,value in expected.items()): return False
    previous=None;window=[];first=None
    for point in observations:
        if not isinstance(point,dict): return False
        stamp=point.get('telemetry_stamp');current=point.get('avg_iq')
        if not bounded(stamp,ack+settings.settle_seconds-time_tolerance,1e12) or not bounded(current,-1e5,1e5): return False
        if previous is not None and (stamp<=previous or stamp-previous>freshness+time_tolerance): return False
        if first is None:first=stamp
        previous=stamp
        if start-time_tolerance<=stamp<=end+time_tolerance:window.append(current)
    if first is None or first>start+time_tolerance or previous<end-time_tolerance: return False
    if len(window)<settings.min_samples or evidence.get('sample_count')!=len(window): return False
    tolerance=expected['tolerance']
    if item.kind=='positive_deceleration':
        below=sum(current<item.current-tolerance for current in window)
        above=sum(current>item.current+tolerance for current in window)
        failed=max(below,above)
    elif item.kind in ('acceleration','braking'):
        sign=1. if item.current>0 else -1.
        failed=sum(sign*current<abs(item.current)-tolerance for current in window)
    else:return False
    return failed/len(window)+1e-9>=settings.failure_fraction


def _reverse_failure_is_supported(row,item,freshness):
    """A C speed-group decision needs fresh negative-wheel evidence after a request."""
    decision=row.get('decision_evidence',{})
    if not isinstance(decision,dict): return False
    obs=decision.get('observation',{});now=decision.get('observed_at')
    if not isinstance(obs,dict) or item.stage!='decel_current' or not bounded(now,1e-12,1e12): return False
    request=row.get('first_current_request_stamp')
    if not bounded(request,1e-12,now) or not bounded(obs.get('wheel_stamp'),request,now): return False
    if not bounded(obs.get('velocity'),-1e5,-1e-12) or obs.get('authority') is not True or obs.get('rc_valid') is not True or obs.get('fault_code')!=0 or obs.get('tracking_reason'):
        return False
    for key in ('wheel_stamp','telemetry_stamp','rc_stamp','authority_stamp','final_received'):
        if not bounded(obs.get(key),now-freshness,now): return False
    echo=obs.get('final_request')
    return isinstance(echo,dict) and echo.get('mode') in (0,1) and bounded(echo.get('stamp'),now-freshness,now)


def summarize_current_results(payload):
    from .engine import TrackingConfig
    from .planner import TrialPlan,load_baseline,build_sweep_plan
    errors=[];config=payload.get('configuration');metadata=payload.get('metadata');plan=payload.get('plan')
    trials=payload.get('trials');skips=payload.get('skipped')
    if not isinstance(config,dict):config={};errors.append('invalid_configuration')
    if not isinstance(metadata,dict):metadata={};errors.append('invalid_metadata')
    if not isinstance(plan,list) or not plan:plan=[];errors.append('invalid_plan')
    if not isinstance(trials,list):trials=[];errors.append('invalid_trials')
    if not isinstance(skips,list):skips=[];errors.append('invalid_skips')
    if metadata.get('evidence_valid') is not True:errors.append('metadata_evidence_invalid')
    if payload.get('current_signal')!='avg_iq' or metadata.get('current_signal')!='avg_iq':errors.append('q_axis_current_identity_missing')
    try:
        if metadata.get('configuration_sha256')!=canonical_hash(config):errors.append('configuration_hash_mismatch')
        if payload.get('plan_sha256')!=canonical_hash(plan):errors.append('plan_hash_mismatch')
        settings_data=payload.get('tracking_configuration')
        settings=TrackingConfig(**settings_data)
        if metadata.get('tracking_configuration_sha256')!=canonical_hash(settings_data):errors.append('tracking_configuration_hash_mismatch')
    except (ValueError,TypeError):
        settings=TrackingConfig();errors.append('invalid_tracking_configuration')
    if metadata.get('plan_mode')=='sweep':
        try:
            from dataclasses import asdict
            baseline=load_baseline('',raw=json.dumps(payload['baseline'],allow_nan=False))
            if metadata.get('baseline_sha256')!=baseline['baseline_sha256']:raise ValueError('baseline identity')
            generated=build_sweep_plan(config['stage'],config['speeds'],baseline,
                current_step=metadata['current_step'],max_current=config['max_current'],repeats=config['repeats'])
            offset=metadata.get('sweep_start_index',0)
            if type(offset) is not int or not 0<=offset<len(generated):raise ValueError('invalid sweep offset')
            if offset:
                full_plan=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(generated)]
                if metadata.get('sweep_full_plan_trials')!=len(generated) or metadata.get('sweep_full_plan_sha256')!=canonical_hash(full_plan):
                    raise ValueError('full sweep plan identity')
                generated=generated[offset:]
            if plan!=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(generated)]:
                raise ValueError('generated plan differs from baseline')
            source=baseline['source']['context']
            keys=['vehicle_sha256']
            if config.get('steering_source')=='pp':keys+=['map_sha256','reference_sha256','calibration_regions_sha256']
            if any(source.get(key) is not None and source[key]!=metadata.get(key) for key in keys):
                raise ValueError('baseline context identity')
        except (ValueError,TypeError,KeyError):errors.append('baseline_or_sweep_plan_invalid')
    if config.get('predictive_exit'):
        try:
            from .protection import ProtectionSettings,load_response_limits
            ProtectionSettings(**payload['protection_configuration'])
            if metadata.get('plan_mode')!='sweep':raise ValueError('protected flow requires frozen sweep')
            if metadata.get('protection_configuration_sha256')!=canonical_hash(payload['protection_configuration']):
                raise ValueError('protection settings hash')
            limits=payload['response_limits']
            if metadata.get('response_limits_content_sha256')!=canonical_hash(limits):raise ValueError('response limits hash')
            load_response_limits('',payload['baseline']['baseline_sha256'],raw=json.dumps(limits,allow_nan=False).encode())
        except (ValueError,TypeError,KeyError):errors.append('invalid_protection_provenance')
    items=[]
    for index,spec in enumerate(plan):
        try:
            if not isinstance(spec,dict) or spec.get('current_signal')!='avg_iq' or spec.get('plan_index')!=index:raise ValueError('plan index')
            item=TrialPlan(**{k:v for k,v in spec.items() if k!='plan_index'})
            if item.stage!=config.get('stage'):raise ValueError('plan stage')
            if max(item.start_speed,item.end_speed)>config.get('max_speed',0) or abs(item.current)>config.get('max_current',0):raise ValueError('plan limit')
            items.append(item)
        except (ValueError,TypeError):items.append(None);errors.append('invalid_plan_item_'+str(index))
    metadata_valid=not errors
    attempts={};seen_ids=set();resolved=set();covered=set();successful=set();unreachable=set();details=[];group_triggers=set()
    group_policy=metadata.get('decel_group_policy')=='skip_on_reverse_or_current_tracking_failure' and config.get('stage')=='decel_current'
    for row in trials:
        if not isinstance(row,dict):errors.append('invalid_trial');continue
        index=row.get('plan_index');attempt=row.get('attempt_id')
        valid_index=type(index) is int and 0<=index<len(items) and items[index] is not None
        valid_id=type(attempt) is int and attempt>0 and attempt not in seen_ids
        if type(attempt) is int:seen_ids.add(attempt)
        if not valid_index or not valid_id:errors.append('invalid_trial_identity');continue
        item=items[index];attempts[attempt]=row
        identity=(row.get('current_signal')=='avg_iq' and row.get('stage')==item.stage and row.get('kind')==item.kind and row.get('repeat')==item.repeat and
            _same_number(row.get('requested_target'),item.current) and _same_number(row.get('current'),item.current) and
            _same_number(row.get('start_target_speed'),item.start_speed) and _same_number(row.get('end_target_speed'),item.end_speed) and
            (row.get('baseline_current') is None if item.baseline_current is None else _same_number(row.get('baseline_current'),item.baseline_current)))
        evidence_valid=row.get('evidence_valid') is True
        if not identity or not evidence_valid:errors.append('invalid_trial_'+str(attempt));continue
        completed=row.get('success') is True
        if completed:
            duration=row.get('duration_s');start=row.get('start_speed');end=row.get('end_speed')
            req=row.get('first_current_request_stamp');ack=row.get('first_current_ack_stamp');start_stamp=row.get('start_stamp');end_stamp=row.get('end_stamp')
            completed=(row.get('reason')=='completed' and bounded(duration,1e-12,1e6) and
                type(row.get('samples')) is int and row['samples']>0 and bounded(row.get('mean_iq'),-1e5,1e5) and bounded(row.get('std_iq'),0.,1e5) and
                bounded(start,0.,1e5) and bounded(end,0.,1e5) and
                (end>=item.end_speed if item.stage=='accel_interval' else end<=item.end_speed) and
                bounded(req,1e-12,1e12) and bounded(ack,req,1e12) and bounded(start_stamp,ack,1e12) and bounded(end_stamp,start_stamp,1e12) and
                _same_number(duration,end_stamp-start_stamp) and _same_number(row.get('response_duration_s'),end_stamp-req) and
                _same_number(row.get('mean_acceleration'),(end-start)/duration))
            if completed:successful.add(index);resolved.add(index);covered.add(index)
            else:errors.append('invalid_completed_trial_'+str(attempt))
        elif row.get('reason')=='current_tracking_unreachable':
            supported=row.get('tracking_status')=='unreachable' and _tracking_failure_is_supported(row,item,settings,config.get('freshness',0))
            if supported:unreachable.add(index);resolved.add(index);covered.add(index)
            else:errors.append('unsupported_current_limit_'+str(attempt))
        elif row.get('reason') in ('reverse_motion','current_tracking_not_achieved') and group_policy:
            supported=(_reverse_failure_is_supported(row,item,config.get('freshness',0)) if row['reason']=='reverse_motion' else
                row.get('tracking_status')=='outside_tolerance' and _tracking_failure_is_supported(row,item,settings,config.get('freshness',0)))
            if supported:group_triggers.add(attempt);resolved.add(index)
            else:errors.append('unsupported_speed_group_decision_'+str(attempt))
        elif row.get('reason')!='straight_region_exit' or row.get('uncovered') is True:
            resolved.add(index)
        details.append(dict(plan_index=index,attempt_id=attempt,target=item.current,kind=item.kind,
                            valid=metadata_valid and completed,reason=row.get('reason','')))
    skipped_indices=set()
    for skipped in skips:
        if not isinstance(skipped,dict):errors.append('invalid_skip');continue
        index=skipped.get('plan_index');trigger_id=skipped.get('triggering_attempt_id')
        trigger=attempts.get(trigger_id) if type(trigger_id) is int else None
        if type(index) is not int or not 0<=index<len(items) or items[index] is None or index in skipped_indices or trigger is None:
            errors.append('invalid_skip_identity');continue
        item=items[index];trigger_index=trigger['plan_index'];origin=items[trigger_index]
        try:identity=TrialPlan(**{k:skipped[k] for k in TrialPlan.__dataclass_fields__})==item
        except (TypeError,ValueError,KeyError):identity=False
        if skipped.get('reason')=='skipped_speed_group':
            scope=(group_policy and trigger_id in group_triggers and index>trigger_index and
                item.start_speed==origin.start_speed and
                all(p is not None and p.start_speed==origin.start_speed for p in items[trigger_index:index+1]))
            evidence=trigger.get('decision_evidence') if trigger.get('reason')=='reverse_motion' else trigger.get('tracking_evidence')
            if not identity or not scope or skipped.get('trigger_reason')!=trigger.get('reason') or skipped.get('evidence')!=evidence:
                errors.append('invalid_skip_scope_'+str(index));continue
            if any(row.get('plan_index')==index for row in trials):errors.append('skipped_item_was_attempted_'+str(index));continue
            skipped_indices.add(index);resolved.add(index)
            # A deliberately skipped C item remains a measurement coverage gap.
            continue
        scope=(index>trigger_index and trigger_index in unreachable and item.start_speed==origin.start_speed and item.kind==origin.kind and
            all(intermediate is not None and intermediate.start_speed==origin.start_speed and intermediate.kind==origin.kind
                for intermediate in items[trigger_index:index+1]))
        if item.kind=='acceleration':scope=scope and item.end_speed==origin.end_speed and item.current>=origin.current
        elif item.kind=='braking':scope=scope and item.current<=origin.current
        elif item.kind=='positive_deceleration':scope=scope and item.current<=origin.current
        else:scope=False
        if not identity or not scope or skipped.get('reason')!='skipped_unreachable_current' or skipped.get('evidence')!=trigger.get('tracking_evidence'):
            errors.append('invalid_skip_scope_'+str(index));continue
        if any(row.get('plan_index')==index for row in trials):errors.append('skipped_item_was_attempted_'+str(index));continue
        skipped_indices.add(index);resolved.add(index);covered.add(index)
    measured_pulses=settled_pulses=0
    if config.get('predictive_exit') and metadata_valid:
        for row in trials:
            if not isinstance(row,dict) or not row.get('pulse_valid'):continue
            duration=row.get('duration_s');start=row.get('start_speed');end=row.get('end_speed')
            valid=(row.get('reason') in ('predictive_exit','completed') and row.get('evidence_valid') is True and
                bounded(duration,1e-12,1e6) and bounded(start,0.,1e5) and bounded(end,0.,1e5) and
                type(row.get('samples')) is int and row['samples']>=2 and
                bounded(row.get('mean_iq'),-1e5,1e5) and bounded(row.get('std_iq'),0.,1e5) and
                bounded(row.get('first_current_request_stamp'),1e-12,1e12) and
                bounded(row.get('first_current_ack_stamp'),row['first_current_request_stamp'],1e12) and
                bounded(row.get('start_stamp'),row['first_current_ack_stamp'],1e12) and
                bounded(row.get('end_stamp'),row['start_stamp'],1e12) and
                _same_number(duration,row['end_stamp']-row['start_stamp']) and
                _same_number(row.get('pulse_mean_acceleration'),(end-start)/duration))
            if not valid:errors.append('invalid_pulse_'+str(row.get('attempt_id')));continue
            measured_pulses+=1
            handoff=row.get('handoff',{})
            if not isinstance(handoff,dict):
                errors.append('invalid_handoff_'+str(row.get('attempt_id')));continue
            if handoff.get('hold_confirmed') is True and not handoff.get('failure'):
                settings=payload.get('protection_configuration',{})
                supported=(bounded(handoff.get('started'),1e-12,1e12) and
                    bounded(handoff.get('ack_stamp'),handoff['started'],1e12) and
                    bounded(handoff.get('last_iq_stamp'),handoff['ack_stamp'],1e12) and
                    bounded(handoff.get('iq_samples'),settings.get('iq_confirm_samples',math.inf),1e6) and
                    bounded(handoff.get('hold_started'),handoff['ack_stamp'],1e12) and
                    bounded(handoff.get('quiet_since'),handoff['hold_started'],1e12) and
                    bounded(handoff.get('hold_confirmed_stamp'),handoff['quiet_since']+config.get('stable_seconds',math.inf),1e12))
                if supported:settled_pulses+=1
                else:errors.append('invalid_handoff_'+str(row.get('attempt_id')))
    missing=[i for i in range(len(items)) if i not in resolved]
    gaps=[i for i in range(len(items)) if i not in covered]
    execution_complete=bool(items) and not missing
    coverage_complete=bool(items) and not gaps
    evidence_valid=metadata_valid and not errors
    return dict(valid=execution_complete and coverage_complete and evidence_valid,
        schema_version=2,current_signal=payload.get('current_signal','unavailable'),execution_complete=execution_complete,evidence_valid=evidence_valid,
        measured_pulses=measured_pulses if evidence_valid else 0,settled_pulses=settled_pulses if evidence_valid else 0,
        coverage_complete=coverage_complete,planned=len(items),records=len(trials),accepted=len(successful) if metadata_valid else 0,
        rejected=len(trials)-sum(r['valid'] for r in details),unreachable=len(unreachable),skipped=len(skipped_indices),
        missing=missing,coverage_gaps=gaps,errors=errors,trials=details)


def _full_handoff_supported(row,config,settings):
    """A response is committed only after measured IQ unload and quiet recovery."""
    handoff=row.get('handoff')
    if not isinstance(handoff,dict) or handoff.get('hold_confirmed') is not True or handoff.get('failure'):
        return False
    started=handoff.get('started');ack=handoff.get('ack_stamp');actual=handoff.get('actual_iq_confirm_stamp')
    last=handoff.get('last_iq_stamp');hold=handoff.get('hold_started');quiet=handoff.get('quiet_since')
    confirmed=handoff.get('hold_confirmed_stamp');end=row.get('end_stamp')
    if not bounded(end,1e-12,1e12):return False
    epsilon=4*math.ulp(float(end))
    return (bounded(started,end-epsilon,1e12) and bounded(ack,started,1e12) and
        bounded(actual,ack,started+settings.unload_timeout+epsilon) and actual>ack and bounded(last,actual,1e12) and
        type(handoff.get('iq_samples')) is int and handoff['iq_samples']>=settings.iq_confirm_samples and
        bounded(hold,actual,1e12) and bounded(quiet,hold,1e12) and
        bounded(confirmed,quiet+config.get('stable_seconds',math.inf)-epsilon,hold+settings.hold_timeout+epsilon))


def _full_steady_supported(evidence,settings,ack,end):
    if not isinstance(evidence,dict) or evidence.get('steady') is not True:return False
    wheel=evidence.get('wheel_stamp');telemetry=evidence.get('telemetry_stamp')
    start=evidence.get('window_start_stamp');current_start=evidence.get('current_window_start_stamp')
    confirmation=evidence.get('confirmation_start_stamp');duration=evidence.get('confirmed_seconds')
    if not all(bounded(v,1e-12,1e12) for v in (ack,end,wheel,telemetry)):return False
    epsilon=4*max(math.ulp(float(v)) for v in (ack,end,wheel,telemetry))
    settled=ack+settings.current_settle_seconds
    return (abs(wheel-end)<=epsilon and abs(wheel-telemetry)<=settings.source_gap+epsilon and bounded(wheel,ack,1e12) and bounded(telemetry,ack,1e12) and
        bounded(start,settled-epsilon,wheel-settings.window_seconds+epsilon) and
        bounded(current_start,settled-epsilon,telemetry-settings.current_window_seconds+epsilon) and
        bounded(confirmation,settled+settings.window_seconds-epsilon,wheel-settings.confirm_seconds+epsilon) and
        bounded(duration,settings.confirm_seconds-epsilon,1e12) and abs(duration-(wheel-confirmation))<=epsilon and
        bounded(evidence.get('slope'),-settings.max_abs_acceleration,settings.max_abs_acceleration) and
        bounded(evidence.get('speed_span'),0.,settings.max_speed_span) and
        type(evidence.get('sample_count')) is int and evidence['sample_count']>=settings.min_samples and
        type(evidence.get('current_sample_count')) is int and evidence['current_sample_count']>=settings.current_min_samples and
        bounded(evidence.get('current_good_fraction'),settings.current_good_fraction,1.))



def summarize_full_step_results(payload):
    """Verify frozen full-range acquisition, separating execution from coverage.

    A steady response may finish before its speed endpoint. Partial responses
    retain usable raw evidence but never become complete measurement coverage.
    Schema 2 speed-group and unreachable-current inference is deliberately absent.
    """
    from dataclasses import asdict
    from .engine import TrackingConfig
    from .planner import TrialPlan,load_baseline,build_full_step_plan
    from .protection import ProtectionSettings
    from .steady import SteadySettings
    errors=[]
    def mapping(name):
        value=payload.get(name)
        if not isinstance(value,dict):errors.append('invalid_'+name);return {}
        return value
    config=mapping('configuration');metadata=mapping('metadata')
    plan=payload.get('plan');trials=payload.get('trials');skips=payload.get('skipped')
    if not isinstance(plan,list) or not plan:plan=[];errors.append('invalid_plan')
    if not isinstance(trials,list):trials=[];errors.append('invalid_trials')
    if not isinstance(skips,list):skips=[];errors.append('invalid_skips')
    if any(obj.get('sweep_protocol')!='full_range_step' for obj in (payload,config,metadata)):
        errors.append('invalid_sweep_protocol')
    if (metadata.get('plan_mode')!='sweep' or config.get('predictive_exit') is not False or
            config.get('measured_current_handoff') is not True):
        errors.append('full_step_requires_measured_current_handoff')
    if any(obj.get('endpoint_exit_policy')!='measured_wheel_speed' or
            obj.get('current_handoff_enabled') is not True or
            'operating_wheel_speed_limit_mps' not in obj or obj['operating_wheel_speed_limit_mps'] is not None
            for obj in (payload,metadata)):
        errors.append('invalid_measured_endpoint_policy')
    if metadata.get('response_limits_role')!='not_used' or payload.get('response_limits') is not None:
        errors.append('invalid_response_limits_role')
    if metadata.get('evidence_valid') is not True:errors.append('metadata_evidence_invalid')
    if payload.get('current_signal')!='avg_iq' or metadata.get('current_signal')!='avg_iq':errors.append('q_axis_current_identity_missing')
    try:
        if metadata.get('configuration_sha256')!=canonical_hash(config):errors.append('configuration_hash_mismatch')
        if payload.get('plan_sha256')!=canonical_hash(plan):errors.append('plan_hash_mismatch')
    except (ValueError,TypeError):errors.append('invalid_hash_input')
    tracking=TrackingConfig();steady=SteadySettings();protection=ProtectionSettings()
    for name,cls in (('tracking_configuration',TrackingConfig),('steady_configuration',SteadySettings),('protection_configuration',ProtectionSettings)):
        try:
            value=payload[name];instance=cls(**value)
            if metadata.get(name+'_sha256')!=canonical_hash(value):raise ValueError('settings hash')
            if name=='tracking_configuration':tracking=instance
            elif name=='steady_configuration':steady=instance
            else:protection=instance
        except (KeyError,ValueError,TypeError):errors.append('invalid_'+name)
    try:
        baseline=load_baseline('',raw=json.dumps(payload['baseline'],allow_nan=False))
        if metadata.get('baseline_sha256')!=baseline['baseline_sha256']:raise ValueError('baseline identity')
        generated=build_full_step_plan(config['stage'],baseline,start_speed=config['start_speed'],end_speed=config['end_speed'],
            current_step=metadata['current_step'],max_current=config['max_current'],repeats=config['repeats'])
        offset=metadata.get('sweep_start_index',0)
        if type(offset) is not int or not 0<=offset<len(generated):raise ValueError('offset')
        full=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(generated)]
        if offset and (metadata.get('sweep_full_plan_trials')!=len(full) or metadata.get('sweep_full_plan_sha256')!=canonical_hash(full)):
            raise ValueError('full plan identity')
        if plan!=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(generated[offset:])]:raise ValueError('plan differs')
        context=baseline['source']['context'];keys=['vehicle_sha256']
        if config.get('steering_source')=='pp':keys+=['map_sha256','reference_sha256','calibration_regions_sha256']
        if any(context.get(k) is not None and context[k]!=metadata.get(k) for k in keys):raise ValueError('baseline context')
    except (KeyError,ValueError,TypeError):errors.append('baseline_or_full_plan_invalid')
    metadata_valid=not errors
    items=[]
    for index,spec in enumerate(plan):
        try:
            if spec.get('plan_index')!=index:raise ValueError('plan index')
            item=TrialPlan(**{k:v for k,v in spec.items() if k!='plan_index'})
            if item.stage!=config.get('stage') or item.current_signal!='avg_iq':raise ValueError('plan identity')
            items.append(item)
        except (AttributeError,ValueError,TypeError):items.append(None);errors.append('invalid_plan_item_'+str(index))
    resolved=set();covered=set();attempted=set();ids=set();details=[];valid_rows={};complete=partial=uncollected=endpoints=settled=0
    for row in trials:
        if not isinstance(row,dict):errors.append('invalid_trial');continue
        index=row.get('plan_index');attempt=row.get('attempt_id')
        if type(index) is not int or not 0<=index<len(items) or items[index] is None or type(attempt) is not int or attempt<=0 or attempt in ids:
            errors.append('invalid_trial_identity');continue
        ids.add(attempt);item=items[index]
        if bounded(row.get('first_current_request_stamp'),1e-12,1e12):attempted.add(index)
        identity=(row.get('current_signal')=='avg_iq' and row.get('stage')==item.stage and row.get('kind')==item.kind and row.get('repeat')==item.repeat and
            _same_number(row.get('current'),item.current) and _same_number(row.get('requested_target'),item.current) and
            _same_number(row.get('start_target_speed'),item.start_speed) and _same_number(row.get('end_target_speed'),item.end_speed) and
            (row.get('baseline_current') is None if item.baseline_current is None else _same_number(row.get('baseline_current'),item.baseline_current)))
        status=row.get('data_status');reason=row.get('stop_reason');endpoint=row.get('endpoint_reached')
        valid=identity and row.get('evidence_valid') is True and type(endpoint) is bool and isinstance(reason,str) and reason==row.get('reason') and status in ('complete','partial','uncollected')
        if reason in ('predictive_exit','response_limit_unavailable','insufficient_exit_headroom'):
            valid=False
        if status in ('complete','partial'):
            req=row.get('first_current_request_stamp');ack=row.get('first_current_ack_stamp');start=row.get('start_stamp');end=row.get('end_stamp');duration=row.get('duration_s')
            evidence=(bounded(req,1e-12,1e12) and bounded(ack,req,1e12) and bounded(start,ack,1e12) and bounded(end,start,1e12) and
                bounded(duration,1e-12 if status=='complete' else 0.,1e6) and _same_number(duration,end-start) and _same_number(row.get('response_duration_s'),end-req) and
                type(row.get('samples')) is int and row['samples']>=(2 if status=='complete' else 1) and bounded(row.get('mean_iq'),-1e5,1e5) and bounded(row.get('std_iq'),0.,1e5) and
                bounded(row.get('wheel_speed_min'),-1e5,1e5) and bounded(row.get('wheel_speed_max'),row.get('wheel_speed_min',math.inf),1e5))
            valid=valid and evidence
            if endpoint:
                reached=bounded(row.get('end_speed'),-1e5,1e5) and (row['end_speed']>=item.end_speed if item.stage=='accel_interval' else row['end_speed']<=item.end_speed)
                valid=valid and reached
            if status=='complete':
                trigger=(endpoint and reason=='speed_boundary') or (reason=='steady_state' and _full_steady_supported(row.get('steady_evidence'),steady,ack,end))
                valid=valid and row.get('success') is True and trigger
            elif row.get('success') is True:valid=False
        elif row.get('success') is True:valid=False
        handoff=_full_handoff_supported(row,config,protection)
        # Unconfirmed handoffs are visible acquisition outcomes; a claimed
        # confirmation with contradictory timestamps invalidates its evidence.
        if isinstance(row.get('handoff'),dict) and row['handoff'].get('hold_confirmed') is True and not handoff:
            errors.append('invalid_handoff_'+str(attempt));valid=False
        if not valid:errors.append('invalid_trial_'+str(attempt))
        else:
            valid_rows[attempt]=row
            if status=='complete':complete+=1;covered.add(index);resolved.add(index)
            elif status=='partial':
                partial+=1
                if row.get('attempts_exhausted') is True and (reason not in
                        ('trial_timeout','straight_headroom','straight_region_exit') or
                        (type(row.get('attempt_number')) is int and row['attempt_number']>=tracking.max_attempts)):
                    resolved.add(index)
            else:
                uncollected+=1
                if (row.get('preflight_rejected') is True and row.get('attempts_exhausted') is True and
                        row.get('first_current_request_stamp') is None and reason=='straight_distance_unavailable'):
                    resolved.add(index)
            if endpoint:endpoints+=1
            if handoff:settled+=1
        details.append(dict(plan_index=index,attempt_id=attempt,target=item.current,kind=item.kind,
            valid=metadata_valid and valid,data_status=status,stop_reason=reason,endpoint_reached=endpoint,
            handoff_committed=handoff,steady_evidence=row.get('steady_evidence'),handoff=row.get('handoff')))
    skipped_indices=set()
    # Full mode records preflight refusals and exhaustion on trial rows. It
    # never infers another current's result from a skip list.
    if skips:errors.append('invalid_full_step_skip')
    missing=[i for i in range(len(items)) if i not in resolved]
    gaps=[i for i in range(len(items)) if i not in covered]
    unexecuted=[i for i in range(len(items)) if i not in attempted and i not in skipped_indices]
    pulses=[row for row in trials if isinstance(row,dict) and bounded(row.get('first_current_request_stamp'),1e-12,1e12)]
    final=not pulses or (pulses[-1].get('attempt_id') in valid_rows and _full_handoff_supported(pulses[-1],config,protection))
    all_handoffs=all(row.get('attempt_id') in valid_rows and _full_handoff_supported(row,config,protection) for row in pulses)
    evidence_valid=not errors
    execution_complete=bool(items) and not missing and final and all_handoffs and evidence_valid
    coverage_complete=bool(items) and not gaps and evidence_valid
    return dict(valid=execution_complete and coverage_complete,schema_version=3,sweep_protocol='full_range_step',current_signal='avg_iq',
        endpoint_exit_policy='measured_wheel_speed',operating_wheel_speed_limit_mps=None,current_handoff_enabled=True,
        evidence_valid=evidence_valid,execution_complete=execution_complete,coverage_complete=coverage_complete,
        final_handoff_committed=final,handoff_required=bool(pulses),planned=len(items),records=len(trials),accepted=complete if metadata_valid else 0,
        rejected=len(trials)-sum(d['valid'] for d in details),complete_responses=complete if metadata_valid else 0,
        partial_responses=partial,uncollected_responses=uncollected,endpoint_reached=endpoints,settled_responses=settled,
        skipped=len(skipped_indices),unexecuted=unexecuted,missing=missing,coverage_gaps=gaps,errors=errors,trials=details)
