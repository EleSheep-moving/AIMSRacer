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
    if payload.get('schema_version')==2: return summarize_current_results(payload)
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
        rejected=len(trials)-accepted,missing=missing,errors=errors,trials=details)


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def _same_number(first,second):
    return bounded(first,-1e12,1e12) and bounded(second,-1e12,1e12) and math.isclose(first,second,rel_tol=0.,abs_tol=1e-8)


def _tracking_failure_is_supported(row,item,settings,freshness):
    """Recompute the gate from captured feedback instead of trusting a flag."""
    evidence=row.get('tracking_evidence',{});observations=row.get('tracking_observations',[])
    if not isinstance(evidence,dict) or not isinstance(observations,list): return False
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
        stamp=point.get('telemetry_stamp');current=point.get('current_motor')
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
            if plan!=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(generated)]:
                raise ValueError('generated plan differs from baseline')
            source=baseline['source']['context']
            keys=['vehicle_sha256']
            if config.get('steering_source')=='pp':keys+=['map_sha256','reference_sha256','calibration_regions_sha256']
            if any(source.get(key) is not None and source[key]!=metadata.get(key) for key in keys):
                raise ValueError('baseline context identity')
        except (ValueError,TypeError,KeyError):errors.append('baseline_or_sweep_plan_invalid')
    items=[]
    for index,spec in enumerate(plan):
        try:
            if not isinstance(spec,dict) or spec.get('plan_index')!=index:raise ValueError('plan index')
            item=TrialPlan(**{k:v for k,v in spec.items() if k!='plan_index'})
            if item.stage!=config.get('stage'):raise ValueError('plan stage')
            if max(item.start_speed,item.end_speed)>config.get('max_speed',0) or abs(item.current)>config.get('max_current',0):raise ValueError('plan limit')
            items.append(item)
        except (ValueError,TypeError):items.append(None);errors.append('invalid_plan_item_'+str(index))
    metadata_valid=not errors
    attempts={};seen_ids=set();resolved=set();covered=set();successful=set();unreachable=set();details=[]
    for row in trials:
        if not isinstance(row,dict):errors.append('invalid_trial');continue
        index=row.get('plan_index');attempt=row.get('attempt_id')
        valid_index=type(index) is int and 0<=index<len(items) and items[index] is not None
        valid_id=type(attempt) is int and attempt>0 and attempt not in seen_ids
        if type(attempt) is int:seen_ids.add(attempt)
        if not valid_index or not valid_id:errors.append('invalid_trial_identity');continue
        item=items[index];attempts[attempt]=row
        identity=(row.get('stage')==item.stage and row.get('kind')==item.kind and row.get('repeat')==item.repeat and
            _same_number(row.get('requested_target'),item.current) and _same_number(row.get('current'),item.current) and
            _same_number(row.get('start_target_speed'),item.start_speed) and _same_number(row.get('end_target_speed'),item.end_speed))
        evidence_valid=row.get('evidence_valid') is True
        if not identity or not evidence_valid:errors.append('invalid_trial_'+str(attempt));continue
        completed=row.get('success') is True
        if completed:
            duration=row.get('duration_s');start=row.get('start_speed');end=row.get('end_speed')
            req=row.get('first_current_request_stamp');ack=row.get('first_current_ack_stamp');start_stamp=row.get('start_stamp');end_stamp=row.get('end_stamp')
            completed=(row.get('reason')=='completed' and bounded(duration,1e-12,1e6) and
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
    missing=[i for i in range(len(items)) if i not in resolved]
    gaps=[i for i in range(len(items)) if i not in covered]
    execution_complete=bool(items) and not missing
    coverage_complete=bool(items) and not gaps
    evidence_valid=metadata_valid and not errors
    return dict(valid=execution_complete and coverage_complete and evidence_valid,
        schema_version=2,execution_complete=execution_complete,evidence_valid=evidence_valid,
        coverage_complete=coverage_complete,planned=len(items),records=len(trials),accepted=len(successful) if metadata_valid else 0,
        rejected=len(trials)-sum(r['valid'] for r in details),unreachable=len(unreachable),skipped=len(skipped_indices),
        missing=missing,coverage_gaps=gaps,errors=errors,trials=details)
