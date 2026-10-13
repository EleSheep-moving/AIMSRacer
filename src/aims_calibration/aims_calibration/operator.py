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
