"""Prepare frozen current plans without changing the Stage A configuration."""
from dataclasses import asdict
import hashlib
from pathlib import Path

from .engine import Config,Engine,TrackingConfig
from .operator import canonical_hash
from .planner import build_sweep_plan,build_full_step_plan,load_baseline,_polynomial_current
from .protection import PulseProtection,ProtectionSettings,load_response_limits


def tracking_settings_from_parameters(values):
    mapping=dict(settle_seconds='current_settle_seconds',window_seconds='current_window_seconds',
        min_samples='current_min_samples',abs_tolerance='current_absolute_tolerance',
        rel_tolerance='current_relative_tolerance',failure_fraction='current_failure_fraction',max_attempts='max_attempts')
    defaults=TrackingConfig()
    return {field:values.get(parameter,getattr(defaults,field)) for field,parameter in mapping.items()}


def prepare_engine(values,plan_mode='manual',base_current_file='',current_step=5.,tracking_settings=None,sweep_start_index=0,response_limits_file='',protection_settings=None):
    if plan_mode not in ('manual','sweep'):raise ValueError('plan_mode must be manual or sweep')
    full_range=values.get('sweep_protocol','interval')=='full_range_step'
    if full_range and plan_mode!='sweep':raise ValueError('full-range steps require a frozen sweep plan')
    if type(sweep_start_index) is not int or sweep_start_index<0 or (plan_mode!='sweep' and sweep_start_index):
        raise ValueError('sweep_start_index requires a nonnegative integer in sweep mode')
    if values.get('stage')=='speed_hold':
        if plan_mode!='manual':raise ValueError('A does not accept a current sweep')
        return Engine(Config(**values)),{},None
    settings=TrackingConfig(**(tracking_settings or {}))
    baseline=None;plan=None;metadata=dict(current_signal='avg_iq',plan_mode=plan_mode,configuration_role='engine_defaults')
    values=dict(values)
    if plan_mode=='sweep':
        if not base_current_file:raise ValueError('B/C sweep requires base_current_file')
        path=Path(base_current_file).expanduser().resolve(strict=True)
        baseline_bytes=path.read_bytes()
        baseline=load_baseline(path,raw=baseline_bytes)
        if full_range:
            plan=build_full_step_plan(values['stage'],baseline,start_speed=values['start_speed'],
                end_speed=values['end_speed'],current_step=current_step,
                max_current=values['max_current'],repeats=values['repeats'])
        else:
            plan=build_sweep_plan(values['stage'],values['speeds'],baseline,current_step=current_step,
                max_current=values['max_current'],repeats=values['repeats'])
        if sweep_start_index>=len(plan):raise ValueError('sweep_start_index exceeds the frozen plan')
        if sweep_start_index:
            full_plan=[dict(plan_index=i,**asdict(item)) for i,item in enumerate(plan)]
            metadata.update(sweep_start_index=sweep_start_index,sweep_full_plan_trials=len(plan),
                sweep_full_plan_sha256=canonical_hash(full_plan))
            plan=plan[sweep_start_index:]
        # The complete manifest owns generated trials. Config retains its
        # legacy signed-single-interval defaults and validation for manual use.
        values.update(start_speed=plan[0].start_speed,end_speed=plan[0].end_speed,
            currents=(plan[0].current,) if values['stage']=='accel_interval' else (-min(current_step,values['max_current']),))
        metadata.update(base_current_file=str(path),base_current_file_sha256=hashlib.sha256(baseline_bytes).hexdigest(),
            baseline_sha256=baseline['baseline_sha256'],current_step=current_step)
    protection=None;protect_settings=None;limits=None
    if values.get('predictive_exit') or values.get('measured_current_handoff'):
        if baseline is None:raise ValueError('current handoff requires a frozen A baseline')
        response_limit=None
        if values.get('predictive_exit'):
            if not response_limits_file:raise ValueError('predictive protection requires frozen A baseline and response limits file')
            response_path=Path(response_limits_file).expanduser().resolve(strict=True)
            response_limit,limits,response_raw=load_response_limits(response_path,baseline['baseline_sha256'])
            metadata.update(response_limits_file=str(response_path),response_limits_sha256=hashlib.sha256(response_raw).hexdigest(),
                response_limits_content_sha256=canonical_hash(limits))
        protect_settings=ProtectionSettings(**(protection_settings or {}))
        lookup={point['speed']:point['mean_iq'] for point in baseline['points']}
        def holding_current(speed):
            if baseline.get('planning_model'):return _polynomial_current(baseline['planning_model'],speed)
            if full_range:
                if not isinstance(speed,(int,float)) or isinstance(speed,bool) or not min(lookup)<=speed<=max(lookup):
                    raise ValueError('holding speed not covered by Stage A')
                if speed in lookup:return lookup[speed]
                left=max(s for s in lookup if s<speed);right=min(s for s in lookup if s>speed)
                return lookup[left]+(lookup[right]-lookup[left])*(speed-left)/(right-left)
            if speed not in lookup:raise ValueError('holding speed not covered by Stage A')
            return lookup[speed]
        protection=PulseProtection(holding_current,response_limit,protect_settings,full_range=full_range)
        metadata['protection_configuration_sha256']=canonical_hash(asdict(protect_settings))
    engine=Engine(Config(**values),trial_plan=plan,tracking_config=settings,protection=protection)
    tracking=asdict(settings);serialized=engine.serializable_plan
    metadata['tracking_configuration_sha256']=canonical_hash(tracking)
    if values['stage']=='decel_current': metadata['decel_group_policy']='independent_full_steps' if full_range else 'skip_on_reverse_or_current_tracking_failure'
    manifest=dict(current_signal='avg_iq',plan=serialized,plan_sha256=canonical_hash(serialized),tracking_configuration=tracking)
    if protect_settings is not None:
        manifest.update(protection_configuration=asdict(protect_settings),response_limits=limits)
    if baseline is not None:manifest['baseline']=baseline
    if full_range:
        steady=asdict(engine.steady_settings)
        metadata.update(sweep_protocol='full_range_step',steady_configuration_sha256=canonical_hash(steady))
        policy=dict(endpoint_exit_policy='measured_wheel_speed',operating_wheel_speed_limit_mps=None,
            current_handoff_enabled=True,response_limits_role='not_used')
        if values['stage']=='decel_current':policy['start_trigger_policy']='first_wheel_at_or_above_start_speed'
        metadata.update(policy)
        if response_limits_file:metadata['ignored_response_limits_file']=str(response_limits_file)
        manifest.update(sweep_protocol='full_range_step',steady_configuration=steady,**policy)
    return engine,metadata,manifest
