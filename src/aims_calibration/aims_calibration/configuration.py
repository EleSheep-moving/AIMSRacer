"""Prepare frozen current plans without changing the Stage A configuration."""
from dataclasses import asdict
import hashlib
from pathlib import Path

from .engine import Config,Engine,TrackingConfig
from .operator import canonical_hash
from .planner import build_sweep_plan,load_baseline


def tracking_settings_from_parameters(values):
    mapping=dict(settle_seconds='current_settle_seconds',window_seconds='current_window_seconds',
        min_samples='current_min_samples',abs_tolerance='current_absolute_tolerance',
        rel_tolerance='current_relative_tolerance',failure_fraction='current_failure_fraction',max_attempts='max_attempts')
    defaults=TrackingConfig()
    return {field:values.get(parameter,getattr(defaults,field)) for field,parameter in mapping.items()}


def prepare_engine(values,plan_mode='manual',base_current_file='',current_step=5.,tracking_settings=None):
    if plan_mode not in ('manual','sweep'):raise ValueError('plan_mode must be manual or sweep')
    if values.get('stage')=='speed_hold':
        if plan_mode!='manual':raise ValueError('A does not accept a current sweep')
        return Engine(Config(**values)),{},None
    settings=TrackingConfig(**(tracking_settings or {}))
    baseline=None;plan=None;metadata=dict(plan_mode=plan_mode,configuration_role='engine_defaults')
    values=dict(values)
    if plan_mode=='sweep':
        if not base_current_file:raise ValueError('B/C sweep requires base_current_file')
        path=Path(base_current_file).expanduser().resolve(strict=True)
        baseline_bytes=path.read_bytes()
        baseline=load_baseline(path,raw=baseline_bytes)
        plan=build_sweep_plan(values['stage'],values['speeds'],baseline,current_step=current_step,
            max_current=values['max_current'],repeats=values['repeats'])
        # The complete manifest owns generated trials. Config retains its
        # legacy signed-single-interval defaults and validation for manual use.
        values.update(start_speed=plan[0].start_speed,end_speed=plan[0].end_speed,
            currents=(plan[0].current,) if values['stage']=='accel_interval' else (-min(current_step,values['max_current']),))
        metadata.update(base_current_file=str(path),base_current_file_sha256=hashlib.sha256(baseline_bytes).hexdigest(),
            baseline_sha256=baseline['baseline_sha256'],current_step=current_step)
    engine=Engine(Config(**values),trial_plan=plan,tracking_config=settings)
    tracking=asdict(settings);serialized=engine.serializable_plan
    metadata['tracking_configuration_sha256']=canonical_hash(tracking)
    manifest=dict(plan=serialized,plan_sha256=canonical_hash(serialized),tracking_configuration=tracking)
    if baseline is not None:manifest['baseline']=baseline
    return engine,metadata,manifest
