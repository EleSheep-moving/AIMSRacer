"""Nominal held-output validation at activation, without output authority.

The longitudinal model consumes internal speed-target slew as ideal physical
acceleration, as the OCP does. Wire speed and its dead zone are recorded
separately; neither establishes a measured motor response. The certificate
covers nominal 20 ms execution, not later timing/expiry/authority overrides.
"""
import copy
import hashlib
import json
import math
from dataclasses import asdict,replace

import numpy as np

from .envelope import evaluate_envelope,independent_rollout,utilization
from .validation import validate_candidate
from .path import ReferencePath


STEP=.02
_REFERENCE_PROJECT=ReferencePath.project
_REFERENCE_PROJECT_THETA=ReferencePath.project_theta


def _reference_positions(states,path,plan):
    positions=np.asarray(states)[:,:2]
    if path.frame_id!='map':
        return positions
    alignment=np.asarray(plan.get('map_alignment'),float)
    if alignment.shape!=(3,) or not np.isfinite(alignment).all():
        raise ValueError('Physical execution projection requires finite map alignment')
    c,s=np.cos(alignment[2]),np.sin(alignment[2])
    rotation=np.array([[c,-s],[s,c]])
    # Retain the original scalar matrix product for every row. A BLAS batch
    # product can round differently and alter a nearest-grid tie.
    return np.asarray([rotation@xy+alignment[:2] for xy in positions])


def _finish_independent(supervisor,remaining):
    from .runtime import speed_envelope
    minimum=supervisor.minimum_speed_stop_distance()
    threshold=max(.05,minimum,.2,minimum+.05)
    return bool(remaining>threshold+1e-9 and
                speed_envelope(remaining,supervisor.config.brake_limit,
                               supervisor.config.jerk_limit,supervisor.PLAN_TTL)>=supervisor.config.cruise_speed)


def _finish_certificate(supervisor,plan,states,path):
    """Bound the EXACT scalar projector's output without its refinements.

    ReferencePath.project selects the first nearest coarse grid point and then
    refines within guess +/- step. Refinements cannot alter an unwrap branch
    when adjacent coarse differences stay more than 2*step from +/-length/2.
    The refined differences telescope: uncertainty relative to the LIVE
    progress base is +/-2*step, including both initial and current errors.
    """
    positions=_reference_positions(states,path,plan)
    if not np.isfinite(positions).all():
        return None
    if hasattr(path,'_projection_grid'):
        grid,points,step=path._projection_grid,path._projection_points,path._projection_step
    else:
        grid=np.linspace(0,path.length,max(100,int(path.length/.05)),endpoint=False)
        points=path.curve.numpy(grid);step=path.length/len(grid)
    distances=np.sum((points[None,:,:]-positions[:,None,:])**2,axis=2)
    guesses=grid[np.argmin(distances,axis=1)]
    deltas=(np.diff(guesses)+path.length/2)%path.length-path.length/2
    if np.any(np.abs(deltas)>=path.length/2-2*step):
        return None
    relative=np.r_[0.,np.cumsum(deltas)]
    center=supervisor.progress+relative
    bounds=np.column_stack((center-2*step,center+2*step))
    bounds[0]=supervisor.progress  # first command consumes the real live base
    remaining=supervisor.lap_goal-bounds[:,1]
    if not all(_finish_independent(supervisor,r) for r in remaining):
        return None
    return dict(proven=True,bound_source='ReferencePath coarse guesses and bounded refinements',
                projection_error_bound_m=float(2*step),grid_step_m=float(step),
                minimum_remaining_m=float(np.min(remaining)),
                physical_progress_bounds_m=bounds.tolist())


def prospective_status(supervisor,actual_speed):
    if (supervisor.status=='RECOVERING' and supervisor.recovery_good_candidates+1>=2
            and actual_speed>=.05):
        return 'RUNNING'
    return supervisor.status


def execution_context(supervisor,plan,initial,applied,now,path=None):
    context=dict(initial=list(initial),applied=dict(applied),activation_time=now,
                recovery_epoch=plan['stamp'],macro_dt=plan.get('dt',.1),
                status=prospective_status(supervisor,initial[3]),
                last_command=asdict(supervisor.last_command),
                last_acceleration=supervisor.last_acceleration,
                last_steering_rate=supervisor.last_steering_rate,last_tick=supervisor.last_tick,
                progress=supervisor.progress,lap_goal=supervisor.lap_goal,
                plan_ttl=supervisor.PLAN_TTL,minimum_stop_distance=supervisor.minimum_speed_stop_distance())
    context['progress_model']=('projected_nominal_physical_position' if hasattr(path,'project') else
                               'integrated_physical_speed_without_reference_projection')
    if path is not None:
        context['reference']=dict(frame_id=path.frame_id,left_width=path.left_width,
                                  right_width=path.right_width,length=getattr(path,'length',None))
        if hasattr(path,'points'):
            context['reference']['points_sha256']=hashlib.sha256(np.asarray(path.points).tobytes()).hexdigest()
    return context


def execution_fingerprint(supervisor,plan,initial,applied,now,path=None):
    payload=dict(context=execution_context(supervisor,plan,initial,applied,now,path),
                 config=asdict(supervisor.config),
                 plan={key:plan.get(key) for key in ('states','controls','execution_speed_targets',
                       'previous_steering','map_alignment','stamp','source_stamp')})
    return hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()


def execution_schedule(supervisor,plan,initial,applied,now,path=None,held_steering=None,force_slow=False):
    """Replay the real smoother on a clone; never change real state/history."""
    if not math.isclose(plan.get('dt',.1),.1,rel_tol=0.,abs_tol=1e-9):
        raise ValueError('Execution schedule requires the runtime 100 ms plan interval')
    initial=np.asarray(initial,float)
    if initial.shape!=(6,) or not np.isfinite(initial).all():
        raise ValueError('Finite physical initial state(6) required')
    context=execution_context(supervisor,plan,initial.tolist(),applied,now,path)
    standard_projection=(isinstance(path,ReferencePath) and
                         getattr(path.project,'__func__',None) is _REFERENCE_PROJECT and
                         getattr(path.project_theta,'__func__',None) is _REFERENCE_PROJECT_THETA)
    # This is only a cheap prefilter. The generated physical trace must prove
    # finish independence through the actual projector's bounds afterward.
    tentative_fast=(not force_slow and context['status']=='RUNNING' and standard_projection and
                    _finish_independent(supervisor,supervisor.lap_goal-supervisor.progress-
                                         supervisor.config.max_speed*len(plan['controls'])*.1))
    context['finish_independent_certificate']=dict(proven=False)
    forecast=copy.copy(supervisor)
    forecast.pending_plan=None;forecast.plan=dict(plan,stamp=now)
    forecast.status=context['status'];forecast.last_usable_update=now
    forecast.consecutive_failures=0
    controls=[];internal=[];wire=[];elapsed=[];statuses=[];rows=[initial.copy()]
    seed=[supervisor.last_acceleration,applied['steering'],applied['steering_rate']]
    previous=seed;physical=initial.copy();progress=supervisor.progress
    physical_progress=[progress]
    def project(state):
        xy=_reference_positions(state[None,:],path,plan)[0]
        return path.project_theta(xy) if standard_projection else path.project(xy)[0]
    wrapped=project(physical) if not tentative_fast and hasattr(path,'project') else None
    for i in range(5*len(plan['controls'])):
        stamp=now+i*STEP
        previous_tick=forecast.last_tick
        interval=max(0.,min(.05,stamp-(previous_tick if previous_tick is not None else stamp)))
        forecast.state=replace(supervisor.state,x=physical[0],y=physical[1],yaw=physical[2],
                               speed=physical[3],steering=physical[5],
                               timestamp=supervisor.state.timestamp+i*STEP)
        forecast.state_received=forecast.mode_received=stamp;forecast.progress=progress
        command=forecast.command(stamp,enforce_plan_age=False)
        if tentative_fast and forecast.braking_budget is not None:
            # Capacity budgeting reads physical steering. Discard every
            # speculative output and restart from the unchanged live prefix.
            return execution_schedule(supervisor,plan,initial,applied,now,path,held_steering,force_slow=True)
        if forecast.status=='FAULT':
            raise ValueError('Nominal execution schedule fault: '+forecast.reason)
        actuator=forecast.actuator_command(command)
        acceleration=forecast.last_acceleration if forecast.active else 0.
        rate=forecast.last_steering_rate if forecast.active else 0.
        index=min(len(plan['controls'])-1,int((stamp-now)/.1))
        virtual_speed=plan['controls'][index][2]
        steering=actuator.steering if held_steering is None else held_steering[i]
        controls.append([acceleration,steering,virtual_speed])
        internal.append([command.speed,command.steering,acceleration,rate])
        wire.append([actuator.speed,steering]);elapsed.append(interval)
        statuses.append(forecast.status)
        before=physical
        if tentative_fast:
            # command() only reads physical speed outside finish decisions.
            # Match the independent integrator's ten held 2 ms increments;
            # all spatial/steering dynamics are rolled out once below.
            for _ in range(10):physical[3]+=.002*acceleration
            continue
        physical=independent_rollout(physical,previous,[controls[-1]],supervisor.config,STEP)[-1]
        rows.append(physical.copy());previous=[acceleration,steering,rate]
        if wrapped is None:
            # Explicit fixture approximation when no reference projection is
            # available. Production supplies the prepared reference below.
            progress+=max(0.,.5*(before[3]+physical[3]))*STEP
        else:
            projected=project(physical)
            progress+=(projected-wrapped+path.length/2)%path.length-path.length/2
            wrapped=projected
        physical_progress.append(progress)
    bounds=None
    if tentative_fast:
        rows=independent_rollout(initial,seed,controls,supervisor.config,STEP)
        certificate=_finish_certificate(supervisor,plan,rows,path)
        if certificate is None:
            return execution_schedule(supervisor,plan,initial,applied,now,path,held_steering,force_slow=True)
        context['finish_independent_certificate']=certificate
        context['progress_model']='bounded_physical_projection_finish_independent'
        bounds=certificate['physical_progress_bounds_m']
        physical_progress=None
    return dict(controls=np.asarray(controls),states=np.asarray(rows),internal=np.asarray(internal),
                wire=np.asarray(wire),elapsed=np.asarray(elapsed),statuses=statuses,
                physical_progress=physical_progress,physical_progress_bounds_m=bounds,context=context,seed=seed)


def _hard_bounds(schedule,config,tolerance):
    internal,wire=schedule['internal'],schedule['wire'];context=schedule['context']
    intervals=schedule['elapsed'];a=internal[:,2];rates=internal[:,3]
    previous_a=np.r_[context['last_acceleration'],a[:-1]]
    previous_rates=np.r_[context['last_steering_rate'],rates[:-1]]
    previous_steering=np.r_[context['applied']['steering'],wire[:-1,1]]
    steering_change=np.abs(wire[:,1]-previous_steering)
    violation=max(float(np.max(a-config.accel_limit)),float(np.max(-config.brake_limit-a)),
                  float(np.max(np.abs(a-previous_a)-config.jerk_limit*intervals)),
                  float(np.max(np.abs(rates)-config.steer_rate)),
                  float(np.max(np.abs(rates-previous_rates)-config.steer_acceleration*intervals)),
                  float(np.max(steering_change-config.steer_rate*intervals)),
                  float(np.max(np.abs(wire[:,1])-config.steer_limit)),
                  float(np.max(-wire[:,0])),float(np.max(wire[:,0]-config.max_speed)),0.)
    return violation,violation<=tolerance


def _envelope_samples(schedule,config):
    states,controls=schedule['states'],schedule['controls']
    # Check both adjacent acceleration values at every output boundary;
    # immutable t=0 is handled separately by the initial candidate gate.
    indices=np.c_[np.arange(len(controls)),np.arange(1,len(controls)+1)].ravel()[1:]
    accelerations=np.repeat(controls[:,0],2)[1:]
    values=np.array([utilization(states[i],a,config) for i,a in zip(indices,accelerations)])
    lateral=np.array([utilization(states[i],0.,config) for i in indices])
    return values,lateral,indices*STEP


def validate_execution(supervisor,plan,initial,applied,now,path=None,tolerance=1e-4):
    """Final activation gate for this context's nominal executed schedule."""
    try:
        config=supervisor.config
        schedule=execution_schedule(supervisor,plan,initial,applied,now,path)
        fingerprint=execution_fingerprint(supervisor,plan,initial,applied,now,path)
        hard_violation,hard_ok=_hard_bounds(schedule,config,tolerance)
        # Reuse the existing physical steering, speed and corridor checks.
        # Soft recovery is compared explicitly below with its ORIGINAL macro
        # prefix/epoch. A microstep evaluator must not redefine that contract.
        strict=replace(config,envelope_soft_enabled=False,recovery_jerk_enabled=False)
        physical_plan=dict(states=schedule['states'].tolist(),controls=schedule['controls'].tolist(),
                           validation_applied=schedule['seed'],dt=STEP,
                           map_alignment=plan.get('map_alignment'))
        physical=validate_candidate(physical_plan,strict,path,tolerance)
        if 'dynamics_max_residual' not in physical:
            raise ValueError(physical['reason'])
        states=schedule['states']
        state_bounds=max(float(np.max(-states[:,3])),float(np.max(states[:,3]-config.max_speed)),
                         float(np.max(np.abs(states[:,5])-config.steer_limit)),0.)
        margin=physical['minimum_margin_m']
        physical_ok=state_bounds<=tolerance and (margin is None or margin>=-tolerance)
        values,lateral,times=_envelope_samples(schedule,config)
        initial_value=utilization(initial,schedule['controls'][0,0],config)
        excess=np.maximum(values-1.,0.);lateral_excess=np.maximum(lateral-1.,0.)
        epoch_offset=max(0.,now-plan['stamp'])
        deadline_ok=bool(np.all(values[times+epoch_offset>=config.envelope_recovery_time-1e-10]<=1.+tolerance))
        cap=config.envelope_slack_limit if config.envelope_soft_enabled else 0.
        bounds=bool((config.envelope_soft_enabled or initial_value<=1.+tolerance) and
                    np.max(excess)<=cap+tolerance and deadline_ok and values[-1]<=1.+tolerance)
        diagnostic=dict(initial_candidate_utilization=float(initial_value),
                        future_slack_max=float(np.max(excess)),terminal_utilization=float(values[-1]),
                        future_violation_duration_s=float(times[excess>tolerance].max(initial=0.)),
                        recovery_deadline_satisfied=deadline_ok,recovery_bounds_satisfied=bounds,
                        active_envelope_slack_limit=cap,recovery_epoch=plan['stamp'],
                        recovery_time_elapsed_at_activation=epoch_offset,
                        candidate_excess_integral=float(np.trapz(excess,times)),
                        reference_control_prefix_intervals=1,reference_braking_begins_s=.1,
                        original_macro_dt=.1)
        reference=None;comparison=True
        recovery_needed=bool(initial_value>1.+tolerance or np.max(excess)>tolerance)
        if config.envelope_soft_enabled:
            raw=evaluate_envelope(initial,[applied[k] for k in ('acceleration','steering','steering_rate')],
                                  plan['controls'],config,.1,tolerance=tolerance)
            reference_controls=np.asarray(raw['reference_controls'])
            targets=[applied['speed']]
            for control in reference_controls:
                targets.append(max(0.,min(config.max_speed,targets[-1]+control[0]*.1)))
            reference_plan=dict(plan,controls=reference_controls.tolist(),execution_speed_targets=targets)
            reference=execution_schedule(supervisor,reference_plan,initial,applied,now,path,
                                         held_steering=schedule['controls'][:,1])
            # Speed/finish decisions may differ after the immutable prefix.
            # Steering is held to the candidate's EXACT emitted trace.
            reference['controls'][:,1]=schedule['controls'][:,1]
            reference['wire'][:,1]=schedule['wire'][:,1]
            reference['internal'][:,1]=schedule['internal'][:,1]
            reference['internal'][:,3]=schedule['internal'][:,3]
            if not np.array_equal(reference['controls'][:5],schedule['controls'][:5]):
                raise ValueError('Executed braking comparator changed the immutable 100 ms prefix')
            rv,rl,_=_envelope_samples(reference,config)
            re=np.maximum(rv-1.,0.);rle=np.maximum(rl-1.,0.)
            ref_hard,ref_hard_ok=_hard_bounds(reference,config,tolerance)
            ref_speed=max(float(np.max(-reference['states'][:,3])),
                          float(np.max(reference['states'][:,3]-config.max_speed)),0.)
            comparisons=dict(reference_feasible=raw['reference_feasible'] and ref_hard_ok and ref_speed<=tolerance,
                             peak_excess=np.max(excess)<=np.max(re)+tolerance,
                             integrated_excess=np.trapz(excess,times)<=np.trapz(re,times)+tolerance,
                             peak_lateral_excess=np.max(lateral_excess)<=np.max(rle)+tolerance,
                             integrated_lateral_excess=np.trapz(lateral_excess,times)<=np.trapz(rle,times)+tolerance,
                             terminal_lateral_excess=lateral_excess[-1]<=rle[-1]+tolerance)
            comparison=all(comparisons.values())
            recovery_needed=bool(recovery_needed or raw['initial_unavoidable_violation'] is None or
                                 raw['initial_unavoidable_violation']>tolerance)
            diagnostic.update(braking_comparison_satisfied=comparison,
                              braking_comparison_failures=[k for k,v in comparisons.items() if not v],
                              reference_hard_control_violation=ref_hard,
                              reference_excess_integral=float(np.trapz(re,times)),
                              reference_slack_max=float(np.max(re)),
                              reference_initial_diagnostic_dt=.1)
        diagnostic['recovery_needed']=recovery_needed
        accepted=bool(hard_ok and physical_ok and bounds and (not recovery_needed or comparison))
        result=dict(accepted=accepted,reason='' if accepted else 'Nominal executed schedule violates physical or recovery bounds',
                    execution_fingerprint=fingerprint,context=schedule['context'],
                    states=states.tolist(),controls=schedule['controls'].tolist(),
                    internal_commands=schedule['internal'].tolist(),wire_commands=schedule['wire'].tolist(),
                    output_intervals_s=schedule['elapsed'].tolist(),statuses=schedule['statuses'],
                    physical_progress=schedule['physical_progress'],
                    physical_progress_bounds_m=schedule['physical_progress_bounds_m'],
                    hard_control_violation=hard_violation,state_bound_violation=state_bounds,
                    minimum_margin_m=margin,envelope=diagnostic,execution_authorized=False,
                    scope='activation_only_nominal_20ms_schedule',
                    longitudinal_model='unidentified ideal acceleration from internal speed-target slew')
        if reference is not None:
            result.update(reference_controls=reference['controls'].tolist(),reference_states=reference['states'].tolist())
        return result
    except (KeyError,ValueError,TypeError,OverflowError) as exc:
        return dict(accepted=False,reason=str(exc),execution_authorized=False,
                    scope='activation_only_nominal_20ms_schedule')
