"""Independent numerical operating-envelope diagnostics, never output authority.

The rollout uses 2 ms midpoint integration and no CasADi/solver transition. The
braking comparator shares the complete first command interval and candidate
steering sequence, then brakes at bounded jerk and available operating-envelope capacity. Consequently it
cannot start braking earlier than the candidate's immutable control prefix.
"""
import math
import numpy as np


def utilization(state, acceleration, config):
    accel, brake, lateral = config.envelope_halfaxes()
    v, delta = float(state[3]), float(state[5])
    ay = v*v*math.tan(delta)/(config.wheelbase*(1+config.understeer_coefficient*v*v))
    return (acceleration/(accel if acceleration >= 0 else brake))**2 + (ay/lateral)**2


def initial_envelope_diagnostic(initial_state, applied, config, dt=.1):
    lo = max(-config.brake_limit, float(applied[0])-config.jerk_limit*dt)
    hi = min(config.accel_limit, float(applied[0])+config.jerk_limit*dt)
    feasible = lo <= hi
    best = float(np.clip(0.,lo,hi)) if feasible else None
    minimum = utilization(initial_state,best,config) if feasible else None
    lateral = utilization(initial_state,0.,config)
    return dict(initial_acceleration_interval=[lo,hi], initial_actuator_jerk_feasible=feasible,
                minimum_initial_utilization=minimum,
                initial_unavoidable_violation=max(0.,minimum-1.) if feasible else None,
                initial_lateral_utilization=lateral, initial_lateral_violation=max(0.,lateral-1.))


def effective_jerk_limit(initial_state, applied, config, dt=.1):
    d = initial_envelope_diagnostic(initial_state,applied,config,dt)
    recovery = (config.envelope_soft_enabled and config.recovery_jerk_enabled and
                d['initial_unavoidable_violation'] is not None and d['initial_unavoidable_violation'] > 1e-8)
    return config.recovery_jerk_limit if recovery else config.jerk_limit


def jerk_limits(initial_state, applied, config, dt, horizon):
    recovery = effective_jerk_limit(initial_state,applied,config,dt)
    return np.array([recovery if k*dt < config.envelope_recovery_time-1e-10 else config.jerk_limit
                     for k in range(horizon)])


def independent_rollout(initial_state, applied, controls, config, dt=.1, steering_bias=0.):
    from .rollout_native import kernel_path,native_rollout
    result=native_rollout(kernel_path(),initial_state,applied,controls,config,dt,steering_bias)
    if result is not None:
        return result
    return python_rollout(initial_state,applied,controls,config,dt,steering_bias)


def python_rollout(initial_state, applied, controls, config, dt=.1, steering_bias=0.):
    """Return unique 20 ms samples; command ramps match the OCP's held microsteps."""
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(steering_bias):
        raise ValueError('finite dt and steering bias required; dt must be positive')
    count = round(dt/.02)
    if count < 1 or not np.isclose(count*.02,dt):
        raise ValueError('dt must be a positive multiple of 20 ms')
    state = np.asarray(initial_state,dtype=float).copy()
    rows = [state.copy()]
    previous_steer = float(applied[1])
    for control in controls:
        for j in range(count):
            commanded = previous_steer+(control[1]-previous_steer)*(j+1)/count+steering_bias
            def rhs(x):
                v, angle = x[3], x[5]
                return np.array([v*math.cos(x[2]),v*math.sin(x[2]),
                                 v*math.tan(angle)/(config.wheelbase*(1+config.understeer_coefficient*v*v)),
                                 control[0],control[2],(commanded-angle)/config.steering_tau])
            for _ in range(10):
                state += .002*rhs(state+.001*rhs(state))
            rows.append(state.copy())
        previous_steer = float(control[1])
    return np.asarray(rows)


def evaluate_envelope(initial_state, applied, controls, config, dt=.1, steering_bias=0., tolerance=1e-4):
    """Raw physical diagnostics, independent braking comparison, no authorization.

    Bounds and recovery are checked independently of solver slack variables.
    Peak/integrated excess and lateral evolution must be no worse than the
    braking reference. Instantaneous longitudinal excess may temporarily rise
    during braking; the finite cap/deadline and strict terminal bound still apply. Passing this function is only one input
    to a separate numerical validator and trusted output protocol.
    """
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(tolerance) or tolerance < 0:
        raise ValueError('finite positive dt and finite nonnegative tolerance required')
    initial_state, applied, controls = [np.asarray(v,dtype=float) for v in (initial_state,applied,controls)]
    if initial_state.shape != (6,) or applied.shape != (3,) or controls.ndim != 2 or controls.shape[1] != 3 or len(controls)<1:
        raise ValueError('expected initial(6), applied(3), controls(N,3)')
    if not all(np.isfinite(v).all() for v in (initial_state,applied,controls)):
        raise ValueError('finite rollout inputs required')
    config.validate()
    limits = jerk_limits(initial_state,applied,config,dt,len(controls))
    reference = controls.copy()
    reference_rows = [initial_state.copy()]
    reference_state = initial_state.copy()
    reference_reserve_feasible = True
    def stop_loss(acceleration):
        if acceleration >= 0.: return 0.
        # Conservative discrete stopping reserve: ramp acceleration back to
        # zero at the strict jerk even when optional recovery jerk is active.
        step = config.jerk_limit*dt
        intervals = int(math.ceil(-acceleration/step))
        return dt*(-intervals*acceleration-step*intervals*(intervals-1)/2.)
    for k in range(len(controls)):
        if k:
            lo = max(-config.brake_limit,reference[k-1,0]-limits[k]*dt)
            hi = min(config.accel_limit,reference[k-1,0]+limits[k]*dt)
            lateral_now = utilization(reference_state,0.,config)
            brake_axis = config.envelope_halfaxes()[1]
            # When lateral utilization alone exceeds 1, braking is still
            # necessary; restrict temporary excess by the independent recovery
            # budget, rather than demanding zero acceleration indefinitely.
            capacity = (config.brake_limit if lateral_now > 1. else
                        min(config.brake_limit,brake_axis*math.sqrt(max(0.,1.-lateral_now))))
            target = float(np.clip(-capacity,lo,hi))
            speed = float(reference_state[3])
            if stop_loss(target) > speed+1e-12:
                if stop_loss(hi) > speed+1e-12:
                    # There is no bounded stopping maneuver for this prefix.
                    # Keep jerk bounds and explicitly reject the comparator.
                    reference_reserve_feasible = False
                    target = hi
                else:
                    left,right = target,hi
                    for _ in range(45):
                        middle = (left+right)/2.
                        if stop_loss(middle)>speed: left=middle
                        else: right=middle
                    target = right
            reference[k,0] = target
        previous_command = applied if k == 0 else np.r_[reference[k-1,:2],0.]
        segment = independent_rollout(reference_state,previous_command,reference[k:k+1],config,dt,steering_bias)
        reference_rows.extend(segment[1:])
        reference_state = segment[-1]
    candidate_states = independent_rollout(initial_state,applied,controls,config,dt,steering_bias)
    reference_states = np.asarray(reference_rows)
    count = round(dt/.02)
    def samples(states,commands):
        values, lateral, times = [],[],[]
        for k,u in enumerate(commands):
            # Include boundary after acceleration changes, but exclude immutable t=0.
            for j in range(0 if k else 1,count+1):
                x = states[k*count+j]
                values.append(utilization(x,u[0],config))
                lateral.append(utilization(x,0.,config))
                times.append(k*dt+j*.02)
        return np.asarray(values),np.asarray(lateral),np.asarray(times)
    values, lateral, times = samples(candidate_states,controls)
    ref_values, ref_lateral, _ = samples(reference_states,reference)
    excess, ref_excess = np.maximum(values-1.,0.),np.maximum(ref_values-1.,0.)
    # Duration is conservatively the last violating time, not a sum that can
    # hide a later second violation after an earlier feasible interval.
    duration = float(times[excess>tolerance].max(initial=0.))
    previous = np.vstack((applied[:2],controls[:-1,:2]))
    rates = (controls[:,1]-previous[:,1])/dt
    previous_rates = np.r_[applied[2],rates[:-1]]
    hard = max(float(np.max(controls[:,0]-config.accel_limit)),
               float(np.max(-config.brake_limit-controls[:,0])),
               float(np.max(np.abs(controls[:,1])-config.steer_limit)),
               float(np.max(np.abs(controls[:,0]-previous[:,0])-limits*dt)),
               float(np.max(np.abs(rates)-config.steer_rate)),
               float(np.max(np.abs(rates-previous_rates)-config.steer_acceleration*dt)),
               float(np.max(-controls[:,2])),float(np.max(controls[:,2]-config.max_speed)),0.)
    speed_violation = max(float(np.max(-candidate_states[:,3])),float(np.max(candidate_states[:,3]-config.max_speed)),0.)
    initial = initial_envelope_diagnostic(initial_state,applied,config,dt)
    initial_candidate = utilization(initial_state,controls[0,0],config)
    recovery_needed = bool(np.max(excess)>tolerance or initial['initial_unavoidable_violation'] is None or
                           initial['initial_unavoidable_violation']>tolerance or initial_candidate>1.+tolerance)
    speed_above = float(np.max(candidate_states[:,3]-reference_states[:,3]))
    candidate_integral = float(np.trapz(excess,times))
    reference_integral = float(np.trapz(ref_excess,times))
    lateral_excess, ref_lateral_excess = np.maximum(lateral-1.,0.),np.maximum(ref_lateral-1.,0.)
    lateral_integral = float(np.trapz(lateral_excess,times))
    ref_lateral_integral = float(np.trapz(ref_lateral_excess,times))
    ref_previous = np.vstack((applied[:2],reference[:-1,:2]))
    reference_hard = max(float(np.max(reference[:,0]-config.accel_limit)),
                         float(np.max(-config.brake_limit-reference[:,0])),
                         float(np.max(np.abs(reference[:,0]-ref_previous[:,0])-limits*dt)),0.)
    reference_speed_violation = max(float(np.max(-reference_states[:,3])),
                                    float(np.max(reference_states[:,3]-config.max_speed)),0.)
    reference_feasible = bool(reference_reserve_feasible and reference_hard<=tolerance and
                              reference_speed_violation<=tolerance and hard<=tolerance)
    # Compare physical violations. A lower lateral acceleration once both
    # candidates are inside the envelope is a preference, not a recovery gate.
    comparisons = dict(reference_feasible=reference_feasible,
        peak_excess=np.max(excess)<=np.max(ref_excess)+tolerance,
        integrated_excess=candidate_integral<=reference_integral+tolerance,
        peak_lateral_excess=np.max(lateral_excess)<=np.max(ref_lateral_excess)+tolerance,
        integrated_lateral_excess=lateral_integral<=ref_lateral_integral+tolerance,
        terminal_lateral_excess=lateral_excess[-1]<=ref_lateral_excess[-1]+tolerance)
    comparison = all(comparisons.values())
    deadline_ok = bool(np.all(values[times>=config.envelope_recovery_time-1e-10]<=1.+tolerance))
    cap = config.envelope_slack_limit if config.envelope_soft_enabled else 0.
    initial_ok = bool(config.envelope_soft_enabled or initial_candidate<=1.+tolerance)
    bounds = bool(hard<=tolerance and speed_violation<=tolerance and initial_ok and
                  np.max(excess)<=cap+tolerance and deadline_ok and
                  duration<=config.envelope_recovery_time+tolerance and values[-1]<=1.+tolerance)
    return dict(**initial, initial_candidate_utilization=initial_candidate,
                future_slack_max=float(np.max(excess)), future_violation_duration_s=duration,
                terminal_utilization=float(values[-1]), candidate_excess_integral=candidate_integral,
                reference_excess_integral=reference_integral,
                candidate_lateral_excess_integral=lateral_integral,
                reference_lateral_excess_integral=ref_lateral_integral,
                reference_feasible=reference_feasible, reference_hard_control_violation=reference_hard,
                reference_speed_bound_violation=reference_speed_violation,
                reference_slack_max=float(np.max(ref_excess)), reference_terminal_utilization=float(ref_values[-1]),
                candidate_excess_above_reference_max=float(np.max(excess-ref_excess)),
                candidate_lateral_above_reference_max=float(np.max(lateral-ref_lateral)),
                candidate_speed_above_reference_max=speed_above,
                hard_control_violation=hard, speed_bound_violation=speed_violation,
                recovery_needed=recovery_needed, recovery_bounds_satisfied=bounds,
                recovery_deadline_satisfied=deadline_ok, initial_envelope_satisfied=initial_ok,
                active_envelope_slack_limit=cap,
                braking_comparison_satisfied=comparison,
                braking_comparison_failures=[key for key,value in comparisons.items() if not value],
                candidate_terminal_lateral_utilization=float(lateral[-1]),
                reference_terminal_lateral_utilization=float(ref_lateral[-1]),
                candidate_terminal_lateral_excess=float(lateral_excess[-1]),
                reference_terminal_lateral_excess=float(ref_lateral_excess[-1]),
                recovery_acceptable=bool(bounds and comparison and initial['initial_actuator_jerk_feasible']),
                execution_authorized=False, active_jerk_limits=limits.tolist(),
                reference_control_prefix_intervals=1, reference_braking_begins_s=dt,
                candidate_samples=candidate_states.tolist(),reference_samples=reference_states.tolist(),
                reference_controls=reference.tolist())


def evaluate_strict_envelope(initial_state, applied, controls, config, dt=.1,
                             steering_bias=0., tolerance=1e-4):
    """Return candidate bounds and independent NumPy samples, without recovery.

    This strict-only diagnostic preserves the full evaluator's command boundary
    samples. Braking reference fields are omitted because no comparison is made.
    The samples remain arrays for dynamics, physical steering and corridor checks.
    """
    if not np.isfinite(dt) or dt<=0 or not np.isfinite(tolerance) or tolerance<0:
        raise ValueError('finite positive dt and finite nonnegative tolerance required')
    if not np.isfinite(steering_bias):
        raise ValueError('finite steering bias required')
    initial_state,applied,controls=[np.asarray(v,dtype=float) for v in (initial_state,applied,controls)]
    if (initial_state.shape!=(6,) or applied.shape!=(3,) or controls.ndim!=2 or
            controls.shape[1]!=3 or len(controls)<1):
        raise ValueError('expected initial(6), applied(3), controls(N,3)')
    if not all(np.isfinite(v).all() for v in (initial_state,applied,controls)):
        raise ValueError('finite rollout inputs required')
    config.validate()
    if config.envelope_soft_enabled:
        raise ValueError('strict envelope helper cannot evaluate soft recovery')
    count=round(dt/.02)
    if count<1 or not np.isclose(count*.02,dt):
        raise ValueError('dt must be a positive multiple of 20 ms')
    limits=jerk_limits(initial_state,applied,config,dt,len(controls))
    states=independent_rollout(initial_state,applied,controls,config,dt,steering_bias)
    if not np.isfinite(states).all():
        raise ValueError('finite independent rollout required')
    # A state at an acceleration change is checked with both adjacent commands.
    # Exclude only the immutable t=0 sample; its utilization is checked below.
    indices=(np.arange(len(controls))[:,None]*count+np.arange(count+1)).ravel()[1:]
    accelerations=np.repeat(controls[:,0],count+1)[1:]
    times=(np.arange(len(controls))[:,None]*dt+np.arange(count+1)*.02).ravel()[1:]
    sample_speed=states[indices,3];sample_steering=states[indices,5]
    accel_axis,brake_axis,lateral_axis=config.envelope_halfaxes()
    ay=(sample_speed**2*np.tan(sample_steering)/
        (config.wheelbase*(1+config.understeer_coefficient*sample_speed**2)))
    lateral=(ay/lateral_axis)**2
    values=(accelerations/np.where(accelerations>=0.,accel_axis,brake_axis))**2+lateral
    if not np.isfinite(values).all():
        raise ValueError('finite independent utilization required')
    excess=np.maximum(values-1.,0.)
    duration=float(times[excess>tolerance].max(initial=0.))
    previous=np.vstack((applied[:2],controls[:-1,:2]))
    rates=(controls[:,1]-previous[:,1])/dt
    previous_rates=np.r_[applied[2],rates[:-1]]
    hard=max(float(np.max(controls[:,0]-config.accel_limit)),
             float(np.max(-config.brake_limit-controls[:,0])),
             float(np.max(np.abs(controls[:,1])-config.steer_limit)),
             float(np.max(np.abs(controls[:,0]-previous[:,0])-limits*dt)),
             float(np.max(np.abs(rates)-config.steer_rate)),
             float(np.max(np.abs(rates-previous_rates)-config.steer_acceleration*dt)),
             float(np.max(-controls[:,2])),float(np.max(controls[:,2]-config.max_speed)),0.)
    speed_violation=max(float(np.max(-states[:,3])),float(np.max(states[:,3]-config.max_speed)),0.)
    steering_violation=max(float(np.max(np.abs(states[:,5])-config.steer_limit)),0.)
    initial=initial_envelope_diagnostic(initial_state,applied,config,dt)
    initial_candidate=utilization(initial_state,controls[0,0],config)
    initial_ok=bool(initial_candidate<=1.+tolerance)
    deadline_ok=bool(np.all(values[times>=config.envelope_recovery_time-1e-10]<=1.+tolerance))
    recovery_needed=bool(np.max(excess)>tolerance or initial['initial_unavoidable_violation'] is None or
                         initial['initial_unavoidable_violation']>tolerance or not initial_ok)
    bounds=bool(hard<=tolerance and speed_violation<=tolerance and initial_ok and
                np.max(excess)<=tolerance and deadline_ok and
                duration<=config.envelope_recovery_time+tolerance and values[-1]<=1.+tolerance)
    diagnostic=dict(**initial,initial_candidate_utilization=initial_candidate,
        future_slack_max=float(np.max(excess)),future_violation_duration_s=duration,
        terminal_utilization=float(values[-1]),candidate_excess_integral=float(np.trapz(excess,times)),
        candidate_lateral_excess_integral=float(np.trapz(np.maximum(lateral-1.,0.),times)),
        hard_control_violation=hard,speed_bound_violation=speed_violation,
        actual_steering_bound_violation=steering_violation,recovery_needed=recovery_needed,
        recovery_bounds_satisfied=bounds,recovery_deadline_satisfied=deadline_ok,
        initial_envelope_satisfied=initial_ok,active_envelope_slack_limit=0.,
        execution_authorized=False,active_jerk_limits=limits.tolist(),
        diagnostic_scope='independent_strict_bounds',braking_comparison_performed=False)
    return diagnostic,states
