"""Clock-explicit command supervision; no ROS or optimizer dependencies."""
from dataclasses import dataclass, asdict
import math


def clip(value, lower, upper):
    return max(lower, min(upper, value))


def angle_difference(a, b):
    return (a-b+math.pi) % (2*math.pi)-math.pi


def speed_envelope(distance, brake, jerk, latency):
    """Conservative braking envelope including jerk ramp and execution delay."""
    delay = brake/jerk + latency
    return brake*(math.sqrt(delay*delay+2*max(0., distance)/brake)-delay)


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float
    speed: float
    steering: float
    timestamp: float  # ROS time, used only for measurement ordering/alignment.


@dataclass(frozen=True)
class Command:
    speed: float
    steering: float


class Supervisor:
    STATE_TIMEOUT = .10
    MODE_TIMEOUT = .10
    PLAN_TTL = 10 * .8 * .1  # Default horizon * allowed age fraction * interval.
    HANDOVER_TOLERANCE = .0005  # sub-ms timer jitter, not a control interval
    HANDOVER_STATE_LIMITS = dict(position=.30, yaw=math.radians(30),
                                 speed=.30, steering=math.radians(20))
    HANDOVER_COMMAND_LIMITS = dict(command_speed=.30,
                                   command_steering=math.radians(20))

    def __init__(self, config, length, plan_ttl=PLAN_TTL, handover_delay=.10, solve_period=None):
        self.config, self.length = config, float(length)
        if not math.isfinite(plan_ttl) or plan_ttl<=0:
            raise ValueError('Positive finite plan lifetime required')
        self.PLAN_TTL=plan_ttl
        if not math.isfinite(handover_delay) or not 0<handover_delay<plan_ttl:
            raise ValueError('Handover delay must be positive and shorter than plan lifetime')
        self.handover_delay=handover_delay
        if solve_period is not None and (not math.isfinite(solve_period) or solve_period<=0):
            raise ValueError('Positive finite solve period required')
        self.solve_period=solve_period
        self.last_usable_update=None
        self.consecutive_failures=0
        self.recovery_good_candidates=0
        self.status, self.reason = 'READY', 'Waiting for fresh inputs'
        self.state = None
        self.state_received = self.mode_received = -math.inf
        self.mode = False
        self.generation = 0
        self.plan = None
        self.pending_plan = None
        self.rejected_plans = 0
        self.handover_error = None
        self.started = self.last_tick = None
        self.progress = self.wrapped_progress = 0.
        self.start_progress = 0.
        self.lap_goal = self.length
        self.cross_track = self.heading_error = 0.
        self.last_command = Command(0., 0.)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.
        self.stationary_since = None

    @property
    def active(self):
        return self.status in ('RUNNING', 'RECOVERING', 'STOPPING')

    def fault(self, reason):
        if self.status != 'FAULT':
            self.generation += 1
        self.status, self.reason, self.plan = 'FAULT', str(reason), None
        self.pending_plan = None
        self.last_command = Command(0., self.last_command.steering)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.

    def observe(self, state, received, progress, cross_track, heading_error=0.):
        if not all(math.isfinite(v) for v in asdict(state).values()):
            self.fault('Nonfinite state'); return
        if self.state is not None:
            dt = state.timestamp-self.state.timestamp
            if dt < 0:
                self.fault('Measurement clock moved backwards')
            if self.active and (math.hypot(state.x-self.state.x, state.y-self.state.y) >
                                self.config.max_speed*max(dt, 0)+.15 or
                                abs(angle_difference(state.yaw, self.state.yaw)) > .5):
                self.fault('Localization discontinuity')
        if self.active:
            delta = (progress-self.wrapped_progress+self.length/2) % self.length-self.length/2
            self.progress += delta
        self.wrapped_progress = progress
        self.cross_track, self.heading_error = cross_track, heading_error
        self.state, self.state_received = state, received

    def set_mode(self, enabled, received):
        if self.solve_period is not None and self.active and self.mode and not enabled:
            self.fault('Autonomy withdrawn; re-enable required')
        self.mode, self.mode_received = bool(enabled), received

    def fresh(self, now):
        return (self.state is not None and 0 <= now-self.state_received <= self.STATE_TIMEOUT
                and 0 <= now-self.mode_received <= self.MODE_TIMEOUT)

    def start(self, now):
        if not self.fresh(now):
            raise ValueError('Fresh odometry and RC selector status required')
        if abs(self.state.speed) > .1:
            raise ValueError('Start requires a stationary vehicle')
        # The ROS adapter checks the full body against the configured corridor
        # before starting. A second fixed centerline-distance gate can reject
        # valid starting poses on a wide corridor.
        if abs(self.heading_error) > math.radians(30):
            raise ValueError('Start requires heading error within 30 degrees of the nearest path point')
        self.generation += 1
        self.status, self.reason, self.plan = 'RUNNING', '', None
        self.pending_plan = None
        self.handover_error = None
        self.started = self.last_tick = now
        self.last_usable_update=now
        self.consecutive_failures=self.recovery_good_candidates=0
        self.start_progress = self.wrapped_progress
        self.progress = self.start_progress
        self.lap_goal = self.start_progress+self.length
        self.stationary_since = None
        self.last_command = Command(0., self.state.steering)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.

    def stop(self):
        if self.active:
            if self.mode:
                self.status, self.reason = 'STOPPING', 'Operator stop requested'
            else:
                self.status, self.reason, self.plan = 'READY', 'Controller disabled in manual mode', None
                self.pending_plan = None
                self.last_command = Command(0., self.last_command.steering)
                self.last_acceleration = self.last_steering_rate = 0.

    def accept(self, result, now):
        if not self.active or result.get('generation') != self.generation:
            return False
        try:
            finite = all(math.isfinite(v) for row in result['states']+result['controls'] for v in row)
            valid = (result['success'] and finite and len(result['states']) >= 2
                     and len(result['controls']) == len(result['states'])-1
                     and all(len(row)==6 for row in result['states'])
                     and all(len(row)==3 for row in result['controls'])
                     and math.isclose(result.get('dt',.1),.1,rel_tol=0.,abs_tol=1e-9)
                     and math.isfinite(result['constraint_violation'])
                     and result['constraint_violation'] < 1e-4
                     and 0 <= now-result['source_stamp'] <= self.PLAN_TTL)
            valid = (valid and result['source_stamp']<=result['submitted_at']+1e-9
                         and result['submitted_at']<=result['stamp']+1e-9
                         and math.isclose(result['stamp']-result['submitted_at'],self.handover_delay,
                                          rel_tol=0.,abs_tol=1e-6))
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            if self.solve_period is None:
                self.fault('Invalid, failed, or expired solver result')
            else:
                self.solver_failure(now, 'Invalid, failed, or expired solver result')
            return False
        # A result can arrive early; it must not change the input prefix
        # used to predict its own initial state. Missing epochs are rejected.
        self.pending_plan = result
        return True

    def solver_failure(self, now, reason):
        """A bad optimization result cannot extend a plan or revoke fresh inputs."""
        self.consecutive_failures+=1
        self.recovery_good_candidates=0
        if (self.active and self.solve_period is not None and
                self.status!='STOPPING' and self.last_usable_update is not None and
                now-self.last_usable_update>=2*self.solve_period-1e-9):
            self.status,self.reason='RECOVERING',str(reason)

    def reproject_controls(self, initial, applied, controls, optimized_applied, dt):
        """Repair only actuator-prefix differences in never-executed controls.

        This does not authorize execution: original and repaired trajectories
        must both pass independent validation, including the operating envelope.
        """
        import hashlib
        import json
        from .envelope import jerk_limits
        limits=jerk_limits(initial,applied,self.config,dt,len(controls))
        acceleration,steering,rate=applied
        repaired=[]
        acceleration_change=steering_change=0.
        changed_intervals=0
        for command,jerk in zip(controls,limits):
            lower=max(-self.config.brake_limit,acceleration-jerk*dt)
            upper=min(self.config.accel_limit,acceleration+jerk*dt)
            rate_lower=max(-self.config.steer_rate,rate-self.config.steer_acceleration*dt,
                           (-self.config.steer_limit-steering)/dt)
            rate_upper=min(self.config.steer_rate,rate+self.config.steer_acceleration*dt,
                           (self.config.steer_limit-steering)/dt)
            if lower>upper or rate_lower>rate_upper:
                raise ValueError('Actual actuator prefix has no bounded continuation')
            acceleration=clip(command[0],lower,upper)
            rate=clip((command[1]-steering)/dt,rate_lower,rate_upper)
            steering+=rate*dt
            repaired.append([acceleration,steering,command[2]])
            da,ds=abs(acceleration-command[0]),abs(steering-command[1])
            acceleration_change=max(acceleration_change,da)
            steering_change=max(steering_change,ds)
            changed_intervals+=int(da>1e-12 or ds>1e-12)
        prefix_difference=[abs(a-b) for a,b in zip(applied,optimized_applied)]
        # A projection must be attributable to the changed input prefix. A
        # different recovery jerk regime or other large repair is rejected.
        if (acceleration_change>prefix_difference[0]+1e-4 or
                steering_change>min(self.HANDOVER_COMMAND_LIMITS['command_steering'],
                    prefix_difference[1]+len(controls)*dt*prefix_difference[2])+1e-4):
            raise ValueError('Control reprojection exceeds actual-prefix correction bound')
        return repaired,dict(
            original_controls_sha256=hashlib.sha256(json.dumps(controls,sort_keys=True,
                allow_nan=False).encode()).hexdigest(),
            actual_prefix_difference=prefix_difference,
            max_acceleration_correction=acceleration_change,
            max_steering_correction=steering_change,changed_intervals=changed_intervals)

    def activate(self,now,actual,actual_command=None,expected_at_activation=None,path=None,progress=None,
                 map_alignment=None):
        """Activate at the scheduled epoch and record prediction error."""
        result=self.pending_plan
        if result is None or now<result['stamp']-self.HANDOVER_TOLERANCE:
            return False
        self.pending_plan=None
        if not self.active or result.get('generation')!=self.generation:
            return False
        if not 0<=now-result['source_stamp']<=self.PLAN_TTL:
            if self.solve_period is None:
                self.fault('Expired plan at handover')
            else:
                self.rejected_plans+=1
                self.solver_failure(now,'Expired candidate at handover')
            return False
        expected=result['states'][0]
        if expected_at_activation is not None:
            expected=[expected_at_activation.x,expected_at_activation.y,expected_at_activation.yaw,
                      expected_at_activation.speed,expected[4],expected_at_activation.steering]
        self.handover_error=dict(position=math.hypot(actual.x-expected[0],actual.y-expected[1]),
                                yaw=abs(angle_difference(actual.yaw,expected[2])),
                                speed=abs(actual.speed-expected[3]),
                                steering=abs(actual.steering-expected[5]))
        # These differences concern the MPCC prediction and its executed input
        # prefix, independently of point-cloud or localization diagnostics.
        limits=dict(self.HANDOVER_STATE_LIMITS)
        if actual_command is not None and 'handover_command' in result:
            boundary=result['handover_command']
            self.handover_error.update(command_speed=abs(actual_command['speed']-boundary['speed']),
                                       command_steering=abs(actual_command['steering']-boundary['steering']))
            limits.update(self.HANDOVER_COMMAND_LIMITS)
        if any(self.handover_error[key]>limit+1e-12 for key,limit in limits.items()):
            self.rejected_plans+=1
            if self.solve_period is not None:
                self.solver_failure(now,'Candidate differs from actual handover state or command')
            return False
        if self.solve_period is not None:
            # New controls were never executed while the solver ran. Replay
            # every interval from the current state, with the real input boundary.
            if actual_command is None:
                self.rejected_plans+=1
                self.solver_failure(now,'Actual applied command required at handover')
                return False
            import time
            from .envelope import independent_rollout
            from .validation import validate_candidate,candidate_fingerprint
            validation_started=time.perf_counter()
            raw_validation=result.get('validation')
            try:
                if (raw_validation is None or
                        raw_validation.get('candidate_fingerprint')!=candidate_fingerprint(result,self.config)):
                    raw_validation=validate_candidate(result,self.config,path)
            except (ValueError,TypeError) as exc:
                raw_validation=dict(accepted=False,reason=str(exc))
            if not raw_validation['accepted']:
                self.rejected_plans+=1
                self.solver_failure(now,raw_validation['reason'])
                return False
            initial=[actual.x,actual.y,actual.yaw,max(0.,actual.speed),
                     expected[4] if progress is None else progress,actual.steering]
            applied=[actual_command[k] for k in ('acceleration','steering','steering_rate')]
            dt=result.get('dt',.1)
            try:
                controls,correction=self.reproject_controls(initial,applied,result['controls'],
                                                             result['validation_applied'],dt)
                samples=independent_rollout(initial,applied,controls,self.config,dt)
                rebased=dict(result,states=samples[::round(dt/.02)].tolist(),controls=controls,
                             handover_reprojection=correction,
                             validation_applied=applied,previous_steering=actual_command['steering'])
                if map_alignment is not None:
                    rebased['map_alignment']=map_alignment
                validation=validate_candidate(rebased,self.config,path)
            except (ValueError,TypeError,OverflowError) as exc:
                validation=dict(accepted=False,reason=str(exc))
            if not validation['accepted']:
                self.rejected_plans+=1
                self.solver_failure(now,validation['reason'])
                return False
            result=dict(rebased,validation=validation,
                        handover_validation_time_s=time.perf_counter()-validation_started)
        # Execution starts here, even if the control tick is a little late.
        # Never skip new inputs that were not executed during that lateness.
        self.plan=dict(result,scheduled_stamp=result['stamp'],stamp=now)
        self.last_usable_update=now
        self.consecutive_failures=0
        if self.status=='RECOVERING':
            self.recovery_good_candidates+=1
            if self.recovery_good_candidates>=2 and actual.speed>=.05:
                self.status,self.reason='RUNNING',''
                self.recovery_good_candidates=0
        return True

    def refs(self, n, dt):
        if self.status in ('STOPPING','RECOVERING'):
            return [0.]*(n+1)
        remaining = max(0., self.lap_goal-self.progress)
        speed = max(0., self.state.speed)
        return [min(self.config.cruise_speed, speed_envelope(
            remaining-i*dt*speed, self.config.brake_limit,
            self.config.jerk_limit, self.PLAN_TTL)) for i in range(n+1)]

    def minimum_speed_stop_distance(self):
        speed = self.config.minimum_drive_speed
        return speed*(self.config.brake_limit/self.config.jerk_limit+self.PLAN_TTL) + speed**2/(2*self.config.brake_limit)

    def actuator_command(self, command):
        """Invert the speed-mode dead zone, preserving explicit stop commands.

        This is a motor setpoint, not a claim that physical speed jumps. The
        continuous OCP/smoothing speed remains separate from this wire value.
        """
        minimum = self.config.minimum_drive_speed
        if not self.active or command.speed <= 1e-6:
            return Command(0., command.steering)
        if command.speed >= minimum:
            return command
        if self.status == 'RUNNING' and self.lap_goal-self.progress > self.minimum_speed_stop_distance():
            return Command(minimum, command.steering)
        return Command(0., command.steering)

    def command(self, now, *, enforce_plan_age=True):
        """Evaluate a command; future bridge simulation may ignore plan expiry.

        Real output always uses the default expiry check. A forecast can use
        remaining prediction controls without turning future expiry into an
        immediate fault or extending the real plan's lifetime.
        """
        if not self.active:
            return Command(0., self.last_command.steering)
        if not self.fresh(now):
            self.fault('Odometry or selector status expired'); return self.last_command
        if self.last_tick is not None and (now < self.last_tick or now-self.last_tick > .10):
            self.fault('Command scheduling discontinuity'); return self.last_command
        dt = clip(now-(self.last_tick if self.last_tick is not None else now), 0., .05)
        self.last_tick = now
        if (enforce_plan_age and self.solve_period is not None and self.status=='RUNNING' and
                self.last_usable_update is not None and
                now-self.last_usable_update>=2*self.solve_period-1e-9):
            self.status,self.reason='RECOVERING','No usable plan update for two planning periods'
            self.recovery_good_candidates=0
        recovering=self.status=='RECOVERING'
        if self.plan is None and not recovering:
            if now-self.started > self.PLAN_TTL:
                self.fault('Initial plan deadline expired')
            return Command(0., self.last_command.steering)
        if self.plan is not None:
            age=now-self.plan.get('source_stamp',self.plan['stamp'])
            phase=now-self.plan['stamp']
            expired=age<0 or (enforce_plan_age and age>self.PLAN_TTL)
            exhausted=phase>=len(self.plan['controls'])*.1
            if phase<0:
                self.fault('Plan activated before handover');return self.last_command
            if expired or exhausted:
                if self.solve_period is None or not enforce_plan_age or age<0:
                    self.fault('Plan expired' if expired else 'Prediction horizon exhausted')
                    return self.last_command
                self.plan=None
                if self.status!='STOPPING':
                    self.status,self.reason='RECOVERING','Plan expired' if expired else 'Prediction horizon exhausted'
                recovering=self.status=='RECOVERING'
        if self.plan is None or recovering:
            target_speed=0.
            target_steer=self.last_command.steering
        else:
            index=min(len(self.plan['controls'])-1,int(phase/.1))
            fraction=clip((phase-index*.1)/.1,0.,1.)
            states,controls=self.plan['states'],self.plan['controls']
            target_speed=states[index][3]*(1-fraction)+states[index+1][3]*fraction
            previous_steer=self.plan.get('previous_steering',self.last_command.steering) if index==0 else controls[index-1][1]
            target_steer=previous_steer+(controls[index][1]-previous_steer)*fraction
        remaining = self.lap_goal-self.progress
        if self.status in ('STOPPING','RECOVERING') or remaining <= max(.05, self.minimum_speed_stop_distance()):
            target_speed = 0.
        # Enforce the finish envelope on executed targets as well as the OCP's
        # soft speed references. The command still passes through smooth limits.
        finish_speed = speed_envelope(remaining, self.config.brake_limit,
                                      self.config.jerk_limit, self.PLAN_TTL)
        target_speed = clip(target_speed, 0., min(self.config.max_speed,
                                                self.config.cruise_speed, finish_speed))
        if dt > 0:
            desired_accel = clip((target_speed-self.last_command.speed)/dt,
                                 -self.config.brake_limit, self.config.accel_limit)
            # Reserve speed change to ramp acceleration to zero before a hard
            # speed bound: a*dt + a*a/(2*jerk) <= remaining speed change.
            jerk = self.config.jerk_limit
            def safe_acceleration(distance):
                return math.sqrt((jerk*dt)**2+2*jerk*max(0., distance))-jerk*dt
            accel_lower = max(-self.config.brake_limit, self.last_acceleration-jerk*dt,
                              -safe_acceleration(self.last_command.speed))
            accel_upper = min(self.config.accel_limit, self.last_acceleration+jerk*dt,
                              safe_acceleration(self.config.max_speed-self.last_command.speed))
            if accel_lower > accel_upper+1e-10:
                self.fault('Acceleration cannot stop within speed limits')
                return self.last_command
            accel = clip(desired_accel, accel_lower, accel_upper)
            speed = clip(self.last_command.speed+accel*dt, 0., self.config.max_speed)
            accel = (speed-self.last_command.speed)/dt
            desired_rate = clip((target_steer-self.last_command.steering)/dt,
                                -self.config.steer_rate,self.config.steer_rate)
            # Reserve enough angular distance to brake the steering rate before
            # a hard angle limit. r*dt + r*r/(2*a) <= remaining angle.
            angular_accel = self.config.steer_acceleration
            def safe_rate(distance):
                return math.sqrt((angular_accel*dt)**2 +
                                 2*angular_accel*max(0., distance))-angular_accel*dt
            upper = min(self.config.steer_rate,
                        self.last_steering_rate+angular_accel*dt,
                        safe_rate(self.config.steer_limit-self.last_command.steering))
            lower = max(-self.config.steer_rate,
                        self.last_steering_rate-angular_accel*dt,
                        -safe_rate(self.config.steer_limit+self.last_command.steering))
            if lower > upper+1e-10:
                self.fault('Steering rate cannot stop within angle limits')
                return self.last_command
            rate = clip(desired_rate, lower, upper)
            steer = clip(self.last_command.steering+rate*dt,
                         -self.config.steer_limit, self.config.steer_limit)
            self.last_steering_rate = (steer-self.last_command.steering)/dt
            self.last_acceleration = accel
        else:
            speed, steer = self.last_command.speed, self.last_command.steering
        self.last_command = Command(speed, clip(steer, -self.config.steer_limit, self.config.steer_limit))
        finish_tolerance = max(.2, self.minimum_speed_stop_distance()+.05)
        finish = abs(remaining) <= finish_tolerance and self.progress >= self.lap_goal-finish_tolerance
        if (finish or self.status in ('STOPPING','RECOVERING')) and abs(self.state.speed)<.05 and speed<1e-6:
            if self.stationary_since is None:
                self.stationary_since = now
            if now-self.stationary_since >= .5:
                recovery_stopped=self.status=='RECOVERING'
                self.status = 'COMPLETE' if finish else 'READY'
                self.reason = 'One lap complete' if finish else ('Recovery stopped; re-enable required' if recovery_stopped else 'Stopped')
                self.plan = None
                self.pending_plan = None
                self.last_command = Command(0., self.last_command.steering)
        else:
            self.stationary_since = None
        if remaining < -.2:
            self.fault('Finish overshoot')
        return self.last_command
