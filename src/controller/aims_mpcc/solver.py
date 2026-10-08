"""CasADi/IPOPT port of f1tenth_mpcc.solver (see NOTICE.md).

Rear axle state order: x,y,yaw,speed,progress,actual steering.
Controls: longitudinal acceleration, steering command, virtual progress speed.
"""
import time
import casadi as ca
import numpy as np
from .config import VehicleConfig
from .path import ReferencePath
from .vendor import global_kinematic_model, contouring_lag, normalized_cost
from .solver_diagnostics import constraint_summary, convergence, json_safe
from .envelope import evaluate_envelope, initial_envelope_diagnostic, jerk_limits

class MPCCSolver:
    def __init__(self, path: ReferencePath, config: VehicleConfig, horizon=10, dt=.1, jit_enabled=False, native_options=None):
        config.validate(require_verified=True)
        path.validate_config(config)
        if isinstance(horizon, bool) or int(horizon)!=horizon or horizon<1: raise ValueError('positive integer horizon required')
        if not np.isfinite(dt) or dt<=0: raise ValueError('positive finite dt required')
        if min(path.left_width,path.right_width)<=config.half_width: raise ValueError('insufficient corridor width')
        self.path,self.config,self.n,self.dt=path,config,int(horizon),float(dt)
        self.reference=path
        self.modules={'model':global_kinematic_model,'errors':contouring_lag,'cost':normalized_cost}
        for key in ['wheelbase','rear_offset','steer_limit','steer_rate','jerk_limit','steering_tau','steer_acceleration','understeer_coefficient']:
            setattr(self,key,getattr(config,key))
        self.speed=config.cruise_speed
        self.actuator_dt=.02
        self.substeps=round(self.dt/self.actuator_dt)
        if self.substeps<1 or not np.isclose(self.substeps*self.actuator_dt,self.dt):
            raise ValueError('dt must be a multiple of 20 ms')
        self.weights=dict(normalized_cost.RATIOS)
        self.weights.pop('steer_rate');self.weights.pop('vtheta_rate')
        self.weights.update(speed=config.speed_weight,heading=config.heading_weight)
        self.corner_offsets=[(along,sy*config.half_width)
                             for along in config.longitudinal_offsets() for sy in (1,-1)]
        self.jit_enabled=bool(jit_enabled)
        self.native_options=dict(native_options or {})
        self.reset()
        self._build()

    def reset(self):
        self.previous=None; self.previous_theta=None;self.previous_yaw=None
        self.previous_elapsed=0.
        self.consecutive_failures=0

    def _dynamics(self, state, control, symbolic=True, steering_bias=0., previous_steering=None):
        if previous_steering is None:
            previous_steering = control[1]
        def rhs(value, commanded):
            if symbolic:
                effective = ca.vertcat(control[0], value[5], control[2])
                base = self.modules["model"].rhs_symbolic(value[:5], effective, self.wheelbase)
                base[2] /= 1 + self.understeer_coefficient * value[3] ** 2
                return ca.vertcat(base, (commanded + steering_bias - value[5]) / self.steering_tau)
            effective = np.array([control[0], value[5], control[2]])
            base = self.modules["model"].rhs_numpy(value[:5], effective, self.wheelbase)
            base[2] /= 1 + self.understeer_coefficient * value[3] ** 2
            return np.r_[base, (commanded + steering_bias - value[5]) / self.steering_tau]
        samples = [state]
        h = self.actuator_dt
        for index in range(self.substeps):
            commanded = previous_steering + (control[1] - previous_steering) * (index + 1) / self.substeps
            k1 = rhs(state, commanded)
            k2 = rhs(state + .5 * h * k1, commanded)
            k3 = rhs(state + .5 * h * k2, commanded)
            k4 = rhs(state + h * k3, commanded)
            state = state + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
            samples.append(state)
        return state, tuple(samples)

    def _build(self):
        op = ca.Opti()
        x, u = op.variable(6, self.n + 1), op.variable(3, self.n)
        self.initial = op.parameter(6)
        self.applied = op.parameter(3)  # acceleration, steering endpoint, previous steering ramp rate
        self.steering_bias = op.parameter()
        self.speed_refs = op.parameter(self.n + 1)
        self.cost_weights = op.parameter(7)
        self.active_jerk = op.parameter(self.n)
        self.envelope_slack = op.variable(self.n) if self.config.envelope_soft_enabled else None
        contour_w, heading_w, speed_w, steering_w, rate_w, rate_accel_w, terminal_w = (
            self.cost_weights[i] for i in range(7))
        self.map_alignment = op.parameter(3) if self.path.frame_id == 'map' else None
        self.constraint_blocks=[]
        def constrain(expression, group, interval=None, substep=None):
            first=op.ng
            op.subject_to(expression)
            self.constraint_blocks.append(dict(first=first,end=op.ng,group=group,
                                               interval=interval,substep=substep))
        constrain(x[:, 0] == self.initial, 'initial_state')
        constrain(op.bounded(0., x[3, :], self.config.max_speed), 'speed_bounds')
        constrain(op.bounded(-self.config.brake_limit, u[0, :], self.config.accel_limit), 'acceleration_bounds')
        constrain(op.bounded(-self.steer_limit, u[1, :], self.steer_limit), 'steering_bounds')
        constrain(op.bounded(0., u[2, :], self.config.max_speed), 'progress_speed_bounds')
        weights = self.weights
        objective = 0
        if self.envelope_slack is not None:
            for k in range(self.n):
                # No slack at/after the bounded recovery deadline. An interval
                # straddling that deadline uses its own early samples only.
                cap = self.config.envelope_slack_limit if k*self.dt < self.config.envelope_recovery_time-1e-10 else 0.
                constrain(op.bounded(0., self.envelope_slack[k], cap), 'envelope_slack_bounds', k)
            objective += self.config.envelope_slack_weight*ca.sumsqr(self.envelope_slack)
        self.margins = []
        self.corridor_rows = []

        def geometry(state, interval, substep=None):
            # Dynamics and warm starts remain continuous in odom. The persistent
            # map spline is compared through one alignment snapshot per solve.
            if self.map_alignment is not None:
                tx, ty, angle = self.map_alignment[0], self.map_alignment[1], self.map_alignment[2]
                c, s = ca.cos(angle), ca.sin(angle)
                reference_state = ca.vertcat(tx + c * state[0] - s * state[1],
                                             ty + s * state[0] + c * state[1],
                                             state[2] + angle, state[3:])
            else:
                reference_state = state
            ec, el, ref = self.modules["errors"].symbolic_errors(reference_state, self.reference)
            tangent = ref["dxy"] / ca.sqrt(ca.dot(ref["dxy"], ref["dxy"]))
            normal = ca.vertcat(-tangent[1], tangent[0])
            heading = ca.vertcat(ca.cos(reference_state[2]), ca.sin(reference_state[2]))
            left = ca.vertcat(-ca.sin(reference_state[2]), ca.cos(reference_state[2]))
            for along, across in self.corner_offsets:
                corner = reference_state[:2] + along * heading + across * left
                lateral = ca.dot(corner - ref["xy"], normal)
                if self.config.enforce_corridor:
                    constrain(op.bounded(-self.path.right_width, lateral, self.path.left_width),
                              'corridor', interval, substep)
                    self.corridor_rows.append(op.ng - 1)
                self.margins.extend((self.path.left_width - lateral, self.path.right_width + lateral))
            # Periodic, wrap-safe heading error: 2*(1-cos(error)) ~ error^2.
            ref["heading_error_squared"] = 2 * (1 - ca.dot(heading, tangent))
            return ec, el, ref

        # Share the microstep integration graph between prediction intervals.
        sx, su = ca.MX.sym("x", 6), ca.MX.sym("u", 3)
        sp, sb = ca.MX.sym("previous_steering"), ca.MX.sym("steering_bias")
        end, stages = self._dynamics(sx, su, steering_bias=sb, previous_steering=sp)
        transition = ca.Function("f1tenth_interval", [sx, su, sp, sb], [end, ca.horzcat(*stages)]).expand()
        self.transition = transition

        for k in range(self.n):
            previous = self.applied if k == 0 else u[:2, k - 1]
            end, sample_matrix = transition(x[:, k], u[:, k], previous[1], self.steering_bias)
            stages = [sample_matrix[:, j] for j in range(self.substeps + 1)]
            constrain(x[:, k + 1] == end, 'dynamics', k)
            constrain(op.bounded(-self.active_jerk[k] * self.dt, u[0, k] - previous[0], self.active_jerk[k] * self.dt),
                      'jerk', k)
            rate = (u[1, k] - previous[1]) / self.dt
            previous_rate = self.applied[2] if k == 0 else (u[1, k - 1] - (self.applied[1] if k == 1 else u[1, k - 2])) / self.dt
            constrain(op.bounded(-self.steer_rate, rate, self.steer_rate), 'steering_rate', k)
            constrain(op.bounded(-self.steer_acceleration * self.dt, rate - previous_rate, self.steer_acceleration * self.dt),
                      'steering_acceleration', k)
            # Check acceleration utilization at each 20 ms integration node.
            for substep, stage in enumerate(stages):
                lateral_accel = stage[3] ** 2 * ca.tan(stage[5]) / (self.wheelbase * (1 + self.understeer_coefficient * stage[3] ** 2))
                accel_axis, brake_axis, lateral_axis = self.config.envelope_halfaxes()
                scale = ca.if_else(u[0, k] >= 0, accel_axis, brake_axis)
                utilization = (u[0, k]/scale)**2 + (lateral_accel/lateral_axis)**2
                if self.envelope_slack is not None:
                    if k == 0 and substep == 0:
                        continue  # immutable initial state is diagnosed independently
                    slack = (self.envelope_slack[k] if k*self.dt+substep*self.actuator_dt
                             < self.config.envelope_recovery_time-1e-10 else 0.)
                    constrain(utilization <= 1.+slack, 'operating_envelope', k, substep)
                else:
                    constrain(utilization <= 1., 'acceleration_ellipse', k, substep)
            ec, el, ref = geometry(x[:, k], k, 0)
            geometry(stages[(self.substeps + 1) // 2], k, (self.substeps + 1) // 2)
            delta_ff = ca.atan(self.wheelbase * (1 + self.understeer_coefficient * x[3, k] ** 2) * ref["curvature"]) - self.steering_bias
            objective += contour_w * (ec / self.config.contour_scale) ** 2 + weights["lag"] * (el / self.config.lag_scale) ** 2
            objective += heading_w * ref["heading_error_squared"] / self.config.heading_scale ** 2
            objective += speed_w * ((x[3, k] - self.speed_refs[k]) / self.config.speed_scale) ** 2
            objective += weights["progress"] * ((u[2, k] - self.speed_refs[k]) / self.config.speed_scale) ** 2
            objective += steering_w * ((u[1, k] - delta_ff) / self.config.steering_scale) ** 2
            objective += weights["accel"] * (u[0, k] / self.config.acceleration_scale) ** 2
            objective += rate_w * (rate / self.steer_rate) ** 2 + rate_accel_w * ((rate - previous_rate) / (self.steer_acceleration * self.dt)) ** 2
        ec, el, ref = geometry(x[:, -1], self.n, 0)
        objective += terminal_w * (contour_w * (ec / self.config.contour_scale) ** 2 + weights["lag"] * (el / self.config.lag_scale) ** 2
                          + heading_w * ref["heading_error_squared"] / self.config.heading_scale ** 2
                          + speed_w * ((x[3, -1] - self.speed_refs[-1]) / self.config.speed_scale) ** 2)
        op.minimize(objective)
        # Standalone diagnostics retain their default; the worker supplies cached
        # -O2 callbacks in a private working directory.
        options = {"print_time": False, "jit": self.jit_enabled, "compiler": "shell",
                   "jit_options": {"flags": ["-O0"]}}
        options.update(self.native_options)
        op.solver("ipopt", options, {"print_level": 0, "sb": "yes", "linear_solver": "mumps",
                   "max_iter": self.config.solver_max_iterations, "tol": 1e-5, "acceptable_tol": 1e-4, "acceptable_iter": 3,
                   "warm_start_init_point": "yes"})
        self.op, self.x, self.u = op, x, u
        self.corridor_rows = np.asarray(self.corridor_rows, dtype=int)

    def _warm_start(self, initial, applied, refs, elapsed):
        states=[initial]
        shifted=None
        if self.previous is not None and elapsed < self.n*self.dt:
            shift=min(self.n,max(1,round(elapsed/self.dt)))
            shifted=np.vstack((self.previous['controls'][shift:],
                               np.repeat(self.previous['controls'][-1:],shift,axis=0)))
        acceleration,steering,rate=applied
        limits=jerk_limits(initial,applied,self.config,self.dt,self.n)
        controls=[]
        for k in range(self.n):
            state=states[-1]
            if shifted is None:
                ref=self.path.at(state[4])
                desired_acceleration=(refs[k+1]-state[3])/self.dt
                desired_steering=np.arctan(self.wheelbase*(1+self.understeer_coefficient*state[3]**2)*ref['curvature'])
            else:
                desired_acceleration,desired_steering=shifted[k,:2]
            acceleration=np.clip(desired_acceleration,
                max(-self.config.brake_limit,acceleration-limits[k]*self.dt),
                min(self.config.accel_limit,acceleration+limits[k]*self.dt))
            desired_rate=(np.clip(desired_steering,-self.steer_limit,self.steer_limit)-steering)/self.dt
            rate=np.clip(desired_rate,max(-self.steer_rate,rate-self.steer_acceleration*self.dt),
                         min(self.steer_rate,rate+self.steer_acceleration*self.dt))
            next_steering=np.clip(steering+rate*self.dt,-self.steer_limit,self.steer_limit)
            rate=(next_steering-steering)/self.dt
            tangent_norm=np.linalg.norm(self.path.curve.numpy(state[4],1))
            progress_speed=max(0.,state[3]+.5*acceleration*self.dt)/tangent_norm
            control=np.array([acceleration,next_steering,min(self.config.max_speed,progress_speed)])
            end,_=self.transition(state,control,steering,0.)
            states.append(np.asarray(end).reshape(-1));controls.append(control);steering=next_steering
        return np.asarray(states),np.asarray(controls)

    def _diagnostics(self, stats, value):
        """Value may be a solution or Opti.debug after a failed IPOPT call."""
        summary=dict(schema_version=1, iteration_limit=self.config.solver_max_iterations,
                     **convergence(stats))
        constraints=None
        try:
            constraints=[np.asarray(value(item),dtype=float).reshape(-1)
                         for item in (self.op.g,self.op.lbg,self.op.ubg)]
            summary.update(constraint_summary(*constraints,self.constraint_blocks))
        except (RuntimeError,ValueError,TypeError) as exc:
            summary.update(max_constraint_violation=None,constraint_violations={},
                           worst_constraints=[],constraint_diagnostic_error=str(exc))
        if summary['objective'] is None:
            try:
                summary['objective']=json_safe(float(value(self.op.f)))
            except (RuntimeError,ValueError,TypeError):
                pass
        return summary,constraints

    def solve(self, state, previous, speed_refs=None, elapsed=.1, map_alignment=None):
        started=time.perf_counter()
        measured=np.array([state[k] for k in ['x','y','yaw','speed','steering']],float)
        applied=np.array([previous[k] for k in ['acceleration','steering','steering_rate']],float)
        if not np.isfinite(measured).all() or not np.isfinite(applied).all() or not np.isfinite(elapsed) or elapsed<=0:
            raise ValueError('finite state, previous command and positive elapsed required')
        # The forward-only OCP uses a nonnegative initial speed. Signed speed
        # remains in supervisor telemetry; it is not a range-rejection gate.
        measured[3] = max(0., measured[3])
        refs=np.full(self.n+1,self.config.cruise_speed) if speed_refs is None else np.asarray(speed_refs,float)
        if refs.shape!=(self.n+1,) or not np.isfinite(refs).all() or np.any(refs<0) or np.any(refs>self.config.max_speed):
            raise ValueError('speed_refs must be n+1 finite values within speed limits')
        if self.map_alignment is not None:
            alignment = np.asarray(map_alignment, float)
            if alignment.shape != (3,) or not np.isfinite(alignment).all():
                raise ValueError('map reference requires a finite map-to-odom alignment')
            c, s = np.cos(alignment[2]), np.sin(alignment[2])
            reference_xy = alignment[:2] + np.array((c * measured[0] - s * measured[1],
                                                       s * measured[0] + c * measured[1]))
            self.op.set_value(self.map_alignment, alignment)
        else:
            if map_alignment is not None:
                raise ValueError('odom reference must not receive a map alignment')
            reference_xy = measured[:2]
        theta,_=self.path.project(reference_xy);yaw=measured[2]
        if self.previous_theta is not None:
            theta=self.previous_theta+(theta-self.previous_theta+self.path.length/2)%self.path.length-self.path.length/2
            yaw=self.previous_yaw+(yaw-self.previous_yaw+np.pi)%(2*np.pi)-np.pi
        initial=np.r_[measured[:2],yaw,measured[3],theta,measured[4]]
        self.op.set_value(self.initial,initial);self.op.set_value(self.applied,applied)
        self.op.set_value(self.active_jerk,jerk_limits(initial,applied,self.config,self.dt,self.n))
        self.op.set_value(self.steering_bias,0.);self.op.set_value(self.speed_refs,refs)
        self.op.set_value(self.cost_weights, [self.config.contour_weight, self.config.heading_weight,
            self.config.speed_weight, self.config.steering_weight, self.config.steering_rate_weight,
            self.config.steering_acceleration_weight, self.config.terminal_weight])
        # elapsed is between REQUESTS. Retained controls belong to the last
        # successful request, so failed cycles must accumulate their full age.
        if self.previous is not None:
            self.previous_elapsed+=elapsed
        cache_age=self.previous_elapsed if self.previous is not None else None
        use_cache=self.previous is not None and cache_age < self.n*self.dt
        shift=min(self.n,max(1,round(cache_age/self.dt))) if use_cache else 0
        warm_states,warm_u=self._warm_start(initial,applied,refs,cache_age if cache_age is not None else elapsed)
        self.op.set_initial(self.x,np.asarray(warm_states).T);self.op.set_initial(self.u,warm_u.T)
        if self.envelope_slack is not None:
            self.op.set_initial(self.envelope_slack,0.)
        prepared=time.perf_counter()
        error=None
        try:
            solution=self.op.solve()
        except RuntimeError as exc:
            solution=None
            error=str(exc)
        optimized=time.perf_counter()
        try:
            stats=self.op.stats()
        except RuntimeError:
            stats={}
        value=solution.value if solution is not None else self.op.debug.value
        diagnostics,constraints=self._diagnostics(stats,value)
        violation=diagnostics['max_constraint_violation']
        states=controls=None
        try:
            states=np.asarray(value(self.x),dtype=float).reshape(6,self.n+1).T
            controls=np.asarray(value(self.u),dtype=float).reshape(3,self.n).T
        except (RuntimeError,ValueError,TypeError):
            pass
        envelope_traces = None
        if not self.config.envelope_soft_enabled:
            # The common validator independently checks accepted trajectories.
            # Keep the baseline solver's hot reply to the cheap x0 proof only.
            diagnostics['envelope'] = dict(
                **initial_envelope_diagnostic(initial,applied,self.config,self.dt),
                execution_authorized=False, diagnostic_scope='initial_state_only')
        elif controls is not None and np.isfinite(controls).all():
            try:
                envelope = evaluate_envelope(initial,applied,controls,self.config,self.dt)
                envelope_traces = {key: envelope.pop(key) for key in
                                   ('candidate_samples','reference_samples','reference_controls')}
                diagnostics['envelope'] = dict(envelope,diagnostic_scope='independent_recovery_summary')
                if self.envelope_slack is not None:
                    diagnostics['envelope']['optimizer_slack_max']=float(np.max(value(self.envelope_slack)))
            except (RuntimeError,ValueError,TypeError,OverflowError) as exc:
                diagnostics['envelope']={'execution_authorized':False,'diagnostic_error':str(exc)}
        success=bool(solution is not None and stats.get('success') and violation is not None and
                     violation<1e-4 and states is not None and controls is not None and
                     np.isfinite(states).all() and np.isfinite(controls).all())
        result=dict(success=success,status=stats.get('return_status','exception'),
                    iterations=int(stats.get('iter_count',0)),constraint_violation=violation,
                    corridor_enforced=self.config.enforce_corridor,
                    envelope_soft_enabled=self.config.envelope_soft_enabled,
                    execution_authorized=False)
        result['solve_input']=json_safe(dict(initial_state=initial,applied=applied,speed_refs=refs,
                                             map_alignment=map_alignment,warm_states=warm_states,warm_controls=warm_u))
        if error is not None:
            result['error']=error
        if success:
            result.update(states=states.tolist(),controls=controls.tolist())
            g,lower,upper=constraints
            rows=self.corridor_rows
            result['minimum_predicted_margin_m']=(float(np.minimum(upper[rows]-g[rows],g[rows]-lower[rows]).min())
                                                  if len(rows) else None)
            self.previous=dict(states=states,controls=controls)
            self.previous_theta=theta;self.previous_yaw=yaw
            self.previous_elapsed=0.
            self.consecutive_failures=0
        else:
            # Failed iterates are diagnostic only. Keep the last VALID seed;
            # next solve re-integrates shifted controls from the new state.
            self.consecutive_failures+=1
            result['failure_snapshot']=json_safe(dict(
                **result['solve_input'],
                candidate_states=states,candidate_controls=controls,
                iterations=stats.get('iterations',{}),
                constraint_values=constraints[0] if constraints is not None else None,
                constraint_lower=constraints[1] if constraints is not None else None,
                constraint_upper=constraints[2] if constraints is not None else None,
                constraint_blocks=self.constraint_blocks,
                envelope_traces=envelope_traces))
        diagnostics.update(warm_start_source='last_success' if use_cache else 'feedforward',
                           warm_start_age_s=cache_age,warm_start_shift_steps=shift,
                           warm_start_cache_expired=cache_age is not None and not use_cache,
                           warm_start_retained=self.previous is not None,
                           consecutive_failures=self.consecutive_failures,
                           preparation_time_s=prepared-started,optimizer_time_s=optimized-prepared,
                           diagnostics_time_s=time.perf_counter()-optimized)
        result['diagnostics']=json_safe(diagnostics)
        result['solve_time_s']=time.perf_counter()-started
        return result
