"""CasADi/IPOPT port of f1tenth_mpcc.solver (see THIRD_PARTY_NOTICES.md).

Rear axle state order: x,y,yaw,speed,progress,actual steering.
Controls: longitudinal acceleration, steering command, virtual progress speed.
"""
import time
import casadi as ca
import numpy as np
from .config import VehicleConfig, REVERSE_SPEED_TOLERANCE
from .path import ReferencePath
from .vendor import global_kinematic_model, contouring_lag, normalized_cost

class MPCCSolver:
    def __init__(self, path: ReferencePath, config: VehicleConfig, horizon=15, dt=.1, jit_enabled=False):
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
        self.weights.update(speed=self.weights['speed']*1000,heading=4.)
        self.corner_offsets=[(config.rear_offset+sx*config.half_length,sy*config.half_width)
                             for sx,sy in ((1,1),(1,-1),(-1,1),(-1,-1))]
        self.jit_enabled=bool(jit_enabled)
        self.reset()
        self._build()

    def reset(self):
        self.previous=None; self.previous_theta=None;self.previous_yaw=None

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
        op.subject_to(x[:, 0] == self.initial)
        op.subject_to(op.bounded(0., x[3, :], self.config.max_speed))
        op.subject_to(op.bounded(-self.config.brake_limit, u[0, :], self.config.accel_limit))
        op.subject_to(op.bounded(-self.steer_limit, u[1, :], self.steer_limit))
        op.subject_to(op.bounded(0., u[2, :], self.config.max_speed))
        weights = self.weights
        objective = 0
        self.margins = []

        def geometry(state):
            ec, el, ref = self.modules["errors"].symbolic_errors(state, self.reference)
            tangent = ref["dxy"] / ca.sqrt(ca.dot(ref["dxy"], ref["dxy"]))
            normal = ca.vertcat(-tangent[1], tangent[0])
            heading = ca.vertcat(ca.cos(state[2]), ca.sin(state[2]))
            left = ca.vertcat(-ca.sin(state[2]), ca.cos(state[2]))
            for along, across in self.corner_offsets:
                corner = state[:2] + along * heading + across * left
                lateral = ca.dot(corner - ref["xy"], normal)
                op.subject_to(op.bounded(-self.path.right_width, lateral, self.path.left_width))
                self.margins.extend((self.path.left_width - lateral, self.path.right_width + lateral))
            # Periodic, wrap-safe heading error: 2*(1-cos(error)) ~ error^2.
            ref["heading_error_squared"] = 2 * (1 - ca.dot(heading, tangent))
            return ec, el, ref

        # Share the microstep integration graph between prediction intervals.
        sx, su = ca.MX.sym("x", 6), ca.MX.sym("u", 3)
        sp, sb = ca.MX.sym("previous_steering"), ca.MX.sym("steering_bias")
        end, stages = self._dynamics(sx, su, steering_bias=sb, previous_steering=sp)
        transition = ca.Function("f1tenth_interval", [sx, su, sp, sb], [end, ca.horzcat(*stages)]).expand()

        for k in range(self.n):
            previous = self.applied if k == 0 else u[:2, k - 1]
            end, sample_matrix = transition(x[:, k], u[:, k], previous[1], self.steering_bias)
            stages = [sample_matrix[:, j] for j in range(self.substeps + 1)]
            op.subject_to(x[:, k + 1] == end)
            op.subject_to(op.bounded(-self.jerk_limit * self.dt, u[0, k] - previous[0], self.jerk_limit * self.dt))
            rate = (u[1, k] - previous[1]) / self.dt
            previous_rate = self.applied[2] if k == 0 else (u[1, k - 1] - (self.applied[1] if k == 1 else u[1, k - 2])) / self.dt
            op.subject_to(op.bounded(-self.steer_rate, rate, self.steer_rate))
            op.subject_to(op.bounded(-self.steer_acceleration * self.dt, rate - previous_rate, self.steer_acceleration * self.dt))
            # Check acceleration utilization at each 20 ms integration node.
            for stage in stages:
                lateral_accel = stage[3] ** 2 * ca.tan(stage[5]) / (self.wheelbase * (1 + self.understeer_coefficient * stage[3] ** 2))
                scale = ca.if_else(u[0, k] >= 0, self.config.accel_limit, self.config.brake_limit)
                op.subject_to((u[0, k] / scale) ** 2 + (lateral_accel / self.config.lateral_accel_limit) ** 2 <= 1.)
            ec, el, ref = geometry(x[:, k])
            geometry(stages[(self.substeps + 1) // 2])
            delta_ff = ca.atan(self.wheelbase * (1 + self.understeer_coefficient * x[3, k] ** 2) * ref["curvature"]) - self.steering_bias
            objective += (ec / .05) ** 2 + weights["lag"] * (el / .20) ** 2
            objective += weights["heading"] * ref["heading_error_squared"] / .05 ** 2
            objective += weights["speed"] * ((x[3, k] - self.speed_refs[k]) / .60) ** 2
            objective += weights["progress"] * ((u[2, k] - self.speed_refs[k]) / .60) ** 2
            objective += weights["steer"] * ((u[1, k] - delta_ff) / .314159) ** 2
            objective += weights["accel"] * (u[0, k] / 2.) ** 2
            objective += .3 * (rate / self.steer_rate) ** 2 + .3 * ((rate - previous_rate) / (self.steer_acceleration * self.dt)) ** 2
        ec, el, ref = geometry(x[:, -1])
        objective += 3 * ((ec / .05) ** 2 + weights["lag"] * (el / .20) ** 2
                          + weights["heading"] * ref["heading_error_squared"] / .05 ** 2
                          + weights["speed"] * ((x[3, -1] - self.speed_refs[-1]) / .60) ** 2)
        op.minimize(objective)
        # Native callbacks preserve the OCP. -O0 compiles quickly and provides
        # adequate measured deadline margin. Production worker owns a temporary cwd.
        op.solver("ipopt", {"print_time": False, "jit": self.jit_enabled, "compiler": "shell",
                   "jit_options": {"flags": ["-O0"]}}, {"print_level": 0, "sb": "yes", "linear_solver": "mumps",
                   "max_iter": 100, "tol": 1e-5, "acceptable_tol": 1e-4, "acceptable_iter": 3,
                   "warm_start_init_point": "yes"})
        self.op, self.x, self.u = op, x, u

    def _warm_start(self, initial, applied, refs, elapsed):
        states=[initial]
        shifted=None
        if self.previous is not None:
            shift=min(self.n,max(1,round(elapsed/self.dt)))
            shifted=np.vstack((self.previous['controls'][shift:],
                               np.repeat(self.previous['controls'][-1:],shift,axis=0)))
        acceleration,steering,rate=applied
        controls=[]
        for k in range(self.n):
            state=states[-1]
            ref=self.path.at(state[4])
            if shifted is None:
                desired_acceleration=(refs[k+1]-state[3])/self.dt
                desired_steering=np.arctan(self.wheelbase*(1+self.understeer_coefficient*state[3]**2)*ref['curvature'])
            else:
                desired_acceleration,desired_steering=shifted[k,:2]
            acceleration=np.clip(desired_acceleration,
                max(-self.config.brake_limit,acceleration-self.jerk_limit*self.dt),
                min(self.config.accel_limit,acceleration+self.jerk_limit*self.dt))
            desired_rate=(np.clip(desired_steering,-self.steer_limit,self.steer_limit)-steering)/self.dt
            rate=np.clip(desired_rate,max(-self.steer_rate,rate-self.steer_acceleration*self.dt),
                         min(self.steer_rate,rate+self.steer_acceleration*self.dt))
            next_steering=np.clip(steering+rate*self.dt,-self.steer_limit,self.steer_limit)
            rate=(next_steering-steering)/self.dt
            tangent_norm=np.linalg.norm(self.path.curve.numpy(state[4],1))
            progress_speed=max(0.,state[3]+.5*acceleration*self.dt)/tangent_norm
            control=np.array([acceleration,next_steering,min(self.config.max_speed,progress_speed)])
            end,_=self._dynamics(state,control,symbolic=False,previous_steering=steering)
            states.append(end);controls.append(control);steering=next_steering
        return np.asarray(states),np.asarray(controls)

    def solve(self, state, previous, speed_refs=None, elapsed=.1):
        started=time.perf_counter()
        measured=np.array([state[k] for k in ['x','y','yaw','speed','steering']],float)
        applied=np.array([previous[k] for k in ['acceleration','steering','steering_rate']],float)
        if not np.isfinite(measured).all() or not np.isfinite(applied).all() or not np.isfinite(elapsed) or elapsed<=0:
            raise ValueError('finite state, previous command and positive elapsed required')
        if measured[3] < -REVERSE_SPEED_TOLERANCE: raise ValueError('reverse motion unsupported')
        # Match the supervisor deadband; the forward-only OCP starts at zero.
        measured[3] = max(0., measured[3])
        refs=np.full(self.n+1,self.config.cruise_speed) if speed_refs is None else np.asarray(speed_refs,float)
        if refs.shape!=(self.n+1,) or not np.isfinite(refs).all() or np.any(refs<0) or np.any(refs>self.config.max_speed):
            raise ValueError('speed_refs must be n+1 finite values within speed limits')
        theta,_=self.path.project(measured[:2]);yaw=measured[2]
        if self.previous_theta is not None:
            theta=self.previous_theta+(theta-self.previous_theta+self.path.length/2)%self.path.length-self.path.length/2
            yaw=self.previous_yaw+(yaw-self.previous_yaw+np.pi)%(2*np.pi)-np.pi
        initial=np.r_[measured[:2],yaw,measured[3],theta,measured[4]]
        self.op.set_value(self.initial,initial);self.op.set_value(self.applied,applied)
        self.op.set_value(self.steering_bias,0.);self.op.set_value(self.speed_refs,refs)
        warm_states,warm_u=self._warm_start(initial,applied,refs,elapsed)
        self.op.set_initial(self.x,np.asarray(warm_states).T);self.op.set_initial(self.u,warm_u.T)
        try:
            solution=self.op.solve()
        except RuntimeError as exc:
            self.previous=None
            return dict(success=False,status=self.op.stats().get('return_status','exception'),
                        solve_time_s=time.perf_counter()-started,error=str(exc))
        states=np.asarray(solution.value(self.x)).reshape(6,self.n+1).T
        controls=np.asarray(solution.value(self.u)).reshape(3,self.n).T
        stats=self.op.stats()
        g,lower,upper=(np.asarray(solution.value(item)).reshape(-1) for item in (self.op.g,self.op.lbg,self.op.ubg))
        violation=float(np.maximum(np.maximum(lower-g,g-upper),0).max())
        success=bool(stats['success'] and violation<1e-4 and np.isfinite(states).all() and np.isfinite(controls).all())
        result=dict(success=success,status=stats['return_status'],states=states.tolist(),controls=controls.tolist(),
                    solve_time_s=time.perf_counter()-started,iterations=int(stats['iter_count']),constraint_violation=violation,
                    minimum_predicted_margin_m=float(np.min(solution.value(ca.vertcat(*self.margins)))))
        if success:
            self.previous=dict(states=states,controls=controls);self.previous_theta=theta;self.previous_yaw=yaw
        else: self.previous=None
        return result
