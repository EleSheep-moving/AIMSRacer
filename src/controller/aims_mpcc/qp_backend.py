"""Experimental fixed sparsity lateral/speed QP, solved by OSQP's native C core.

Lateral prediction freezes speed and curvature at each seed node. The returned
candidate is independently rolled through the full nonlinear six state model.
The common validator remains responsible for accepting any candidate.
"""
import time
import numpy as np
import scipy.sparse as sparse
import casadi as ca
from .backend_models import NumericalBackend
from .envelope import evaluate_strict_envelope,jerk_limits
from .path import ReferencePath
from .vendor.normalized_cost import RATIOS


# Match the former scalar facet arithmetic exactly; angles never depend on
# vehicle configuration or the current seed.
ELLIPSE_FACETS=tuple((np.cos(angle),np.sin(angle))
                    for angle in np.arange(16)*2*np.pi/16)
_REFERENCE_AT=ReferencePath.at


class QPSolver(NumericalBackend):
    def __init__(self,path,config,horizon=10,dt=.1):
        super().__init__(path,config,horizon,dt)
        import osqp
        self.dimension=4*(self.n+1)+2*self.n
        self._transition=self._linear_transition()
        # Build once with an explicit pattern. Zeros are retained in both CSC
        # matrices, so speed/curvature changes never change OSQP's sparsity.
        initial=np.zeros(6);initial[0:2]=path.points[0]
        initial[2]=path.at(0.)['yaw']; initial[3]=config.cruise_speed
        seed=self.seed(initial,np.zeros(3),np.full(self.n+1,config.cruise_speed))
        P,q,A,l,u=self._assemble(initial,np.zeros(3),np.full(self.n+1,config.cruise_speed),np.zeros(3),seed)
        self._p_pattern=sparse.csc_matrix(np.triu(self._p_structure.astype(float)))
        self._a_pattern=sparse.csc_matrix(self._a_structure.astype(float))
        self._p_columns=np.repeat(np.arange(self.dimension),np.diff(self._p_pattern.indptr))
        self._a_columns=np.repeat(np.arange(self.dimension),np.diff(self._a_pattern.indptr))
        self._native=osqp.OSQP()
        self._native.setup(P=self._p_values(P),q=q,A=self._a_values(A),l=l,u=u,
            verbose=False,warm_starting=True,polishing=True,eps_abs=1e-7,eps_rel=1e-7,max_iter=4000)
        self.last_native=None

    def reset(self):
        super().reset()
        self._longitudinal_refresh=False
        self._longitudinal_refresh_reason='no_cache'
        self._longitudinal_seed_source='feedforward'
        if hasattr(self,'_native'):
            self._native.warm_start(x=np.zeros(self.dimension),y=np.zeros(self._a_pattern.shape[0]))
        self.last_native=None

    def seed(self,initial,applied,refs):
        """Retain shifted steering, refresh stopped-reference longitudinal goals."""
        use_cache=self.previous is not None and self.previous_elapsed<self.n*self.dt
        if self.previous is None:reason='no_cache'
        elif not use_cache:reason='cache_expired'
        elif np.shape(self.previous.get('speed_refs'))!=(self.n+1,):
            reason='previous_reference_unavailable'
        elif not np.all(self.previous['speed_refs']==0.):reason='previous_reference_not_stopped'
        elif self.config.cruise_speed<=0.:reason='configured_cruise_not_positive'
        elif refs[0]!=self.config.cruise_speed:reason='current_reference_not_cruise'
        else:reason='stopped_reference_to_cruise'
        refresh=bool(reason=='stopped_reference_to_cruise')
        self._longitudinal_refresh=refresh
        self._longitudinal_refresh_reason=reason
        self._longitudinal_seed_source=('current_reference' if refresh else
                                        'last_success' if use_cache else 'feedforward')
        return super().seed(initial,applied,refs,refresh_longitudinal=refresh)

    def finish(self,result,initial,applied,refs,alignment,started,prepared,optimized):
        result['diagnostics'].update(
            warm_start_longitudinal_refresh=self._longitudinal_refresh,
            warm_start_longitudinal_refresh_reason=self._longitudinal_refresh_reason,
            warm_start_longitudinal_source=self._longitudinal_seed_source)
        result=super().finish(result,initial,applied,refs,alignment,started,prepared,optimized)
        if result['success']:
            # Provenance belongs to the same successful trajectory cache. A
            # failed request retains both its controls and its copied goals.
            self.previous['speed_refs']=np.asarray(refs).copy()
        done=time.perf_counter()
        result['solve_time_s']=done-started
        result['diagnostics']['diagnostics_time_s']=done-optimized
        return result

    def _p_values(self,P):
        result=self._p_pattern.copy()
        result.data[:]=P[result.indices,self._p_columns]
        return result

    def _a_values(self,A):
        result=self._a_pattern.copy()
        result.data[:]=A[result.indices,self._a_columns]
        return result

    def _linear_transition(self):
        z=ca.SX.sym('lateral_state',3);command=ca.SX.sym('command');previous=ca.SX.sym('previous')
        speed=ca.SX.sym('speed');curvature=ca.SX.sym('curvature');count=round(self.dt/.02)
        current=z
        def rhs(value,target):
            return ca.vertcat(speed*value[1],
                speed*value[2]/(self.config.wheelbase*(1+self.config.understeer_coefficient*speed**2))-speed*curvature,
                (target-value[2])/self.config.steering_tau)
        for j in range(count):
            target=previous+(command-previous)*(j+1)/count
            a=rhs(current,target);b=rhs(current+.01*a,target);c=rhs(current+.01*b,target);d=rhs(current+.02*c,target)
            current=current+.02*(a+2*b+2*c+d)/6
        coeff=ca.jacobian(current,ca.vertcat(z,command,previous))
        bias=ca.substitute(current,ca.vertcat(z,command,previous),ca.DM.zeros(5))
        return ca.Function('qp_lateral_interval',[speed,curvature],[coeff,bias])

    def _assemble(self,initial,applied,refs,alignment,seed):
        cfg=self.config;D=self.dimension;P=np.eye(D)*1e-10;q=np.zeros(D)
        rows=[];structures=[];low=[];high=[];P_structure=np.eye(D,dtype=bool)
        def constraint(values,lo,hi):
            row=np.zeros(D)
            for i,v in values.items():row[i]+=v
            structure=np.zeros(D,dtype=bool);structure[list(values)]=True
            rows.append(row);structures.append(structure);low.append(lo);high.append(hi)
        def cost(values,target,weight):
            # Each objective term involves at most three variables. Preserve
            # its accumulation order while updating only those active entries.
            for i,vi in values.items():
                q[i]-=2*weight*target*vi
                for j,vj in values.items():
                    P[i,j]+=2*weight*(vi*vj)
                    P_structure[i,j]=True
        state=lambda k,j:4*k+j
        control=lambda k,j:4*(self.n+1)+2*k+j
        geometries=self.geometries(seed[0][:,4],alignment)
        g=geometries[0];normal=np.array([-np.sin(g[2]),np.cos(g[2])])
        lateral=float(np.dot(initial[:2]-g[:2],normal));heading=(initial[2]-g[2]+np.pi)%(2*np.pi)-np.pi
        for j,value in enumerate((lateral,heading,initial[5],initial[3])):
            constraint({state(0,j):1.},value,value)
        for k in range(self.n):
            coeff,bias=self._transition(float(seed[0][k,3]),geometries[k][3]);coeff=np.asarray(coeff);bias=np.asarray(bias).ravel()
            for j in range(3):
                equation={state(k+1,j):1.,control(k,1):-coeff[j,3]}
                for m in range(3):equation[state(k,m)]=-coeff[j,m]
                if k:equation[control(k-1,1)]=-coeff[j,4];offset=bias[j]
                else:offset=bias[j]+coeff[j,4]*applied[1]
                constraint(equation,offset,offset)
            constraint({state(k+1,3):1.,state(k,3):-1.,control(k,0):-self.dt},0.,0.)
        offsets=cfg.longitudinal_offsets() if cfg.enforce_corridor else ()
        for k in range(self.n+1):
            terminal=cfg.terminal_weight if k==self.n else 1.
            cost({state(k,0):1.},0.,terminal*cfg.contour_weight/cfg.contour_scale**2)
            cost({state(k,1):1.},0.,terminal*cfg.heading_weight/cfg.heading_scale**2)
            cost({state(k,3):1.},refs[k],terminal*cfg.speed_weight/cfg.speed_scale**2)
            constraint({state(k,3):1.},0.,cfg.max_speed)
            constraint({state(k,2):1.},-cfg.steer_limit,cfg.steer_limit)
            if cfg.enforce_corridor:
                for along in offsets:
                    constraint({state(k,0):1.,state(k,1):along},-self.path.right_width+cfg.half_width,self.path.left_width-cfg.half_width)
        jerk=jerk_limits(initial,applied,cfg,self.dt,self.n)
        accel_axis=min(cfg.envelope_halfaxes()[:2]);lateral_axis=cfg.envelope_halfaxes()[2]
        envelope_radius=np.cos(np.pi/16)*np.sqrt(1.-cfg.optimization_envelope_margin)
        for k in range(self.n):
            accel,steer=control(k,0),control(k,1)
            constraint({accel:1.},-cfg.brake_limit,cfg.accel_limit)
            constraint({steer:1.},-cfg.steer_limit,cfg.steer_limit)
            delta={steer:1.}
            if k:delta[control(k-1,1)]=-1.;previous=0.
            else:previous=applied[1]
            constraint(delta,previous-cfg.steer_rate*self.dt,previous+cfg.steer_rate*self.dt)
            acceleration={accel:1.}
            if k:acceleration[control(k-1,0)]=-1.;pa=0.
            else:pa=applied[0]
            constraint(acceleration,pa-jerk[k]*self.dt,pa+jerk[k]*self.dt)
            second=dict(delta);bound_offset=0.
            if k==0:bound_offset=applied[1]+applied[2]*self.dt
            elif k==1:
                second[control(k-1,1)]=-2.;bound_offset=-applied[1]
            else:
                second[control(k-1,1)]=-2.;second[control(k-2,1)]=1.
            constraint(second,bound_offset-cfg.steer_acceleration*self.dt**2,bound_offset+cfg.steer_acceleration*self.dt**2)
            ff=np.arctan(cfg.wheelbase*(1+cfg.understeer_coefficient*seed[0][k,3]**2)*geometries[k][3])
            cost({steer:1.},ff,cfg.steering_weight/cfg.steering_scale**2)
            cost({accel:1.},0.,RATIOS['accel']/cfg.acceleration_scale**2)
            cost(delta,previous,cfg.steering_rate_weight/(self.dt*cfg.steer_rate)**2)
            cost(second,bound_offset,cfg.steering_acceleration_weight/(cfg.steer_acceleration*self.dt**2)**2)
            # Inscribed 16-sided ellipse approximation; exact nonlinear
            # utilization is checked again on the independent output rollout.
            for node in (k,k+1):
                v=float(seed[0][node,3]);d=float(seed[0][node,5])
                factor=v*v/(cfg.wheelbase*(1+cfg.understeer_coefficient*v*v)*lateral_axis)
                slope=factor/(np.cos(d)**2);offset=factor*np.tan(d)-slope*d
                for a,b in ELLIPSE_FACETS:
                    constraint({accel:a/accel_axis,state(node,2):b*slope},-np.inf,envelope_radius-b*offset)
        self._p_structure=P_structure;self._a_structure=np.asarray(structures)
        return P,q,np.asarray(rows),np.asarray(low),np.asarray(high)

    def _warm_start_vector(self,seed,alignment):
        # Native QP coordinates are path-relative, while retained trajectories
        # stay in odom. Re-project every node through this request's alignment.
        states=[]
        geometries=self.geometries(seed[0][:,4],alignment)
        for x,geometry in zip(seed[0],geometries):
            normal=np.array([-np.sin(geometry[2]),np.cos(geometry[2])])
            lateral=float(np.dot(x[:2]-geometry[:2],normal))
            heading=(x[2]-geometry[2]+np.pi)%(2*np.pi)-np.pi
            states.append([lateral,heading,x[5],x[3]])
        return np.r_[np.asarray(states).ravel(),seed[1][:,:2].ravel()]

    def solve(self,state,previous,speed_refs=None,elapsed=.1,map_alignment=None):
        started=time.perf_counter();initial,applied,refs,alignment,seed=self.inputs(state,previous,speed_refs,elapsed,map_alignment)
        if self.config.envelope_soft_enabled:
            now=time.perf_counter()
            return self.finish(dict(success=False,status='unsupported',iterations=0,constraint_violation=None,
                error='QP backend does not support nonlinear soft-envelope recovery; use acados or ipopt',
                diagnostics=dict(native_core='OSQP C',unsupported_constraints=['soft_operating_envelope'])),
                initial,applied,refs,alignment,started,now,now)
        cache_age=self.previous_elapsed if self.previous is not None else None
        use_cache=self.previous is not None and cache_age<self.n*self.dt
        shift=min(self.n,max(1,round(cache_age/self.dt))) if use_cache else 0
        P,q,A,lower,upper=self._assemble(initial,applied,refs,alignment,seed)
        self._native.update(Px=self._p_values(P).data,Ax=self._a_values(A).data,q=q,l=lower,u=upper)
        primal=self._warm_start_vector(seed,alignment)
        # Bound rows change their numerical geometry and the immutable initial
        # row changes every request. Their old duals have no valid stage mapping.
        self._native.warm_start(x=primal,y=np.zeros(A.shape[0]))
        prepared=time.perf_counter();solution=self._native.solve(raise_error=False);optimized=time.perf_counter()
        self.last_native=solution
        # OSQP status 2 meets its optimality test at 10x solver tolerance.
        # It may supply a candidate under the SAME physical/matrix bounds;
        # status 7 (maximum iterations) remains ineligible. Neither status
        # eligibility nor candidate feasibility establishes output authority.
        status_val=int(solution.info.status_val)
        primal_residual=float(solution.info.prim_res);dual_residual=float(solution.info.dual_res)
        primal_finite=bool(np.isfinite(primal_residual));dual_finite=bool(np.isfinite(dual_residual))
        residuals_finite=primal_finite and dual_finite
        eligible=status_val in (1,2) and residuals_finite
        success=False;controls=None;states=None;violation=None
        diagnostics=dict(native_core='OSQP C',osqp_status=solution.info.status,
            osqp_status_val=status_val,osqp_primal_residual=primal_residual if primal_finite else None,
            osqp_dual_residual=dual_residual if dual_finite else None,
            native_residuals_finite=residuals_finite,
            native_error_kind=None if residuals_finite else 'nonfinite_residual_metadata',
            native_converged=status_val==1 and residuals_finite,
            native_candidate_status_eligible=eligible,candidate_feasible=False,
            success_scope='candidate_feasibility',
            optimization_envelope_margin=self.config.optimization_envelope_margin,
            warm_start_source='last_success' if use_cache else 'feedforward',
            warm_start_age_s=cache_age,warm_start_shift_steps=shift,
            warm_start_cache_expired=cache_age is not None and not use_cache,
            native_primal_reprojected=True,native_dual_start='zero',
            osqp_native_run_time_s=float(solution.info.run_time),model='frozen-speed lateral QP with independent nonlinear output rollout')
        if solution.x is not None and np.isfinite(solution.x).all():
            z=np.asarray(solution.x); native_u=z[4*(self.n+1):].reshape(self.n,2)
            controls=[];current=initial.copy()
            for k,(a,d) in enumerate(native_u):
                progress=np.clip((current[3]+.5*a*self.dt)/np.linalg.norm(self.path.curve.numpy(current[4],1)),0.,self.config.max_speed)
                u=np.array([a,d,progress]);controls.append(u)
                current[3]+=a*self.dt;current[4]+=progress*self.dt
            controls=np.asarray(controls)
            envelope,microstates=evaluate_strict_envelope(initial,applied,controls,self.config,self.dt)
            states=microstates[::round(self.dt/.02)]
            value=A@z;qp_violation=max(float(np.max(lower-value)),float(np.max(value-upper)),0.)
            violation=max(qp_violation,envelope['hard_control_violation'],envelope['speed_bound_violation'],
                          envelope['actual_steering_bound_violation'],
                          envelope['future_slack_max'],max(0.,envelope['initial_candidate_utilization']-1.))
            margin=np.inf
            if self.config.enforce_corridor:
                offsets=self.config.longitudinal_offsets()
                c,s=np.cos(alignment[2]),np.sin(alignment[2]);rotation=np.array([[c,-s],[s,c]])
                reference=tangent=None
                if (type(self.path) is ReferencePath and
                        getattr(self.path.at,'__func__',None) is _REFERENCE_AT):
                    # Keep SciPy's per-point polynomial arithmetic, omitting
                    # unused curvature. Custom paths retain their scalar at().
                    reference=self.path.curve.numpy(microstates[:,4])
                    tangent=self.path.curve.numpy(microstates[:,4],1)
                for i,x in enumerate(microstates):
                    xy=rotation@x[:2]+alignment[:2]
                    ref=(self.path.at(x[4]) if reference is None else
                         dict(x=float(reference[i,0]),y=float(reference[i,1]),
                              yaw=float(np.arctan2(tangent[i,1],tangent[i,0]))))
                    normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
                    yaw=x[2]+alignment[2];forward=np.array([np.cos(yaw),np.sin(yaw)]);left=np.array([-np.sin(yaw),np.cos(yaw)])
                    for along in offsets:
                        for sign in (-1,1):
                            lateral=np.dot(xy+along*forward+sign*self.config.half_width*left-np.array([ref['x'],ref['y']]),normal)
                            margin=min(margin,self.path.left_width-lateral,self.path.right_width+lateral)
                violation=max(violation,-margin)
            diagnostics.update(envelope=envelope,max_constraint_violation=violation,qp_constraint_violation=qp_violation,
                               minimum_predicted_margin_m=float(margin) if np.isfinite(margin) else None)
            diagnostics['candidate_feasible']=bool(violation<1e-4)
            success=bool(eligible and diagnostics['candidate_feasible'])
        result=dict(success=success,status=solution.info.status if success else 'qp_candidate_rejected:'+solution.info.status,
                    iterations=int(solution.info.iter),constraint_violation=violation,diagnostics=diagnostics,
                    solve_input=dict(warm_states=seed[0].tolist(),warm_controls=seed[1].tolist()))
        if states is not None:result.update(states=states.tolist(),controls=controls.tolist())
        return self.finish(result,initial,applied,refs,alignment,started,prepared,optimized)
