"""Numerical input and trajectory utilities for experimental native backends."""
import math
import time
import numpy as np
from .envelope import independent_rollout, jerk_limits, evaluate_envelope
from .solver_diagnostics import json_safe


class NumericalBackend:
    def __init__(self,path,config,horizon=10,dt=.1):
        config.validate(require_verified=True);path.validate_config(config)
        if isinstance(horizon,bool) or int(horizon)!=horizon or horizon<1:
            raise ValueError('positive integer horizon required')
        if not np.isfinite(dt) or dt<=0 or not np.isclose(round(dt/.02)*.02,dt):
            raise ValueError('dt must be a positive multiple of 20 ms')
        self.path,self.config,self.n,self.dt=path,config,int(horizon),float(dt)
        self.reset()

    def reset(self):
        self.previous=None;self.previous_theta=None;self.previous_yaw=None
        self.previous_elapsed=0.;self.consecutive_failures=0

    def inputs(self,state,previous,speed_refs,elapsed,map_alignment):
        measured=np.array([state[k] for k in ('x','y','yaw','speed','steering')],float)
        applied=np.array([previous[k] for k in ('acceleration','steering','steering_rate')],float)
        if not np.isfinite(measured).all() or not np.isfinite(applied).all() or not np.isfinite(elapsed) or elapsed<=0:
            raise ValueError('finite state, previous command and positive elapsed required')
        measured[3]=max(0.,measured[3])
        refs=np.full(self.n+1,self.config.cruise_speed) if speed_refs is None else np.asarray(speed_refs,float)
        if refs.shape!=(self.n+1,) or not np.isfinite(refs).all() or np.any(refs<0) or np.any(refs>self.config.max_speed):
            raise ValueError('speed_refs must be n+1 finite values within speed limits')
        if self.path.frame_id=='map':
            alignment=np.asarray(map_alignment,float)
            if alignment.shape!=(3,) or not np.isfinite(alignment).all():
                raise ValueError('map reference requires a finite map-to-odom alignment')
        else:
            if map_alignment is not None: raise ValueError('odom reference must not receive a map alignment')
            alignment=np.zeros(3)
        c,s=np.cos(alignment[2]),np.sin(alignment[2]); rotation=np.array([[c,-s],[s,c]])
        theta,_=self.path.project(rotation@measured[:2]+alignment[:2]);yaw=measured[2]
        if self.previous_theta is not None:
            theta=self.previous_theta+(theta-self.previous_theta+self.path.length/2)%self.path.length-self.path.length/2
            yaw=self.previous_yaw+(yaw-self.previous_yaw+np.pi)%(2*np.pi)-np.pi
        initial=np.r_[measured[:2],yaw,measured[3],theta,measured[4]]
        self.previous_elapsed+=elapsed if self.previous is not None else 0.
        seed=self.seed(initial,applied,refs)
        return initial,applied,refs,alignment,seed

    def seed(self,initial,applied,refs):
        shifted=None
        if self.previous is not None and self.previous_elapsed<self.n*self.dt:
            shift=min(self.n,max(1,round(self.previous_elapsed/self.dt)))
            shifted=np.vstack((self.previous['controls'][shift:],np.repeat(self.previous['controls'][-1:],shift,axis=0)))
        states=[initial];controls=[];acceleration,steering,rate=applied
        limits=jerk_limits(initial,applied,self.config,self.dt,self.n)
        for k in range(self.n):
            x=states[-1];ref=self.path.at(x[4])
            desired_accel=(refs[k+1]-x[3])/self.dt
            desired_steer=math.atan(self.config.wheelbase*(1+self.config.understeer_coefficient*x[3]**2)*ref['curvature'])
            if shifted is not None: desired_accel,desired_steer=shifted[k,:2]
            acceleration=float(np.clip(desired_accel,max(-self.config.brake_limit,acceleration-limits[k]*self.dt),
                                        min(self.config.accel_limit,acceleration+limits[k]*self.dt)))
            desired_rate=(np.clip(desired_steer,-self.config.steer_limit,self.config.steer_limit)-steering)/self.dt
            rate=float(np.clip(desired_rate,max(-self.config.steer_rate,rate-self.config.steer_acceleration*self.dt),
                                min(self.config.steer_rate,rate+self.config.steer_acceleration*self.dt)))
            endpoint=float(np.clip(steering+rate*self.dt,-self.config.steer_limit,self.config.steer_limit))
            progress=max(0.,x[3]+.5*acceleration*self.dt)/np.linalg.norm(self.path.curve.numpy(x[4],1))
            control=np.array([acceleration,endpoint,min(self.config.max_speed,progress)])
            states.append(independent_rollout(x,[acceleration,steering,rate],[control],self.config,self.dt)[-1])
            controls.append(control);rate=(endpoint-steering)/self.dt;steering=endpoint
        return np.asarray(states),np.asarray(controls)

    def geometry(self,theta,alignment):
        ref=self.path.at(theta);c,s=np.cos(alignment[2]),np.sin(alignment[2])
        rotation=np.array([[c,-s],[s,c]])
        xy=rotation.T@(np.array([ref['x'],ref['y']])-alignment[:2])
        return np.r_[xy,ref['yaw']-alignment[2],ref['curvature'],theta,
                     np.linalg.norm(self.path.curve.numpy(theta,1))]

    def finish(self,result,initial,applied,refs,alignment,started,prepared,optimized):
        result.update(corridor_enforced=self.config.enforce_corridor,
                      envelope_soft_enabled=self.config.envelope_soft_enabled,execution_authorized=False)
        result.setdefault('solve_input',{}).update(initial_state=initial.tolist(),applied=applied.tolist(),
                                  speed_refs=refs.tolist(),map_alignment=alignment.tolist() if self.path.frame_id=='map' else None)
        if result.get('controls') is not None and 'envelope' not in result['diagnostics']:
            try:
                envelope=evaluate_envelope(initial,applied,result['controls'],self.config,self.dt)
                for k in ('candidate_samples','reference_samples','reference_controls'):envelope.pop(k)
                result['diagnostics']['envelope']=envelope
            except (ValueError,OverflowError) as exc:
                result['diagnostics']['envelope']=dict(diagnostic_error=str(exc),execution_authorized=False)
        if result['success']:
            self.previous=dict(states=np.asarray(result['states']),controls=np.asarray(result['controls']))
            self.previous_theta,self.previous_yaw=initial[4],initial[2]
            self.previous_elapsed=0.;self.consecutive_failures=0
        else:self.consecutive_failures+=1
        result=json_safe(result)
        done=time.perf_counter()
        result['diagnostics'].update(preparation_time_s=prepared-started,optimizer_time_s=optimized-prepared,
            diagnostics_time_s=done-optimized,consecutive_failures=self.consecutive_failures,
            warm_start_retained=self.previous is not None)
        result['solve_time_s']=done-started
        return result


def interval_symbolic(x,u,previous_steering,config,dt):
    """Discrete rear axle dynamics: 20 ms command ramp and RK4 actuator lag."""
    import casadi as ca
    state=x; samples=[state];count=round(dt/.02)
    def rhs(v,command):
        return ca.vertcat(v[3]*ca.cos(v[2]),v[3]*ca.sin(v[2]),
            v[3]*ca.tan(v[5])/(config.wheelbase*(1+config.understeer_coefficient*v[3]**2)),
            u[0],u[2],(command-v[5])/config.steering_tau)
    for j in range(count):
        commanded=previous_steering+(u[1]-previous_steering)*(j+1)/count
        k1=rhs(state,commanded);k2=rhs(state+.01*k1,commanded)
        k3=rhs(state+.01*k2,commanded);k4=rhs(state+.02*k3,commanded)
        state=state+.02*(k1+2*k2+2*k3+k4)/6;samples.append(state)
    return state,samples
