"""Global kinematic bicycle with virtual progress and classical RK4."""

from __future__ import annotations

import casadi as ca
import numpy as np

WHEELBASE_M = .324
DT_S = .05


def rhs_numpy(state, control, wheelbase: float = WHEELBASE_M):
    x,y,yaw,speed,theta=np.asarray(state,float);acceleration,steering,vtheta=np.asarray(control,float)
    return np.asarray([speed*np.cos(yaw),speed*np.sin(yaw),speed*np.tan(steering)/wheelbase,acceleration,vtheta])


def rhs_symbolic(state, control, wheelbase: float = WHEELBASE_M):
    return ca.vertcat(state[3]*ca.cos(state[2]),state[3]*ca.sin(state[2]),state[3]*ca.tan(control[1])/wheelbase,control[0],control[2])


def rk4_numpy(state, control, dt: float = DT_S, wheelbase: float = WHEELBASE_M):
    x1=np.asarray(state,float);k1=rhs_numpy(x1,control,wheelbase)
    x2=x1+.5*dt*k1;k2=rhs_numpy(x2,control,wheelbase)
    x3=x1+.5*dt*k2;k3=rhs_numpy(x3,control,wheelbase)
    x4=x1+dt*k3;k4=rhs_numpy(x4,control,wheelbase)
    end=x1+dt*(k1+2*k2+2*k3+k4)/6
    return end,np.stack((x1,x2,x3,x4))


def rk4_symbolic(state, control, dt: float = DT_S, wheelbase: float = WHEELBASE_M):
    k1=rhs_symbolic(state,control,wheelbase);x2=state+.5*dt*k1
    k2=rhs_symbolic(x2,control,wheelbase);x3=state+.5*dt*k2
    k3=rhs_symbolic(x3,control,wheelbase);x4=state+dt*k3
    k4=rhs_symbolic(x4,control,wheelbase)
    return state+dt*(k1+2*k2+2*k3+k4)/6,(state,x2,x3,x4)


def rollout(initial, controls):
    controls=np.asarray(controls,float);states=np.empty((len(controls)+1,5));stages=np.empty((len(controls),4,5));states[0]=initial
    for index,control in enumerate(controls):states[index+1],stages[index]=rk4_numpy(states[index],control)
    return states,stages


def identity_audit() -> dict:
    state=np.asarray([.2,-.3,.17,2.4,16.95]);control=np.asarray([-.3,.12,2.3])
    end,stages=rk4_numpy(state,control);x=ca.MX.sym("d7_rk4_x",5);u=ca.MX.sym("d7_rk4_u",3);sym,sym_stages=rk4_symbolic(x,u)
    fun=ca.Function("d7_rk4_identity",[x,u],[sym,ca.horzcat(*sym_stages)])
    actual=fun(state,control);errors=np.r_[np.asarray(actual[0]).reshape(-1)-end,np.asarray(actual[1]).T.reshape(-1)-stages.reshape(-1)]
    return {"max_abs_error":float(np.max(np.abs(errors))),"state_dimension":5,"control_dimension":3,"rk4_stages":4,"passed":bool(np.max(np.abs(errors))<=1e-12)}

