"""Frozen dimensionless MPCC cost in NumPy and CasADi."""

from __future__ import annotations

import casadi as ca
import numpy as np

from .contouring_lag import numpy_errors, symbolic_errors


RATIOS={"lag":800/30,"speed":4/30,"progress":40/30,"steer":20/30,"accel":10/30,"steer_rate":900/30,"vtheta_rate":0.}


def numpy_stage(state,control,previous,reference):
    ec,el,ref=numpy_errors(state,reference);delta_ff=np.clip(np.arctan(.324*ref["curvature"]),-.314159,.314159)
    values=np.asarray([ec/.05,el/.20,(state[3]-ref["v"])/.60,(control[2]-ref["v"])/.60,(control[1]-delta_ff)/.314159,control[0]/2.,(control[1]-previous[1])/.314159,(control[2]-previous[2])/.40])
    weights=np.asarray([1,RATIOS["lag"],RATIOS["speed"],RATIOS["progress"],RATIOS["steer"],RATIOS["accel"],RATIOS["steer_rate"],RATIOS["vtheta_rate"]])
    return float(np.dot(weights,values*values)),values


def symbolic_stage(state,control,previous,reference):
    ec,el,ref=symbolic_errors(state,reference);delta_ff=ca.fmin(.314159,ca.fmax(-.314159,ca.atan(.324*ref["curvature"])))
    values=ca.vertcat(ec/.05,el/.20,(state[3]-ref["v"])/.60,(control[2]-ref["v"])/.60,(control[1]-delta_ff)/.314159,control[0]/2.,(control[1]-previous[1])/.314159,(control[2]-previous[2])/.40)
    weights=ca.DM([1,RATIOS["lag"],RATIOS["speed"],RATIOS["progress"],RATIOS["steer"],RATIOS["accel"],RATIOS["steer_rate"],RATIOS["vtheta_rate"]])
    return ca.dot(weights,values*values),values


def numpy_terminal(state,reference):
    ec,el,ref=numpy_errors(state,reference);values=np.asarray([ec/.05,el/.20,(state[3]-ref["v"])/.60]);weights=np.asarray([1,RATIOS["lag"],RATIOS["speed"]])
    return float(np.dot(weights,values*values)),values


def symbolic_terminal(state,reference):
    ec,el,ref=symbolic_errors(state,reference);values=ca.vertcat(ec/.05,el/.20,(state[3]-ref["v"])/.60);weights=ca.DM([1,RATIOS["lag"],RATIOS["speed"]])
    return ca.dot(weights,values*values),values


def trajectory_numpy(states,controls,current_steering,current_vtheta,reference):
    total=0.;terms=[];previous=np.asarray([0.,current_steering,current_vtheta])
    for state,control in zip(states[:-1],controls):
        value,row=numpy_stage(state,control,previous,reference);total+=value;terms.append(row);previous=control
    terminal,trow=numpy_terminal(states[-1],reference);total+=terminal
    return float(total),np.asarray(terms),trow


def canary(reference) -> dict:
    probes=[];states=[]
    base=reference.numpy(.37*reference.length);t=np.asarray([np.cos(base["yaw"]),np.sin(base["yaw"])]);n=np.asarray([-t[1],t[0]]);p=np.asarray([base["x"],base["y"]])
    states.append(("exact",np.r_[p,base["yaw"],base["v"],.37*reference.length],[0,np.arctan(.324*base["curvature"]),base["v"]]))
    states.append(("left",np.r_[p+.05*n,base["yaw"],base["v"],.37*reference.length],[0,0,base["v"]]))
    states.append(("right",np.r_[p-.05*n,base["yaw"],base["v"],.37*reference.length],[0,0,base["v"]]))
    states.append(("lag_forward",np.r_[p+.10*t,base["yaw"],base["v"],.37*reference.length],[0,0,base["v"]]))
    states.append(("speed",np.r_[p,base["yaw"],base["v"]-.6,.37*reference.length],[0,0,base["v"]]))
    states.append(("high_steer_rate",np.r_[p,base["yaw"],base["v"],.37*reference.length],[0,.314159,base["v"]]))
    x=ca.MX.sym("d7_cost_x",5);u=ca.MX.sym("d7_cost_u",3);prev=ca.MX.sym("d7_cost_prev",3);expr,components=symbolic_stage(x,u,prev,reference);fun=ca.Function("d7_cost_canary",[x,u,prev],[expr,components])
    errors=[]
    for name,state,control in states:
        previous=np.asarray([0.,-.314159,base["v"]]);expected,components_np=numpy_stage(state,control,previous,reference);actual=fun(state,control,previous);error=abs(float(actual[0])-expected);errors.append(error)
        probes.append({"name":name,"numpy_cost":expected,"casadi_cost":float(actual[0]),"cost_error":error,"components":components_np.tolist(),"finite":bool(np.isfinite(expected)),"nonnegative":expected>=0})
    contour_sign=probes[1]["components"][0]>0 and probes[2]["components"][0]<0
    lag_sign=probes[3]["components"][1]>0
    return {"probes":probes,"max_cost_error":max(errors),"reference_contouring_lag_near_zero":max(abs(probes[0]["components"][0]),abs(probes[0]["components"][1]))<=1e-10,"contouring_sign_correct":contour_sign,"lag_sign_correct":lag_sign,"all_finite_nonnegative":all(x["finite"] and x["nonnegative"] for x in probes),"passed":bool(max(errors)<=1e-9 and contour_sign and lag_sign and all(x["finite"] and x["nonnegative"] for x in probes))}
