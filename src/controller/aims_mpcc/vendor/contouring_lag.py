"""Independent NumPy and CasADi contouring/lag definitions."""

from __future__ import annotations

import casadi as ca
import numpy as np


def numpy_errors(state, reference):
    ref = reference.numpy(float(state[4]))
    tangent = np.asarray([np.cos(ref["yaw"]), np.sin(ref["yaw"])])
    normal = np.asarray([-tangent[1], tangent[0]])
    displacement = np.asarray(state[:2], float)-np.asarray([ref["x"], ref["y"]])
    return float(normal@displacement), float(tangent@displacement), ref


def symbolic_errors(state, reference):
    ref = reference.symbolic(state[4])
    tangent = ref["dxy"]/ca.sqrt(ca.dot(ref["dxy"], ref["dxy"]))
    normal = ca.vertcat(-tangent[1], tangent[0])
    displacement = state[:2]-ref["xy"]
    return ca.dot(normal, displacement), ca.dot(tangent, displacement), ref


def equivalence_audit(reference, count: int = 128) -> dict:
    q = np.linspace(-reference.length, 2*reference.length, count, endpoint=False)
    states=[]
    for i, theta in enumerate(q):
        ref=reference.numpy(theta);normal=np.asarray([-np.sin(ref["yaw"]),np.cos(ref["yaw"])])
        tangent=np.asarray([np.cos(ref["yaw"]),np.sin(ref["yaw"])])
        xy=np.asarray([ref["x"],ref["y"]])+(.08*np.sin(.37*i))*normal+(.11*np.cos(.23*i))*tangent
        states.append([xy[0],xy[1],ref["yaw"],ref["v"],theta])
    x=ca.MX.sym("d7_error_state",5);ec,el,_=symbolic_errors(x,reference)
    fun=ca.Function("d7_error_equivalence",[x],[ec,el])
    differences=[]; signs=[]
    for state in states:
        expected=numpy_errors(np.asarray(state),reference)[:2]
        actual=np.asarray(fun(state)).reshape(-1)
        differences.extend((actual-np.asarray(expected)).tolist())
        signs.append((np.sign(actual)==np.sign(expected)) | (np.abs(actual)<1e-13))
    diff=np.asarray(differences)
    return {"probes":count,"max_abs_error":float(np.max(np.abs(diff))),"rms_error":float(np.sqrt(np.mean(diff*diff))),"signs_match":bool(np.all(signs)),"passed":bool(np.max(np.abs(diff))<=1e-10)}

