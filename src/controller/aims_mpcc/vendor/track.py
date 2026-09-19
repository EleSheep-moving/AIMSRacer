# Adapted periodic arc-length representation inspired by alexliniger/MPCC.
# Upstream: https://github.com/alexliniger/MPCC, commit bd331621ba47ae3326711922a863bdb1cdf2d2ea
# Copyright (c) 2018-2020 Alexander Liniger; Apache-2.0.
# Modification: independent Python/CasADi implementation using repository BARC data.
"""Single-coefficient periodic TrackSpline and corrected PathReference."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

import casadi as ca
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize_scalar


def wrap_s(value, length: float = 17.0):
    if isinstance(value, (ca.SX, ca.MX, ca.DM)):
        return value - length * ca.floor(value / length)
    return np.mod(value, length)


class PeriodicCubic:
    """Periodic cubic with exactly one coefficient table for NumPy and CasADi."""

    def __init__(self, s: Sequence[float], values: np.ndarray, length: float = 17.0, name: str = "spline"):
        self.length, self.name = float(length), str(name)
        self.s = np.asarray(s, dtype=float)
        self.values = np.asarray(values, dtype=float)
        if self.s.ndim != 1 or len(self.s) < 4 or abs(self.s[0]) > 1e-12 or abs(self.s[-1]-self.length) > 1e-10:
            raise ValueError("periodic knots must span [0,length]")
        if not np.all(np.diff(self.s) > 0):
            raise ValueError("spline knots must be strictly increasing")
        if self.values.shape[0] != len(self.s):
            raise ValueError("values and knots differ")
        if np.max(np.abs(self.values[-1]-self.values[0])) > 1e-9:
            raise ValueError(f"{name} endpoint is not periodic")
        self._spline = CubicSpline(self.s, self.values, bc_type="periodic", axis=0)
        self.coefficients = np.asarray(self._spline.c, dtype=float)
        self.output_shape = self.values.shape[1:]

    def numpy(self, theta, derivative: int = 0):
        return np.asarray(self._spline(wrap_s(theta, self.length), nu=derivative))

    def symbolic(self, theta, derivative: int = 0):
        sw = wrap_s(theta, self.length)
        columns = int(np.prod(self.output_shape)) if self.output_shape else 1
        coeff = self.coefficients.reshape((4, len(self.s)-1, columns))
        # MX supports a compact binary-search + dynamic coefficient lookup.
        # This retains the exact SciPy coefficient table while avoiding a
        # multi-megabyte if_else graph in generated acados code.
        if isinstance(theta, ca.MX):
            index = ca.low(ca.MX(ca.DM(self.s)), sw, {})
            index = ca.fmin(index, len(self.s)-2)
            knot = ca.MX(ca.DM(self.s))[index]
            d = sw-knot; values=[]
            for j in range(columns):
                c0=ca.MX(ca.DM(coeff[0,:,j]))[index];c1=ca.MX(ca.DM(coeff[1,:,j]))[index]
                c2=ca.MX(ca.DM(coeff[2,:,j]))[index];c3=ca.MX(ca.DM(coeff[3,:,j]))[index]
                if derivative == 0: item=c0*d**3+c1*d**2+c2*d+c3
                elif derivative == 1: item=3*c0*d**2+2*c1*d+c2
                elif derivative == 2: item=6*c0*d+2*c1
                else: raise ValueError("only derivative orders 0,1,2 are supported")
                values.append(item)
            return ca.vertcat(*values)
        pieces = []
        for i in range(len(self.s)-1):
            d = sw-self.s[i]
            val=[]
            for j in range(columns):
                if derivative == 0:
                    item = coeff[0,i,j]*d**3 + coeff[1,i,j]*d**2 + coeff[2,i,j]*d + coeff[3,i,j]
                elif derivative == 1:
                    item = 3*coeff[0,i,j]*d**2 + 2*coeff[1,i,j]*d + coeff[2,i,j]
                elif derivative == 2:
                    item = 6*coeff[0,i,j]*d + 2*coeff[1,i,j]
                else:
                    raise ValueError("only derivative orders 0,1,2 are supported")
                val.append(item)
            pieces.append(ca.vertcat(*val))
        out = pieces[-1]
        for i in range(len(pieces)-2, -1, -1):
            out = ca.if_else(sw < self.s[i+1], pieces[i], out)
        return out

