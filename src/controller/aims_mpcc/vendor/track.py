# Adapted periodic arc-length representation inspired by alexliniger/MPCC.
# Upstream: https://github.com/alexliniger/MPCC, commit bd331621ba47ae3326711922a863bdb1cdf2d2ea
# Copyright (c) 2018-2020 Alexander Liniger; Apache-2.0.
# Modification: independent Python/CasADi implementation, extended with a
# periodic quintic reference for smooth curvature-dependent optimization costs.
"""Single-coefficient periodic TrackSpline and corrected PathReference."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

import casadi as ca
import numpy as np
from scipy.interpolate import CubicSpline, PPoly, make_interp_spline
from scipy.optimize import minimize_scalar


def wrap_s(value, length: float = 17.0):
    if isinstance(value, (ca.SX, ca.MX, ca.DM)):
        return value - length * ca.floor(value / length)
    return np.mod(value, length)


class _PeriodicPolynomial:
    """Periodic interpolation with one coefficient table for NumPy and CasADi."""

    degree = 3

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
        self.output_shape = self.values.shape[1:]
        if self.degree == 3:
            self._spline = CubicSpline(self.s, self.values, bc_type="periodic", axis=0)
            self.coefficients = np.asarray(self._spline.c, dtype=float)
        else:
            columns = self.values.reshape(len(self.s), -1)
            spline = make_interp_spline(self.s, columns, k=self.degree,
                                       bc_type="periodic", axis=0)
            pieces = [PPoly.from_spline((spline.t, spline.c[:, j], spline.k))
                      for j in range(columns.shape[1])]
            # Periodic B-splines include knots outside [0, length]. Keep only
            # the actual lap intervals, without changing the recorded knots.
            knots = pieces[0].x
            indices = np.flatnonzero((knots[:-1] >= 0.) &
                                     (knots[:-1] < self.length) & (np.diff(knots) > 0.))
            if not np.array_equal(knots[indices], self.s[:-1]):
                raise ValueError("periodic spline intervals differ from reference knots")
            self.coefficients = np.stack([piece.c[:, indices] for piece in pieces], axis=-1)
            self.coefficients = self.coefficients.reshape(
                (self.degree + 1, len(self.s) - 1) + self.output_shape)
            self._spline = PPoly(self.coefficients, self.s, extrapolate="periodic")

    def numpy(self, theta, derivative: int = 0):
        return np.asarray(self._spline(wrap_s(theta, self.length), nu=derivative))

    def symbolic(self, theta, derivative: int = 0):
        if derivative not in (0, 1, 2):
            raise ValueError("only derivative orders 0,1,2 are supported")
        sw = wrap_s(theta, self.length)
        columns = int(np.prod(self.output_shape)) if self.output_shape else 1
        coeff = self.coefficients.reshape((self.degree + 1, len(self.s)-1, columns))

        def polynomial(d, coefficient):
            # Horner evaluation also applies to the first two derivatives.
            item = 0
            for row in range(self.degree + 1 - derivative):
                power = self.degree - row
                factor = math.prod(power - order for order in range(derivative))
                item = item * d + factor * coefficient(row)
            return item
        # MX supports a compact binary-search + dynamic coefficient lookup.
        # This retains the exact SciPy coefficient table while avoiding a
        # multi-megabyte if_else graph in generated acados code.
        if isinstance(theta, ca.MX):
            index = ca.low(ca.MX(ca.DM(self.s)), sw, {})
            index = ca.fmin(index, len(self.s)-2)
            knot = ca.MX(ca.DM(self.s))[index]
            d = sw-knot; values=[]
            for j in range(columns):
                values.append(polynomial(d, lambda row: ca.MX(ca.DM(coeff[row,:,j]))[index]))
            return ca.vertcat(*values)
        pieces = []
        for i in range(len(self.s)-1):
            d = sw-self.s[i]
            val=[]
            for j in range(columns):
                val.append(polynomial(d, lambda row: coeff[row,i,j]))
            pieces.append(ca.vertcat(*val))
        out = pieces[-1]
        for i in range(len(pieces)-2, -1, -1):
            out = ca.if_else(sw < self.s[i+1], pieces[i], out)
        return out


class PeriodicCubic(_PeriodicPolynomial):
    """C2 periodic cubic, retained for compatibility and comparisons."""

    degree = 3


class PeriodicQuintic(_PeriodicPolynomial):
    """C4 periodic quintic; curvature has two continuous derivatives."""

    degree = 5
