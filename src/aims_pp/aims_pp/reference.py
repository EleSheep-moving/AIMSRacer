"""Canonical periodic quintic evaluation and projection without SciPy/CasADi."""
import json
import math
from pathlib import Path
import re

import numpy as np


class NumericReference:
    @classmethod
    def load(cls, directory):
        file = Path(directory) / 'reference.json'
        # Limit untrusted startup work; runtime never opens reference files.
        if file.stat().st_size > 64*1024*1024:
            raise ValueError('reference exceeds 64 MiB')
        return cls(json.loads(file.read_text()))

    def __init__(self, data):
        if data.get('schema_version', 1) != 1:
            raise ValueError('unsupported reference schema')
        self.knots = np.asarray(data['knots'], dtype=float)
        self.coefficients = np.asarray(data['coefficients'], dtype=float)
        self.length = float(data['length'])
        self.left_width, self.right_width = float(data['left_width']), float(data['right_width'])
        self.frame_id, self.metadata = data['frame_id'], data['metadata']
        if self.frame_id != 'map' or not isinstance(self.metadata, dict) or self.metadata.get('closed_lap') is not True:
            raise ValueError('closed forward map reference required')
        if self.metadata.get('source_frame') != 'base_link' or self.metadata.get('coordinate_reference', 'rear_axle') != 'rear_axle':
            raise ValueError('base_link source and rear_axle coordinate reference required')
        self.map_sha256 = self.metadata.get('map_sha256', '')
        if not isinstance(self.map_sha256, str) or not re.fullmatch('[0-9a-f]{64}', self.map_sha256):
            raise ValueError('map SHA-256 required')
        if not isinstance(self.metadata.get('vehicle_geometry'), dict):
            raise ValueError('complete vehicle geometry required')
        if not np.isfinite([self.length, self.left_width, self.right_width]).all() or min(self.length, self.left_width, self.right_width) <= 0:
            raise ValueError('positive finite length and widths required')
        if self.knots.ndim != 1 or not 3 <= len(self.knots) <= 20001 or self.knots[0] != 0 or abs(self.knots[-1]-self.length)>1e-9 or not np.isfinite(self.knots).all() or np.any(np.diff(self.knots)<=0):
            raise ValueError('strict increasing knots spanning the length required')
        if self.coefficients.shape != (6, len(self.knots)-1, 2) or not np.isfinite(self.coefficients).all():
            raise ValueError('finite degree-5 coefficients[6][interval][2] required')
        speed = data['speed_profile']
        self.speed_positions = np.asarray(speed['positions'], dtype=float)
        self.speeds = np.asarray(speed['speeds'], dtype=float)
        if speed.get('schema_version') != 1 or self.speed_positions.ndim != 1 or self.speeds.shape != self.speed_positions.shape or len(self.speeds)<1 or not np.isfinite(self.speed_positions).all() or not np.isfinite(self.speeds).all() or self.speed_positions[0] != 0 or self.speed_positions[-1]>=self.length or np.any(np.diff(self.speed_positions)<=0) or np.any(self.speeds<0):
            raise ValueError('invalid frozen speed profile')
        # Check every interval and closure, including derivatives. A flag alone
        # cannot turn an open or discontinuous path into a periodic reference.
        for i in range(len(self.knots)-1):
            j = (i+1) % (len(self.knots)-1)
            h = self.knots[i+1]-self.knots[i]
            for derivative in range(3):
                if not np.allclose(self._evaluate(i, h, derivative), self._evaluate(j, 0., derivative), rtol=0, atol=1e-5):
                    raise ValueError('quintic reference has a discontinuous seam')
        count = max(64, int(math.ceil(self.length/.05)))
        if count > 20000:
            raise ValueError('reference exceeds bounded projection length')
        self.sample_positions = np.linspace(0, self.length, count, endpoint=False)
        self.sample_xy = np.array([[self.at(s)['x'], self.at(s)['y']] for s in self.sample_positions])
        if np.any([np.linalg.norm(self.derivatives(s)[1]) < 1e-6 for s in self.sample_positions]):
            raise ValueError('reference contains zero tangent')

    def validate_config(self, config):
        config.validate()
        geometry = self.metadata['vehicle_geometry']
        for key in ['wheelbase', 'rear_offset', 'half_length', 'front_extent', 'rear_extent', 'half_width']:
            expected, value = getattr(config, key), geometry.get(key)
            if value is None and expected is None and key in geometry:
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or expected is None or not math.isclose(value, expected, rel_tol=0, abs_tol=1e-9):
                raise ValueError(f'vehicle geometry mismatch: {key}')
        if min(self.left_width, self.right_width) <= config.half_width or np.max(self.speeds)>config.max_speed:
            raise ValueError('reference width or speed incompatible with vehicle')
        return self

    def _evaluate(self, interval, h, derivative=0):
        result = np.zeros(2)
        for k in range(6-derivative):
            factor = math.prod(range(5-k-derivative+1, 5-k+1))
            result = result*h + factor*self.coefficients[k, interval]
        return result

    def derivatives(self, s):
        if not math.isfinite(s):
            raise ValueError('finite progress required')
        sw = s % self.length
        index = min(int(np.searchsorted(self.knots, sw, side='right'))-1, len(self.knots)-2)
        h = sw-self.knots[index]
        return tuple(self._evaluate(index, h, d) for d in range(3))

    def at(self, s):
        p, d, dd = self.derivatives(s)
        norm = float(np.dot(d, d))
        if norm < 1e-12:
            raise ValueError('zero reference tangent')
        return dict(x=float(p[0]), y=float(p[1]), yaw=math.atan2(d[1], d[0]), curvature=float((d[0]*dd[1]-d[1]*dd[0])/norm**1.5))

    def project(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('finite planar position required')
        index = int(np.argmin(np.sum((self.sample_xy-xy)**2, axis=1)))
        candidates = []
        for offset in [-1, 0, 1]:
            s = self.sample_positions[(index+offset)%len(self.sample_positions)]
            for _ in range(8):
                p, d, dd = self.derivatives(s)
                denominator = float(d@d+(p-xy)@dd)
                if abs(denominator)<1e-10:
                    break
                step = float((p-xy)@d)/denominator
                s = (s-float(np.clip(step, -.1, .1))) % self.length
            p, d, _ = self.derivatives(s)
            candidates.append((float((p-xy)@(p-xy)), s, p, d))
        _, s, p, d = min(candidates, key=lambda entry: entry[0])
        normal = np.array([-d[1], d[0]])/np.linalg.norm(d)
        return float(s), float((xy-p)@normal)

    def speed_at(self, s):
        if not math.isfinite(s):
            raise ValueError('finite progress required')
        return float(np.interp(s%self.length, np.r_[self.speed_positions, self.length], np.r_[self.speeds, self.speeds[0]]))
