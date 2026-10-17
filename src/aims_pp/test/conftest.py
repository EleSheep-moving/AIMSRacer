import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))


@pytest.fixture
def circle_payload():
    # C2 quintic Hermite circle, radius 2m, parameterized in metres.
    radius = 2.
    knots = np.linspace(0, 2 * math.pi * radius, 17)
    coeff = np.zeros((6, len(knots) - 1, 2))
    for i, (a, b) in enumerate(zip(knots[:-1], knots[1:])):
        h = b - a
        p = lambda s: np.array([radius * math.cos(s / radius), radius * math.sin(s / radius)])
        d = lambda s: np.array([-math.sin(s / radius), math.cos(s / radius)])
        dd = lambda s: np.array([-math.cos(s / radius), -math.sin(s / radius)]) / radius
        low = np.array([p(a), d(a), dd(a) / 2])
        matrix = [[h**3, h**4, h**5], [3*h**2, 4*h**3, 5*h**4], [6*h, 12*h**2, 20*h**3]]
        rhs = [p(b)-low[0]-h*low[1]-h*h*low[2], d(b)-low[1]-2*h*low[2], dd(b)-2*low[2]]
        high = np.linalg.solve(matrix, rhs)
        coeff[:, i, :] = np.vstack([low, high])[::-1]
    return dict(schema_version=1, knots=knots.tolist(), coefficients=coeff.tolist(), length=float(knots[-1]),
                left_width=.8, right_width=.8, frame_id='map',
                metadata=dict(map_sha256='a'*64, closed_lap=True, source_frame='base_link', coordinate_reference='rear_axle',
                              vehicle_geometry=dict(wheelbase=.36, rear_offset=0., half_length=None, front_extent=.52, rear_extent=.1, half_width=.16)),
                speed_profile=dict(schema_version=1, positions=[0., 3., 6., 9.], speeds=[1., 1.5, 2., .5]))


@pytest.fixture
def reference(tmp_path, circle_payload):
    from aims_pp.reference import NumericReference
    (tmp_path / 'reference.json').write_text(json.dumps(circle_payload))
    return NumericReference.load(tmp_path)


def health_values(sequence=1, stamp_ns=10_000_000_000, epoch='one', health_sequence=None):
    return dict(protocol_version='1', epoch=epoch, health_sequence=str(health_sequence or sequence), anchor_sequence=str(sequence),
                last_anchor_stamp_ns=str(stamp_ns), ready='true', map_sha256='a'*64, map_valid='true', alignment_valid='true',
                alignment_epoch=epoch, alignment_anchor_sequence=str(sequence), alignment_stamp_ns=str(stamp_ns),
                map_odom_x='0', map_odom_y='0', map_odom_z='0', map_odom_qx='0', map_odom_qy='0', map_odom_qz='0', map_odom_qw='1')


@pytest.fixture
def inputs(reference):
    from aims_pp.core import PPConfig
    from aims_pp.tracking import TrackingInputs
    state = TrackingInputs(PPConfig(), reference)
    refresh(state, 10., 10.)
    return state


def refresh(state, ros, mono, sequence=1, pose=None):
    ns = round(ros * 1e9)
    state.observe_map('a'*64)
    state.observe_map_valid(True)
    state.observe_health(health_values(sequence, ns), ns, mono)
    state.observe_authority(True, mono)
    state.observe_command(0, 0., 1., ns, ns, mono)
    if pose is None:
        pose = (2., 0., math.pi/2, 1.)
    state.observe_odometry(*pose, ns, ns, mono)
