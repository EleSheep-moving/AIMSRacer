#!/usr/bin/env python3
"""Compare actual legacy Opti objective with compiled frozen native costs.

Inputs are arbitrary objective-evaluation points, not feasible trajectories or
closed-loop regression evidence. No optimization, artifact generation or ROS
node is run. Requires the pinned native bundle and legacy Python dependencies.
"""
import argparse
import json
from pathlib import Path
import sys

import casadi as ca
import numpy as np


def run(args):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'src/controller'))
    from aims_mpcc.config import VehicleConfig
    from aims_mpcc.path import ReferencePath
    from aims_mpcc.solver import MPCCSolver
    cfg = VehicleConfig(**json.loads((args.bundle/'config.json').read_text()))
    manifest = json.loads((args.bundle/'manifest.json').read_text())
    assert manifest['horizon'] == 10 and manifest['dt'] == .1 and not cfg.envelope_soft_enabled
    assert manifest['cost_scaling'] == [1.]*11, 'comparison requires unscaled N10/100 ms costs'
    path = ReferencePath.load(args.bundle/'input_reference')
    assert path.frame_id == 'odom', 'this focused cost fixture uses odom geometry'
    solver = MPCCSolver(path, cfg, horizon=10, dt=.1, jit_enabled=False)
    exact = ca.Function('actual_legacy_objective',
                        [solver.x, solver.u, solver.applied, solver.steering_bias,
                         solver.speed_refs, solver.cost_weights], [solver.op.f])
    lib = str((args.bundle/'libaims_mpcc_bundle.so').resolve())
    stage = ca.external('rt_cost', lib)
    terminal = ca.external('rt_terminal_cost', lib)
    rows = []
    for departure in (0., .05, .2, .5):
        seed = .8
        theta = seed + departure
        r = path.at(theta)
        angle = np.arctan(cfg.wheelbase*(1+cfg.understeer_coefficient*cfg.cruise_speed**2)*r['curvature'])
        physical = np.array([r['x'], r['y'], r['yaw'], cfg.cruise_speed, theta, angle])
        control = np.array([0., angle, cfg.cruise_speed])
        previous = np.array([0., angle, 0.])
        weights = [cfg.contour_weight, cfg.heading_weight, cfg.speed_weight,
                   cfg.steering_weight, cfg.steering_rate_weight,
                   cfg.steering_acceleration_weight, cfg.terminal_weight]
        legacy = float(exact(np.tile(physical[:, None], (1, 11)),
                             np.tile(control[:, None], (1, 10)), previous, 0.,
                             np.full(11, cfg.cruise_speed), weights))
        frozen = path.at(seed)
        norm = np.linalg.norm(path.curve.numpy(seed, 1))
        p = np.array([frozen['x'], frozen['y'], frozen['yaw'], frozen['curvature'],
                      seed, norm, cfg.cruise_speed, cfg.jerk_limit, 0., 1.])
        internal = np.r_[physical, previous]
        native = 10*float(stage(internal, control, p))+float(terminal(internal, p))
        h = np.array([np.cos(frozen['yaw']), np.sin(frozen['yaw'])])
        offset = physical[:2]-(np.array([frozen['x'], frozen['y']])+h*norm*departure)
        rows.append(dict(seed_progress=seed, candidate_progress=theta,
                         legacy_objective=legacy, native_frozen_objective=native,
                         frozen_contour_m=float(offset@np.array([-h[1], h[0]])),
                         frozen_lag_m=float(offset@h), heading_difference_rad=r['yaw']-frozen['yaw']))
    assert abs(rows[0]['legacy_objective']-rows[0]['native_frozen_objective']) < 1e-8
    assert rows[-1]['native_frozen_objective'] > rows[-1]['legacy_objective']+1.
    report = dict(scope='arbitrary objective points, not feasible/closed-loop trajectories',
                  bundle_fingerprint=manifest['fingerprint'], config=vars(cfg), rows=rows)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    run(p.parse_args())
