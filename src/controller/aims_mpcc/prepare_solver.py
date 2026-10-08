"""Build/refresh the native solver cache without ROS or actuator access."""
import argparse
import json
import time
from contextlib import nullcontext
from pathlib import Path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path_directory')
    parser.add_argument('--vehicle-config', required=True)
    parser.add_argument('--horizon', type=int, default=10,
                        help='Number of 0.1 s prediction intervals (default: 10)')
    parser.add_argument('--backend',choices=['ipopt','acados','qp'],default='ipopt')
    parser.add_argument('--artifact-directory')
    args = parser.parse_args(args)
    if args.horizon < 1:
        parser.error('--horizon must be a positive integer')
    from .io import load_config
    from .path import ReferencePath
    from .backends import create_solver
    from .native import build_context, solver_options
    path = ReferencePath.load(Path(args.path_directory).resolve())
    config = load_config(Path(args.vehicle_config).resolve())
    started = time.monotonic()
    from .rollout_native import prepare_kernel
    print(f'Independent rollout library: {prepare_kernel()}',flush=True)
    with (build_context(allow_compile=True) if args.backend=='ipopt' else nullcontext(args.artifact_directory)) as cache:
        print(f'Preparing {args.backend} solver cache: {cache}', flush=True)
        solver = create_solver(args.backend,path,config,horizon=args.horizon,prepare=True,
                               artifact_directory=args.artifact_directory,jit_enabled=args.backend=='ipopt',
                               native_options=solver_options() if args.backend=='ipopt' else None)
        point = path.at(0.)
        result = solver.solve(
            dict(x=point['x'], y=point['y'], yaw=point['yaw'], speed=0., steering=0.),
            dict(acceleration=0., steering=0., steering_rate=0.), [0.] * (solver.n + 1),
            map_alignment=(0., 0., 0.) if path.frame_id == 'map' else None)
        print(json.dumps(dict(status=result['status'],iterations=result['iterations'],
                              solve_time_s=result['solve_time_s'],diagnostics=result['diagnostics']),
                         allow_nan=False),flush=True)
        if not result['success']:
            raise RuntimeError('Solver preparation failed: ' + result['status'])
        del solver
    print(f'MPCC solver cache ready in {time.monotonic() - started:.2f} s', flush=True)
