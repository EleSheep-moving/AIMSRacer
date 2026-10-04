"""Build/refresh the native solver cache without ROS or actuator access."""
import argparse
import time
from pathlib import Path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path_directory')
    parser.add_argument('--vehicle-config', required=True)
    parser.add_argument('--horizon', type=int, default=10,
                        help='Number of 0.1 s prediction intervals (default: 10)')
    args = parser.parse_args(args)
    if args.horizon < 1:
        parser.error('--horizon must be a positive integer')
    from .io import load_config
    from .path import ReferencePath
    from .solver import MPCCSolver
    from .native import build_context, solver_options
    path = ReferencePath.load(Path(args.path_directory).resolve())
    config = load_config(Path(args.vehicle_config).resolve())
    started = time.monotonic()
    with build_context(allow_compile=True) as cache:
        print(f'Preparing MPCC solver cache: {cache}', flush=True)
        solver = MPCCSolver(path, config, horizon=args.horizon, jit_enabled=True, native_options=solver_options())
        point = path.at(0.)
        result = solver.solve(
            dict(x=point['x'], y=point['y'], yaw=point['yaw'], speed=0., steering=0.),
            dict(acceleration=0., steering=0., steering_rate=0.), [0.] * (solver.n + 1),
            map_alignment=(0., 0., 0.) if path.frame_id == 'map' else None)
        if not result['success']:
            raise RuntimeError('Solver preparation failed: ' + result['status'])
        del solver
    print(f'MPCC solver cache ready in {time.monotonic() - started:.2f} s', flush=True)
