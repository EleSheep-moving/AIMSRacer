"""Build/refresh the native solver cache without ROS or actuator access."""
import argparse
import time
from pathlib import Path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path_directory')
    parser.add_argument('--vehicle-config', required=True)
    args = parser.parse_args(args)
    from .io import load_config
    from .path import ReferencePath
    from .solver import MPCCSolver
    from .native import build_context, solver_options
    path = ReferencePath.load(Path(args.path_directory).resolve())
    config = load_config(Path(args.vehicle_config).resolve())
    started = time.monotonic()
    with build_context(allow_compile=True) as cache:
        print(f'Preparing MPCC solver cache: {cache}', flush=True)
        solver = MPCCSolver(path, config, jit_enabled=True, native_options=solver_options())
        point = path.at(0.)
        result = solver.solve(
            dict(x=point['x'], y=point['y'], yaw=point['yaw'], speed=0., steering=0.),
            dict(acceleration=0., steering=0., steering_rate=0.), [0.] * (solver.n + 1))
        if not result['success']:
            raise RuntimeError('Solver preparation failed: ' + result['status'])
        del solver
    print(f'MPCC solver cache ready in {time.monotonic() - started:.2f} s', flush=True)
