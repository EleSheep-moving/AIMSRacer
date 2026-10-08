"""Hardware-free fixed-request comparison; never publishes ROS commands."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np


def summarize(rows, budget_s=.05):
    durations = [row['full_request_s'] for row in rows]
    run = longest = 0
    for duration in durations:
        run = run + 1 if duration > budget_s else 0
        longest = max(longest, run)
    checked=[row for row in rows if row.get('validation') is not None]
    return dict(requests=len(rows), failures=sum(not row['success'] for row in rows),
                independently_checked=len(checked),
                executable_candidates=sum(row['success'] and row['validation']['accepted'] for row in checked),
                overruns=sum(t > budget_s for t in durations),
                max_consecutive_overruns=longest,
                full_request_p95_s=float(np.percentile(durations, 95)) if durations else None,
                full_request_p99_s=float(np.percentile(durations, 99)) if durations else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True)
    parser.add_argument('--fixtures', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--vehicle-config', help='Override recorded vehicle profile for the candidate')
    parser.add_argument('--backend', choices=['ipopt', 'acados', 'qp'], default='ipopt')
    parser.add_argument('--artifact-directory')
    parser.add_argument('--recovery-speed-refs',action='store_true',
                        help='Compare the recovery policy with zero speed references; keep recorded state and applied inputs')
    parser.add_argument('--repeats', type=int, default=1)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    from .config import VehicleConfig
    from .io import load_config
    from .path import ReferencePath
    from .solver_diagnostics import json_safe
    fixtures_file = Path(args.fixtures)
    fixtures = json.loads(fixtures_file.read_text())
    path = ReferencePath.load(args.reference)
    rows = []
    solvers = {}
    for repeat in range(args.repeats):
        for fixture in fixtures:
            config = load_config(args.vehicle_config) if args.vehicle_config else VehicleConfig(**fixture['config'])
            key = json.dumps(asdict(config), sort_keys=True) + str(fixture['horizon'])
            if key not in solvers:
                if args.backend == 'ipopt':
                    from .solver import MPCCSolver
                    solvers[key] = MPCCSolver(path, config, horizon=fixture['horizon'], dt=fixture['dt'])
                else:
                    from .backends import create_solver
                    solvers[key] = create_solver(args.backend, path, config, fixture['horizon'], fixture['dt'],
                                                 artifact_directory=args.artifact_directory)
            solver = solvers[key]
            solver.reset()
            # Reproduce the original IPOPT primal seed rather than inferring it
            # from another run's history. Other cores use their native warm start.
            if args.backend == 'ipopt':
                solver._warm_start = lambda *unused, f=fixture: (
                    np.asarray(f['recorded_warm_states']), np.asarray(f['recorded_warm_controls']))
                solver.previous_theta = fixture['recorded_initial'][4]
                solver.previous_yaw = fixture['recorded_initial'][2]
            request = dict(fixture['request'])
            if args.recovery_speed_refs:
                request['speed_refs']=[0.]*(fixture['horizon']+1)
            start = time.perf_counter()
            result = solver.solve(request['state'], request['previous'], request['speed_refs'],
                                  request['elapsed'], request.get('map_alignment'))
            validation=None
            if result.get('success') and hasattr(config,'envelope_soft_enabled'):
                from .validation import validate_candidate
                validation=validate_candidate(dict(result, dt=fixture['dt'],
                    validation_applied=[request['previous'][k] for k in ('acceleration','steering','steering_rate')],
                    map_alignment=request.get('map_alignment')),config,path)
            duration = time.perf_counter() - start
            rows.append(dict(id=fixture['id'], repeat=repeat, backend=args.backend,
                             config=asdict(config), full_request_s=duration, validation=validation,
                             **{k: v for k, v in result.items() if k not in ['failure_snapshot', 'solve_input']}))
    report = dict(schema_version=1, layer='fixed-input-direct-call', hardware_validated=False,
                  includes_ipc=False, includes_solver_construction=False,
                  speed_reference_policy='recovery-zero-speed' if args.recovery_speed_refs else 'recorded',
                  fixtures_sha256=hashlib.sha256(fixtures_file.read_bytes()).hexdigest(),
                  reference_sha256=hashlib.sha256((Path(args.reference)/'path.csv').read_bytes()).hexdigest(),
                  summary=summarize(rows), requests=rows)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(json_safe(report), allow_nan=False, indent=2) + '\n')
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
