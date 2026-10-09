#!/usr/bin/env python3
"""Reproduce migration counterexamples offline against a frozen runtime source.

Build the actual native callbacks in a separate executable using the existing
test friend. No executor spins, no physical driver is opened, and no production
binary/bundle is modified. Success means counterexamples reproduced; it does
not qualify the runtime. Source the desktop ROS overlay before running.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys


def run(args):
    root = Path(__file__).resolve().parents[3]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    build = args.build_dir.resolve()
    bundles = {}
    for name, directory in (('synthetic', args.synthetic_bundle), ('measured', args.measured_bundle)):
        manifest = json.loads((directory/'manifest.json').read_text())
        config = json.loads((directory/'config.json').read_text())
        assert manifest['horizon'] == 10 and manifest['dt'] == .1 and not config['envelope_soft_enabled']
        bundles[name] = dict(directory=str(directory.resolve()), fingerprint=manifest['fingerprint'],
                             config=config, native_manifest=json.loads((directory/'native_manifest.json').read_text()))
    source = Path(__file__).with_suffix('.cpp')
    flags = (build / 'CMakeFiles/test_runtime_clock.dir/flags.make').read_text()
    compiler_flags = []
    for name in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        compiler_flags += shlex.split(re.search(r'^' + name + r' = (.*)$', flags, re.M)[1])
    compile_command = ['/usr/bin/c++', *compiler_flags, '-c', str(source), '-o', str(out/'audit.o')]
    link_command = shlex.split((build/'CMakeFiles/test_runtime_clock.dir/link.txt').read_text())
    link_command = [str(out/'audit.o') if x.endswith('test_runtime_clock.cpp.o') else x
                    for x in link_command]
    link_command[link_command.index('-o')+1] = str(out/'audit_contracts')
    for command in (compile_command, link_command):
        result = subprocess.run(command, cwd=build, text=True, capture_output=True)
        with (out/'build.log').open('a') as log:
            log.write(result.stdout + result.stderr)
        result.check_returncode()
    result = subprocess.run([str(out/'audit_contracts'), str(args.synthetic_bundle.resolve()),
                             str(args.measured_bundle.resolve())], text=True, capture_output=True,
                            timeout=20)
    (out/'native.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    marker = next(line[len('AUDIT_JSON '):] for line in result.stdout.splitlines()
                  if line.startswith('AUDIT_JSON '))
    report = json.loads(marker)

    # Independent legacy implementation: validate the same strict macro controls
    # and its actual 20 ms held-output schedule, with the same live input prefix.
    sys.path.insert(0, str(root/'src/controller'))
    from aims_mpcc.config import VehicleConfig
    from aims_mpcc.execution import validate_execution
    from aims_mpcc.runtime import State, Command, Supervisor
    from aims_mpcc.envelope import independent_rollout
    from aims_mpcc.validation import validate_candidate
    import numpy as np
    config = VehicleConfig(**json.loads((args.measured_bundle/'config.json').read_text()))
    initial = [0., 0., 0., .79, 0., .4]
    previous = [.4, .4, 0.]
    controls = [[max(0., .3-.1*k), .4, .79] for k in range(10)]
    applied = dict(speed=.79, steering=.4, acceleration=.4, steering_rate=0.)
    supervisor = Supervisor(config, 100., handover_delay=.02, solve_period=.05)
    supervisor.observe(State(0., 0., 0., 0., .4, 100.), 10., 0., 0.)
    supervisor.set_mode(True, 10.)
    supervisor.start(10.)
    supervisor.state = State(0., 0., 0., .79, .4, 100.02)
    supervisor.state_received = supervisor.mode_received = 10.02
    supervisor.last_command = Command(.79, .4)
    supervisor.last_acceleration = .4
    supervisor.last_steering_rate = 0.
    plan = dict(success=True, states=independent_rollout(initial, previous, controls, config)[::5].tolist(),
                controls=controls, validation_applied=previous, previous_steering=.4,
                source_stamp=10., submitted_at=10., stamp=10.02, dt=.1)
    plan['execution_speed_targets'] = [.79]
    for control in controls:
        plan['execution_speed_targets'].append(plan['execution_speed_targets'][-1]+control[0]*.1)
    macro = validate_candidate(plan, config)
    execution = validate_execution(supervisor, plan, initial, applied, 10.02)
    assert macro['accepted'] and not execution['accepted'], 'legacy counterexample did not reproduce'
    first = np.asarray(execution['controls'])[:5, 0]
    np.testing.assert_allclose(first, np.asarray(report['execution']['trace'])[:5, 2], atol=1e-12)
    report['legacy_execution'] = dict(macro_accepted=macro['accepted'], execution_accepted=execution['accepted'],
                                     reason=execution['reason'], first_accelerations=first.tolist(),
                                     envelope=execution['envelope'])
    report['scope'] = 'offline counterexample reproduction; no runtime acceptance or physical test'
    report['runtime_commit'] = args.runtime_commit or subprocess.check_output(
        ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    report['runtime_commit_supplied'] = bool(args.runtime_commit)
    report['bundles'] = bundles
    files = [root/'src/aims_mpcc_rt/src/node.cpp', root/'src/aims_mpcc_rt/src/core.cpp', source,
             root/'src/aims_mpcc_rt/include/aims_mpcc_rt/output.hpp',
             root/'src/controller/aims_mpcc/execution.py']
    files += [Path(__file__).resolve(), build/'libmpcc_rt_core.a', out/'audit_contracts']
    report['sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('execution', 'sha256')}, indent=2))
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('build-dir', 'synthetic-bundle', 'measured-bundle', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--runtime-commit', help='Host-verified commit for containers with an unmounted worktree gitdir')
    raise SystemExit(run(parser.parse_args()))
