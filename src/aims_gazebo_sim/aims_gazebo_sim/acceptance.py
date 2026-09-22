"""Run an MPCC lap through Gazebo Fortress vehicle and contact physics."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool

from aims_mpcc.path import ReferencePath
from aims_mpcc_sim.acceptance import AcceptanceObserver, launch_initial_pose_arguments, write_artifacts
from aims_mpcc_sim.metrics import meets_acceptance_thresholds, summarize_tracking


def run_command(command, environment):
    subprocess.run(command, check=True, env=environment)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--timeout', type=float, default=100.0)
    parser.add_argument('--track', choices=('circle', 'figure_eight'), default='circle')
    parser.add_argument('--radius', type=float, default=8.0)
    parser.add_argument('--waist-ratio', type=float, default=0.3)
    parsed = parser.parse_args(args)
    output = Path(parsed.output).resolve()
    if output.exists():
        raise FileExistsError(f'refusing to overwrite {output}')
    output.mkdir(parents=True)
    environment = dict(os.environ)
    # Let Docker mount a shared compiler cache. The output directory still keeps
    # the reference, controller log, trajectory, and immutable summary together.
    environment.setdefault('AIMS_MPCC_CACHE_DIR', str(output / 'ccache'))
    share = Path(get_package_share_directory('aims_gazebo_sim'))
    config = share / 'config' / 'gazebo.yaml'
    fixture = ['ros2', 'run', 'aims_mpcc_sim', 'prepare_circle', str(output / 'fixture'),
               '--vehicle-config', str(config), '--shape', parsed.track,
               '--radius', str(parsed.radius)]
    if parsed.track == 'figure_eight':
        fixture.extend(['--waist-ratio', str(parsed.waist_ratio)])
    run_command(fixture, environment)
    reference_dir = output / 'fixture' / 'reference'
    run_command(['ros2', 'run', 'aims_mpcc', 'prepare_solver', str(reference_dir),
                 '--vehicle-config', str(config)], environment)
    reference = ReferencePath.load(reference_dir)
    log = (output / 'launch.log').open('w')
    launch = subprocess.Popen([
        'ros2', 'launch', 'aims_gazebo_sim', 'closed_loop.launch.py',
        f'path_directory:={reference_dir}', f'vehicle_config:={config}',
        f'log_directory:={output / "controller"}', *launch_initial_pose_arguments(reference),
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
    rclpy.init(args=args)
    observer = AcceptanceObserver(reference)
    outcome = {
        'status': 'FAIL', 'simulation': 'Gazebo Fortress four-wheel contact Ackermann vehicle',
        'track': parsed.track, 'radius_m': parsed.radius,
        'waist_ratio': parsed.waist_ratio if parsed.track == 'figure_eight' else None,
        'hardware_validated': False,
    }
    started = time.monotonic()
    enabled = False
    try:
        while time.monotonic() - started < parsed.timeout:
            rclpy.spin_once(observer, timeout_sec=0.05)
            if launch.poll() is not None:
                raise RuntimeError(f'launch exited with {launch.returncode}')
            if not enabled and observer.values.get('worker_ready') and observer.enable_client.service_is_ready():
                request = SetBool.Request(); request.data = True
                future = observer.enable_client.call_async(request)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and not future.done():
                    rclpy.spin_once(observer, timeout_sec=0.05)
                if not future.done() or not future.result().success:
                    raise RuntimeError('MPCC enable request was rejected or timed out')
                enabled = True
            if enabled and observer.values.get('status') == 'COMPLETE':
                break
        else:
            raise TimeoutError('Gazebo acceptance lap timed out')
        if not observer.samples:
            raise RuntimeError('no physical Gazebo odometry samples received')
        initial = reference.at(0.0)
        final = observer.samples[-1]
        summary = summarize_tracking(observer.samples, (final[2] - initial['x'], final[3] - initial['y']))
        if not meets_acceptance_thresholds(summary, observer.values):
            raise RuntimeError('tracking or deadline acceptance threshold failed')
        outcome['max_observed_speed_mps'] = max(abs(row[4]) for row in observer.samples)
        outcome['status'] = 'PASS'
    except BaseException as error:
        outcome['error'] = repr(error)
        raise
    finally:
        write_artifacts(output, observer, reference, outcome, started)
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGTERM)
            try:
                launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        log.close()
    print(json.dumps(outcome, indent=2, sort_keys=True))
