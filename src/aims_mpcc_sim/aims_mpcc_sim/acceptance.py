"""Run an independent-process ROS numerical MPCC acceptance lap."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from aims_mpcc.path import ReferencePath
from .metrics import meets_acceptance_thresholds, summarize_tracking


class AcceptanceObserver(Node):
    def __init__(self, reference):
        super().__init__('aims_mpcc_sim_acceptance_observer')
        self.reference = reference
        self.samples = []
        self.status = None
        self.values = {}
        self.create_subscription(Odometry, '/odometry/filtered', self._odometry, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/mpcc/status', self._status, 10)
        self.enable_client = self.create_client(SetBool, '/mpcc/enable')

    def _odometry(self, message):
        progress, error = self.reference.project([
            message.pose.pose.position.x, message.pose.pose.position.y,
        ])
        self.samples.append((
            time.monotonic(), error, message.pose.pose.position.x,
            message.pose.pose.position.y, message.twist.twist.linear.x, progress,
        ))

    def _status(self, message):
        for item in message.status:
            if item.name != 'aims_mpcc':
                continue
            self.status = item.message
            self.values = {pair.key: json.loads(pair.value) for pair in item.values}


def run_command(command, environment):
    subprocess.run(command, check=True, env=environment)


def launch_initial_pose_arguments(reference):
    start = reference.at(0.0)
    return [
        f'initial_x:={start["x"]}',
        f'initial_y:={start["y"]}',
        f'initial_yaw:={start["yaw"]}',
    ]


def write_artifacts(directory, observer, reference, outcome, started):
    if observer.samples:
        initial = reference.at(0.0)
        final = observer.samples[-1]
        summary = summarize_tracking(observer.samples, (final[2] - initial['x'], final[3] - initial['y']))
        outcome.update(summary)
    outcome['elapsed_s'] = time.monotonic() - started
    outcome['controller'] = observer.values
    with (directory / 'trajectory.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['monotonic_s', 'cross_track_m', 'x_m', 'y_m', 'speed_mps', 'progress_m'])
        writer.writerows(observer.samples)
    (directory / 'summary.json').write_text(json.dumps(outcome, indent=2, sort_keys=True) + '\n')
    if not observer.samples:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = observer.samples
    points = reference.points
    figure, axes = plt.subplots(1, 3, figsize=(12, 3.5))
    axes[0].plot(points[:, 0], points[:, 1], '--', label='reference')
    axes[0].plot([row[2] for row in rows], [row[3] for row in rows], label='vehicle')
    axes[0].set_aspect('equal'); axes[0].legend(); axes[0].set(xlabel='x [m]', ylabel='y [m]')
    t0 = rows[0][0]
    axes[1].plot([row[0] - t0 for row in rows], [row[4] for row in rows])
    axes[1].set(xlabel='time [s]', ylabel='speed [m/s]')
    axes[2].plot([row[0] - t0 for row in rows], [row[1] for row in rows])
    axes[2].set(xlabel='time [s]', ylabel='cross-track [m]')
    figure.tight_layout()
    figure.savefig(directory / 'tracking.png', dpi=150)
    plt.close(figure)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--timeout', type=float, default=70.0)
    parser.add_argument('--track', choices=('circle', 'figure_eight'), default='circle')
    parser.add_argument('--radius', type=float)
    parser.add_argument('--waist-ratio', type=float, default=0.5)
    parsed = parser.parse_args(args)
    output = Path(parsed.output).resolve()
    if output.exists():
        raise FileExistsError(f'refusing to overwrite {output}')
    output.mkdir(parents=True)
    environment = dict(os.environ)
    environment['AIMS_MPCC_CACHE_DIR'] = str(output / 'ccache')
    config = Path(get_package_share_directory('aims_mpcc_sim')) / 'config' / 'numerical.yaml'
    fixture_command = ['ros2', 'run', 'aims_mpcc_sim', 'prepare_circle', str(output / 'fixture'),
                       '--vehicle-config', str(config), '--shape', parsed.track]
    if parsed.radius is not None:
        fixture_command.extend(['--radius', str(parsed.radius)])
    if parsed.track == 'figure_eight':
        fixture_command.extend(['--waist-ratio', str(parsed.waist_ratio)])
    run_command(fixture_command, environment)
    reference_dir = output / 'fixture' / 'reference'
    run_command(['ros2', 'run', 'aims_mpcc', 'prepare_solver', str(reference_dir),
                 '--vehicle-config', str(config)], environment)
    reference = ReferencePath.load(reference_dir)
    log = (output / 'launch.log').open('w')
    command = ['ros2', 'launch', 'aims_mpcc_sim', 'closed_loop.launch.py',
               f'path_directory:={reference_dir}', f'vehicle_config:={config}',
               f'log_directory:={output / "controller"}',
               *launch_initial_pose_arguments(reference)]
    launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True, env=environment)
    rclpy.init(args=args)
    observer = AcceptanceObserver(reference)
    outcome = {'status': 'FAIL', 'simulation': 'independent real-time lagged kinematic bicycle',
               'track': parsed.track, 'radius_m': parsed.radius,
               'waist_ratio': parsed.waist_ratio if parsed.track == 'figure_eight' else None,
               'hardware_validated': False}
    started = time.monotonic()
    enabled = False
    try:
        while time.monotonic() - started < parsed.timeout:
            rclpy.spin_once(observer, timeout_sec=0.05)
            if launch.poll() is not None:
                raise RuntimeError(f'launch exited with {launch.returncode}')
            if not enabled and observer.values.get('worker_ready') and observer.enable_client.service_is_ready():
                request = SetBool.Request(); request.data = True
                response = observer.enable_client.call_async(request)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and not response.done():
                    rclpy.spin_once(observer, timeout_sec=0.05)
                if not response.done() or not response.result().success:
                    raise RuntimeError('MPCC enable request was rejected or timed out')
                enabled = True
            if enabled and observer.values.get('status') == 'COMPLETE':
                break
        else:
            raise TimeoutError('acceptance lap timed out')
        if not observer.samples:
            raise RuntimeError('no odometry samples received')
        initial = reference.at(0.0)
        final = observer.samples[-1]
        summary = summarize_tracking(observer.samples, (final[2] - initial['x'], final[3] - initial['y']))
        if not meets_acceptance_thresholds(summary, observer.values):
            raise RuntimeError('tracking or deadline acceptance threshold failed')
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
