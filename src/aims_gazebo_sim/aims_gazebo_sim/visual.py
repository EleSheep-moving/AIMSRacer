"""Prepare a reference, then replace this process with the GUI closed loop."""

import argparse
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from aims_mpcc.path import ReferencePath
from aims_mpcc_sim.acceptance import launch_initial_pose_arguments


def run_command(command, environment):
    subprocess.run(command, check=True, env=environment)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', default='/results')
    parser.add_argument('--track', choices=('circle', 'figure_eight'), default='figure_eight')
    parser.add_argument('--radius', type=float, default=6.0)
    parser.add_argument('--waist-ratio', type=float, default=0.3)
    parsed = parser.parse_args(args)
    output = (Path(parsed.output_root) / f'visual-{time.strftime("%Y%m%d-%H%M%S")}').resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ)
    environment.setdefault('AIMS_MPCC_CACHE_DIR', str(output / 'ccache'))
    config = Path(get_package_share_directory('aims_gazebo_sim')) / 'config' / 'gazebo.yaml'
    fixture = ['ros2', 'run', 'aims_mpcc_sim', 'prepare_circle', str(output / 'fixture'),
               '--vehicle-config', str(config), '--shape', parsed.track, '--radius', str(parsed.radius)]
    if parsed.track == 'figure_eight':
        fixture.extend(['--waist-ratio', str(parsed.waist_ratio)])
    run_command(fixture, environment)
    reference_dir = output / 'fixture' / 'reference'
    run_command(['ros2', 'run', 'aims_mpcc', 'prepare_solver', str(reference_dir),
                 '--vehicle-config', str(config)], environment)
    reference = ReferencePath.load(reference_dir)
    command = ['ros2', 'launch', 'aims_gazebo_sim', 'closed_loop.launch.py',
               f'path_directory:={reference_dir}', f'vehicle_config:={config}',
               f'log_directory:={output / "controller"}', 'gazebo_gui:=true',
               'rviz:=true', 'autostart:=true', *launch_initial_pose_arguments(reference)]
    os.execvpe(command[0], command, environment)
