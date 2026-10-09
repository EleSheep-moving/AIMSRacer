"""Opt-in synthetic ROS interface checks after building the C++ runtime.

Run inside the runtime container with AIMS_MPCC_PROTOCOL_BUNDLE and
AIMS_MPCC_PROTOCOL_OUTPUT set; each invocation gets a new evidence directory.
"""
import json
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('scenario', ['health', 'odometry', 'ownership', 'shadow', 'freshness'])
def test_isolated_ros_protocol(scenario):
    bundle = os.environ.get('AIMS_MPCC_PROTOCOL_BUNDLE')
    if not bundle:
        pytest.skip('set AIMS_MPCC_PROTOCOL_BUNDLE for opt-in ROS validation')
    script = Path(__file__).resolve().parents[1] / 'tools' / 'protocol_probe.py'
    assert script.is_file(), 'functional ROS protocol probe has not been implemented'
    output = Path(os.environ['AIMS_MPCC_PROTOCOL_OUTPUT']) / scenario
    env = dict(os.environ, ROS_DOMAIN_ID='225', ROS_LOCALHOST_ONLY='1')
    command = ['python3', str(script), '--bundle', bundle, '--output', str(output),
               '--scenario', scenario]
    result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads((output / 'report.json').read_text())
    assert report['overall_pass'] and report['checks']
    assert all(n == 0 for n in report['unprefixed_driving_publishers'].values())
