"""Run after colcon build; verifies fail-closed startup without ROS hardware."""
import os
from pathlib import Path
import subprocess
import pytest


def executable():
    root=Path(__file__).resolve().parents[3]
    node=Path(os.environ.get('AIMS_MPCC_RT_NODE',root/'install/aims_mpcc_rt/lib/aims_mpcc_rt/mpcc_rt_node'))
    assert node.is_file(), 'C++ ROS controller executable has not been built'
    return str(node)


def test_missing_artifact_cannot_start_a_controller():
    result=subprocess.run([executable(),'--ros-args','-p','artifact_directory:='],
                          text=True,capture_output=True,timeout=15)
    assert result.returncode!=0
    assert 'artifact_directory' in result.stdout+result.stderr


def test_nonexistent_artifact_cannot_start_a_controller():
    result=subprocess.run([executable(),'--ros-args','-p','artifact_directory:=/missing/aims-artifact'],
                          text=True,capture_output=True,timeout=15)
    assert result.returncode!=0
    assert 'artifact' in (result.stdout+result.stderr).lower()


@pytest.mark.parametrize('parameter,value', [('solve_frequency','.nan'),('solver_timeout','.nan'),
    ('plan_ttl','.nan'),('handover_delay','.nan'),('horizon','999')])
def test_invalid_runtime_parameter_rejected_before_ros_controller_start(parameter,value):
    bundle=os.environ.get('AIMS_MPCC_PROTOCOL_BUNDLE')
    if not bundle:pytest.skip('set AIMS_MPCC_PROTOCOL_BUNDLE for valid artifact startup checks')
    result=subprocess.run([executable(),'--ros-args','-p','artifact_directory:='+bundle,
        '-p','simulation:=true','-p',parameter+':='+value],text=True,capture_output=True,timeout=15)
    assert result.returncode!=0
    text=result.stdout+result.stderr
    assert 'runtime ready' not in text
    assert 'horizon' in text.lower() if parameter=='horizon' else 'timing' in text.lower() or parameter in text.lower()
