import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import json
import pytest


def description():
    file=Path(__file__).resolve().parents[1]/'launch'/'mpcc.launch.py'
    spec=importlib.util.spec_from_file_location('runtime_launch',file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_default_keeps_legacy_and_cpp_is_an_exclusive_selection():
    actions=description().entities
    options={a.name:a for a in actions if isinstance(a,DeclareLaunchArgument)}
    assert 'implementation' in options
    context=LaunchContext()
    assert ''.join(s.perform(context) for s in options['implementation'].default_value)=='legacy'
    nodes=[a for a in actions if isinstance(a,Node)]
    assert len(nodes)==2
    for choice in ('legacy','acados_cpp'):
        context.launch_configurations['implementation']=choice
        enabled=[n for n in nodes if n.condition is None or n.condition.evaluate(context)]
        assert len(enabled)==1
    context.launch_configurations['implementation']='unexpected'
    assert not any(n.condition.evaluate(context) for n in nodes)


def test_cpp_defaults_have_distinct_solve_rate_and_standard_control_interface():
    options={a.name:a for a in description().entities if isinstance(a,DeclareLaunchArgument)}
    context=LaunchContext()
    assert 'cpp_solve_frequency' in options
    assert ''.join(s.perform(context) for s in options['cpp_solve_frequency'].default_value)=='20.0'
    assert 'shadow' not in options


def profile(context, key):
    file=Path(__file__).resolve().parents[1]/'launch'/'mpcc.launch.py'
    spec=importlib.util.spec_from_file_location('runtime_profile',file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.RuntimeProfile(key).perform(context)


def test_native_profile_uses_artifact_horizon_and_respects_budget_override(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps(dict(horizon=5,dt=.1)))
    context=LaunchContext()
    context.launch_configurations.update(implementation='acados_cpp',artifact_directory=str(tmp_path),
                                         horizon='',plan_ttl='',solver_timeout='')
    assert float(profile(context,'horizon'))==5
    assert float(profile(context,'plan_ttl'))==pytest.approx(.4)
    assert float(profile(context,'solver_timeout'))==.05
    context.launch_configurations['solver_timeout']='.07'
    assert float(profile(context,'solver_timeout'))==.07
    context.launch_configurations['horizon']='10'
    with pytest.raises(ValueError,match='artifact'):profile(context,'horizon')


def test_legacy_profile_preserves_original_defaults_without_artifact():
    context=LaunchContext()
    context.launch_configurations.update(implementation='legacy',artifact_directory='',
                                         horizon='',plan_ttl='',solver_timeout='')
    assert float(profile(context,'horizon'))==10
    assert float(profile(context,'plan_ttl'))==pytest.approx(.8)
    assert float(profile(context,'solver_timeout'))==.25


def test_native_command_launch_prefix_runs_process_supervisor():
    context=LaunchContext()
    context.launch_configurations['implementation']='acados_cpp'
    nodes=[a for a in description().entities if isinstance(a,Node)]
    native=next(n for n in nodes if n.condition.evaluate(context))
    # Exercise the actual process action's prefix substitutions.
    text=''.join(s.perform(context) for s in native.process_description._Executable__prefix)
    assert 'runtime_supervisor.py' in text and '--status-topic /mpcc/status' in text


def test_launch_passes_resolved_profile_to_native_node(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps(dict(horizon=5,dt=.2)))
    context=LaunchContext()
    context.launch_configurations.update(implementation='acados_cpp',artifact_directory=str(tmp_path),
        horizon='',plan_ttl='',solver_timeout='.07',path_directory='/reference',vehicle_config='/config.yaml',
        odom_topic='/test/odom',log_directory='',simulation='true',cpp_solve_frequency='20.',
        handover_delay='.02')
    native=next(n for n in description().entities if isinstance(n,Node) and n.condition.evaluate(context))
    parameters=evaluate_parameters(context,native._Node__parameters)[0]
    assert parameters['horizon']==5
    assert parameters['plan_ttl']==pytest.approx(.8)
    assert parameters['solver_timeout']==.07
    assert 'shadow' not in parameters
