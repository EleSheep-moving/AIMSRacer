import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


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


def test_cpp_defaults_have_distinct_solve_rate_and_shadow_control():
    options={a.name:a for a in description().entities if isinstance(a,DeclareLaunchArgument)}
    context=LaunchContext()
    assert 'cpp_solve_frequency' in options
    assert ''.join(s.perform(context) for s in options['cpp_solve_frequency'].default_value)=='20.0'
    assert ''.join(s.perform(context) for s in options['shadow'].default_value)=='true'
