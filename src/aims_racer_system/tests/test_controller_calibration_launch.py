"""Control selection, calibration isolation and experiment recording contracts."""
import importlib.util
from pathlib import Path
import sys

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), ROOT/'launch'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(ROOT)
    module.include = lambda package, filename, arguments=None: {
        'package': package, 'include': filename, 'arguments': arguments or {}}
    module.Node = lambda **kwargs: kwargs
    return module


def context(tmp_path, **overrides):
    map_file = tmp_path/'map.pcd'
    map_file.write_text('map')
    bundle = tmp_path/'bundle'
    bundle.mkdir(exist_ok=True)
    reference = tmp_path/'reference'
    reference.mkdir(exist_ok=True)
    cfg = tmp_path/'calibration.yaml'
    cfg.write_text('test: true\n')
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        map_file=str(map_file), artifact_directory=str(bundle),
        reference_directory=str(reference), vehicle_config='/vehicle.yaml',
        pp_config='', controller='mpcc', speed_limit='1.0',
        calibration_config=str(cfg), stage='speed_hold', steering_source='pp',
        auto_start='false', repeat_laps='false', record='false',
        initial_pose='', session_directory=str(tmp_path/'session'),
        log_directory='', longitudinal_acceleration='true')
    ctx.launch_configurations.update(overrides)
    return ctx


def test_race_starts_exactly_selected_controller(tmp_path):
    module = load('race.launch.py')
    for controller, expected in [('mpcc', 'aims_mpcc_rt'), ('pp', 'aims_pp')]:
        actions = module.build_race(context(tmp_path, controller=controller))
        controllers = [a for a in actions if isinstance(a, dict) and
                       a.get('package') in ('aims_mpcc_rt', 'aims_pp')]
        assert len(controllers) == 1
        assert controllers[0]['package'] == expected


def test_pp_does_not_require_generated_solver_bundle(tmp_path):
    actions = load('race.launch.py').build_race(
        context(tmp_path, controller='pp', artifact_directory=''))
    pp = next(a for a in actions if isinstance(a, dict) and a.get('package') == 'aims_pp')
    assert pp['arguments']['reference_directory'] == str(tmp_path/'reference')


def test_bad_controller_fails_before_recording(tmp_path):
    with pytest.raises(ValueError, match='controller'):
        load('race.launch.py').build_race(context(tmp_path, controller='other', record='true'))
    assert not (tmp_path/'session').exists()


def test_calibration_disables_startup_helper_and_has_one_command_producer(tmp_path):
    module = load('calibration.launch.py')
    actions = module.build_calibration(context(tmp_path))
    includes = [a for a in actions if isinstance(a, dict) and 'include' in a]
    vehicle = next(a for a in includes if a['include'] == 'vehicle.launch.py')
    assert vehicle['arguments']['current_startup_enabled'] == 'false'
    assert vehicle['arguments']['longitudinal_acceleration'] == 'true'
    assert len([a for a in includes if a['package'] == 'aims_calibration']) == 1
    assert not any(a['package'] in ('aims_pp', 'aims_mpcc_rt') for a in includes)


def test_rc_calibration_needs_no_map_or_localizer(tmp_path):
    actions = load('calibration.launch.py').build_calibration(
        context(tmp_path, steering_source='rc', map_file='', reference_directory=''))
    assert not any(isinstance(a, dict) and
                   a.get('include') == 'known_map_localization.launch.py' for a in actions)


def test_vehicle_override_changes_only_driver_startup(tmp_path):
    module = load('vehicle.launch.py')
    ctx = context(tmp_path, mapping='false', vesc_config='/vesc.yaml',
                  lio_config='/lio.yaml', mid360_config='/mid.json',
                  current_startup_enabled='false')
    nodes = [a for a in module.build_vehicle(ctx) if isinstance(a, dict) and 'executable' in a]
    driver = next(a for a in nodes if a['executable'] == 'vesc_driver_node')
    assert driver['parameters'][-1]['current_startup_enabled'] is False
    selector = next(a for a in nodes if a['executable'] == 'joystick_control_v2')
    assert selector['parameters'][-1]['current_limit_max_current'] == 100.0


def test_calibration_recording_keeps_requests_results_and_config(tmp_path, monkeypatch):
    import aims_racer_system.launch_support as support
    ctx = context(tmp_path, record='true')
    actions = support.recording_actions(ctx, ROOT, mode='calibration')
    record = next(a for a in actions if isinstance(a, ExecuteProcess))
    cmd = [perform_substitutions(ctx, item) for item in record.cmd]
    assert {'/calib/vesc_command', '/vesc/command', '/control/calibration_enabled',
            '/calib/status', '/sensors/core', '/state_estimation/longitudinal_acceleration',
            '/localization/status'} <= set(cmd)
    assert (tmp_path/'session/params/calibration_selected.yaml').read_text() == 'test: true\n'


def test_nine_speed_profile_resolves_all_tuning_from_installed_packages(tmp_path):
    module=load('calibration.launch.py')
    module.get_package_share_directory=lambda package: str(ROOT.parent/package)
    ctx=context(tmp_path,profile='speed_hold_1to5',calibration_config='',vehicle_config='',pp_config='')
    actions=module.build_calibration(ctx)
    assert ctx.launch_configurations['calibration_config'].endswith('/aims_calibration/config/speed_hold_1to5.yaml')
    assert ctx.launch_configurations['vehicle_config'].endswith('/aims_calibration/config/speed_hold_vehicle.yaml')
    assert ctx.launch_configurations['pp_config'].endswith('/aims_calibration/config/speed_hold_pp.yaml')
    calibration=next(a for a in actions if isinstance(a,dict) and a.get('package')=='aims_calibration')
    assert calibration['arguments']['require_acceleration']=='true'


def test_nine_speed_profile_cannot_silently_select_current_stage(tmp_path):
    with pytest.raises(ValueError,match='profile'):
        load('calibration.launch.py').build_calibration(context(tmp_path,profile='speed_hold_1to5',stage='accel_interval'))


def test_calibration_rviz_uses_packaged_map_reference_and_software_option(tmp_path):
    module=load('calibration.launch.py')
    actions=module.build_calibration(context(tmp_path,rviz='true',rviz_software_rendering='true'))
    rviz=next(a for a in actions if isinstance(a,dict) and a.get('executable')=='rviz2')
    assert rviz['arguments'][-1].endswith('/rviz/calibration.rviz')
    assert rviz['additional_env']['LIBGL_ALWAYS_SOFTWARE']=='1'
    import yaml
    config=yaml.safe_load((ROOT/'rviz/calibration.rviz').read_text())['Visualization Manager']
    assert config['Global Options']['Fixed Frame']=='map'
    paths=[d for d in config['Displays'] if d['Class']=='rviz_default_plugins/Path']
    assert paths[0]['Topic']['Value']=='/calib/reference'
    assert paths[0]['Topic']['Durability Policy']=='Transient Local'


@pytest.mark.parametrize('profile,stage',[('accel_sweep_1to5','accel_interval'),('decel_sweep_1to5','decel_current')])
def test_current_profiles_resolve_stage_and_require_baseline_before_recording(tmp_path,profile,stage):
    module=load('calibration.launch.py')
    module.get_package_share_directory=lambda package:str(ROOT.parent/package)
    ctx=context(tmp_path,profile=profile,stage='auto',calibration_config='',vehicle_config='',pp_config='',record='true')
    with pytest.raises(ValueError,match='base_current_file'):
        module.build_calibration(ctx)
    assert not (tmp_path/'session').exists()


def test_default_calibration_auto_stage_preserves_a(tmp_path):
    actions=load('calibration.launch.py').build_calibration(context(tmp_path,stage='auto'))
    calibration=next(a for a in actions if isinstance(a,dict) and a.get('package')=='aims_calibration')
    assert calibration['arguments']['stage']=='speed_hold'


@pytest.mark.parametrize('steering,pose',[('pp',''),('pp','1,2,0,0,0,0.5'),('rc','')])
def test_sweep_keeps_initial_pose_separate_from_config(tmp_path,steering,pose):
    import json
    from aims_calibration.planner import export_baseline
    import importlib.util
    fixture=ROOT.parent/'aims_calibration/tests/test_cli.py'
    spec=importlib.util.spec_from_file_location('calib_fixture',fixture)
    source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
    baseline=tmp_path/'baseline.json';baseline.write_text(json.dumps(export_baseline(source.stage_a_payload())))
    module=load('calibration.launch.py')
    module.get_package_share_directory=lambda package:str(ROOT.parent/package)
    actions=module.build_calibration(context(tmp_path,profile='accel_sweep_1to5',stage='auto',
        calibration_config='',vehicle_config='',pp_config='',base_current_file=str(baseline),
        steering_source=steering,initial_pose=pose))
    nodes=[a for a in actions if isinstance(a,dict) and a.get('executable')=='relocalize_known_map.py']
    assert len(nodes)==bool(pose)
    if pose:assert nodes[0]['arguments'][3:]==['--x','1.0','--y','2.0','--z','0.0','--roll','0.0','--pitch','0.0','--yaw','0.5']


def test_invalid_sweep_tracking_settings_fail_before_recording(tmp_path):
    import json,yaml
    from aims_calibration.planner import export_baseline
    fixture=ROOT.parent/'aims_calibration/tests/test_cli.py'
    spec=importlib.util.spec_from_file_location('calib_bad_settings_fixture',fixture)
    source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
    baseline=tmp_path/'baseline.json';baseline.write_text(json.dumps(export_baseline(source.stage_a_payload())))
    params=yaml.safe_load((ROOT.parent/'aims_calibration/config/accel_sweep_1to5.yaml').read_text())
    params['longitudinal_calibration']['ros__parameters']['current_min_samples']=0
    config=tmp_path/'bad.yaml';config.write_text(yaml.safe_dump(params))
    module=load('calibration.launch.py');module.get_package_share_directory=lambda package:str(ROOT.parent/package)
    ctx=context(tmp_path,profile='accel_sweep_1to5',stage='auto',calibration_config=str(config),
        vehicle_config='',pp_config='',base_current_file=str(baseline),record='true')
    with pytest.raises(ValueError,match='sample'):
        module.build_calibration(ctx)
    assert not (tmp_path/'session').exists()
