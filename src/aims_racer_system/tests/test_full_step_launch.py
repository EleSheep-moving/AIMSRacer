"""Full-step launch keeps experiment feedback wheel-only and startup disabled."""
import json
import pytest
from test_controller_calibration_launch import load, context, package_source, stage_a_payload


@pytest.mark.parametrize('profile,stage',[('accel_full_step_1to5','accel_interval'),('decel_full_step_5to1','decel_current')])
def test_full_profiles_require_baseline_before_recording(tmp_path,profile,stage):
    module=load('calibration.launch.py')
    module.get_package_share_directory=package_source
    ctx=context(tmp_path,profile=profile,stage='auto',calibration_config='',vehicle_config='',pp_config='',record='true')
    with pytest.raises(ValueError,match='base_current_file'):module.build_calibration(ctx)
    assert ctx.launch_configurations['stage']==stage
    assert not (tmp_path/'session').exists()


def test_full_profile_disables_acceleration_observer_gate_and_keeps_localizer(tmp_path):
    from aims_calibration.planner import export_baseline
    baseline=export_baseline(stage_a_payload())
    a=tmp_path/'A.json';a.write_text(json.dumps(baseline))
    limits=tmp_path/'not-needed-and-does-not-exist.json'
    module=load('calibration.launch.py');module.get_package_share_directory=package_source
    ctx=context(tmp_path,profile='accel_full_step_1to5',stage='auto',calibration_config='',vehicle_config='',pp_config='',
        base_current_file=str(a),response_limits_file=str(limits),longitudinal_acceleration='true')
    actions=module.build_calibration(ctx)
    vehicle=next(a for a in actions if isinstance(a,dict) and a.get('include')=='vehicle.launch.py')
    calib=next(a for a in actions if isinstance(a,dict) and a.get('package')=='aims_calibration')
    assert vehicle['arguments']['longitudinal_acceleration']=='false'
    assert calib['arguments']['require_acceleration']=='false' and calib['arguments']['auto_start']=='false'
    assert not calib['arguments'].get('response_limits_file')
    assert any(isinstance(a,dict) and a.get('include')=='known_map_localization.launch.py' for a in actions)
