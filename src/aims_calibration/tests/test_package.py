from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[1]


def test_package_launch_and_required_explicit_currents():
    assert ET.parse(ROOT/'package.xml').getroot().findtext('name')=='aims_calibration'
    values=yaml.safe_load((ROOT/'config/calibration.yaml').read_text())['longitudinal_calibration']['ros__parameters']
    assert values['currents']==[] and values['auto_start'] is False
    assert values['speeds']==[1.,2.,3.]
    assert values['hold_sampling']=='cumulative'
    sweep=yaml.safe_load((ROOT/'config/speed_hold_1to5.yaml').read_text())['longitudinal_calibration']['ros__parameters']
    assert sweep['speeds']==[1.+.5*i for i in range(9)]
    assert sweep['measurement_seconds']==5. and sweep['max_speed']==5.
    assert sweep['hold_sampling']=='cumulative' and sweep['auto_start'] is False and sweep['repeats']==1
    launch=(ROOT/'launch/longitudinal_calibration.launch.py').read_text()
    for arg in ('config','reference_directory','vehicle_config','pp_config','log_directory','stage','steering_source','auto_start'):
        assert arg in launch
    assert "executable='longitudinal_calibration'" in launch
    assert "executable='joystick" not in launch


def test_current_sweep_profiles_and_offline_operator_entries():
    for name,stage in [('accel_sweep_1to5','accel_interval'),('decel_sweep_1to5','decel_current')]:
        values=yaml.safe_load((ROOT/'config'/f'{name}.yaml').read_text())['longitudinal_calibration']['ros__parameters']
        assert values['stage']==stage and values['plan_mode']=='sweep'
        assert values['max_speed']==5. and values['max_current']==100.
        assert values['current_step']==5. and values['trial_timeout']==8.
        assert values['stable_seconds']==1. and values['stable_tolerance']==.2
        assert values['auto_start'] is False and values['repeats']==1
    setup=(ROOT/'setup.py').read_text()
    assert 'baseline = aims_calibration.cli:baseline_main' in setup
    assert 'plan = aims_calibration.cli:plan_main' in setup
