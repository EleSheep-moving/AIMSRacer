from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[1]


def test_package_launch_and_required_explicit_currents():
    assert ET.parse(ROOT/'package.xml').getroot().findtext('name')=='aims_calibration'
    values=yaml.safe_load((ROOT/'config/calibration.yaml').read_text())['longitudinal_calibration']['ros__parameters']
    assert values['currents']==[] and values['auto_start'] is False
    assert values['speeds']==[1.,2.,3.]
    launch=(ROOT/'launch/longitudinal_calibration.launch.py').read_text()
    for arg in ('config','reference_directory','vehicle_config','pp_config','log_directory','stage','steering_source','auto_start'):
        assert arg in launch
    assert "executable='longitudinal_calibration'" in launch
    assert "executable='joystick" not in launch
