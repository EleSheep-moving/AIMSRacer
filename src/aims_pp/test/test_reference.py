import copy
import json
import math

import pytest


def test_numeric_quintic_at_wrap_project_speed(reference):
    at = reference.at(0)
    assert at == pytest.approx(dict(x=2., y=0., yaw=math.pi/2, curvature=.5))
    assert reference.at(reference.length) == pytest.approx(at)
    assert reference.at(-.2) == pytest.approx(reference.at(reference.length-.2))
    s, lateral = reference.project([2.1, 0.])
    assert min(s, reference.length-s) < 1e-6
    assert lateral == pytest.approx(-.1, abs=1e-6)
    assert reference.speed_at(1.5) == 1.25
    assert reference.speed_at(reference.length) == 1.
    assert .5 < reference.speed_at(11.) < 1.


@pytest.mark.parametrize('damage', ['hash', 'geometry', 'frame', 'seam', 'knots', 'speed', 'closed', 'nan'])
def test_loader_rejects_invalid_reference(tmp_path, circle_payload, damage):
    from aims_pp.reference import NumericReference
    from aims_pp.core import PPConfig
    data = copy.deepcopy(circle_payload)
    if damage == 'hash': data['metadata']['map_sha256'] = 'bad'
    if damage == 'geometry': data['metadata']['vehicle_geometry']['wheelbase'] = .4
    if damage == 'frame': data['frame_id'] = 'odom'
    if damage == 'seam': data['coefficients'][-1][0][0] += .1
    if damage == 'knots': data['knots'][1] = data['knots'][0]
    if damage == 'speed': data['speed_profile']['positions'] = [0, 3, 3, 9]
    if damage == 'closed': data['metadata']['closed_lap'] = False
    if damage == 'nan': data['coefficients'][0][0][0] = float('nan')
    (tmp_path/'reference.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        NumericReference.load(tmp_path).validate_config(PPConfig())
