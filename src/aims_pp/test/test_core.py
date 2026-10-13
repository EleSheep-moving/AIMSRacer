import ast
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest


def test_package_is_available():
    from aims_pp.core import EnhancedPP, PPConfig
    assert EnhancedPP(PPConfig()).compute_lookahead(0) == .3


def legacy_controller():
    # Bundled verbatim numeric fixture has no ROS/hardware imports or host paths.
    source = Path(__file__).parent/'fixtures'/'recovered_enhanced_pp.py'
    provenance=json.loads(source.with_name('provenance.json').read_text())
    assert hashlib.sha256(source.read_bytes()).hexdigest()==provenance['fixture_sha256']
    tree = ast.parse(source.read_text())
    helper = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_angle_wrap')
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PurePursuitController')
    assert hashlib.sha256(ast.get_source_segment(source.read_text(),cls).encode()).hexdigest()==provenance['class_sha256']
    scope = {'math': math, 'np': np}
    exec(compile(ast.Module(body=[helper,cls], type_ignores=[]), str(source), 'exec'), scope)
    return scope['PurePursuitController'](max_steering=.45)


def test_recovered_enhanced_pp_numeric_parity():
    from aims_pp.core import EnhancedPP, PPConfig
    old, new = legacy_controller(), EnhancedPP(PPConfig())
    rng = np.random.default_rng(130)
    for v in [0, .3, 1, 3.5, 4, 5, 6, 9, -2]:
        assert new.compute_lookahead(v) == old.compute_lookahead(v)
        for _ in range(12):
            args = (rng.normal(size=2), rng.uniform(-4, 4), rng.normal(size=2), rng.uniform(-4, 4), rng.normal(), v)
            expected, old_debug = old.compute_steering(*args)
            result, debug = new.compute_steering(*args)
            assert result == pytest.approx(expected, abs=1e-13)
            assert debug == pytest.approx(old_debug)


def test_straight_turn_and_nonfinite_rejection():
    from aims_pp.core import EnhancedPP, PPConfig
    pp = EnhancedPP(PPConfig())
    assert pp.compute_steering([0, 0], 0, [1, 0], 0, 0, 1)[0] == 0
    assert pp.compute_steering([0, 0], 0, [1, 1], .5, .4, 1)[0] > 0
    assert pp.steering_limit_for_speed(5) == pytest.approx(.315)
    with pytest.raises(ValueError):
        pp.compute_lookahead(float('nan'))
    with pytest.raises(ValueError):
        pp.compute_steering([0, float('inf')], 0, [1, 1], 0, 0, 1)


def test_vehicle_config_controls_geometry(tmp_path):
    from aims_pp.core import PPConfig
    vehicle = tmp_path / 'vehicle.yaml'
    vehicle.write_text('wheelbase: 0.36\nsteer_limit: 0.45\nrear_offset: 0.1\nhalf_length: null\nfront_extent: 0.52\nrear_extent: 0.1\nhalf_width: 0.16\nmax_speed: 4.0\ngeometry_verified: true\n')
    cfg = PPConfig.from_files(vehicle)
    assert cfg.wheelbase == .36 and cfg.steer_limit == .45 and cfg.rear_offset == .1
    assert cfg.enforce_corridor is False
    vehicle.write_text(vehicle.read_text()+'enforce_corridor: true\n')
    assert PPConfig.from_files(vehicle).enforce_corridor is True
    override = tmp_path / 'pp.yaml'
    override.write_text('wheelbase: 0.9\n')
    with pytest.raises(ValueError, match='vehicle'):
        PPConfig.from_files(vehicle, override)
