"""Synthetic runtime contract tests; no measurement-fitting dependencies."""
import json
import math
import random

import pytest


def payload():
    return {
        'schema_version': 1, 'kind': 'wheel_current_model',
        'current_signal': 'avg_iq', 'outcome': 'mean_wheel_acceleration',
        'threshold_iq_a': 50.,
        'baseline': {'coefficients_ascending': [10., 2., 0., 0.],
                     'speed_domain_mps': [1., 5.]},
        'rows': [
            {'speed_mps': 1., 'u': [-1., 0., .5, 1.],
             'acceleration_mps2': [-4., 0., 1., 4.],
             'support_intervals': [[-1., 1.]]},
            {'speed_mps': 5., 'u': [-1., 0., .25, 1.],
             'acceleration_mps2': [-8., 0., 2., 4.],
             'support_intervals': [[-1., 1.]]}],
        'high': {'iq_a': [50., 75., 100.], 'acceleration_mps2': [4., 6., 8.],
                 'support_intervals': [[50., 100.]], 'speed_domain_mps': [1., 5.]},
        'provenance': {'description': 'synthetic'}, 'validation': {'held_out': []}}


_DEFAULT = object()


def model(p=_DEFAULT):
    from aims_calibration.wheel_current_model import WheelCurrentModel
    return WheelCurrentModel.from_dict(payload() if p is _DEFAULT else p)


def test_low_interpolation_uses_same_normalized_current_with_different_knots():
    m = model()
    # At v=3, Iq0=16 and u=.5. The two rows give 1 and 8/3.
    result = m.predict_acceleration(33., 3.)
    assert result.valid and result.reason == 'ok' and result.segment == 'low'
    assert result.value == pytest.approx(11/6)
    assert m.predict_acceleration(16., 3.).value == 0.
    assert m.predict_acceleration(0., 3.).value < 0.
    assert m.predict_acceleration(-10., 3.).value < 0.
    assert m.predict_acceleration(10., 3.).value < 0.


def test_high_pool_and_boundary_continuity():
    m = model()
    assert m.predict_acceleration(75., 1.).value == 6.
    assert m.predict_acceleration(75., 5.).value == 6.
    for speed in (1., 2.2, 5.):
        assert m.predict_acceleration(50., speed).segment == 'high'
        assert m.predict_acceleration(50.-1e-8, speed).value == pytest.approx(4.)
        assert m.predict_acceleration(50.+1e-8, speed).value == pytest.approx(4.)


def test_inverse_roundtrip_and_zero_baseline():
    m = model()
    rng = random.Random(21)
    for _ in range(100):
        speed = rng.uniform(1., 5.)
        baseline = 10.+2.*speed
        iq = rng.uniform(2.*baseline-50., 100.)
        forward = m.predict_acceleration(iq, speed)
        inverse = m.current_for_acceleration(forward.value, speed)
        assert inverse.valid
        assert inverse.value == pytest.approx(iq)
    zero = m.current_for_acceleration(0., 3.)
    assert zero.value == 16. and zero.segment == 'baseline'


def test_support_gaps_are_not_bridged_and_rows_intersect_at_same_u():
    p = payload()
    p['rows'][0]['support_intervals'] = [[-1., -.5], [.2, .6], [.8, 1.]]
    p['rows'][1]['support_intervals'] = [[-.7, -.2], [.4, .9]]
    m = model(p)
    for u, expected in [(-.6, True), (-.3, False), (.3, False), (.5, True), (.7, False), (.85, True)]:
        assert m.predict_acceleration(16.+34.*u, 3.).valid is expected
    assert m.predict_acceleration(12.+38.*.3, 1.).valid
    assert not m.predict_acceleration(16.+34.*.3, 3.).valid
    # u=.7 target falls in an unsupported gap even though its acceleration
    # lies inside the global supported acceleration range.
    full = model()
    target = full.predict_acceleration(16.+34.*.7, 3.).value
    assert not m.current_for_acceleration(target, 3.).valid
    assert m.predict_acceleration(16., 3.).value == 0.


def test_inverse_handles_plateaus_and_high_support_gaps():
    p = payload()
    for row in p['rows']:
        row['u'] = [-1., -.5, 0., .25, .5, .75, 1.]
        row['acceleration_mps2'] = [-2., -2., 0., 1., 1., 3., 4.]
    p['high']['acceleration_mps2'] = [4., 4., 8.]
    m = model(p)
    assert m.current_for_acceleration(-2., 3.).value == pytest.approx(-1.)
    assert m.current_for_acceleration(1., 3.).value == pytest.approx(24.5)
    assert m.current_for_acceleration(4., 3.).value == 50.
    p['high']['support_intervals'] = [[50., 60.], [80., 100.]]
    m = model(p)
    assert not m.predict_acceleration(70., 3.).valid
    assert not m.current_for_acceleration(4.5, 3.).valid
    assert m.current_for_acceleration(6., 3.).value == pytest.approx(87.5)


def test_baseline_and_high_are_available_without_low_row_coverage():
    p = payload()
    p['rows'][0]['speed_mps'] = 2.
    p['rows'][1]['speed_mps'] = 4.
    p['high']['speed_domain_mps'] = [1., 3.]
    m = model(p)
    assert not m.predict_acceleration(20., 1.).valid
    assert m.current_for_acceleration(0., 1.).value == 12.
    assert m.predict_acceleration(12., 1.).value == 0.
    assert m.predict_acceleration(75., 1.).value == 6.
    assert m.current_for_acceleration(6., 1.).value == 75.
    assert not m.predict_acceleration(75., 4.).valid
    assert not m.current_for_acceleration(6., 4.).valid
    assert not m.current_for_acceleration(4., 4.).valid


def test_empty_row_support_retains_only_baseline_anchor():
    p = payload()
    for row in p['rows']:
        row['support_intervals'] = []
    m = model(p)
    assert m.predict_acceleration(16., 3.).value == 0.
    assert not m.predict_acceleration(17., 3.).valid
    assert not m.current_for_acceleration(.01, 3.).valid


def test_affine_support_boundary_currents_remain_supported_without_accepting_outside():
    p = payload()
    # Decimal support edges need not round-trip exactly through affine u/Iq.
    for row in p['rows']:
        row['support_intervals'] = [[-.7, -.2], [.1, .3], [.8, 1.]]
    m = model(p)
    for speed in (1., 2.3, 3., 4.7, 5.):
        baseline = 10.+2.*speed
        for ulo, uhi in p['rows'][0]['support_intervals']:
            low = baseline+(50.-baseline)*ulo
            high = baseline+(50.-baseline)*uhi
            for iq in (low, high):
                result = m.predict_acceleration(iq, speed)
                assert result.valid, (speed, iq, result)
                inverse = m.current_for_acceleration(result.value, speed)
                assert inverse.valid
                assert inverse.value == pytest.approx(iq)
            assert not m.predict_acceleration(math.nextafter(low, -math.inf), speed).valid
            # u=1 is the supported high branch, so only check low branch gaps.
            if uhi != 1.:
                assert not m.predict_acceleration(math.nextafter(high, math.inf), speed).valid


def test_interior_cubic_maximum_is_checked_not_only_endpoints():
    p = payload()
    # Iq0 = 51 - 10*(v-3)^2: domain ends are 11, interior peak is 51.
    p['baseline']['coefficients_ascending'] = [-39., 60., -10., 0.]
    with pytest.raises(ValueError): model(p)


def test_baseline_derivative_scaling_excludes_large_constant_term():
    p = payload()
    domain = [1e163, 2e163]
    p['baseline'] = {
        'coefficients_ascending': [-978.1312916824929, 1.4821969375237394e-160, -5e-324, 0.],
        'speed_domain_mps': domain}
    p['rows'][0]['speed_mps'], p['rows'][1]['speed_mps'] = domain
    p['high']['speed_domain_mps'] = domain
    # Both endpoints evaluate near 10 A, but the interior maximum exceeds
    # 50 A. Scaling the derivative by c0 underflows its quadratic term.
    with pytest.raises(ValueError): model(p)


def test_load_does_not_alias_input_and_accepts_one_speed_row():
    p = payload()
    p['rows'] = p['rows'][:1]
    m = model(p)
    p['rows'][0]['support_intervals'].clear()
    assert m.predict_acceleration(31., 1.).valid
    assert not m.predict_acceleration(31., 2.).valid
    assert m.current_for_acceleration(6., 2.).value == 75.


def test_low_rows_with_different_minimum_knots_are_inverted_on_common_domain():
    p = payload()
    p['rows'][1]['u'][0] = -.5
    p['rows'][1]['acceleration_mps2'][0] = -3.
    p['rows'][1]['support_intervals'] = [[-.5, 1.]]
    m = model(p)
    assert not m.predict_acceleration(16.-34.*.75, 3.).valid
    forward = m.predict_acceleration(16.-34.*.4, 3.)
    assert forward.valid
    assert m.current_for_acceleration(forward.value, 3.).value == pytest.approx(16.-34.*.4)


def test_inverse_rejects_float_collapsed_current_that_does_not_match_target():
    p = payload()
    baseline = 49.99999999999999
    p['baseline']['coefficients_ascending'] = [baseline, 0., 0., 0.]
    for row in p['rows']:
        row['u'] = [0., 1.]
        row['acceleration_mps2'] = [0., 4.]
        row['support_intervals'] = [[0., 1.]]
    m = model(p)
    # No float exists strictly between this baseline and 50 A. A rounded
    # candidate at baseline/50 A is supported but predicts 0/4, not 1/2/3.
    for target in (1., 2., 3.):
        result = m.current_for_acceleration(target, 3.)
        assert not result.valid and result.value is None
        assert result.reason == 'unsupported_acceleration'
    for target in (0., 4., 5., 6.):
        result = m.current_for_acceleration(target, 3.)
        assert result.valid
        assert m.predict_acceleration(result.value, 3.).value == pytest.approx(target)


def test_real_plateau_forward_inverse_roundtrip_preserves_exact_ordinate():
    p = payload()
    p['baseline']['coefficients_ascending'] = [5.77545, 3.51367, -.594009, .0451659]
    plateau = -18.47677459954029
    # Minimal subset of the real speed-2 row that reproduced the failure.
    p['rows'] = [{'speed_mps': 2.,
                  'u': [-2.1640793937056193, -2.120050248955134, 0., 1.],
                  'acceleration_mps2': [plateau, plateau, 0., 4.],
                  'support_intervals': [[-2.1640793937056193, 1.]]}]
    m = model(p)
    forward = m.predict_acceleration(-73.95121296693455, 2.)
    inverse = m.current_for_acceleration(forward.value, 2.)
    assert inverse.valid, (forward, inverse)
    assert forward.value == plateau
    assert m.predict_acceleration(inverse.value, 2.).value == plateau


def test_identical_plateaus_remain_exact_between_speed_rows():
    p = payload()
    plateau = -18.47677459954029
    for row in p['rows']:
        row['u'] = [-2., -1., 0., 1.]
        row['acceleration_mps2'] = [plateau, plateau, 0., 4.]
        row['support_intervals'] = [[-2., 1.]]
    m = model(p)
    for i in range(41):
        speed = 1.+i*.1
        baseline = 10.+2.*speed
        forward = m.predict_acceleration(baseline+(50.-baseline)*-1.7, speed)
        assert forward.value == plateau
        assert m.current_for_acceleration(forward.value, speed).valid


def test_inverse_avoids_intermediate_overflow_for_large_finite_high_curve():
    p = payload()
    p['high']['iq_a'] = [50., 1e200]
    p['high']['acceleration_mps2'] = [4., 1e200]
    p['high']['support_intervals'] = [[50., 1e200]]
    m = model(p)
    forward = m.predict_acceleration(5e199, 3.)
    assert forward.valid and math.isfinite(forward.value)
    inverse = m.current_for_acceleration(forward.value, 3.)
    assert inverse.valid
    assert inverse.value == pytest.approx(5e199)


@pytest.mark.parametrize('bad', [None, True, False, '2', float('nan'), float('inf'), -float('inf')])
def test_nonfinite_and_nonnumeric_queries_return_invalid(bad):
    m = model()
    for call in (m.predict_acceleration, m.current_for_acceleration):
        for result in (call(bad, 3.), call(1., bad)):
            assert not result.valid and result.value is None and result.reason == 'invalid_input'


def test_no_extrapolation_or_clipping():
    m = model()
    for speed in (.9999, 5.0001):
        assert not m.predict_acceleration(50., speed).valid
        assert not m.current_for_acceleration(0., speed).valid
    for iq in (-26.0001, 100.0001):
        assert not m.predict_acceleration(iq, 1.).valid
    for accel in (-4.0001, 8.0001):
        assert not m.current_for_acceleration(accel, 1.).valid


def test_load_and_serialization_are_independent_copies(tmp_path):
    from aims_calibration.wheel_current_model import WheelCurrentModel
    p = payload()
    source = tmp_path/'model.json'
    source.write_text(json.dumps(p))
    m = WheelCurrentModel.load(source)
    assert m.to_dict() == p
    exported = m.to_dict()
    exported['rows'][0]['acceleration_mps2'][0] = -999.
    assert m.predict_acceleration(-26., 1.).value == -4.
    result = m.predict_acceleration(50., 3.)
    assert json.loads(json.dumps(result.to_dict()))['valid'] is True
    source.write_text('{broken')
    with pytest.raises(ValueError):
        WheelCurrentModel.load(source)


@pytest.mark.parametrize('damage', [
    'schema', 'kind', 'signal', 'outcome', 'threshold', 'coefficients', 'baseline_nan',
    'baseline_high', 'baseline_negative', 'baseline_interior_extremum',
    'speed_domain', 'rows_missing', 'rows_order', 'row_speed', 'coordinates',
    'matching', 'monotone', 'zero_anchor', 'high_anchor', 'support', 'support_reversed',
    'high_missing', 'high_single', 'high_coordinate', 'high_monotone', 'high_support',
    'high_speed_domain', 'numeric_bool', 'numeric_nan',
])
def test_loader_rejects_malformed_model(damage):
    p = payload()
    if damage == 'schema': p['schema_version'] = True
    elif damage == 'kind': p['kind'] = 'other'
    elif damage == 'signal': p['current_signal'] = 'current_motor'
    elif damage == 'outcome': p['outcome'] = 'imu_acceleration'
    elif damage == 'threshold': p['threshold_iq_a'] = 49.
    elif damage == 'coefficients': p['baseline']['coefficients_ascending'] = [10., 2.]
    elif damage == 'baseline_nan': p['baseline']['coefficients_ascending'][0] = math.nan
    elif damage == 'baseline_high': p['baseline']['coefficients_ascending'] = [50., 0., 0., 0.]
    elif damage == 'baseline_negative': p['baseline']['coefficients_ascending'] = [-1., 0., 0., 0.]
    # Positive at both domain ends, negative at an interior derivative root.
    elif damage == 'baseline_interior_extremum': p['baseline']['coefficients_ascending'] = [8., -6., 1., 0.]
    elif damage == 'speed_domain': p['baseline']['speed_domain_mps'] = [5., 1.]
    elif damage == 'rows_missing': p['rows'] = []
    elif damage == 'rows_order': p['rows'].reverse()
    elif damage == 'row_speed': p['rows'][0]['speed_mps'] = .5
    elif damage == 'coordinates': p['rows'][0]['u'][1] = -1.
    elif damage == 'matching': p['rows'][0]['acceleration_mps2'].pop()
    elif damage == 'monotone': p['rows'][0]['acceleration_mps2'][2] = -.1
    elif damage == 'zero_anchor': p['rows'][0]['acceleration_mps2'][1] = .1
    elif damage == 'high_anchor': p['rows'][0]['acceleration_mps2'][-1] = 5.
    elif damage == 'support': p['rows'][0]['support_intervals'] = [[-2., 1.]]
    elif damage == 'support_reversed': p['rows'][0]['support_intervals'] = [[.8, .2]]
    elif damage == 'high_missing': p.pop('high')
    elif damage == 'high_single':
        p['high']['iq_a'] = [50.]; p['high']['acceleration_mps2'] = [4.]
    elif damage == 'high_coordinate': p['high']['iq_a'][0] = 49.
    elif damage == 'high_monotone': p['high']['acceleration_mps2'][1] = 3.
    elif damage == 'high_support': p['high']['support_intervals'] = [[50., 101.]]
    elif damage == 'high_speed_domain': p['high']['speed_domain_mps'] = [0., 5.]
    elif damage == 'numeric_bool': p['rows'][0]['u'][0] = False
    elif damage == 'numeric_nan': p['high']['acceleration_mps2'][1] = math.nan
    with pytest.raises(ValueError): model(p)


def test_loader_rejects_nonobjects_and_missing_nested_fields():
    for bad in (None, [], {}, {'schema_version': 1}):
        with pytest.raises(ValueError): model(bad)
    for field in ('baseline', 'rows', 'high'):
        p = payload(); p[field] = None
        with pytest.raises(ValueError): model(p)


def test_runtime_imports_only_standard_library():
    import ast
    import aims_calibration.wheel_current_model as runtime
    from pathlib import Path
    tree = ast.parse(Path(runtime.__file__).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import): imported.update(n.name.split('.')[0] for n in node.names)
        elif isinstance(node, ast.ImportFrom): imported.add(node.module.split('.')[0])
    assert imported <= {'bisect', 'copy', 'dataclasses', 'json', 'math', 'pathlib', 'typing', '__future__'}
