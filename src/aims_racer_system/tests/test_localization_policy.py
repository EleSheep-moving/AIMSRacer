import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'localization_policy.py'


def load():
    assert PATH.exists(), 'localization policy is missing'
    spec = importlib.util.spec_from_file_location('localization_policy', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_three_distinct_measurements_required_and_bridge_does_not_refresh():
    policy = load().MeasurementHealth()
    for stamp in (1., 1.1):
        policy.observe(stamp, True, 'ok')
        assert policy.state(stamp, True) == 'initializing'
    policy.observe(1.2, True, 'ok')
    assert policy.state(1.3, True) == 'tracking'
    policy.observe(1.2, True, 'ok')  # Duplicate latched status.
    assert policy.state(1.7, True) == 'degraded'
    assert policy.state(3.21, True) == 'lost'


def test_rejection_input_loss_and_rewind_reset_readiness():
    policy = load().MeasurementHealth()
    for stamp in (10., 10.1, 10.2):
        policy.observe(stamp, True, 'ok')
    assert policy.state(10.3, False) == 'lost'
    policy.observe(10.4, False, 'seed_correction_guard_rejected')
    assert policy.state(10.4, True) == 'degraded'
    assert policy.accepts == 0
    policy.observe(1., True, 'ok')
    assert policy.accepts == 1
    assert policy.state(1., True) == 'initializing'


def test_corrections_accumulate_in_window():
    policy = load().MeasurementHealth()
    policy.observe(1., True, 'ok', .2, 2.)
    policy.observe(2., True, 'ok', .3, 3.)
    assert policy.correction_totals(2.) == pytest.approx((.5, 5.))
    assert policy.correction_totals(12.) == pytest.approx((.3, 3.))


def test_source_time_pose_interpolation_refuses_extrapolation():
    import numpy as np
    policy = load()
    history = [(1., np.eye(4)), (2., np.eye(4))]
    history[1][1][0, 3] = 2.
    assert policy.interpolate_pose(history, .9) is None
    assert policy.interpolate_pose(history, 2.1) is None
    assert policy.interpolate_pose(history, 1.5)[0, 3] == pytest.approx(1.)


def test_anchor_metadata_rejects_missing_full_rotation_and_bridge_output():
    policy = load()
    values = {'has_converged': 'true', 'fitness_score': '1.2',
              'correction_rotation_deg': '5.', 'correction_translation_m': '.1', 'correction_yaw_deg': '2.'}
    assert policy.trustworthy_alignment(values, True)
    assert not policy.trustworthy_alignment(values, False)
    values.pop('correction_rotation_deg')
    assert not policy.trustworthy_alignment(values, True)
    values['correction_rotation_deg'] = '11.'
    assert not policy.trustworthy_alignment(values, True)


def test_map_odom_packets_replace_same_stamp_after_future_timer_and_never_interpolate():
    import numpy as np
    policy = load()
    cache = policy.MapOdomPackets()
    old, accepted = np.eye(4), np.eye(4)
    accepted[0, 3] = .1
    cache.add(1000000000, old)
    cache.add(2000000000, old)
    cache.add(1000000000, accepted)
    assert cache.exact(1000000000)[0, 3] == pytest.approx(.1)
    assert cache.exact(1500000000) is None
    assert cache.held(1500000000)[0, 3] == pytest.approx(.1)
    assert cache.held(999999999) is None
    cache.add(8000000000, old)
    assert cache.exact(1000000000) is None
    cache.clear()
    assert cache.held(8000000000) is None
