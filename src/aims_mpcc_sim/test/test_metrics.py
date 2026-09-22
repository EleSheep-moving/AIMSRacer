import pytest

from aims_mpcc_sim.metrics import summarize_tracking, meets_acceptance_thresholds


def test_tracking_summary_reports_rms_maximum_and_finish_error():
    summary = summarize_tracking(
        [(0.0, 0.0, 0.0), (0.1, 0.06, 0.0), (0.2, -0.08, 0.0)],
        finish=(0.12, -0.16),
    )

    assert summary['cross_track_rms_m'] == pytest.approx((0.01 / 3.0) ** 0.5)
    assert summary['cross_track_max_m'] == pytest.approx(0.08)
    assert summary['finish_error_m'] == pytest.approx(0.2)


def test_acceptance_thresholds_reject_excessive_error_or_deadline_miss():
    summary = {'cross_track_rms_m': 0.02, 'cross_track_max_m': 0.08, 'finish_error_m': 0.1}

    assert meets_acceptance_thresholds(summary, {'deadline_misses': 0})
    assert not meets_acceptance_thresholds(summary, {'deadline_misses': 1})
    assert not meets_acceptance_thresholds({**summary, 'cross_track_max_m': 0.251}, {'deadline_misses': 0})
