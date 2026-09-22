import math

import pytest

from aims_mpcc_sim.fixture import circle_samples, figure_eight_samples


def test_circle_samples_start_at_rear_axle_pose_and_close_one_lap():
    samples = list(circle_samples(radius=2.0, speed=0.5, samples=4))

    assert samples[0] == (0.0, 2.0, 0.0, math.pi / 2.0, 0.5, 'odom', 'base_link')
    assert samples[-1] == (0.2, 2.0, 0.0, math.pi / 2.0 + 2.0 * math.pi, 0.5, 'odom', 'base_link')


def test_figure_eight_samples_form_a_closed_double_lobe_without_a_crossing():
    samples = list(figure_eight_samples(radius=4.0, waist_ratio=0.5, speed=1.2, samples=8))

    assert samples[0][:5] == pytest.approx((0.0, 6.0, 0.0, math.pi / 2.0, 1.2))
    assert samples[-1][0] == pytest.approx(0.4)
    assert samples[-1][1:5] == pytest.approx((6.0, 0.0, math.pi / 2.0 + 2.0 * math.pi, 1.2))
    assert min(sample[1] for sample in samples) < -5.0
