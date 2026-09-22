import math

from aims_mpcc_sim.fixture import circle_samples


def test_circle_samples_start_at_rear_axle_pose_and_close_one_lap():
    samples = list(circle_samples(radius=2.0, speed=0.5, samples=4))

    assert samples[0] == (0.0, 2.0, 0.0, math.pi / 2.0, 0.5, 'odom', 'base_link')
    assert samples[-1] == (0.2, 2.0, 0.0, math.pi / 2.0 + 2.0 * math.pi, 0.5, 'odom', 'base_link')
