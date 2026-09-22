import math

import pytest

from aims_gazebo_sim.bridge import vesc_to_twist


def test_vesc_to_twist_converts_speed_and_servo_to_ackermann_yaw_rate():
    speed, yaw_rate = vesc_to_twist(
        erpm=4650.0 * 1.2,
        servo=0.506 - 0.5137 * 0.2,
        speed_to_erpm_gain=4650.0,
        steering_gain=-0.5137,
        steering_offset=0.506,
        wheelbase=0.36,
        steer_limit=0.4,
    )

    assert speed == pytest.approx(1.2)
    assert yaw_rate == pytest.approx(1.2 * math.tan(0.2) / 0.36)


def test_vesc_to_twist_clamps_the_physical_steering_limit():
    speed, yaw_rate = vesc_to_twist(
        erpm=4650.0,
        servo=0.506 - 0.5137 * 0.8,
        speed_to_erpm_gain=4650.0,
        steering_gain=-0.5137,
        steering_offset=0.506,
        wheelbase=0.36,
        steer_limit=0.4,
    )

    assert speed == pytest.approx(1.0)
    assert yaw_rate == pytest.approx(math.tan(0.4) / 0.36)
