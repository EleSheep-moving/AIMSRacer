import math

import pytest

from aims_mpcc_sim.plant import (
    LaggedBicyclePlant,
    autonomous_channels,
    odometry_fields,
)


def test_plant_converts_vesc_outputs_and_integrates_rear_axle_state():
    plant = LaggedBicyclePlant(
        wheelbase=0.36,
        speed_to_erpm_gain=4650.0,
        steering_gain=-0.5137,
        steering_offset=0.506,
        speed_tau=0.2,
        steering_tau=0.15,
    )

    plant.receive_erpm(4650.0)
    plant.receive_servo(0.506 - 0.5137 * 0.2)
    for _ in range(500):
        plant.step(0.002)

    assert plant.speed == pytest.approx(0.993, abs=0.01)
    assert plant.steering == pytest.approx(0.2, abs=0.01)
    assert plant.x > 0.75
    assert plant.y > 0.15
    assert plant.yaw == pytest.approx(0.41, abs=0.03)


def test_autonomous_channels_select_speed_mode_and_mpcc_control_source():
    channels = autonomous_channels()

    assert channels[0] == 992
    assert channels[2] == 992
    assert channels[4] == 1810
    assert channels[5] == 172
    assert channels[6] == 1810
    assert channels[7] == 172
    assert channels[9] == 172


def test_odometry_fields_are_published_at_the_rear_axle_base_link():
    plant = LaggedBicyclePlant(0.36, 4650.0, -0.5137, 0.506, 0.2, 0.15,
                               x=1.2, y=-0.4, yaw=0.3, speed=0.7, steering=0.1)

    odom = odometry_fields(plant)

    assert odom == pytest.approx({
        'x': 1.2,
        'y': -0.4,
        'yaw': 0.3,
        'speed': 0.7,
        'yaw_rate': 0.7 * math.tan(0.1) / 0.36,
    })
