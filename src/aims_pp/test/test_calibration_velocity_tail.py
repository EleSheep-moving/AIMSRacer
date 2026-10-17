"""Command speed limits do not become an extra measured-speed cap in calibration."""
import pytest


def test_regular_pp_retains_its_observed_velocity_limit(reference):
    from aims_pp.core import PPConfig
    from aims_pp.tracking import TrackingInputs
    state=TrackingInputs(PPConfig(max_speed=5.),reference)
    with pytest.raises(ValueError):state.observe_odometry(0.,0.,0.,5.6,10_000_000_000,10_000_000_000,10.)


def test_full_step_option_accepts_tail_speed_and_continuous_position(reference):
    from aims_pp.core import PPConfig
    from aims_pp.tracking import TrackingInputs
    state=TrackingInputs(PPConfig(max_speed=5.),reference,allow_speed_overshoot=True)
    state.observe_odometry(0.,0.,0.,8.,10_000_000_000,10_000_000_000,10.)
    state.observe_odometry(1.6,0.,0.,8.,10_200_000_000,10_200_000_000,10.2)
    assert state.pose[3]==8.
    with pytest.raises(ValueError,match='odometry discontinuity'):
        state.observe_odometry(4.,0.,0.,8.,10_210_000_000,10_210_000_000,10.21)


def test_full_step_option_keeps_frame_finite_and_reverse_checks(reference):
    from aims_pp.core import PPConfig
    from aims_pp.tracking import TrackingInputs
    for value,frame in [(float('nan'),'odom'),(8.,'map'),(-1.,'odom')]:
        state=TrackingInputs(PPConfig(max_speed=5.),reference,allow_speed_overshoot=True)
        with pytest.raises(ValueError):
            state.observe_odometry(0.,0.,0.,value,10_000_000_000,10_000_000_000,10.,frame_id=frame)
