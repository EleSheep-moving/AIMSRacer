"""Unit-safe conversion between VESC outputs and Gazebo Ackermann commands."""

import math


def vesc_to_twist(erpm, servo, speed_to_erpm_gain, steering_gain,
                  steering_offset, wheelbase, steer_limit):
    """Return longitudinal speed and yaw rate for Gazebo's Ackermann system."""
    speed = erpm / speed_to_erpm_gain
    steering = (servo - steering_offset) / steering_gain
    steering = max(-steer_limit, min(steer_limit, steering))
    return speed, speed * math.tan(steering) / wheelbase
