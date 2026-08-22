#!/usr/bin/env python3

import rclpy

from joystick_control_v2 import JoystickControl as BaseJoystickControl


class JoystickControl(BaseJoystickControl):
    """RC profile for steering CH1, throttle CH3, and auxiliary CH5-CH10."""

    CHANNEL_PROFILE = "steering_ch1_throttle_ch3_aux_ch5_to_ch10"
    DEFAULT_CHANNELS = {
        "speed_channel": 3,
        "steering_channel": 1,
        "lock_channel": 5,
        "esc_mode_channel": 6,
        "control_source_channel": 7,
        "calib_mode_channel": 8,
        # CH9 is intentionally unused/reserved in this transmitter profile.
        "limit_channel": 10,
    }


def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    joystick_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
