ackermann_mux
=========

based on twist_mux
Ackermann multiplexer with support for
[ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)
topics and
[std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) locks with priorities.

<!-- See [documentation](http://wiki.ros.org/twist_mux). -->

## RC controller variants

| Executable | Throttle | Steering | Lock | ESC mode | Control source | Limit | Calibration |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `joystick_control_v2.py` | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 |
| `joystick_control_v2_ch3_ch1.py` | CH3 | CH1 | CH5 | CH6 | CH7 | CH10 | CH8 |

The CH3/CH1 profile intentionally leaves CH2, CH4, and CH9 unused. Channel
parameters remain overridable through ROS parameters when a transmitter needs
a custom mapping.
