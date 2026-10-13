# VESC vehicle command

**English** · [简体中文](vesc-command.zh-CN.md)

`/vesc/command` uses `vesc_msgs/msg/VescCommand`. The RC selector publishes at
100 Hz; VESC core and IMU polling also run at 100 Hz and 921600 baud. MPCC retains
its 50 Hz `/drive` speed proposal and reads the final command for input history.

| control_mode | Active target | Units / meaning |
| --- | --- | --- |
| 0 MODE_SPEED | target_speed_mps | signed m/s, forward positive |
| 1 MODE_CURRENT | target_current | signed motor current, A |
| 2 MODE_DUTY | target_duty | [-1, 1] |
| 3 MODE_BRAKE_CURRENT | target_brake_current | nonnegative braking magnitude, A |
| 4 MODE_ACCELERATION | target_acceleration_mps2 | reserved signed body-vx derivative; always rejected |

`steering_angle` is radians, left positive, in every mode. Header stamp is the
central publisher's ROS time and frame_id is `base_link`. All fields must be
finite and all inactive targets exactly zero. Reliable KeepLast(1) and a 100 ms
DDS lifespan replace an entire pending command, including its mode and steering.
The driver and MPCC check the header and available DDS source time; expired
samples do not refresh the driver's monotonic watchdog. Invalid messages send
literal zero ERPM and retain steering. Watchdog timeout is 100 ms and also retains
steering. Central RC lock/expiry and source expiry publish zero speed and recenter.
No command is sent on driver startup before a valid command arrives.

Vehicle calibration: ERPM = 3465 * target_speed_mps + 0; servo = -0.5137 *
steering_angle + 0.506, clipped to [0.264, 0.750]. Nonzero speed is bounded by
configured electrical RPM limits [-40000, 40000]. Zero speed and safe stops
always send zero ERPM, regardless of calibration offset or minimum limits.
The driver sends motor and servo packets together under one serial write mutex.
`/sensors/servo_position_command` remains a commanded-position echo for odometry,
not measured steering. Wheel speed covariance remains 0.04 (m/s)^2 by default.

RC retains speed/current/duty switches, limits, direction, locks and priorities.
The vehicle CH10 manual current request range remains **3–100 A**; the driver
`vesc.yaml` clips the final target to **−80 to +80 A**. These bound the request
and serial target separately; a 100 A request is not a measured 100 A current.
`current_startup_enabled=true` retains the 40 A startup helper
(`startup_duty_current=40`, `startup_timeout_ms=100`): requested current below
0.1 A sends zero; otherwise use signed startup current until absolute ERPM > 1
is confirmed for 100 ms using fresh telemetry. Motor current bounds still apply.
Disable the helper explicitly to send the requested current directly. Recorded
`target_current` is the requested target; during startup it can differ from the
serial target. Configuration snapshots preserve the helper settings.

Nonzero braking is disabled by vehicle and standalone defaults (`brake_min=0`,
`brake_max=0`). The protocol path is retained for later use. Acceleration-to-current
control belongs in the VESC-side ROS module in a later implementation, after
identification and limits are established; no proportional placeholder exists.

Navigation remains `/drive` (`AckermannDriveStamped`, jerk=0). MPCC reads only
MODE_SPEED, directly in m/s and radians; a matching recent own proposal within
100 ms and 1e-6 speed/steering tolerance restores acceleration/steering-rate metadata
inside its prediction history. Non-speed commands do not enter speed history and
withdraw active autonomous control through its existing fault flow.

Calibration is `/calib/vesc_command` using the same native message, with fresh
source timestamps, RC arming/rearming and source TTL. Old `/ackermann_cmd`, old
Float64 motor/servo control subscriptions and `ackermann_to_vesc_node` are removed.
There is no `/vesc/applied_command`. Existing bags retain their old message schema;
new bags record the native command, authority, `/drive` and sensor telemetry.

This migration's acceptance uses private DDS domains, synthetic publishers and
PTY VESC emulators only. It opens no vehicle serial port and sends no physical
motor commands. Hardware tests require the owner on site and explicit direction.
