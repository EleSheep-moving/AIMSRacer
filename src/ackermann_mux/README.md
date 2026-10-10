ackermann_mux
=========

Current production installs the C++ RC selector `joystick_control_v2`. It selects manual or autonomous commands and publishes `/ackermann_cmd`; the unified vehicle launch supplies the field-tested CH3/CH1 transmitter profile. Original f1tenth/twist_mux source attribution is retained in package metadata and license materials.

## RC controller variants

Both profiles use the same C++ executable, `joystick_control_v2`. Select the
profile with the read-only `channel_profile` parameter.

| `channel_profile` | Throttle | Steering | Lock | ESC mode | Control source | Limit | Calibration |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `sequential_ch1_ch2` (default) | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 |
| `steering_ch1_throttle_ch3_aux_ch5_to_ch10` (vehicle launch) | CH3 | CH1 | CH5 | CH6 | CH7 | CH10 | CH8 |

The CH3/CH1 profile intentionally leaves CH2, CH4, and CH9 unused. Channel
parameters remain overridable through ROS parameters when a transmitter needs
a custom mapping.

```bash
ros2 run ackermann_mux joystick_control_v2 --ros-args \
  -p channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10
```

Production installs only `joystick_control_v2`. The old mux and Python utilities are no longer built or installed; the unified vehicle launch owns startup.

## Arbitration and calibration safety

Subscriptions retain only the latest sample using best-effort QoS. At 200 Hz,
the selector publishes `/ackermann_cmd` and `/control/autonomy_speed_enabled`.
Lock and RC validity take priority over calibration, navigation and manual
commands. The autonomy flag describes RC selection independently of `/drive`
availability, so a producer can start without an enable dependency cycle.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `publish_rate_hz` | 200.0 | Wall timer rate; independent of ROS simulation time |
| `rc_timeout_sec` | 0.2 | Maximum time since receiving a valid RC frame |
| `nav_timeout_sec` | 0.2 | Maximum receipt/source age of a navigation command |
| `calib_timeout_sec` | 0.2 | Maximum receipt/source age of a calibration command |
| `require_calibration_stamp` | true | Reject calibration commands without a source timestamp |
| `command_speed_limit` | `speed_limit_max_speed` (12.0) | Absolute external speed bound, m/s |
| `command_current_limit` | 100.0 | Absolute external current bound, A |
| `command_duty_limit` | 0.8 | Absolute duty bound; also the manual knob's maximum |

All receipt ageing uses `std::chrono::steady_clock`. Nonzero source timestamps
are compared against ROS time on receipt; that source age then advances using
steady time. This rejects old/future messages and prevents a paused or reset ROS
clock from extending an already accepted command. Producers must share the
selector's ROS clock. Unstamped legacy navigation commands use receipt age;
unstamped calibration is rejected by default.

Calibration accepts `drive.jerk=0` (speed), `2` (current in `acceleration`) and
`3` (duty in `acceleration`), regardless of the RC ESC switch. Entering calibration,
unlocking or recovering RC clears its cached command; a new command must arrive
after arming, and its source timestamp must not predate that arming. When the
producer disappears, the selector sends zero speed on the
first output cycle after the command expires. It never renews freshness by
restamping its own output. The actual stop latency also includes scheduling,
transport and motor response; this is not a hard real-time guarantee.

`jerk=1` acceleration feedforward is rejected because the vehicle's
`ackermann_to_vesc` has no implementation for it. Navigation supports speed mode
only. Unsupported modes, missing/expired inputs, malformed RC, nonfinite command
values and commands exceeding external speed/current/duty limits all produce an
explicit zero speed command. Steering is clamped to `steering_limit`. A persistent
RC outage keeps the node alive and stopped; manual control can recover when RC
returns. Calibration still requires a new command after that recovery.

The current vehicle, mapping and race launches all use the field-tested manual
current range **3–100 A**, controlled by CH10; CH3 controls the throttle fraction
and the throttle deadzone is 50. The executable's fallback when run directly
without vehicle parameters is 3–20 A; that fallback is not the vehicle launch
configuration. External calibration current has its own bound. Existing
`channel8_*`, `speed_channel8_*`, `current_channel8_*` and `steering_channel_mid`
aliases remain fallbacks; canonical names take precedence. All selector parameters
are validated at startup and read-only thereafter. Logging occurs on state changes.

## Verification

```bash
colcon build --symlink-install --packages-select ackermann_mux
```

Local native tests covered both profiles against 120 recorded Python manual outputs,
timeouts, arming transitions, takeover, unsupported modes and invalid commands.
The ROS pipeline tests start only the selector and synthetic publishers in an
isolated DDS domain, including a paused simulation clock. They do not start the
vehicle receiver or VESC driver.
Regression sources and fixtures are under `tests/` and are used during development;
production installation contains only the selector executable.

On this host (2026-10-03), an isolated synthetic workload with 100 Hz RC,
50 Hz navigation input and 200 Hz output measured 29.0–32.0% of one CPU core for
the original Python selector and 4.3–4.7% for C++ (three 3 s windows after warmup).
Both produced approximately 200 commands/s. These are process CPU measurements,
not a bound on worst-case latency or a vehicle driving test.
