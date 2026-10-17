# Body longitudinal acceleration from IMU and delayed LIO

> Publication scope (2026-10-18): this records the deployed workspace. This documentation commit does not include the associated new runtime sources. Commands require that deployed environment. For current rear/CG outputs, see [the state guide (Chinese)](state-observer.zh-CN.md). This is the historical standalone branch.


**English** · [简体中文](body-acceleration.zh-CN.md) · Implementation plan（recorded workspace path：`docs/plans/2026-10-15-body-acceleration.md`）

`body_acceleration` is an independent C++ observation node. Version 1 reports
inertial acceleration at the rear `base_link`, projected along the body forward
axis. It subscribes to no wheel speed, navigation EKF or current signal and
publishes no commands or TF. Geometry and accelerometer calibration are initially
unconfirmed. Software validation does not establish dynamic absolute accuracy or
closed-loop vehicle qualification.

## Interfaces

| Topic | Meaning |
| --- | --- |
| `/livox/imu_bias_corrected` | Startup gyro-bias-corrected Livox samples; acceleration in g, gyro in rad/s, `livox_frame` |
| `/rear_axle/lio_odom` | Source-stamped `odom` / `base_link` orientation; velocity fields are unused |
| `/state_estimation/body_acceleration` | `AccelWithCovarianceStamped`; only `accel.accel.linear.x` is valid, m/s² in `base_link`, stamped at the IMU source time |
| `/state_estimation/body_acceleration/status` | 10 Hz validity, source/receipt ages, calibration flags, replay/rejection counters and callback timings |
| `/state_estimation/body_acceleration/debug` | 10 Hz stage quantities: force, gravity, gyro, angular acceleration, lever terms and reconstructed/filtered longitudinal acceleration; source time in `source_stamp_ns` |

Other acceleration components are placeholders, not measurements of zero. Their
covariance diagonals are `1e6`; inspect `x_valid/y_valid/z_valid/angular_valid`.
Reported variance is an uncalibrated model uncertainty, not an accuracy guarantee.
All-zero raw Livox covariance means unknown.

The new body-projected inertial acceleration differs from the older wheel-fused
`/state_estimation/longitudinal_acceleration` speed derivative:

\[
a^B=\dot v^B+\omega^B\times v^B.
\]

For planar motion, `ax = d(vx)/dt - r*vy`. A wheel-speed derivative during a turn
is not independent acceleration truth. The [older observer](longitudinal-acceleration.md)
and its topic remain available with their original semantics.

## Time and compensation

After unit and mounting conversion,

\[
a_{\rm rear}^B=f_{\rm IMU}^B-b_a^B+g^B
-\alpha^B\times\ell-\omega^B\times(\omega^B\times\ell).
\]

The lever points from the rear reference to the IMU. The inherited
`[0.30, 0, 0.03]` m geometry is approximate and is not a center-of-gravity offset.
Angular acceleration uses causal window regression; its noise and response
affect the lever correction. Fixed acceleration bias defaults to zero. One static
pose cannot separately identify acceleration bias, scale and mounting tilt.

LIO supplies historical tilt anchors; all three gyro axes propagate orientation.
A delayed anchor restores and replays subsequent orientation, compensation and
acceleration-filter states. Published history remains unchanged. Current output
does not wait for another LIO scan. History has time/count bounds, and gyro gaps,
stale anchors and invalid inputs have explicit diagnostics.

LIO shares the physical IMU with this observer. Agreement is a consistency check,
not independent accuracy evidence. Planar navigation-EKF roll/pitch is unused.

Live status/debug report `alpha_window_span_s`, `alpha_sample_center_age_s`,
`estimate_lio_anchor_age_s` and `latest_received_lio_source_age_s` separately.
The first pair describes gyro regression sample support, not universal phase
delay. The latter pair distinguishes the historical anchor actually applied at
the current IMU source time from the latest received LIO source time.

## Launch and recording

With the existing corrected-IMU and LIO inputs running:

```bash
ros2 launch aims_racer_system body_acceleration.launch.py
```

This starts only the observer. Optional arguments are
`config:=/absolute/body_acceleration.yaml`,
`geometry_config:=/absolute/rear_axle_geometry.yaml`, and
`use_sim_time:=true` for replay.

The NX validation build is installed separately at
`/home/aims/aimsracer-body-observer-20261015`; it does not replace the main workspace
installation. In a fresh terminal, source the existing workspace and then this
overlay:

```bash
source /home/aims/AIMSRacer/install/setup.bash
source /home/aims/aimsracer-body-observer-20261015/install/local_setup.bash
ros2 launch aims_racer_system body_acceleration.launch.py
```

Vehicle/race launches accept the opt-in arguments below, alongside the existing
required vehicle configuration:

```text
body_acceleration:=true record:=true
```

`body_acceleration` defaults to false. Recording captures the new topics and
configuration. Run only one instance for the output namespace.

```bash
ros2 topic echo /state_estimation/body_acceleration/status --once
ros2 topic echo /state_estimation/body_acceleration --once
```

Invalid estimates are suppressed while diagnostics continue during outages.
Backward ROS time resets history. Source ages (`imu_age_s/lio_age_s`) and time
since reception (`*_receipt_age_s`) are distinct; neither is motion-response lag.

## Calibration and motion capture

Parameters are in `params/body_acceleration.yaml`; mounting geometry is shared
through `params/rear_axle_geometry.yaml`. Force/gyro variance and tilt deviation
are initial model assumptions. `extrinsics_calibrated` and
`accelerometer_calibrated` default to false.

The offline tools（recorded workspace path：`verification/body_acceleration/README.md`） provide read-only
bag extraction, per-axis statistics, spectra, Allan analysis, static calibration
and native replay in arrival order. Long stationary, temperature, fixture and
new field measurements still require physical collection.

The future motion-capture interface retains sample/receive times, clock mapping
and rigid-body extrinsics. Reference acceleration is differentiated offline with
explicit bandwidth/edge handling and never enters the online observer. Validate
synthetic physics, timing faults, recorded data and isolated NX DDS before field
and motion-capture qualification.

## Current and lateral-acceleration calibration

[Current calibration A/B/C and selected offline models](calibration.md) · [Lateral AY validation and grip status](lateral-acceleration-calibration.md). Interval-current fits and lateral tire capability require separate validation; these reports do not install a new feedforward controller or lateral-acceleration limit.
