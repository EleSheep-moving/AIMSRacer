# Real-car deployment: remaining work

This branch is a low-speed MPCC prototype. Docker validates software behavior;
it does not establish vehicle readiness or racing performance. The shipped
`vehicle.yaml` deliberately leaves footprint dimensions unset and
`geometry_verified: false`, so normal activation is blocked until measured.
Current calibration is outside this implementation.

## Current estimator and command contract

| Source | EKF input | Frame / meaning |
|---|---|---|
| FAST-LIO via rear-axle adapter | Existing LIO pose and velocity selection | Pose in `odom`, twist at rear axle in `base_link` |
| `/rear_axle/imu` | Yaw rate only, index 11 | Rear-frame gyro; acceleration and orientation are not fused |
| VESC `/rear_axle/wheel_odom` | Forward velocity only, index 6 | `base_link` vx in m/s; exclude integrated wheel pose and steering-derived yaw rate |

EKF publishes `/odometry/filtered` and owns `odom -> base_link` in V2/V3 driving.
The rear-axle adapter owns that transform in mapping, where EKF is absent.
VESC does not publish TF. The compensated IMU acceleration remains available for
diagnostics; the gravity-removal EKF option is inactive while acceleration
fusion is disabled. Raw `/livox/imu` remains FAST-LIO's input.

MPCC sends speed and steering through `/drive -> joystick_control_v2 ->
/ackermann_cmd -> ackermann_to_vesc -> VESC`. Existing RC selection and command
watchdogs remain in that chain. No additional LIO-loss watchdog is implemented,
as requested. Fresh EKF output alone does not prove that LIO is still updating.

## Prepare the vehicle workspace

The FAST-LIO submodule remains unmodified. Main-repository launch files remap the
upstream FAST-LIO `/tf` output to `/fastlio2/tf`; global driving TF remains owned
by EKF. See the [frame migration guide](../../aims_racer_system/docs/rear-axle-frames.md#upstream-fast-lio-integration).

All external Livox measurements use `livox_frame` with a shared approximate mounting
origin; internal FAST-LIO `r_il/t_il` remain unchanged. The rear-axle adapter uses only
the common external mounting transform, not an additional internal IMU displacement.

Dependency/build and synthetic pipeline checks do not establish scan-matching accuracy
or real-car TF behavior. Use the controller's documented system-Python setup and measure
solve latency on Orin. See the current [frame guide](../../aims_racer_system/docs/rear-axle-frames.md)
for topic names and validation scope.

## Before the first autonomous lap

- **Measure geometry.** Rear-axle `base_link`, wheelbase 0.36 m, and LiDAR forward
  offset approximately 0.30 m are the intended geometry. Sensor height,
  orientation, lateral offset and inherited LIO extrinsics still need physical
  checks. Fill `half_length` with the larger rear-axle-to-front/rear body extent
  and `half_width` with the larger left/right extent. Only then set
  `geometry_verified: true`. Recheck any V3 camera extrinsics from this origin.
- **Verify frames and signs with real data.** Straight motion should produce
  positive body vx. A left turn should produce positive steering and yaw rate;
  negative steering means right. Inspect pose and twist at the rear axle and
  verify exactly one owner per TF edge during driving and mapping.
- **Check wheel-speed scale and uncertainty.** The retained ERPM gain is 4650,
  offset zero. Compare `/rear_axle/wheel_odom` vx with independent ground displacement/time and
  LIO during modest straight runs. Variance 0.04 (m/s)^2 is an assumed sigma of
  0.2 m/s, not a measured calibration. Driven-wheel slip can bias it; a constant
  covariance is not a slip estimator. Check measured telemetry rate/dropouts.
- **Check steering and braking.** Steering limit is ±0.45 rad; configured rate
  limit is 2 rad/s (reported measured capability about 5 rad/s). There is no
  steering-angle sensor; steering lag 0.115 s is an assumption. Measure command
  response, speed-loop response and stopping distance. Zero speed requests do
  not establish an immediate stop or a specific braking force.
- **Measure Orin timing and estimator delay.** Check actual LIO/IMU/VESC source
  stamps, arrival delays, clock consistency, solve p95/p99 and deadline misses.
  Targets are 200 Hz EKF output, 10 Hz solving, 50 Hz commands; these are not
  measured hardware rates. Run `prepare_solver` with the selected path and vehicle
  configuration before startup; online workers require the cached native solver.
  Warm-up must finish before READY. Delayed-LIO
  smoothing/history and prediction settings still need joint validation.
- **Record the reference in the same localization session.** Re-record after
  frame migration. Do not restart localization between recording and driving
  without a separately validated alignment. Measure conservative free corridor
  widths and inspect the prepared path/footprint; a hand-driven lap alone does
  not measure boundaries. Start near the recorded start with matching heading.
- **Check authority and fault response at low speed.** Keep Nav2 and other
  `/drive` publishers stopped. Verify RC takeover, RC loss, stale commands,
  explicit enable/disable and physical stop behavior. Use the measured vehicle
  profile, `simulation:=false`, 0.5 m/s cruise and 1 m/s cap for initial work.
  Record sensor inputs, fused odometry, forwarded commands and MPCC status.

Useful recording command (choose an external/local ignored data directory):

```bash
ros2 bag record -o /data/mpcc-run /livox/imu /rear_axle/imu \
  /fastlio2/lio_odom /rear_axle/lio_odom /rear_axle/wheel_odom /odometry/filtered \
  /sensors/core /sensors/servo_position_command /drive /ackermann_cmd \
  /control/autonomy_speed_enabled /mpcc/status /tf /tf_static
```

## Known work before faster driving

| Missing part | Practical consequence / next work |
|---|---|
| Delayed LIO measurement replay | Prior synthetic 80 ms delay produced substantial position error; VESC vx does not demonstrate this issue is solved. Validate estimator history and CPU cost. |
| Sensor-to-actuation prediction alignment | Solver starts from a measured state and skips ahead on its new plan, without fully propagating actual applied controls to actuation time. |
| Manual-driving shadow evaluation | Shadow requires autonomous RC selection and uses hypothetical steering while active. It is a preview, not validated prediction tracking during manual laps. |
| Identified actuator / dynamic model | Current kinematic model lacks tire-slip dynamics and an identified longitudinal speed-loop response. Validate held-out prediction error before raising limits. |
| Reusable map/session alignment | Recorded `odom` paths have no automatic association/alignment after localization restart. |
| Progress association and track boundaries | Global nearest projection and constant tangent-strip corridors need improvement for nearby track sections or complex boundaries. |

See the [engineering review](ENGINEERING_REVIEW_20260920.md) for evidence and
scope. These are explicitly open deployment/research items, not claims resolved
by passing the integration suite. This branch is not ready for 5 m/s vehicle
operation solely because a synthetic 5 m/s estimator test passes.
