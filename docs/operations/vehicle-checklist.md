# Real-car deployment: remaining work

This branch is a low-speed MPCC prototype. Package tests and stationary checks
do not establish moving-vehicle readiness or racing performance. The shipped
`vehicle.yaml` now contains the supplied 620 mm × 320 mm footprint with the
rear axle 100 mm ahead of the tail. This enables the geometry check but does not
establish moving-vehicle readiness; the physical envelope and track must be checked.
Current calibration is outside this implementation.

## Prerequisites

Complete [installation](../installation.md), [bringup](bringup.md) and the
[MPCC preparation steps](../../src/controller/docs/usage.md). The
[architecture](../architecture.md) defines the current sensor topics, frame
origins, TF ownership and control chain. Fresh EKF output alone does not prove
that LIO is still updating.

## Before the first autonomous lap

- **Check geometry.** Rear-axle `base_link`, wheelbase 0.36 m, and LiDAR forward
  offset approximately 0.30 m are the intended geometry. Sensor height,
  orientation, lateral offset and inherited LIO extrinsics still need physical
  checks. The supplied body model uses front/rear extents 0.52/0.10 m and
  half-width 0.16 m from the rear axle. Check these against protruding hardware
  and recheck any V3 camera extrinsics from this origin.
- **Verify frames and signs with real data.** Straight motion should produce
  positive body vx. A left turn should produce positive steering and yaw rate;
  negative steering means right. Inspect pose and twist at the rear axle and
  verify exactly one owner per TF edge during driving and mapping.
- **Check wheel-speed scale and uncertainty.** The provisional ERPM gain is 3465
  (4650 / the 2026-09-27 bag's straight-line LIO/wheel ratio 1.342), offset zero.
  Compare `/rear_axle/wheel_odom` vx with independent ground displacement/time and
  LIO during modest straight runs. Variance 0.04 (m/s)^2 is an assumed sigma of
  0.2 m/s, not a measured calibration. Driven-wheel slip can bias it; a constant
  covariance is not a slip estimator. Check measured telemetry rate/dropouts.
- **Check steering and braking.** Steering limit is ±0.45 rad; configured command
  rate limit is 2 rad/s. The 2026-09-27 bag suggests roughly 4.6 rad/s median
  low-speed *effective* turn-angle response, inferred from yaw rate and speed,
  not measured steering hardware capability. There is no steering-angle sensor;
  the MPCC's 0.08 s steering time constant is a low-speed lumped response fit,
  not a measured servo constant. Measure command response, speed-loop response
  and stopping distance. Zero speed requests do
  not establish an immediate stop or a specific braking force.
- **Measure Orin timing and estimator delay.** Check actual LIO/IMU/VESC source
  stamps, arrival delays, clock consistency, solve p95/p99 and deadline misses.
  Default targets are 200 Hz EKF output, 5 Hz solving and 50 Hz commands; these are not
  measured hardware rates. Run `prepare_solver` with the selected path and vehicle
  configuration before startup; online workers require the cached native solver.
  Warm-up must finish before READY. Delayed-LIO history replay is enabled after
  an isolated bag replay; check the actual 200 Hz output rate, CPU load and
  source-time pose error with the full vehicle stack running.
- **Tie the reference to its localization frame.** An `odom` reference needs the
  same uninterrupted localization session. A `map` reference needs the matching
  saved PGO map, successful relocalization and a fresh `map -> odom` transform;
  the [known-map procedure](known-map-mpcc.md) is still unverified on the moving
  vehicle. The current model assumes a centered path in a 1.0 m-wide course;
  check the actual minimum free widths and inspect the prepared path/footprint.
  A hand-driven lap alone does not measure boundaries. Start
  near the recorded start with matching heading.
- **Check authority and fault response at low speed.** Keep Nav2 and other
  `/drive` publishers stopped. Verify RC takeover, RC loss, stale commands,
  explicit enable/disable and physical stop behavior. Use the measured vehicle
  profile, `simulation:=false`, 1.0 m/s cruise and 1.5 m/s cap for initial work.
  Record sensor inputs, fused odometry, forwarded commands and MPCC status.

Use the [recording guide](recording.md) for sensor and controller evidence.

## Known work before faster driving

| Missing part | Practical consequence / next work |
|---|---|
| Full-stack delayed LIO validation | A 2 s EKF history substantially reduced yaw distortion in the incident-input replay; validate estimator rate, CPU cost and source-time pose error on the moving vehicle. It does not remove LIO delay. |
| LIO build and cumulative delay | Rebuild the fork containing the corrected rotation Jacobian. The configured five-iteration cap remains unwired (effective default ten). Record raw scans, per-frame stages/iterations, queue depth and system load; the incident's roughly one-second backlog is not yet reproduced. See the [LIO/EKF findings](../reports/2026-10-04-lio-delay.md). |
| Sensor-to-actuation prediction alignment | The bridge propagates the source-time state using actual forwarded command history and forecasts to scheduled takeover. The longitudinal response remains an acceleration-bounded approximation; validate physical response and handover errors on the moving vehicle. |
| Manual-driving MPCC evaluation | Explicitly enabled MPCC calculates in manual mode and uses actual forwarded command history. Moving-lap prediction accuracy still needs validation. |
| Identified actuator / dynamic model | Current kinematic model lacks tire-slip dynamics and an identified longitudinal speed-loop response. Validate held-out prediction error before raising limits. |
| Known-map validation | Map-frame references, map-file identity checks and a relocalization add-on exist, but the full restart/moving-car workflow has not been validated. The upstream check latches after initial ICP success and does not report later ICP quality. |
| Progress association and track boundaries | Global nearest projection and constant tangent-strip corridors need improvement for nearby track sections or complex boundaries. |

See the [engineering review](../reports/2026-09-20-engineering-review.md) for evidence and
scope. These are explicitly open deployment/research items, not claims resolved
by passing the integration suite. This branch is not ready for 5 m/s vehicle
operation solely because a synthetic 5 m/s estimator test passes.
