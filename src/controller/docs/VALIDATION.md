# Docker validation — 2026-09-20

The first section records the original controller-only checkpoint. See the
pre-push section below for the current combined regression and acceptance results.

Image: `aimsracer-mpcc:humble`.
Image ID: `sha256:eb1b802bd857960908e1bc5734f0efb391236cb72c91136025ee5dd122b14f32`.

**62 tests and all 10 ROS integration scenarios passed.** The 27 packaged Python source files match the workspace, verified by SHA-256.

| Scenario | Result | Final controller state | Detail |
|---|---|---|---|
| nominal | PASS | COMPLETE | finish error 2.17 cm; solver p95 50.1 ms; 0 deadline misses |
| odom_drop | PASS | FAULT | fault detected in 97.4 ms; zero-speed converter output verified |
| manual | PASS | FAULT | fault detected in 15.1 ms; zero-speed converter output verified |
| rc_loss | PASS | FAULT | fault detected in 195.6 ms; zero-speed converter output verified |
| clock_reset | PASS | FAULT | fault detected in 10.5 ms; zero-speed converter output verified |
| solver_stall | PASS | FAULT | fault detected in 214.4 ms; zero-speed converter output verified |
| solver_crash | PASS | FAULT | fault detected in 77.5 ms; zero-speed converter output verified |
| disable | PASS | READY | Controlled stop, no fault, zero speed target |
| clockwise_mismatch | PASS | COMPLETE | finish error 0.61 cm; solver p95 51.7 ms; 0 deadline misses |
| delayed_odometry | PASS | COMPLETE | finish error 1.43 cm; solver p95 51.7 ms; 0 deadline misses |

## Evidence and reproduction

The full evidence pack is [`results/acceptance-20260920`](../results/acceptance-20260920/):
`summary.json`, image metadata, exact Python dependencies, source hashes, test output,
per-scenario results, controller JSONL, RC/converter logs, raw and prepared reference,
trajectory CSV, and tracking PNG. Generated result files are intentionally gitignored.

Reproduce from the AIMSRacer root:

```bash
bash src/controller/docker/test.sh
```

The pipeline exits nonzero for a failing assertion. Tests run without hardware devices
or host networking, using the real RC selector and VESC converter. The independent
plant is a radius-2 m bicycle with speed/steering lag. The clockwise test changes
those lags to 0.30/0.22 s; delayed odometry retains original capture timestamps.
Lap completion is independently checked using integrated plant angle, and footprint
clearance uses the exact synthetic annulus rather than the solver tangent constraint.

## Limits

These are CPU software-integration results on this x86 host, not measured Orin
performance or real-car acceptance. The synthetic localization is otherwise noiseless;
there are no tire-force, perception, obstacle, or localization-failure-recovery claims.
Fault tests verify zero-speed requests, not instantaneous physical braking.

Measured rear-axle offset, verified footprint, checked corridor widths, live frame
and steering-sign checks, and an Orin timing check remain prerequisites for driving.
Current calibration is not part of this implementation. The real vehicle configuration
ships with missing geometry and `geometry_verified: false`; synthetic geometry requires
an explicit simulation setting and is rejected in ordinary real-drive mode.

Native CasADi callbacks are compiled before READY in an isolated temporary directory.
The worker initialization deadline is 180 s; the moving solve deadline remains 150 ms.
All three complete-lap scenarios finished with zero moving solve deadline misses.

## Pre-push verification — current prototype

Image `aimsracer-mpcc:rear-frames`: `sha256:8763b273613a3529cfdbbc448e04ef8e9d2d4482ed5777749c57b41a24879297`.

**100 regression tests passed in 67.41 s; all 10 ROS acceptance scenarios passed.**

The full regression suite includes rear-axle transformations, real ROS EKF and VESC conversion, yaw-only/vx-only selection, a synthetic 5 m/s turn, rejection of conflicting TF configs, and consistent standstill velocity-noise handling. Earlier failing tests reproduced the TF and velocity-noise defects before their fixes.

| Scenario | Result | Final state | Detail |
|---|---|---|---|
| nominal | PASS | COMPLETE | finish error 2.15 cm; solver p95 49.3 ms; 0 deadline misses |
| odom_drop | PASS | FAULT | zero-speed output checked; detection 97.4 ms |
| manual | PASS | FAULT | zero-speed output checked; detection 15.0 ms |
| rc_loss | PASS | FAULT | zero-speed output checked; detection 200.0 ms |
| clock_reset | PASS | FAULT | zero-speed output checked; detection 10.5 ms |
| solver_stall | PASS | FAULT | zero-speed output checked; detection 216.2 ms |
| solver_crash | PASS | FAULT | zero-speed output checked; detection 75.4 ms |
| disable | PASS | READY | normal decelerating stop; zero-speed output checked |
| clockwise_mismatch | PASS | COMPLETE | finish error 0.55 cm; solver p95 50.7 ms; 0 deadline misses |
| delayed_odometry | PASS | COMPLETE | finish error 1.52 cm; solver p95 49.1 ms; 0 deadline misses |

The FAST-LIO patch was applied to a clean checkout of its pinned revision and compared byte-for-byte with the reviewed local source. Repeated application was idempotent; conflicting changes were refused without modification. This is patch validation, not compilation or scan-matching validation.

Installed controller/localization Python, RC/VESC source and parsed localization/VESC configuration were compared with the workspace. Independent source review covered localization and controller changes. Staged whitespace, generated-artifact exclusion and credential-pattern checks passed.

Generated logs and per-scenario outputs are local under `src/controller/results/prepush-20260920/` and intentionally gitignored. The table preserves the reviewable results in Git. Reproduce from a fresh checkout (Docker required):

```bash
docker build -f src/controller/docker/Dockerfile -t aimsracer-mpcc:humble .
docker build -f src/aims_racer_system/docker/Dockerfile.frames -t aimsracer-mpcc:rear-frames .
docker run --rm --network none aimsracer-mpcc:rear-frames
bash src/controller/docker/test.sh
```

The 10 acceptance scenarios use an independent synthetic bicycle and synthesized odometry, not FAST-LIO/EKF scan matching. The EKF tests run separately. The delayed-odometry controller scenario does not close the delayed-LIO estimator issue. Physical geometry, wheel-slip/variance tuning, actuator identification, Orin timing and FAST-LIO build/runtime remain open in the [real-car checklist](REAL_CAR_CHECKLIST.md).
