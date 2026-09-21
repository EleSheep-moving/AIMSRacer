# Unified Livox frame and rear-axle pipeline (V2/V3)

`base_link` is the rear-axle midpoint at the existing base_link height, with x
forward, y left and z up. It is not the rear edge of the chassis. MPCC uses
`rear_offset: 0` because its odometry already refers to that point.

## External frame and mounting convention

All external Livox data use `livox_frame`: the driver's lidar and raw IMU,
FAST-LIO `lio_odom` and `body_cloud`. We deliberately approximate the centimetre-scale
lidar/IMU separation as zero for external transforms and rear-axle compensation.
`laser`, `livox_imu` and `imu_link` are not published as Livox aliases in the shared Livox pipeline.

`params/rear_axle_geometry.yaml` provides the ONE external mounting transform:
`livox_translation: [0.30, 0, 0.03]` metres and `livox_rpy: [0, 0, 0]` radians,
expressed relative to the rear axle. These replace `lidar_translation/lidar_rpy`.
The forward distance is approximate; height and rotation are inherited values.

FAST-LIO retains `r_il = I` and `t_il = [-0.011, -0.02329, 0.04412]` internally.
Its state remains IMU-origin and its body cloud is transformed into IMU axes;
using the common external name accepts that positional approximation explicitly.
The rear-axle launch does NOT compose or subtract these internal extrinsics again.
This convention assumes aligned lidar/IMU axes as in the current configuration;
a future nontrivial internal rotation requires revisiting the unified-frame assumption.
Online extrinsic estimation remains disabled.

## Topics and compensation

| Topic | Meaning | Message frames |
| --- | --- | --- |
| `/livox/lidar` | Raw lidar | `livox_frame` |
| `/livox/imu` | Raw IMU, acceleration in g | `livox_frame` |
| `/fastlio2/lio_odom` | Raw LIO pose/velocity | `odom / livox_frame` |
| `/fastlio2/body_cloud` | LIO body cloud for PGO/localization | `livox_frame` |
| `/rear_axle/lio_odom` | Rear-axle pose, twist and covariance | `odom / base_link` |
| `/rear_axle/imu` | Rear-axle compensated IMU, acceleration in m/s² | `base_link` |
| `/rear_axle/wheel_odom` | VESC wheel odometry | `odom / base_link` |
| `/odometry/filtered` | Fused rear-axle state for MPCC/Nav2 | `odom / base_link` |

The LIO adapter calculates `T_odom_base = T_odom_livox * inverse(T_base_livox)`.
It rotates linear velocity into base axes and subtracts `omega × r`, where `r`
points from rear axle to the common Livox origin. It preserves LIO timestamps,
rotates/propagates covariance and supplies explicit floors for upstream zeros.
Gyro must be no later than the LIO timestamp and no more than 50 ms old;
otherwise LIO output is dropped. Both adapters reject raw IMU with a different frame.

The IMU adapter rotates gyro/specific force into base axes, scales g to m/s²,
and compensates `f_rear = f_livox - alpha × r - omega × (omega × r)`.
Gravity stays in specific force. Angular acceleration uses a causal 25 ms window,
at least three samples, and resets on a gap over 30 ms or a non-increasing stamp.
Recent LIO attitude (at most 250 ms old) is propagated with gyro to the IMU stamp.
Until derivative/attitude are available, it publishes gyro only and marks orientation
and acceleration unavailable via covariance[0] = -1. Covariance includes explicit
noise/model floors; these are assumptions, not measured calibration.

VESC already supplies rear-axle forward speed from ERPM; no sensor lever-arm
translation is applied to it. V2/V3 and mapping remap its `odom` output to
`/rear_axle/wheel_odom`. Gain 4650 and longitudinal variance 0.04 (m/s)² are retained.

## EKF and TF ownership

EKF fuses rear-axle LIO pose/linear velocity, wheel `vx` only, and IMU yaw rate
only. IMU acceleration/orientation and wheel-integrated pose/steering-derived yaw
rate remain excluded. `two_d_mode: true` produces a planar output at a target
200 Hz. Gravity removal is configured but inactive while IMU acceleration is excluded.
The LIO and raw IMU streams remain correlated; this is not independent sensing.

```text
map                         optional: PGO while mapping, localizer with an existing map
 └─ odom
     └─ base_link           driving: EKF; mapping without EKF: LIO rear-axle adapter
         ├─ livox_frame     one static mounting transform
         ├─ base_footprint  existing zero transform, not a measured ground projection
         └─ zed2i_camera_link → camera internals (V3, when measured mounting is enabled)
```

VESC must not broadcast vehicle TF in this pipeline. Upstream FAST-LIO's TF is
remapped to the private `/fastlio2/tf` topic by each host launch file, so it does
not enter the global tree. The frame adapter publishes `odom -> base_link` only
when `publish_odom_tf:=true` (mapping). V2/V3 set it false and let EKF publish
that edge. PGO retains the matching RAW LIO
body-cloud/pose pair; do not replace just its raw odometry input with rear-axle data.
V2 defines a localizer but does not add it to its launch description, so V2/V3
alone do not produce `map -> odom`. MPCC's same-session recorded path uses `odom`.
ZED vehicle mounting and IMU TF remain disabled by default; camera tracking is off.

## Upstream FAST-LIO integration

Do not modify the FAST-LIO submodule. Upstream FAST-LIO currently broadcasts TF
regardless of the compatibility `publish_tf` key in the YAML. Every main-repository
launch file therefore remaps its `/tf` output to `/fastlio2/tf`; this preserves raw
LIO diagnostics while keeping global TF ownership in the main pipeline. Build the
unmodified submodule and the main packages normally after updating either workspace.

## Migration and validation

Old V2 topics `/fastlio2/base_odom`, `/livox/imu_ekf` and wheel `/odom` become the
three `/rear_axle/` topics above. Calibration defaults and current recording examples
use those names. Fused `/odometry/filtered` and MPCC's frame contract are unchanged.
Custom geometry files must use the new `livox_translation/livox_rpy` keys.
Custom adapter parameters use `livox_translation/livox_quaternion`.
Re-record references when changing the sensor-origin convention or localization session.

Legacy bringup/mapping/localizer entrypoints now include the same rear-axle adapters,
mounting TF and topic contract. Their original choices of hardware/control/localizer
nodes remain unchanged (some base entrypoints still expect external LIO).
The compatibility filenames fastlio.yaml, ekf.yaml and pgo.yaml also use this contract.
The old standalone livox_imu_to_ekf.py helper is not launched by any entrypoint; use
the shared rear_axle_frames launch for compensated data. V3 inherits V2.
Existing footprint dimensions remain assumptions and must be measured separately.
No new LIO-loss watchdog or EKF fusion-mask change is introduced.

Tests in `src/aims_racer_system/tests` exercise the current convention. The pipeline
integration tests start ONLY frame adapters, EKF and the VESC converter, feed synthetic
measurements, and require an isolated localhost ROS domain; they do not start hardware
or control publishers. Run after building/sourcing the workspace:

```bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m pytest -q src/aims_racer_system/tests
```

Current Orin validation (2026-09-20): `fastlio2` and `aims_racer_system` built
successfully. **7 tests passed in 18.63 s**, including both synthetic ROS modes,
rotated mounting, turn velocity compensation, tangential/centripetal compensation,
and verification of one `odom -> base_link` TF publisher. These tests feed LIO
output directly; they do not exercise FAST-LIO scan matching or vehicle motion.

A powered-vehicle stationary check also passed the frame/TF/position/velocity
contract; see [the hardware report](v2-frame-hardware-check-20260921.md) for measured
rates, remaining gyro bias and LIO latency, and the localhost DDS configuration.

The following historical results describe earlier frame conventions and are retained
as history, not evidence for the current changes.

## Historical validation — initial frame migration (2026-09-20)

Image `aimsracer-mpcc:rear-frames`, ID `sha256:846fde410d0138d9d3c9243553eaba841ebedc4476abdf7caf8c533d5024a77a`: **74 tests passed in 23.86 s** (62 existing MPCC tests and 12 frame/pipeline tests). Both installed V2 launch descriptions and the shared frame launch passed argument expansion; MPCC config loading verified rear_offset=0, steering limit=0.45 and rate=2.0. At that historical validation checkpoint, FAST-LIO was unchanged at f516daac08bc46e50e814a2e7d6c8352ed8141bb; the current local patch described above was added later.

Build/test logs and image ID: `src/controller/results/rear-frames-20260920/` (generated, gitignored). Full ten-scenario MPCC acceptance from the previous image was not repeated for this localization migration. Hardware data and FAST-LIO scan matching were not exercised.

## Historical validation — yaw-rate-only baseline (superseded)

At that checkpoint, the rear-axle adapter and matching IMU TF were retained, and V2/V3 fused only IMU yaw rate. The current configuration above supersedes that baseline. The raw IMU topic remains unchanged for FAST-LIO. No additional LIO freshness watchdog is added, as explicitly requested. Legacy `ekf.yaml` is unchanged; V2 uses `ekf_rear.yaml`.

The integration regression injects incorrect IMU acceleration while keeping synthetic LIO motion correct. Before the configuration change, estimated forward speed was about 9.67 m/s instead of 1 m/s and the test failed. That earlier yaw-only configuration passed the corrupted-acceleration case. The subsequent acceleration-enabled checkpoint instead tested physically valid compensated acceleration. The current VESC-plus-gyro configuration again excludes IMU acceleration and tests rejection of those unselected fields.

Yaw-only validation: **75 tests passed in 30.17 s**, including both ordinary and corrupted-acceleration EKF cases. Rebuilt image `aimsracer-mpcc:rear-frames`, ID `sha256:a6a7b089d2794a43f8af4a35cc3cb1fd2c017a53c0f869382f4fa844672aff68`. Evidence: `src/controller/results/yaw-only-20260920/`. Installed V2 config was also checked to use the rear-axle odometry topic and only IMU update index 11 (yaw rate). These are Docker software tests, not real-car acceptance.

## Historical validation — compensated IMU acceleration

**86 tests passed in 43.13 s**, including 3D centripetal/tangential corrections, causal differentiation and attitude propagation, covariance checks, invalid timestamps/data, startup/stale-attitude gyro-only behavior, and real-EKF cases for turning, changing yaw rate, straight acceleration and tilted rest. Image `aimsracer-mpcc:rear-frames`: `sha256:22560eb61d10f2aa58697ecf73289d58bdf65b6979916975a8db171fad833eff`. Installed configuration was verified to fuse only indices 11, 12, 13 with EKF gravity removal enabled. Evidence: `src/controller/results/rear-imu-20260920/`. These are synthetic software tests, not measured hardware timing or sensor calibration.

## VESC longitudinal-speed fusion (2026-09-20)

V2/V3 use LIO rear-axle odometry, IMU yaw rate only, and VESC longitudinal speed only. Mapping still uses the rear-axle adapter without EKF. No LIO-loss watchdog has been added. Delayed LIO handling and other controller findings in `controller/docs/ENGINEERING_REVIEW_20260920.md` remain separate outstanding work.

After updating the workspace, rebuild `vesc_ackermann` and `aims_racer_system` and restart bringup to load both the new covariance publication and EKF configuration. The earlier local FAST-LIO TF patch also requires rebuilding `fastlio2` if it has not already been rebuilt.

Validation: **90 tests passed in 58.49 s** in image `sha256:7b625a39a26000e27ac8478cb232dfc8f428ca3a8f7a008f2d1cd684e6101d67`. Includes the real VESC converter with two variance settings, rejection of unselected wheel/IMU fields, and the synthetic 5 m/s turn. Evidence: `src/controller/results/vesc-fusion-20260920/`. This validates software behavior with synthetic inputs, not wheel-slip calibration or real-car high-speed operation.
