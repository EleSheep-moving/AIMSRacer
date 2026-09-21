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
