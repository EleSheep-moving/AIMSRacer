# Lateral-acceleration calibration, AY validation and lateral grip

[简体中文](lateral-acceleration-calibration.zh-CN.md) · [Calibration entry](calibration.md) · [Project overview](../../README.md#calibration)

## Current status

| Work | Quantities to establish | Status |
|---|---|---|
| IMU and mounting calibration | Bias, scale, axes and IMU-to-rear/CG extrinsics | Complete six-position and 3D extrinsic calibration pending; the IMU is approximately 3 cm above the CG, with that height difference estimated |
| AY output validation | Rear/CG inertial acceleration bias, dynamic errors, coverage and response | Independent outputs deployed on NX; Vicon acquisition/analysis ready, independent vehicle accuracy pending |
| Lateral grip and slip | Left/right turning capability, slip onset and tire/surface grip | Stepped-speed fixed-radius procedure is historical; its old automatic script is not a current usable tool |

[State deployment evidence (Chinese)](../reports/2026-10-18-state-observer-nx-deployment.zh-CN.md) establishes installation and software outputs. The [historical AY report](../reports/2026-10-15-planar-acceleration-quality.md) records phase and time-support issues in the earlier scalar branch. Neither establishes independent dynamic accuracy. This page adds no automatic lateral driving program.

## Choose the correct AY

- `/state_estimation/rear_axle/acceleration`: `base_link`, `linear.y` is rear-axle AY.
- `/state_estimation/center_of_mass/acceleration`: `center_of_mass`, `linear.y` is CG AY.

Units are m/s²; axes are forward/left/up. These outputs represent inertial acceleration projected into body axes, after unit, gravity/orientation and rigid-point handling. Raw IMU lateral specific force cannot simply be relabeled as CG AY. See the [state guide (Chinese)](state-observer.zh-CN.md) for inputs and filtering.

At a common point in planar motion, `a_y=dot(v_y)+v_x*r`. The `v_x*r` approximation requires sufficiently small lateral-velocity derivative and is not independent truth. A controller `lateral_accel_limit` is a speed-planning setting, not a measured tire limit.

## Existing Vicon validation workflow

1. Fill [motion-capture configuration (Chinese)](mocap-validation.zh-CN.md): connection, clock mapping and marker/body/rear/CG extrinsics. Waveform alignment cannot replace independent synchronization.
2. Record corrected IMU, both state references/status, raw/rear LIO, navigation EKF, wheel speed, final commands and VESC feedback together. Motion uses the existing field procedures; this tool publishes no driving commands.
3. Run `extract`, then `analyze --task state` or `--task all`. Transform marker pose to each target point before differentiating world position twice and projecting acceleration into body axes.
4. Compare 0.08/0.12/0.18 s derivative windows, with 0.12 s primary. Main errors remain unshifted and unbiased; static, straight, turning and S-turn regimes are separated, with missing support and occlusion visible.
5. Separate accuracy, source age, waveform offset and command-to-vehicle response. Centered differentiation is an offline accuracy reference, not a physical-onset detector. No vehicle acceptance thresholds have been imposed.

[Delivery and tests (Chinese)](../reports/2026-10-18-mocap-validation.zh-CN.md) · [Explicitly synthetic demonstration](../reports/assets/2026-10-18-mocap-validation/demo/report.zh-CN.md)

<a id="lateral-grip"></a>
## Lateral-grip calibration: historical entry

The [English](../reports/2026-10-10-calibration-guide-en-archive.md#historical-lateral-grip) and [Chinese](../reports/2026-10-10-calibration-guide-archive.md#historical-lateral-grip) archives preserve fixed-radius left/right stepped-speed designs and historical output definitions.

The 2026-10-18 inspection found no usable `lateral_grip_calib` launch/executable in the current NX main installation. Residual Python bytecode is not a deployed tool. Historical launch commands are not a validated current procedure.

Independent state accuracy, geometry and synchronization remain to be validated. No new `mu_safe` or lateral limit is inferred from raw IMU peaks or `v_x*r`; MPCC planning and constraints are unchanged.

[Current calibration](calibration.md#current-calibration) · [In-repository evidence](../reports/assets/README.zh-CN.md)
