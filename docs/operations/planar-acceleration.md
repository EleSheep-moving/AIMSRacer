# Rear-axle planar acceleration and G

> 兼容单分量接口说明：当前推荐使用[后轴／质心统一五量节点](state-observer.zh-CN.md)。对应源码保留在 `src/aims_racer_system/`，按当前构建流程生成。


**English** · [简体中文](planar-acceleration.zh-CN.md) · [Quality report](../reports/2026-10-15-planar-acceleration-quality.md)

The optional C++ observer now provides rear-axle ax/ay and dimensionless G for recording and validation. It has no controller consumer, sideslip classifier or friction-boundary fit. A known compensation timing error remains: with the rear point fixed and sinusoidal yaw rate of amplitude 1 rad/s at 3 Hz, synthetic false lateral G has **0.1405g absolute P95**. `y_valid` indicates valid inputs/computation, not verified dynamic accuracy.

## Processing and interfaces

Inputs are corrected three-axis IMU specific force/gyro and LIO orientation only. There are no wheel, current, navigation EKF or independent yaw-observer inputs. The existing [body acceleration](body-acceleration.md) history replay applies late LIO orientation at its source time, then replays gyro, physical reconstruction and filter states to the current IMU source time.

The reference is rear-axle `base_link`, x forward/y left/z up, not the centre of gravity. Shared mounting rotation and approximately `[0.30,0,0.03]` m rear-to-IMU lever are used:

```text
a_rear = R_body_from_imu*f_imu - b_accel + g_body
         - alpha×lever - omega×(omega×lever)
```

The retained causal 25 ms three-axis gyro regression estimates alpha. Separate `[ax,jx]` and `[ay,jy]` means use the same model covariance/gain (`jerk_psd=1e4`, effective force variance `0.01`). This matches final XY filter response, not the response of every upstream compensation term. Accelerometer bias is not estimated online. XY correlation remains unidentified; diagonal covariances are conditional effective model values, not calibrated accuracy.

| Topic | Contract |
| --- | --- |
| `/state_estimation/planar_acceleration` | `AccelWithCovarianceStamped`, linear x/y in m/s², covariance[0]/[7] effective variances |
| `/state_estimation/planar_acceleration/g` | `Vector3Stamped`, x=ax/9.80665 and y=ay/9.80665 |
| `/state_estimation/planar_acceleration/status` | Validity, input ages, rejection/replay counters and callback window timing |
| `/state_estimation/planar_acceleration/debug` | Force, gravity, tangential/centripetal terms and reconstructed/filtered acceleration |

SI/G share the IMU source stamp and `base_link`. G z is unobserved; other SI axes are also unobserved with diagonal variance 1e6. Placeholder zeros are not physical measurements. No TF or control commands are published.

Existing gates reject wrong frames, nonfinite, duplicate, future and out-of-history data. Late IMU/LIO inside retained history can be inserted and replayed; numerical publication remains source-time monotonic within each clock epoch. Maximum source ages are 50 ms IMU and 500 ms LIO; IMU source gaps above 30 ms reset state. Outages suppress numerical output while diagnostics continue. Clock rollback starts a new epoch. Input age and callback P99 over at most 512 samples are not physical response delay or whole-run latency.

The 14 October B recording also exposes a burst-source-time failure: alpha regression spans only about 41–62 microseconds, the tangential term reaches 106.3 m/s² and filtered ay peaks at 92.66 m/s² (9.45g), while computational validity remains true. No minimum regression time-span credibility gate currently exists. These samples are not credible vehicle G or tyre capability; retain original timing and compensation components to investigate the source-time pathology.

## Reported centre-of-mass geometry

On 2026-10-15 the owner reported **wheelbase L=0.365 m** and the internal MID-360 IMU-to-CG displacement **d=[−0.09,−0.03,dz] m**, expressed in ROS vehicle axes x forward/y left/z up. The CG is 9 cm behind and 3 cm right of the IMU. The owner confirms the IMU is above the CG: **dz<0, magnitude unknown**. Measurement uncertainty is unavailable. See the measurement record（recorded workspace path：`src/aims_racer_system/params/vehicle_geometry_measurements.yaml`）; its date is the reporting date, not a claimed measurement date.

The full transfer is `a_CG=a_IMU+alpha×d+omega×(omega×d)`, where a_IMU is mounting/gravity corrected but has not been shifted to the rear axle. Under strictly planar yaw motion, known x/y alone give:

```text
ax_CG = ax_IMU + 0.03*alpha_z + 0.09*r²
ay_CG = ay_IMU - 0.09*alpha_z + 0.03*r²
```

These describe a future transfer, not the current output. Full 3D compensation still needs dz; zero height is not a measured assumption during roll/pitch motion. Current figures/topics remain rear-axle referenced. Do not apply an IMU-to-CG displacement directly to already rear-shifted acceleration. Existing approximate common external Livox geometry does not establish precise rear-to-CG geometry.

Controller/VESC configuration still uses historical wheelbase 0.36 m. Before switching the model, reconcile fitted parameters, generated bundles and actual installation. This update records measurements only, without runtime parameter changes.

## Run and record

Legacy `body_acceleration` remains x-only by default (`lateral_enabled=false`). The new launch opts in using a separate node name, config and topics, and may run alongside the legacy node for comparison. NX has a separate software installation; the main installation is unchanged. With corrected IMU and LIO already running:

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-planar-observer-20261015/install/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system planar_acceleration.launch.py
```

Ctrl-C stops the observer. This launch starts no driving stack. Optional arguments are `config:=/absolute/planar_acceleration.yaml`, `geometry_config:=/absolute/rear_axle_geometry.yaml` and `use_sim_time:=true` for bag playback with `/clock`.

```bash
ros2 topic echo /state_estimation/planar_acceleration/status --once
ros2 topic echo /state_estimation/planar_acceleration/g --once
```

Vehicle/race expose `planar_acceleration:=true`, default false. Keep other driving configuration as usual. `record:=true` records SI/G/status/debug and snapshots parameters. The future Vicon topic must be added under its actual bridge name; see the [Vicon acquisition contract (Chinese)](vicon-observer-validation.zh-CN.md).

Next validation needs clock/extrinsic-calibrated Vicon, particularly rapid reversals and static bias, before changing alpha prediction/alignment. Extra smoothing alone cannot establish compensation accuracy. Lateral velocity is not implemented; in the planar relation `ay=d(vy)/dt+r*vx`, `vx*r` is not independent acceleration truth.

## Current and lateral-acceleration calibration

[Current calibration A/B/C and selected results](calibration.md#current-calibration) · [Lateral AY validation and grip status](lateral-acceleration-calibration.md). Source is unified under `src/calibration/`; use the restored C++ lateral entry. Historical Python commands are provenance only.
