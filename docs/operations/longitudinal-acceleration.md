# Longitudinal acceleration observation

**English** · [简体中文](longitudinal-acceleration.zh-CN.md)

The selected wheel + IMU observer supplies an independent longitudinal acceleration estimate for recording and downstream reference. Enable it explicitly for field evaluation. It does not alter navigation EKF, TF, MPCC, VESC or driving authority. Acceleration feedback control is not connected.

## Inputs and outputs

| Topic | Field / meaning |
|---|---|
| `/rear_axle/wheel_odom` | Input body longitudinal speed, `twist.twist.linear.x`; child frame must be `base_link` |
| `/rear_axle/imu` | Input rear-axle specific force, `linear_acceleration.x`, already in m/s² and `base_link`; unavailable covariance is respected |
| `/state_estimation/longitudinal_acceleration` | `geometry_msgs/AccelWithCovarianceStamped`; `accel.accel.linear.x` estimates `d(body_vx)/dt`, m/s² |
| `/state_estimation/longitudinal_acceleration/status` | `DiagnosticArray`: initialization, source ages, force validity, counters and callback timings |

Specific force is **f = a − g**, with all vectors expressed in the same frame. In a z-up world, `g_world = [0,0,−9.80665] m/s²`. Resting z-up accelerometers read approximately +9.81 m/s²; restoring inertial acceleration requires **a = f + g**. The rear adapter converts units/mount/lever only. This observer estimates `[v,a,d]` from `v_wheel≈v` and `f_x≈a+d`; the disturbance includes `−g_body,x`, accelerometer bias and `(ω×v)_x`. It does not reconstruct attitude. Its scalar body-speed derivative differs from full inertial vector acceleration during rotation.

Other output mean components are zero and their covariance diagonals are 1e6 because they are unobserved. Covariance[0] is the model acceleration variance, not field-calibrated uncertainty. Do not treat this message as a fully measured six-axis acceleration vector or automatically feed it into navigation EKF.

## Start

For the existing race command, add:

```bash
longitudinal_acceleration:=true record:=true
```

Complete example; use the map and bundle matching the current field:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  longitudinal_acceleration:=true record:=true
```

Both vehicle/race default this flag to false until field validation. Existing auto-start remains false. The acceleration observer is observational; use existing [enable/stop procedures](known-map-mpcc.md) for ordinary driving. `record=true` records both new topics plus the original wheel/IMU/LIO/EKF/control inputs, and snapshots `longitudinal_acceleration.yaml` with the other parameters.

If the vehicle graph is already running without the observer, start only the observer in another sourced terminal:

```bash
ros2 launch aims_racer_system longitudinal_acceleration.launch.py
```

Use one observer instance: standalone or integrated. The standalone launch starts no sensors, motor drivers, EKF, TF or controller. For ROS bag playback add `use_sim_time:=true` and provide `/clock` from playback.

## Time and initialization

Source stamps enter the observer as exact integer nanoseconds when callbacks arrive. Wheel callbacks update state; only IMU callbacks trigger acceleration publication. Estimate headers use current ROS estimate time after scalar prediction; they do not modify upstream observation stamps. No periodic fabricated acceleration output is generated while IMU messages are absent.

The first output requires valid wheel and force observations. After initialization, unavailable force is skipped, not fused as its default zero; an IMU heartbeat may still trigger a model/wheel estimate. Source-age warnings are diagnostics, not output-freeze gates. If ROS time jumps backwards, state/history and initialization are reset. Invalid frames, nonfinite data, future stamps and historical retention failures are counted.

The dedicated process serializes input/status callbacks. The two-state/three-state mathematics is shared with the offline comparison. There is no extra output low-pass filter, LIO prediction or LIO-age dependency.

## Inspect and record two laps

```bash
ros2 topic echo /state_estimation/longitudinal_acceleration --once
ros2 topic hz /state_estimation/longitudinal_acceleration
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

Check `initialized=true`, `force_valid`, wheel/IMU/force source ages, numerical errors and callback timing. Approximate output cadence follows the IMU, around 200 Hz on previous recordings, not a guaranteed 5 ms deadline. Publisher header age alone does not reveal sensor age; use status source-age fields.

For field validation, record a stationary interval, then two ordinary laps at low and usual speed, preserving current control behavior. Compare acceleration/deceleration response, stop recovery, source-to-output delay, publication gaps and callback cost under FAST-LIO/EKF/NDT load. Wheel near-zero handling, slope changes, vibration and slip remain relevant; target speed is not measured speed.

## Settings and evidence

Installed settings are in `params/longitudinal_acceleration.yaml`: jerk PSD 30, disturbance PSD 0.1, wheel variance 0.01, force variance 16, history 2 s / 4,096 observations. These are frozen effective tuning values from six training runs and two held-out runs. Gravity/LIO observations are disabled in the runtime B node.

The [offline comparison](../reports/2026-10-12-longitudinal-acc-observer.md) establishes recorded-input behavior; [verification tools](../../verification/acceleration/README.md) reproduce it. Installed software, synthetic DDS replay, vehicle joint-load testing and real field driving are distinct evidence stages. Keep the acceleration output observational until field response and data quality are established.

[Launch guide](bringup.md) · [Recording](recording.md) · [Architecture](../architecture.md)
