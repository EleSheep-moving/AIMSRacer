<a id="当前车辆架构"></a>
# Current vehicle architecture

**English** · [简体中文](architecture.zh-CN.md)

The main workflow uses one ROS 2 Humble workspace. `vehicle.launch.py` runs the sensors, local estimation, and chassis; `mapping.launch.py` runs mapping; `race.launch.py` adds known-map NDT and native acados/C++ MPCC to vehicle. [Bringup](operations/bringup.md) explains how to choose an entry point.

<a id="数据与命令链"></a>
## Data and command chain

```mermaid
flowchart LR
    Livox[MID360 lidar / raw IMU] --> LIO[FAST-LIO2]
    Livox --> Bias[gyro bias correction]
    Bias --> RearIMU[rear axle IMU]
    LIO --> RearLIO[rear axle odometry]
    RearIMU --> EKF[EKF]
    RearLIO --> EKF
    Wheel[VESC wheel speed] --> EKF
    LIO --> NDT[NDT known map]
    EKF --> NDT
    NDT --> Health[localization monitor]
    EKF --> MPCC[native MPCC]
    Health --> MPCC
    Bundle[immutable bundle] --> MPCC
    MPCC --> Drive[/drive]
    Drive --> RC[RC selector]
    RC --> Applied[/ackermann_cmd]
    Applied --> VESC[VESC converter / driver]
    Applied --> MPCC
```

MPCC publishes proposals. The RC selector decides whether to forward them and in which mode; the controller uses the actual `/ackermann_cmd` history to predict motion between measurement and execution. Motor/servo commands and measured speed/steering angle are different data. The current control interface uses speed mode; complete longitudinal identification is planned with the future acceleration interface.

<a id="坐标与安装约定"></a>
## Frame and mounting conventions

`base_link` is at the rear axle midpoint, with x forward, y left, and z up. `rear_offset=0`; the default footprint extends 0.52 m forward, 0.10 m rearward, and 0.16 m to either side of this point.

All external Livox lidar, IMU, and LIO body clouds use `livox_frame`. The external mounting transform is defined centrally in [rear_axle_geometry.yaml](../src/aims_racer_system/params/rear_axle_geometry.yaml): translation `[0.30,0,0.03] m`, RPY `[0,0,0] rad`, approximating the current installation. FAST-LIO still uses its own internal lidar/IMU extrinsics; rear axle adaptation does not compose them again. The centimeter-scale difference between the lidar/IMU origins is approximately neglected in the external frames.

Rear axle LIO adaptation transforms pose, velocity, and covariance; velocity includes compensation for the rotational lever arm from the sensor to the rear axle. Rear axle IMU adaptation converts acceleration from g to m/s² and compensates the lever arm, retaining the original source stamp. Gyro bias correction publishes a separate topic; the raw Livox input remains available for LIO and recording.

```text
map
 └─ odom
     └─ base_link
         ├─ livox_frame
         └─ base_footprint
```

| TF edge | race / vehicle | mapping |
| --- | --- | --- |
| `map → odom` | NDT in race; not provided by vehicle alone | PGO |
| `odom → base_link` | EKF | Rear axle LIO adapter |
| `base_link → livox_frame` | Single static mounting TF | Same as at left |
| Raw FAST-LIO TF | Isolated on `/fastlio2/tf` | Same as at left |

VESC does not publish this vehicle TF. PGO and NDT use separate entry points to preserve sole ownership of the map edge. `base_footprint` is an existing zero transform, not a measured ground projection.

## Topics and compensation

The following rates are configured targets or nominal values driven by inputs; field message rates must be established from recordings. A timer rate does not imply a new measurement rate.

| Topic | Source | Content / time / consumers |
| --- | --- | --- |
| `/livox/lidar` | Livox | Configured 10 Hz; raw CustomMsg, scan-start stamp and per-point offsets; LIO, replay |
| `/livox/imu` | Livox | Raw `livox_frame` IMU, recorded input around 200 Hz, acceleration in g; LIO, bias, recording |
| `/livox/imu_bias_corrected` | Gyro bias | Corrected gyro, retaining source time; rear axle adaptation |
| `/imu/gyro_bias/status` | Gyro bias | Calibration status and diagnostics |
| `/fastlio2/lio_odom` | LIO | `odom/livox_frame`, processed scan-end stamp; rear axle adaptation, PGO |
| `/fastlio2/body_cloud` | LIO | Processed scan in `livox_frame`, same stamp as raw LIO; NDT, PGO, display |
| `/fastlio2/tf` | LIO | Private raw TF, excluded from the public TF tree |
| `/fastlio2/visualization/world_cloud` | LIO | Optional raw LIO world display, disabled by default; not used for localization or control |
| `/rear_axle/lio_odom` | Rear axle adaptation | `odom/base_link`, retaining LIO stamp; EKF |
| `/rear_axle/imu` | Rear axle adaptation | `base_link` IMU, m/s², retaining IMU stamp; EKF |
| `/rear_axle/wheel_odom` | VESC | Rear axle wheel vx, nominally around 50 Hz; EKF |
| `/odometry/filtered` | EKF | `odom/base_link`, configured 200 Hz; MPCC, scan-time TF prediction |
| `/localization/ndt_pose` | NDT | Pose output from registration |
| `/localization/odom_bridge_pose` | NDT | Pose bridging continuous local motion and the map anchor |
| `/localization/ndt_status` | NDT | Registration diagnostics |
| `/localization/anchor_status` | NDT | Protocol-v1 trusted anchor, epoch, sequence, and alignment payload |
| `/localization/map_sha256` | Monitor | Saved PCD identity, transient-local |
| `/localization/map_valid` | Monitor | Localization eligibility flag; not an independent accuracy measurement |
| `/localization/status` | Monitor | Ready state, health, freshness, and map identity; native control eligibility checks |
| `/drive` | Native MPCC | 50 Hz proposals, at most 20 Hz optimization requests; RC selects execution |
| `/ackermann_cmd` | RC selector | Selected forwarded command, configured 200 Hz; VESC and MPCC input history |
| `/control/autonomy_speed_enabled` | RC selector | Permission for autonomous speed mode |
| `/commands/motor/speed` | VESC converter | Target ERPM; not measured vehicle speed |
| `/commands/servo/position` | VESC converter | Servo target; not a steering angle measurement |
| `/sensors/core` | VESC driver | Actual ERPM, current, voltage, and other telemetry |
| `/sensors/servo_position_command` | VESC driver | Servo command echo |
| `/mpcc/status` | Native MPCC | Execution phase, auto start, eligibility, worker/takeover/output diagnostics |
| `/mpcc/reference` | Native MPCC | Fixed reference from the bundle; transient-local |
| `/mpcc/prediction` | Native MPCC | Prediction of the accepted candidate, for display |

<a id="ekf-与-lio-时间"></a>
## EKF and LIO timing

Rear axle odometry retains the exact processed LIO scan-end timestamp, even when the message arrives late. Its rotational velocity correction uses gyro measured at that source time: an exact IMU knot or interpolation between bracketing knots (maximum interval 30 ms). It waits for missing upper IMU coverage without predicting LIO pose or replacing its stamp with receipt time.

Rear axle IMU publishes immediately from the latest corrected IMU input, retaining its exact source stamp. It converts g to m/s², rotates by the fixed mounting transform, and removes rotational lever acceleration. It does not subscribe to LIO or estimate orientation; `orientation_covariance[0] = -1`. Acceleration remains gravity-containing specific force. A short gyro-history startup/reset can mark acceleration unavailable, but LIO latency cannot. No additional aligned-IMU topic is published. Offline gravity removal pairs this stream with LIO attitude by source time; the current planar EKF does not fuse acceleration.

The EKF uses rear axle LIO pose/linear velocity, wheel vx, and IMU yaw rate; it does not fuse IMU acceleration/orientation, integrated wheel pose, or yaw rate inferred from steering. `two_d_mode=true`, delayed measurement history is 2 s, and `predict_to_current_time=true` after replay. LIO and IMU data are correlated, not independent observation sources.

FAST-LIO uses a single worker to process scans and complete IMU intervals. The default pending lidar depth is 2; overload retains newer scans while preserving the IMU needed for propagation; raw IMU uses reliable depth 4096. Checks for time reversal, nonfinite values, overflow, and scan-end coverage remain. A fresh EKF header does not imply zero LIO delay; see the [field review](reports/2026-10-10-mpcc-field-review.md) for specific versions and measurements.

## Persistent reference and local control frames

The saved reference is in `map`, bound to the exact PCD hash. Control state and model propagation stay in continuous `odom/base_link`; trusted `map ← odom` alignment is used for reference projection, errors, and optional boundary calculations. The initial value is the **six-dimensional pose of base_link in map**; see the [known-map workflow](operations/known-map-mpcc.md).

NDT's 50 Hz map TF may hold the last trusted correction; it does not mean registration is repeated at 50 Hz. Both the monitor and native controller check epoch, sequence, source and receive ages, map identity, and a consistent trusted alignment payload. Fresh TF alone cannot grant autonomous control.

<a id="native-mpcc-与当前边界"></a>
## Native MPCC and current limits

The model has six physical states and three controls, plus auxiliary history states; it uses rear axle kinematic propagation, first-order steering lag, a periodic quintic reference, and curvature-based speed planning, with generated acados SQP-RTI called from C++. The current bundle uses N15/dt=.1, at most two RTI passes, a 50 ms complete delivery budget, a 20 ms expected takeover lead time, and a raw state TTL of `0.8×N×dt=1.2 s`. Commands publish at 50 Hz; rejecting a new candidate does not refresh the old plan's source epoch.

The default cruise target is 3.5, hard cap 4, acceleration/braking 1, and curvature planning ay=1; `combined_accel_constraint_enabled=false`, `enforce_corridor=false`. Independent input/state, finite-number, localization, and command ownership checks remain. Planning ay is not a runtime hard tire constraint; manually assigned reference widths are not real track boundaries. The [implementation guide](../src/controller/docs/implementation.md) explains costs, execution history, and certification.

The [2026-10-10 field run](reports/2026-10-10-mpcc-field-review.md) had a planned peak of 2.99 and measured peak of approximately 2.89 m/s; its motion window was not a flying lap. These recordings do not demonstrate higher grip capability or performance on other maps.
