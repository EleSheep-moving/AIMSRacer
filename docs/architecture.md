# Vehicle architecture (V2/V3)

`base_link` is the rear-axle midpoint at the existing base_link height, with x
forward, y left and z up. It is not the rear edge of the chassis. MPCC uses
`rear_offset: 0` because its odometry already refers to that point.

## External frame and mounting convention

All external Livox data use `livox_frame`: the driver's lidar and raw IMU,
FAST-LIO `lio_odom` and `body_cloud`. We deliberately approximate the centimetre-scale
lidar/IMU separation as zero for external transforms and rear-axle compensation.
`laser`, `livox_imu` and `imu_link` are not published as Livox aliases in the shared Livox pipeline.

[`rear_axle_geometry.yaml`](../src/aims_racer_system/params/rear_axle_geometry.yaml) provides the ONE external mounting transform:
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

These contracts describe **V2 + the known-map add-on + MPCC**. An installed
publisher or a listed topic does not guarantee that messages are being produced;
optional outputs and consumer demand are noted explicitly. Runtime ROS graph
inspection remains necessary for a particular launch/session.

Frequencies below are configured targets or input-driven nominal rates, **not**
guaranteed live measurements. `≈200 Hz (measured)` identifies the MID360 IMU
rate observed in the recorded bags. Enabled/subscribed conditions still apply;
disabled outputs produce no messages. A timer rate is not necessarily the rate
of new information: localizer raw TF can repeat an old correction, and MPCC
publishes commands faster than it generates new plans.

| Topic | Publisher | Frequency (Hz) | Meaning / frames | Functional consumers |
| --- | --- | --- | --- | --- |
| `/livox/lidar` | MID360 driver | 10 configured | Raw `CustomMsg`, `livox_frame`; header is scan start, point offsets describe acquisition times | FAST-LIO; bag replay |
| `/livox/imu` | MID360 driver | ≈200 (measured) | Raw IMU, `livox_frame`, acceleration in g; source sample stamp | FAST-LIO and rear-axle adapters |
| `/fastlio2/lio_odom` | FAST-LIO | ≈10; follows processed scans | Raw pose and body-frame linear velocity, `odom / livox_frame`; scan-end stamp | ICP/PGO, both rear-axle adapters, consistency diagnostics |
| `/fastlio2/body_cloud` | FAST-LIO | ≈10; follows processed scans | Current processed scan in `livox_frame`; same scan-end stamp as raw LIO odometry | ICP/PGO, consistency diagnostics; also RViz |
| `/fastlio2/visualization/world_cloud` | FAST-LIO, host remapping | Disabled; ≤5 when enabled | Current scan already placed in `odom` by raw LIO, **not** the accumulated/global map | Visualization only; publication disabled by default; optional maximum 5 Hz / 2000 points |
| `/fastlio2/lio_path` | FAST-LIO | Disabled; ≤1 when enabled | Accumulated raw LIO poses in `odom` | Visualization only; publication disabled by default |
| `/fastlio2/tf` | FAST-LIO, host remapping | ≈10; follows LIO output | Private raw `odom -> livox_frame` TF | Raw LIO diagnostics; **not** the public TF tree |
| `/rear_axle/lio_odom` | `lio_to_rear_axle.py` | ≈10; valid LIO samples only | Compensated rear-axle pose, twist and covariance, `odom / base_link`; original LIO stamp | EKF |
| `/rear_axle/imu` | `imu_to_rear_axle` | ≈200; follows raw IMU | Compensated IMU in `base_link`, acceleration in m/s²; original IMU stamp | EKF (see selected components below) |
| `/rear_axle/wheel_odom` | VESC odometry converter | ≈50; follows VESC telemetry | Wheel speed/odometry, `odom / base_link`; VESC state stamp | EKF |
| `/odometry/filtered` | EKF | 200 configured | Fused rear-axle state, `odom / base_link` | MPCC/Nav2; RViz |
| `/localizer/raw_tf` | Native localizer, add-on remapping | ≤100 timer; correction attempts ≈1; ICP can pause publication | Native `map -> odom` correction with upstream scan epoch | `map_tf_gate`; no direct public-TF ownership |
| `/localizer/map_cloud` | Native localizer | ≈1 after matching, when subscribed | Loaded/refined saved map in `map`; repeated after matching when subscribed | RViz only; not a new live scan and not a heartbeat |
| `/localization/map_valid` | `map_tf_gate` | ≈10 | Verified initialization plus an available map transform; **not** scan quality or guaranteed localization accuracy | Initialization tool/operator; MPCC does not subscribe |
| `/localization/map_sha256` | `map_tf_gate` | Once at startup; transient-local | Latched exact saved-map identity, not a health flag | Initialization tool and MPCC reference/map identity check |
| `/localization/status` | `map_tf_gate` | ≈10; consistency recalculated at most 2 | Input ages and fixed-alignment nearest-neighbor scan/map consistency diagnostics | Operator and bag analysis; never gates TF or MPCC |
| `/tf` | EKF and `map_tf_gate` | EKF 200; map correction holder 50 | EKF owns `odom -> base_link`; gate holds/renews verified `map -> odom` at 50 Hz | Coordinate conversion/RViz; no cloud publication heartbeat required |
| `/tf_static` | Mounting publishers | On publication/discovery; transient-local | `base_link -> livox_frame`, `base_footprint`, optional ZED static mounting | Coordinate conversion/RViz |

**Control and actuator interfaces** have different meanings from physical feedback:

| Topic | Publisher | Frequency (Hz) | Role / consumers |
| --- | --- | --- | --- |
| `/rc/channels` | CRSF receiver | Input-driven, ≤100 polling rate | Actual received RC selections/inputs; RC selector and calibration tools |
| `/rc/link` | CRSF receiver | Event/input-driven, ≤100 polling rate | Radio link telemetry; diagnostic, not navigation state |
| `/drive` | MPCC (or a separately selected Nav2 controller) | 50 with MPCC; new-plan scheduling target 5 | Proposed speed/steering; RC selector decides execution authority. One producer per session |
| `/ackermann_cmd` | RC selector | 200 configured | Selected/forwarded command sent to VESC conversion; MPCC uses this for applied-command history. This is a command, not measured actuator motion |
| `/control/autonomy_speed_enabled` | RC selector | 200 configured | RC permission for unlocked navigation in speed mode; MPCC command routing |
| `/commands/motor/speed` | Ackermann-to-VESC | Follows selected commands, typically ≈200 in speed mode | Target ERPM; VESC driver |
| `/commands/motor/current`, `/commands/motor/duty_cycle` | Ackermann-to-VESC | Follows selected commands, typically ≈200 in its mode | Targets used in the corresponding RC/calibration mode; not MPCC speed-mode feedback |
| `/commands/servo/position` | Ackermann-to-VESC | Follows selected commands, typically ≈200 | Normalized servo target; VESC driver; not actual steering angle |
| `/sensors/core` | VESC driver | ≈50; driver polling target 50 | Actual VESC telemetry, including ERPM/current/voltage; wheel odometry and calibration |
| `/sensors/servo_position_command` | VESC driver | Follows servo commands, typically ≈200 | Echo of the servo command, **not** a physical steering-angle measurement |
| `/sensors/imu`, `/sensors/imu/raw` | VESC driver, hardware-dependent | Hardware-dependent; driver polling target 50 | Optional VESC IMU telemetry/custom and standard forms; not inputs of the current EKF |
| `/mpcc/status` | MPCC | 50 timer target | Controller/worker/plan status; operator diagnostics |
| `/mpcc/reference` | MPCC | Once at startup; transient-local | Reference path in its configured `map` or `odom` frame; visualization, not a live reference-input interface |
| `/mpcc/prediction` | MPCC | On accepted solver replies, nominal ≤5 | Accepted predicted path in `odom`; visualization, not an executed-path measurement |

With the **mapping launch instead of V2**, PGO consumes the same raw
`body_cloud + lio_odom` pair and owns `map -> odom`. `/pgo/loop_markers` visualizes
loop constraints when queued scan/pose pairs are processed (timer ceiling 20 Hz);
`/pgo/save_maps` is a service, not a map-cloud topic. That launch
has no EKF, and the rear-axle LIO adapter owns `odom -> base_link`. Do not run
PGO and the known-map localizer as competing map-TF owners.

Calibration programs separately publish `/calib/ackermann_cmd` (selector input)
and `/calib/current_trajectory`, `/calib/lookahead_point`, `/calib/status_text`
(visualization). Legacy Nav2 launches may add a body-cloud-to-`/scan` conversion;
these are not launched by the MPCC known-map workflow. V3 additionally enables
ZED RGB/depth (30 Hz target) and IMU (100 Hz target) for perception/diagnostics;
ZED vehicle tracking, dynamic
vehicle TF and ZED point-cloud publication are disabled. ROS `/rosout`,
`/parameter_events` and package diagnostics are framework/monitoring outputs,
not interchangeable localization or control inputs.

**Data-role rules:** visualization topics may be disabled without invalidating
localization or MPCC. A pose and a scan must be paired by source timestamp;
never substitute EKF TF for raw LIO pose in ICP/PGO or raw-LIO consistency checks.
The nearest-neighbor consistency metric is not the native ICP fitness or an ICP
success flag. Scan errors/missing pairs remain diagnostic; the TF holding loop
does not wait on the consistency worker.

The old `/fastlio2/world_cloud` name is remapped at each host FAST-LIO launch;
no duplicate alias is published. Old bags retain their original names, and
submodule-provided launch/RViz samples retain upstream names. For replay,
remap the old world topic to `/fastlio2/visualization/world_cloud` if displaying
it; diagnostics now require the original body-cloud/raw-LIO pair.

## Rear-axle compensation

The LIO adapter calculates `T_odom_base = T_odom_livox * inverse(T_base_livox)`.
It rotates linear velocity into base axes and subtracts `omega × r`, where `r`
points from rear axle to the common Livox origin. It preserves LIO timestamps,
rotates/propagates covariance and supplies explicit floors for upstream zeros.
Upstream FAST-LIO fills pose and body-frame linear velocity in its Odometry
message, but leaves angular velocity and both covariances at zero. The rear-axle
adapter adds angular velocity from a source-time-matched raw IMU sample. Neither
Odometry topic carries linear or angular acceleration; the compensated linear
acceleration is published separately on `/rear_axle/imu` and is not currently
fused by the EKF.
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
`/rear_axle/wheel_odom`. The ERPM gain is 3465, provisionally calibrated from
the 2026-09-27 bag's straight-line LIO/wheel speed ratio of 1.342; longitudinal
variance 0.04 (m/s)² remains an assumption.

## EKF and TF ownership

EKF fuses rear-axle LIO pose/linear velocity, wheel `vx` only, and IMU yaw rate
only. IMU acceleration/orientation and wheel-integrated pose/steering-derived yaw
rate remain excluded. `two_d_mode: true` produces a planar output at a target
200 Hz. Gravity removal is configured but inactive while IMU acceleration is excluded.
The LIO and raw IMU streams remain correlated; this is not independent sensing.
The EKF keeps 2.0 s of measurement history and replays newer IMU/wheel updates
when delayed LIO arrives. `predict_to_current_time: true` then predicts the
corrected state to the output cycle's current time, so published state epochs
do not alternate between sensor time and current time. This corrects
measurement-time alignment; it does not
make the LIO result available earlier or guarantee a 200 Hz output under load.
The 2026-10-04 recording reached about 1.17 s of LIO receive age. Preserving
that arrival schedule in an isolated replay reproduced yaw distortion with the
former 0.5 s window; a 2 s window substantially reduced it. A fresh EKF header
alone does not establish correct handling of delayed measurements. See the
[dated LIO/EKF findings](reports/2026-10-04-lio-delay.md) for measurements and limits.

FAST-LIO point matching uses the IMU-frame point `R_il * p_lidar + t_il` in
its rotation Jacobian. The earlier binary incorrectly substituted world position
`t_wi`, increasing iteration counts and making convergence depend on map origin.
The fork source includes the correction; deployment must rebuild it rather than
reuse the incident binary. OpenMP defaults to two matching threads. The YAML
`ieskf_max_iter: 5` is still only read into configuration: it is not passed to
`IESKF::setMaxIter`, so the effective default cap remains ten iterations.
This configuration wiring issue remains open; lowering the cap has not been
approved as a production change.

The fork source separates ROS input callbacks from one joined LIO worker. The
worker owns conversion, IMU packaging, estimation, map updates and publishing.
Its pending LiDAR queue defaults to two scans; overflow retains the newest scan
and preserves all intervening IMU. IMU source gaps or buffer overflow cause an
explicit error exit. Both inputs remain reliable; LiDAR depth remains ten and
IMU DDS depth follows the propagation-buffer capacity (default 4096). Output
frames, scan-end stamps and body-cloud density remain unchanged. This source
change has a separate desktop replay
[acceptance report](reports/2026-10-05-fastlio-thread.md); production deployment
requires rebuilding the selected submodule commit.

```text
map                         optional: PGO while mapping, localizer with an existing map
 └─ odom
     └─ base_link           driving: EKF; mapping without EKF: LIO rear-axle adapter
         ├─ livox_frame     one static mounting transform
         ├─ base_footprint  existing zero transform, not a measured ground projection
         └─ zed2i_camera_link → camera internals (V3, approximate static mounting)
```

VESC must not broadcast vehicle TF in this pipeline. Upstream FAST-LIO's TF is
remapped to the private `/fastlio2/tf` topic by each host launch file, so it does
not enter the global tree. The frame adapter publishes `odom -> base_link` only
when `publish_odom_tf:=true` (mapping). V2/V3 set it false and let EKF publish
that edge. PGO retains the matching RAW LIO
body-cloud/pose pair; do not replace just its raw odometry input with rear-axle data.
V2 defines a localizer but does not add it to its launch description, so V2/V3
alone do not produce `map -> odom`. The separate
[known-map add-on](operations/known-map-mpcc.md) runs the upstream localizer
without duplicating LIO/hardware, isolates its raw TF and forwards `map -> odom`
after verified initial localization and receipt of a finite upstream transform;
source ages and consistency checks are diagnostic only. MPCC accepts
either a same-session `odom` reference or a `map` reference tied to the exact
saved map hash; EKF odometry remains `odom/base_link` in both cases.
V3 publishes an approximate static ZED mounting transform at
`base_link -> zed2i_camera_link = [0.33, 0, 0.03]` metres, assuming the ZED
mounting point is 3 cm ahead of `livox_frame` at the same height and orientation.
The complete extrinsic remains to be measured. ZED dynamic vehicle and IMU TF
remain disabled, and camera tracking is off.

## Upstream FAST-LIO integration

Maintain FAST-LIO changes in the configured fork and pin its commit in this
repository. FAST-LIO currently broadcasts TF
without reading a `publish_tf` configuration switch. The main-repository FAST-LIO
launch files therefore remap its `/tf` output to `/fastlio2/tf`; this preserves raw
LIO diagnostics while keeping global TF ownership in the main pipeline. Build the
pinned submodule and the main packages normally after updating either workspace.

TF queries for `odom -> livox_frame` return the EKF pose composed with the static
mounting transform. Consumers needing raw LIO pose must use `/fastlio2/lio_odom`.
Odometry frame identifiers do not themselves publish TF. Remapping changes the TF
topic only; the raw odometry and point clouds retain their frames and timestamps.
Starting upstream FAST-LIO outside these launch files requires the same remapping.

## Control chain

```text
/rc/channels -----------------------------+
/drive (MPCC or Nav2) --------------------+--> joystick_control_v2 (C++)
/calib/ackermann_cmd ---------------------+       |
                                                 v
                                          /ackermann_cmd
                                                 |
                                          ackermann_to_vesc
                                                 |
                                                VESC
```

V2 selects the C++ `joystick_control_v2` with the CH3/CH1 profile; V3 includes V2. The selected command
is an `AckermannDriveStamped` on `/ackermann_cmd`. MPCC requests speed (m/s) and
steering (rad). Calibration also encodes its mode using `drive.jerk`; the
[calibration guide](../src/aims_racer_system/docs/calibration.en.md) defines that
separate convention. RC channel mappings belong to the
[controller profile documentation](../src/ackermann_mux/README.md#rc-controller-variants).
RC, navigation and calibration each have a 0.2 s timeout measured with a monotonic
clock. Every rejected or expired selection publishes an explicit zero speed
command. Calibration also requires a fresh command after entry, unlock or RC
recovery; restamping the cached output never extends the producer's lifetime.
Only one autonomous producer should own `/drive` during a run. MPCC publishes
proposals on that topic whenever running; the RC selector owns execution
routing. Explicitly enabled MPCC continues calculating in manual mode and uses
actual `/ackermann_cmd` feedback for prediction history. Manual takeover changes
actuator authority without cancelling computation.

MPCC defaults to horizon 10 (1.0 s), 5 Hz optimization and 50 Hz command output.
Requests replay real inputs from the EKF epoch and forecast the old controller
(or hold actual manual targets) to takeover one solve period after submission (200 ms at 5 Hz).
Early results wait until takeover; the latest state is checked before activation.
Plan interpolation starts at takeover, while the horizon-derived age guard
(`horizon * 0.8 * 0.1 s`, 800 ms for horizon 10) starts
at the original EKF measurement. The request-to-reply acceptance budget is 250 ms; late results are skipped. The 100 ms state/RC
freshness guards remain unchanged. Late replies still require state/target checks,
and an exhausted prediction stops instead of repeating its last control. This bridge and its mismatch thresholds are experimental
low-speed assumptions; see the [timing contract](../src/controller/docs/implementation.md#measurement-time-computation-and-takeover).
Parameter units, time origins and explicit-override behavior are maintained in
the [runtime timing table](../src/controller/docs/usage.md#runtime-timing-parameters).

MPCC currently loads a recorded closed-lap reference at startup. Its outgoing
`nav_msgs/Path` topics visualize the reference and predictions; they are not
local-trajectory input interfaces. See the [MPCC implementation](../src/controller/docs/implementation.md).

## Persistent reference and local control frames

[ROS REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
defines `map` as a long-term reference whose estimated robot pose may jump, and
`odom` as a continuous short-term frame that may drift. Nav2 Humble's
[example configuration](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
uses `map` for the global costmap and `odom` for the local costmap. Its
[MPPI path handler](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/path_handler.cpp)
transforms the relevant global-plan segment into the local costmap frame;
[regulated pure pursuit](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)
instead transforms the near path into the robot base frame. These are concrete
implementations, not a rule that every controller must use the same frame.

For this MPCC, the saved course stays in `map` so it survives an `odom` restart.
The EKF state, vehicle dynamics, solver warm start and `/mpcc/prediction` stay
in continuous `odom`. At each solve a fresh planar `map <- odom` snapshot is
passed as a runtime parameter; only path association, tracking error and
footprint-corridor evaluation use the map alignment. `/mpcc/reference` remains
in `map`. TF updates do not recompile the embedded map spline. MPCC does not
subscribe to localization-valid status or apply map-TF age/correction-size
limits. It checks reference map identity and transform availability; EKF
odometry must remain no older than 100 ms. Map roll/pitch and measured speed operating range have no rejection gate; map
alignment is projected into x/y/yaw. Handover tolerances are 30 cm position,
30 degrees yaw, 20 degrees estimated/command steering, and 0.30 m/s
estimated/command speed. Its TF subscription queue is bounded
to 10 messages to avoid a backlog under load.
The external localization publisher verifies initial alignment and renews the
latest upstream correction at 50 Hz across ICP/input pauses. A bounded
background scan/map check reports match fraction, RMSE and runtime without
blocking TF renewal. Source ages are diagnostic only. Localization policy is
separate from MPCC's solver, actual-input, takeover and plan-validity checks.
Details are in the [known-map workflow](operations/known-map-mpcc.md).
Their timestamps can differ; moving-car validation is still required. Neither
Nav2's precedent nor consistency scores establish unique global localization.

## Configuration and verification

Custom geometry files use `livox_translation/livox_rpy`; adapter parameters use
`livox_translation/livox_quaternion`. Re-record references after changing the
sensor origin or restarting localization without an established alignment.
The older `/fastlio2/base_odom`, `/livox/imu_ekf` and wheel `/odom` interfaces have
been replaced by the three `/rear_axle/` topics in the table above.

Synthetic frame tests and their scope are documented in the
[system package](../src/aims_racer_system/README.md). The
[2026-09-21 hardware report](reports/2026-09-21-frame-check.md) records a stationary
check with the earlier local FAST-LIO modification; it does not validate the
subsequent unmodified-submodule launch configuration or moving-vehicle behavior.
Delayed-LIO replay remains an open item in the
[vehicle checklist](operations/vehicle-checklist.md).
