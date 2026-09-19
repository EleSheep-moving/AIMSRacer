# Rear-axle reference (V2 driving/mapping and inherited V3)

`base_link` is now at the rear-axle midpoint, with x forward, y left, z up. Its height is unchanged from the previous base_link. `base_footprint` retains the old zero transform; ground height has not been measured, so this is not a newly verified ground projection.

## Configuration

`params/rear_axle_geometry.yaml` is the mounting source. LiDAR x=0.30 m is the owner's approximate confirmed forward distance. y=0, z=0.03 m and zero mounting rotation are inherited assumptions. For the existing identity `r_il` and `t_il=[-0.011,-0.02329,0.04412]`, the raw IMU location in base_link is `[0.311,0.02329,-0.01412]` m. The negative z is a consequence of existing extrinsics, not a new measurement: verify sensor height and extrinsics on the car.

The shared launch derives `T_base_imu = T_base_lidar * inverse(T_imu_lidar)` using the SAME `lio_config` passed to FAST-LIO. Online extrinsic estimation (`esti_il: true`) is rejected because static transforms would otherwise diverge.

V2/V3 now launch `imu_to_rear_axle.py`; the legacy `livox_imu_to_ekf.py` remains unchanged for older launches. The new output `/livox/imu_ekf` uses `base_link`, with no extra 180-degree data rotation. `imu_link` is now a physical raw-IMU frame alias, not the output frame of this topic.

## Rear-axle IMU acceleration and gravity

The converter rotates raw gyro and acceleration into base axes, converts acceleration from g to m/s², then computes:

`f_rear = f_imu - alpha × r - omega × (omega × r)`

Here `r` points from rear axle to IMU and all vectors use base axes. The tangential and centripetal terms are both removed. Gravity is deliberately retained as part of specific force. `ekf_rear.yaml` now fuses only IMU yaw rate (index 11). Compensated acceleration and orientation remain available on the topic for diagnostics, but neither is fused. `imu0_remove_gravitational_acceleration: true` remains configured and is inactive while all acceleration update flags are false. The helper itself does not subtract gravity.

Angular acceleration is estimated with a causal 25 ms least-squares gyro window, requiring at least three samples. This reduces differentiation noise but can lag rapid changes. A gyro gap over 30 ms or a non-increasing timestamp resets the window. The output uses the input source timestamp, never the wall-clock publication time.

For gravity direction, use the latest nonfuture raw LIO attitude, transformed into base orientation and gyro-propagated to the IMU source timestamp. Maximum attitude age is 250 ms. This preserves roll/pitch information even with a planar EKF. Startup, unavailable gyro history, or unavailable/stale attitude produces gyro-only IMU messages (`linear_acceleration_covariance[0] = -1`, orientation unavailable). This is validity handling for acceleration compensation, not a new MPCC LIO-stop watchdog.

Sensor covariance is rotated/scaled, and gyro/derivative uncertainty is propagated through the lever-arm terms with a conservative correlation bound. Gravity-direction uncertainty is also added. Zero input covariance receives explicit assumed floors; bias, mounting error, filter-window lag, and temporal noise correlation still need real-data characterization. LIO and the raw IMU remain correlated sources, so this is not a statistically independent second measurement system.

## Data and TF ownership

- `/livox/imu`: raw MID360 IMU; input to FAST-LIO and the rear-axle adapter.
- `/fastlio2/lio_odom`: raw IMU-origin odometry, `odom / livox_imu`.
- `/fastlio2/base_odom`: pose, twist and covariance converted to `odom / base_link`.
- `/livox/imu_ekf`: rear-axle specific force and gyro in `base_link`, plus LIO/gyro-derived attitude for gravity removal; EKF fuses yaw rate only.
- `/odom`: VESC motor-speed odometry in `base_link`; EKF `odom1` fuses only `twist.linear.x` (index 6). Wheel-integrated pose, assumed zero lateral velocity and servo-derived yaw rate are excluded.
- `/odometry/filtered`: EKF rear-axle odometry used by MPCC.

VESC converts motor ERPM using the existing gain 4650 and zero offset. `vesc_to_odom_node.longitudinal_velocity_variance` in `params/vesc.yaml` sets `twist.covariance[0]`; default 0.04 (m/s)² corresponds to an assumed standard deviation of 0.2 m/s. It is configurable and must be finite and positive. This is an initial uncertainty assumption, not a measured speed calibration or a wheel-slip model. VESC keeps `publish_tf: false`, and the existing LIO fusion mask is retained.

The adapter uses the latest raw gyro at or before each LIO timestamp, at most 50 ms old. Missing/stale samples cause dropped output, never an uncorrected velocity. It subtracts the rotational lever-arm velocity. Raw gyro bias is not estimated by this adapter; configured covariance floors reflect assumed uncertainty, not measured calibration. FAST-LIO's currently zero covariances receive nonzero floors.

The local FAST-LIO submodule now reads its YAML `publish_tf` flag and guards TF publication. V2 uses `publish_tf: false`; the temporary TF-topic remapping has been removed. Rebuild `fastlio2` together with `aims_racer_system` before deploying these launch changes; an older FAST-LIO binary ignores the flag. Omitted flags default to true, preserving upstream behavior. This change is distributed as a [versioned patch](../../../patches/README.md); run `bash scripts/apply_fastlio_patch.sh` on a fresh checkout before building. The upstream gitlink stays pinned and the patched submodule working tree is intentionally modified. During driving, EKF alone owns `odom -> base_link`. During mapping (without EKF), the adapter owns that transform. Mapping PGO uses `local_frame: odom` and retains the raw IMU cloud/pose pairing.

## MPCC and footprint migration

`controller/config/vehicle.yaml` now has `rear_offset: 0`, steering limit 0.45 rad, rate limit 2 rad/s. Body dimensions remain unmeasured and `geometry_verified: false`.

MPCC still uses a symmetric conservative rectangle centered on base_link. Set `half_length` to the larger of rear-axle-to-front-edge and rear-axle-to-rear-edge distances; similarly enclose both sides with `half_width`. This can overestimate rear clearance until asymmetric footprint support is added.

Existing Nav2 footprint vertices were shifted forward by 0.17 m (`x=[-0.33,0.67]`), preserving the old represented body relative to the new origin. These are inherited envelope dimensions, not newly measured car dimensions. V1 launch/configs retain their old sensor frame definitions and should not be combined with the migrated Nav2 footprint or rear-frame MPCC config. V3 inherits V2; any externally supplied ZED mounting x must be re-expressed from the rear axle (old x + 0.17 m, if only the origin changed).

Re-record/re-prepare reference laps after fixing localization frames. Do not merely relabel old recorded sensor poses. Confirm straight-driving yaw, left-turn positive yaw rate, TF ownership and actual mounting on the Orin before enabling MPCC.

## Local isolated validation

The test directories and controller Docker tooling are excluded from Git. The
commands below require a development workspace retaining those local files and
the existing `aimsracer-mpcc:humble` image; a fresh checkout alone is insufficient.
From the AIMSRacer root:

```bash
docker build -f src/aims_racer_system/docker/Dockerfile.frames -t aimsracer-mpcc:rear-frames .
docker run --rm --network none aimsracer-mpcc:rear-frames
```

Tests include lever-arm pose/twist, inverted sensor orientation, covariance, frame rejection, stale/future gyro rejection, and a real ROS EKF pipeline driven by synthetic LIO/IMU messages. The pipeline checks turning (including 5 m/s on a 5 m radius), changing yaw rate, straight-line acceleration and tilted stationary behavior, and verifies a single dynamic TF publisher. Separate tests run the real VESC converter, check configurable velocity variance, and inject bogus wheel pose/lateral velocity/yaw rate plus IMU acceleration to verify that EKF ignores the excluded fields. This does not execute FAST-LIO scan matching or validate hardware geometry.

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
