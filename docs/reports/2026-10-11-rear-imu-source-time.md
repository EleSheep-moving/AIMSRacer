# Rear axle IMU and LIO source-time correction

**English** · [简体中文](2026-10-11-rear-imu-source-time.zh-CN.md)

## Required timing contract

- `/rear_axle/lio_odom`: retain the processed LIO scan-end stamp. At source time 10.000 s, use gyro measured at 10.000 s, even if LIO arrives at 10.170 s. Match exact knots or interpolate between measured IMU knots; do not extrapolate pose or gyro. The interpolation interval is limited to 30 ms for source continuity. A small pending queue waits for an upper IMU knot if the LIO callback arrives first.
- `/rear_axle/imu`: publish from every incoming corrected IMU sample immediately, retaining its source stamp. Convert units and fixed mounting axes, and remove rotational lever effects. No subscription to LIO, attitude prediction, 250 ms attitude-age gate, or extra aligned-IMU topic.
- IMU output is still gravity-containing specific force. Orientation is unavailable (`orientation_covariance[0] = -1`). Force availability depends on finite input, valid covariance and sufficient gyro history to estimate angular acceleration; startup or history resets can still leave acceleration unavailable. Its default zero must not be used as a measurement when covariance[0] is -1.

The current two-dimensional EKF continues to fuse LIO, wheel vx and IMU yaw rate only. Acceleration/orientation fusion stays disabled. The existing EKF `predict_to_current_time=true` remains; it predicts the fused state, not the upstream LIO observation. FAST-LIO's internal IMU propagation, scan deskew, output frames and scan-end publication have not been changed.

## Cause and fix

The previous force adapter required a recent LIO attitude and gyro-propagated it to each incoming IMU time. That attitude is useful for gravity removal but not required by the lever correction itself:

`f_rear = R_mount f_imu - alpha × r - omega × (omega × r)`.

The coupled gate discarded usable force when LIO was late. Removing this coupling preserves source-time force conversion. Offline gravity removal must use LIO attitude aligned to the acceleration's source time, with explicitly identified interpolation; the historical report's propagated-orientation metrics are not automatically measurements of this new alignment policy.

## Recorded-input replay

Eight completed 2026-10-10 laps were replayed through the production C++ core. Raw Livox acceleration was paired by exact integer nanosecond stamps with the recorded rear gyro (the gyro-bias-corrected input, since mounting rotation is identity in these bags). The active window is first enabled RUNNING through first COMPLETE, using rear-message bag receipt times consistently for both versions.

| Run | Previous force unavailable | Corrected force unavailable |
| --- | ---: | ---: |
| 175129_v10_lap01 | 2.1836% | 0.0000% |
| 185351_v12_n15_independent_lap01 | 5.8445% | 0.0307% |
| 190404_v15_n15_cap4_lap01 | 1.4318% | 0.0000% |
| 190848_v18_n15_accel1_cap4_lap01 | 3.4767% | 0.0589% |
| 192020_v22_n15_accel1_cap4_lap02_clockfix | 5.8283% | 0.0209% |
| 192725_v26_n15_accel1_cap4_lap01 | 4.6814% | 0.0217% |
| 192955_v35_n15_accel1_cap4_lap01 | 5.8814% | 0.0158% |
| 193524_v35_n15_accel1_cap4_lap02 | 1.9676% | 0.0000% |

This is numerical replay of matched recorded inputs, not a measurement of live ROS delivery or a new vehicle run. Missing extracted inputs and callback order cannot be fully reconstructed. The remaining unavailable samples reflect insufficient angular-acceleration history. Covariance retains sensor/lever uncertainty; the old heuristic gravity/attitude inflation is removed from the specific-force stream. It is not an empirical downstream acceleration covariance calibration.

## Verification and artifacts

- Two baseline regression checks reproduced missing force without LIO and gyro-based attitude extrapolation.
- A ROS regression reproduced the old latest-past gyro error: 0.397 rad/s selected instead of the source-time 0.400 rad/s.
- Local package build and `colcon test-result`: **75 tests, 0 errors, 0 failures**. The ROS test checks exact nanosecond header retention, unchanged pose, source-time velocity correction, and live IMU before LIO arrival.
- NX package build and tests also passed: **75 tests, 0 errors, 0 failures**. No vehicle-driver processes or driving commands were started; ROS regression nodes used isolated domain 188.

Evidence: `~/aimsracer-data/sessions/2026-10-11/imu-source-time-alignment/` (`baseline-failing-tests.txt`, `source-time-failing-test.txt`, `test-result.txt`, `replay_metrics.csv`, replay source and manifests). Original bags and prior extraction files are retained unchanged.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select aims_racer_system --event-handlers console_direct+
colcon test-result --verbose
```

[Architecture](../architecture.md#ekf-and-lio-timing) · [System package](../../src/aims_racer_system/README.md)
