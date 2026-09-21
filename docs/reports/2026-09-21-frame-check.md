# V2 hardware frame check — 2026-09-21

Final sample ended at 2026-09-20T16:07:13.344695+00:00.
This is a 30-second read-only ROS observation on the powered Orin/MID360/VESC
vehicle using `base_orin_livox_bringup_v2.launch.py`. V2 was stopped after the test.
RC was locked, wheel speed was zero, autonomy was disabled, and all observed
Ackermann speed/acceleration/jerk commands were zero. No MPCC, Nav2 or calibration
nodes were started, and the observer published no messages.

This is a historical stationary measurement. Use the current [bringup guide](../operations/bringup.md)
for startup; the tested source configuration differs from the current TF isolation.

## Results

- All required topics were present with the intended frame identifiers.
- 300 source-time-matched raw/rear LIO messages were checked.
- Position residual against `p_rear = p_livox - R * [0.30, 0, 0.03]` was zero.
- Velocity residual against `v_rear = v_livox - omega × [0.30, 0, 0.03]`
  was zero, using the selected gyro reported in the rear odometry message.
- Orientation difference was numerical round-off (maximum 4.3e-18 rad).
- Comparing with the observer's independently received latest gyro instead
  gives a maximum velocity difference of 0.00105 m/s; independent subscriber
  delivery can select a different gyro sample from the adapter.
- `/tf` had exactly one publisher, `ekf_filter_node`, and one observed edge:
  `odom -> base_link`.
- `/tf_static` contained `base_link -> livox_frame` at [0.30, 0, 0.03] m,
  identity rotation, plus the existing identity `base_link -> base_footprint`.
- No `laser`, `livox_imu`, or `imu_link` alias appeared in the TF tree.
- Compensated IMU acceleration was valid throughout the final observation.

The raw point cloud was sampled once to verify `livox_frame`; then its
subscription was removed to avoid Python point deserialization distorting the
high-rate observer. The table reports observed receive rates, not guaranteed deadlines.

| Topic | Samples | Receive Hz | Age P95 ms | Age max ms | Frames |
| --- | ---: | ---: | ---: | ---: | --- |
| `/livox/imu` | 5956 | 199.7 | 1.40 | 25.27 | livox_frame |
| `/fastlio2/lio_odom` | 300 | 10.0 | 88.72 | 155.22 | odom → livox_frame |
| `/rear_axle/lio_odom` | 300 | 10.0 | 91.60 | 155.47 | odom → base_link |
| `/rear_axle/imu` | 5957 | 199.7 | 6.99 | 28.35 | base_link |
| `/rear_axle/wheel_odom` | 1497 | 50.2 | 1.74 | 99.99 | odom → base_link |
| `/odometry/filtered` | 5763 | 193.2 | 9.62 | 31.47 | odom → base_link |

## Findings outside the coordinate conversion

1. Under the stationary/zero-wheel-speed condition, raw gyro yaw-rate mean was
   0.007749 rad/s (0.444 deg/s); the EKF yaw-rate estimate inherited
   approximately the same value. An earlier stationary window measured 0.01449 rad/s.
   This suggests a time-varying gyro zero-rate bias that needs separate stationary
   calibration/estimation. No bias value was hard-coded during this test.
2. EKF received/output observations were about 193 Hz versus a 200 Hz target;
   an update-rate warning was observed. Rear LIO age reached about 155 ms despite
   a P95 near 92 ms. A fresh EKF timestamp does not remove underlying LIO latency.
3. The 0.30 m mounting offset and orientation were checked for correct software
   application, not measured physically. Stationary data do not validate the
   sign/magnitude of compensation during actual turns or acceleration. The
   earlier synthetic tests cover those calculations, not real vehicle dynamics.

## Startup fixes and reproducibility

The first localhost-only startup exhausted CycloneDDS participant indices; several
of the twelve V2 processes failed to join. The test used a session-local XML with
loopback-only networking, multicast disabled, explicit localhost peer discovery,
and `MaxAutoParticipantIndex=63`. With an explicit `lo` interface, use
`ROS_LOCALHOST_ONLY=0` to avoid duplicate loopback selection by the RMW layer.
The XML still restricts DDS to loopback. Livox's separate Ethernet UDP transport
continues to use 192.168.1.5 / 192.168.1.198.

The recorded observation used a local FAST-LIO source modification that is no
longer part of the repository policy. Current main-repository launch files instead
remap upstream FAST-LIO `/tf` to `/fastlio2/tf`; repeat the hardware check after
updating the unmodified-submodule configuration.

Local evidence (gitignored): `log/v2-tf-hardware-20260920-01/` contains
`verified-launch.log`, `verified-audit.log`, `summary.json`, `samples.json`,
`cyclonedds.xml`, and the observer script. Earlier startup/observation logs are
retained separately. The directory name reflects the beginning of this session.

The local evidence files are not available in a fresh checkout. See the current
[recording guide](../operations/recording.md) for a new measurement session.
