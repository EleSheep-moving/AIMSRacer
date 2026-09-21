# Record sensor and controller data

Start the selected [vehicle bringup](bringup.md), verify the
[frame contract](../architecture.md), and source the same environment in the
recording terminal. Use a new output directory on a disk with sufficient space.
The commands below subscribe to data; they do not enable autonomous control.

## Sensor and estimator bag

Replace `/data/session-001` with your chosen new directory:

```bash
ros2 bag record -o /data/session-001 \
  /livox/lidar /livox/imu \
  /fastlio2/lio_odom /rear_axle/lio_odom /rear_axle/imu \
  /rear_axle/wheel_odom /odometry/filtered \
  /sensors/core /sensors/servo_position_command \
  /rc/channels /ackermann_cmd /control/autonomy_speed_enabled \
  /tf /tf_static
```

Raw point clouds increase disk throughput and size, but are needed to rerun LIO.
For a controller experiment, append `/drive /mpcc/status /mpcc/reference
/mpcc/prediction` to the same command. For calibration, also capture the
`/calib/` topics specified in the [calibration guide](../../src/aims_racer_system/docs/calibration.en.md).
Append `/fastlio2/tf` only when the isolated raw LIO TF is useful for diagnostics;
raw LIO pose is already available in `/fastlio2/lio_odom`.

Stop with Ctrl-C and inspect the result:

```bash
ros2 bag info /data/session-001
```

Confirm the expected topics, message counts and duration, including `/tf_static`.
Keep the effective vehicle/geometry/EKF/LIO configurations, repository and submodule
commit IDs, software versions, and a note of the maneuver and localization session
beside the bag. Do not commit large bags or generated logs to the source repository.

## Timing interpretation

Receive age is observation time minus the message's source timestamp; the two
must use a consistent clock domain. It includes scheduling and delivery as well
as processing, and is not IPOPT or LIO core execution time alone.

Livox CustomMsg stamps represent the scan start. Point `offset_time` values are
relative to it. FAST-LIO computes the scan end from the last retained point and
stamps its odometry with that end time; the rear-axle adapter preserves the stamp.
LIO odometry age therefore excludes the scan acquisition interval. Record raw
clouds and IMU if you need to inspect those stages.

P50 is the median; P95 is the age below which 95% of samples fall. Report the
sample window, operating conditions and clock basis with those numbers. A fresh
EKF output stamp does not prove old LIO measurements were correctly replayed.

## MPCC reference recording

A rosbag and a prepared MPCC reference serve different purposes. The controller
currently loads a processed closed lap from disk. Follow [record and prepare one
lap](../../src/controller/docs/usage.md#record-and-prepare-one-lap) to create it;
keep localization running between that recording and execution. Live local-path
topic input is not implemented.
