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
For a known-map run, also record `/localization/map_valid`,
`/localization/map_sha256` and `/localizer/raw_tf` so the accepted alignment and
the localizer's source transform can be reviewed with `/tf`.

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
loads a processed closed lap from disk. Follow [record and prepare one
lap](../../src/controller/docs/usage.md#record-and-prepare-one-lap) for an
`odom` reference, keeping localization running between recording and execution.
For a saved `map` reference and restart, use the
[known-map workflow](known-map-mpcc.md). Live local-path topic input is not
implemented.

Mapping and CSV capture can run in one mapping session, but that does **not**
make the CSV a persistent map-frame reference. The separate
`base_orin_livox_mapping_v2.launch.py` runs LIO and PGO without the EKF, so its
reference-CSV command must select rear-axle LIO odometry explicitly:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run aims_mpcc record_path --ros-args \
  -p odom_topic:=/rear_axle/lio_odom -p output:=/home/aims/mapping-lap-run1.csv
```

That CSV is still `odom/base_link`. PGO publishes a changing `map -> odom`
transform and saves a map in `map`; do not treat the last transform as the
whole lap's map alignment. Do not start the mapping launch beside V2: both
launch hardware/LIO and would duplicate TF ownership. The
[known-map workflow](known-map-mpcc.md) shows how to recover a map-frame
candidate from saved optimized PGO poses, prepare it against an exact map file,
and relocalize after a restart. This remains subject to live validation.

For the initial low-speed calibration, keep the VESC in **speed mode** and record
the actual speed for each maneuver. About 0.8–1.2 m/s covers the current
controller's operating region; do not mix in current/duty mode or runs above
2 m/s when fitting that low-speed response. Record one continuous bag before
any motion:

```bash
ros2 bag record -o /home/aims/mpcc-prep-run1 \
  /livox/lidar /livox/imu \
  /fastlio2/lio_odom /rear_axle/lio_odom /rear_axle/imu \
  /rear_axle/wheel_odom /odometry/filtered \
  /sensors/core /sensors/servo_position_command \
  /commands/motor/speed /commands/servo/position \
  /rc/channels /ackermann_cmd /control/autonomy_speed_enabled \
  /tf /tf_static
```

Choose a new output directory for every attempt and check that the disk can
sustain raw LiDAR recording. `/ackermann_cmd` gives the requested speed, steering
and mode (`drive.jerk == 0` for speed mode); the two `/commands/` topics show
what reached the VESC driver. `/sensors/servo_position_command` is a command
echo, **not** a physical steering-angle measurement. Raw IMU and the odometry
streams allow command-to-yaw/speed fits and localization-freshness checks. The
LiDAR topic enables later inspection or replay of LIO timing.

Use this sequence, noting approximate bag times for each segment:

1. Hold still for 10–15 s to measure IMU bias and baseline noise.
2. On a clear straight, repeat 3–5 speed-mode starts from rest to a safe
   low speed near 1 m/s, hold steady for at least 2 s, then command zero and let the car
   stop. Include a few 0.6 ↔ 1.0 m/s steps if space allows. This measures both
   first motion and the speed-loop/braking response with the current ERPM scale.
3. At a steady low speed near 1 m/s, apply repeatable left/right steering
   0 → about ±0.1–0.2 rad → 0, with 1–2 s steady sections before and after
   each change; repeat each direction 3–5 times. Leave enough room for the
   turn and record the actual command rather than assuming the hand motion was
   an ideal step. The steady arcs also check curvature, steering sign and
   left/right asymmetry. Avoid full-lock tests for this low-speed MPCC fit.
4. Start `record_path` in another terminal just before the reference lap,
   drive one forward closed lap with slight overlap at a safe low speed, then
   stop that recorder. Keep the bag running through this lap. See the linked
   usage guide for the exact CSV command and path-preparation step.
5. Stop the bag after another stationary 10 s. Record which bag interval is the
   lap, the speed mode used, and the effective configuration/commit IDs.

The bag contains the observations, but current `prepare_path` takes the
`record_path` CSV, not a bag directly. Measure the car footprint and minimum
left/right free corridor widths separately; a driven lap does not establish
track boundaries. Review speed units, sign, LIO age and missing data before
fitting a new steering time constant or attempting MPCC control.
When using the mapping launch, `/odometry/filtered` is absent: omit it from
the bag command, keep `/rear_axle/lio_odom` and `/tf`, and add
`/pgo/loop_markers` if loop-closure diagnostics are useful. Save the PGO map
separately with `bash preprocess_script/save_map.sh`, which calls
`/pgo/save_maps` and defaults to a dated directory under `~/maps/`. Check the
service response and the resulting `map.pcd`/`poses.txt`: the helper checks
the reported `success: true` and that both files are nonempty. The bag does not
save that map artifact.
