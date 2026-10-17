# Body acceleration quality and causal replay

This tool audits six-axis IMU data and replays the **native** body acceleration observer. It never reads EKF arrays, estimates unknown accelerometer bias from a single stationary pose, or uses wheel speed as a tuning target. Large raw inputs, event logs and complete traces live in the external data tree.

Use a new output directory for every command (the CLI refuses existing directories):

```sh
PY=/home/elesheep/aimsracer-data/sessions/2026-10-13/longitudinal-acceleration-review/.venv/bin/python3
$PY -m unittest discover -s verification/body_acceleration -v

g++ -std=c++17 -O2 -I/usr/include/eigen3 -Isrc/aims_racer_system/include src/aims_racer_system/src/body_acceleration.cpp src/aims_racer_system/tests/body_acceleration_trace.cpp -o /tmp/body_acceleration_trace

$PY verification/body_acceleration/quality.py \
  --input /home/elesheep/aimsracer-data/sessions/2026-10-15/022604-mpcc-review/yaw-rate-quality/input \
  --exact-input /home/elesheep/aimsracer-data/sessions/2026-10-15/body-acceleration-observer/source-quality-20261015-01 \
  --trace /tmp/body_acceleration_trace \
  --output /home/elesheep/aimsracer-data/sessions/2026-10-15/body-acceleration-observer/my-new-run
```

If host Eigen is unavailable, compile inside the source-mounted ROS container, then copy only the executable out. The executable hash and source hashes enter `metrics.json`. Additional native flags use `--trace-arg=--no-filter` or repeated `--trace-arg` for each flag/value. Native defaults are frozen by their source hash; no tuning is performed by this script.

`extract.py` runs in sourced ROS Humble with NumPy. It opens the bag using `mode=ro`, selects only raw/corrected/rear IMU and rear LIO, retains source/receipt times and all covariance matrices, and hashes selected CDR with source size/mtime before and after:

```sh
python3 verification/body_acceleration/extract.py --bag /path/bag_0.db3 --output /path/new-extract
```

The source-match audit derives gyro bias from raw minus processed gyro at exactly equal source stamps. With `--exact-input`, replay uses the actual corrected IMU values and receipt times. Without it, rear receipt is a conservative proxy, explicitly recorded. Streams merge by receipt only; delayed LIO retains its original source stamp. Corrected IMU has no fabricated samples before bias readiness. Exact replay refuses cross-topic receipt ties, because the current NPZ does not retain sqlite IDs; this artifact has zero ties. Only accepted, valid IMU trace rows represent publication candidates. LIO rows expose internal revision at the latest IMU stamp. The legacy native CSV column `raw_ax` means reconstructed **unfiltered inertial acceleration at the rear axle**, after gravity and lever correction; it is not raw IMU acceleration or specific force. `ax` is the filtered longitudinal estimate; `force_x` is mounted specific force before those corrections.

Welch PSD uses local interpolation within contiguous source segments; gaps above 1.5 times the median sample period split segments. Segments shorter than 512 samples are excluded. The report states supporting samples/windows, so these PSDs cannot be treated as whole-run statistics. Stationary overlapping Allan deviation reports each tau, cluster count and overlapping pair count; overlapping pairs are correlated. The short available recording cannot identify long-term bias instability.

Future independent stationary data can be supplied with `--static-input` as NPZ (`time_s`, `values[N,6]`) or CSV (`time_s,ax,ay,az,wx,wy,wz`), all SI. The CLI requires increasing near-uniform timestamps and refuses gaps. Separate externally verified stationary/motor-on/motor-off datasets should each get new output directories. Known-pose accelerometer calibration uses `--calibration known-poses.json` containing `measured` and `expected` Nx3 SI arrays. It fits only diagonal positive scale and bias, requires at least six poses and sufficient variation on every axis, and rejects a single tilted stationary pose. This is an interface, not a calibration performed on this car.

Future motion-capture CSV requires `source_ns,receive_ns,px,py,pz,qx,qy,qz,qw`. Optional `cov_00` through `cov_55` (all 36 row-major entries) are retained as original pose covariance; partial covariance is rejected and missing covariance is explicitly marked unknown. These pose covariance values are not treated as calibrated differentiated-acceleration covariance. Supply `--reference-csv` and `--reference-metadata` together. Metadata is mandatory:

```json
{
  "clock_scale": 1.0,
  "clock_offset_ns": 0,
  "clock_uncertainty_ns": 1000000,
  "rotation_xyzw": [0, 0, 0, 1],
  "translation_m": [0, 0, 0],
  "body_origin_in_marker_m": [0, 0, 0],
  "body_orientation_in_marker_xyzw": [0, 0, 0, 1],
  "window_s": 0.11,
  "polyorder": 3
}
```

The clock maps mocap source to IMU clock as `mapped_ns = clock_scale * source_ns + clock_offset_ns`. Decimal arithmetic retains the mapped epoch origin; relative timestamps retain precision. `rotation_xyzw, translation_m` transform mocap world coordinates to the observer world. `body_origin_in_marker_m` is the rear/body origin expressed in the tracked marker frame; its rotated offset is applied before differentiating. `body_orientation_in_marker_xyzw` rotates body vectors into marker coordinates. Body orientations are SLERP-interpolated, position is locally resampled, and a centered Savitzky–Golay second derivative supplies offline world/body acceleration. The actual odd window length, invalid half-window edges, original source/receive times, mapped times, source hashes and transform metadata are retained. This centered reference uses future samples and is saved in a separate file; it never enters native online replay. No actual mocap measurement is present in the current evidence.

For the current rear-axle/CG output interface, see the [unified state guide (Chinese)](../../docs/operations/state-observer.zh-CN.md).

## One-timespeed ROS receipt replay

In sourced ROS Humble, `runtime_replay.py` starts only the native observer in
localhost-only domain 193 and sends a selected recorded receipt window at 1×.
The observer uses wall-clock time; source stamps are mapped with one shared offset
to preserve nominal recorded source ages at scheduled publication. Actual
scheduling and DDS delivery add their own jitter; the original DDS callback order
is not enforced. The complete selected arrays
are loaded before timing starts. No hardware or control process starts.

```sh
python3 verification/body_acceleration/runtime_replay.py \
  --input /path/to/exact-extraction/imu-quality-input.npz \
  --output /path/to/new-runtime-evidence \
  --executable /path/to/install/aims_racer_system/lib/aims_racer_system/body_acceleration \
  --duration 20
```

The default window is the first 20 s of this recorded RUNNING interval; override
`--origin-ns` and `--start-source-offset` for another dataset. Output checks cover
finite values, exact source-stamp membership, monotonic publication and node graph
independence. Raw outputs, status snapshots and input/executable/helper hashes are
retained. Callback timing is the maximum/final reported P99 over rolling windows
of up to 512 callbacks; it is neither whole-run P99 nor sensor/actuator latency.
Warmup and post-replay drain statuses are classified separately. This test uses
recorded LIO, not a concurrently running localization stack.
