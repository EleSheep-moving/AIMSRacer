# Planar acceleration verification

[Operations](../../docs/operations/planar-acceleration.md) · [Dated evidence](../../docs/reports/2026-10-15-planar-acceleration-quality.md) · [Vicon acquisition](../../docs/operations/vicon-observer-validation.zh-CN.md)

`evaluate.py` replays corrected IMU and LIO orientation by recorded receipt order
through `body_acceleration_trace --lateral`, preserving source stamps. It verifies
input hashes, compares lateral-enabled/disabled ax and optionally the accepted old
binary, writes full traces and plots rear-axle G-G sample occupancy. It also runs
pure lateral translation and fixed-rear-point yaw-pivot synthetic tests. Raw trace
`raw_ay` means compensated unfiltered inertial ay, not raw IMU specific force.

MPCC is the primary ay dynamic test; B/C straight-line calibration replays are
auxiliary timing/rest/cross-axis checks. `mpcc_review.py` verifies the original
trace digest, partitions valid motion samples using explicit gyro thresholds and
exports the full-motion overview plus automatically selected transient/peak
windows. It performs no tuning and introduces no estimator changes:

```bash
python3 verification/planar_acceleration/mpcc_review.py \
  --quality /absolute/planar-acceleration-20261015/quality-02 \
  --output /absolute/new-mpcc-review
```

Groups partition samples, not sustained manoeuvres or independently labelled
truth. Differences between specific force, reconstructed ay and filtered ay are
not accuracy measurements. See the dated report for exact thresholds and timing
anomalies; dense source intervals are retained, not silently removed.

`kinematic_comparison.py` evaluates the user's proposed own-ax/own-rate route.
It integrates ax from the last recorded pre-motion stationary sample (vx=0),
matches our yaw-rate output by exact IMU source stamp and compares the product
against reconstructed ay. A static-mean-subtracted integral is a sensitivity
case. Wheel vx × our r is a separate comparison, not an observer input:

```bash
python3 verification/planar_acceleration/kinematic_comparison.py \
  --planar-quality /absolute/planar-acceleration-20261015/quality-02 \
  --yaw-quality /absolute/yaw-rate-20261015/quality-02 \
  --comparison-input /absolute/022604-mpcc-review/yaw-rate-quality/input \
  --output /absolute/new-kinematic-comparison
```

It verifies trace/input hashes and original-bag identity. Trapezoidal integration
uses valid ax source intervals at most 30 ms; wheel interpolation cannot span
more than 30 ms. Common-sample statistics and first/last 10 s summaries are
retained without gain/time fits. Planar `dot(vx)=ax+r*vy`; dropping r*vy and
accumulating measurement errors makes an unbounded ax integral unsuitable as
independent speed truth. The source IMU is shared between ax, ay and r, and wheel
speed is also not independent ground truth. No velocity state or observer
parameter is changed by this offline comparison.

Build the ROS package with testing enabled and locate `body_acceleration_trace` in
its build directory. Offline dependencies: Python 3, NumPy and Matplotlib. Inputs
are external artifacts from the accepted body-acceleration analysis, not bundled
in Git. Output must be a new directory.

```bash
python3 verification/planar_acceleration/evaluate.py \
  --binary /absolute/build/body_acceleration_trace \
  --mpcc-input /absolute/source-quality-20261015-01 \
  --legacy-binary /absolute/accepted/desktop-trace-cmake.elf \
  --bc-root /absolute/calibration-14night-01 \
  --output /absolute/new-planar-quality
```

Run the standalone 20 s 1× ROS/DDS test in a sourced Humble environment with
NumPy available. It uses localhost domain 197 and starts only the observer:

```bash
/usr/bin/python3 verification/planar_acceleration/runtime_replay.py \
  --input /absolute/source-quality-20261015-01/imu-quality-input.npz \
  --output /absolute/new-planar-runtime \
  --executable /absolute/install/lib/aims_racer_system/body_acceleration \
  --trace-executable /absolute/build/body_acceleration_trace
```

This tool validates the default model/geometry only; custom parameter files are
not accepted because the native trace oracle uses defaults. ROS source sequences
are compared against native valid IMU publications; finite XY variance, SI/G
source matching/unit conversion and subscription independence are also checked.
Numerical native/ROS acceleration equality is not checked: inter-topic callback
ordering may differ. It retains int64 timestamps separately from float signals.
The common epoch shift preserves recorded source ages while actual DDS scheduling
adds jitter. This is software replay, not hardware/vehicle latency acceptance.

CTest adds core physical/replay tests and `planar_acceleration_runtime` DDS/launch
tests; existing default x-only observer tests remain. See the dated report for
the known dynamic phase error and burst-source-time failure. Computational
validity and covariance must not be interpreted as independent accuracy evidence.
