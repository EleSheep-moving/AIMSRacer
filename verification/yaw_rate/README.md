# Independent yaw-rate verification

Build core without ROS:

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc/aims_racer_system/include src/aims_racer_system/src/yaw_rate.cpp src/aims_racer_system/tests/test_yaw_rate.cpp -o /tmp/yaw_rate_test
/tmp/yaw_rate_test
g++ -std=c++17 -O2 -Isrc/aims_racer_system/include src/aims_racer_system/src/yaw_rate.cpp src/aims_racer_system/tests/yaw_rate_trace.cpp -o /tmp/yaw_rate_trace
```

Python offline dependencies: NumPy, SciPy, Matplotlib, Chinese Noto CJK font.
Exact extraction schema comes from `verification/body_acceleration/extract.py`.
Use a new output directory:

```sh
python3 -m unittest discover -s verification/yaw_rate -v
python3 verification/yaw_rate/evaluate.py --binary /tmp/yaw_rate_trace \
  --mpcc-input /path/to/exact-mpcc-extraction \
  --comparison /path/to/yaw-quality/input \
  --bc-root /path/to/calibration-14night-01 --output /path/to/new-evidence
```

`--comparison` and `--bc-root` are optional. MPCC source origin/windows are
explicitly frozen to the latest 022604 recording; do not reuse them silently for
a different recording. The comparison NPZ contains EKF only for plot display.
The observer trace input contains only source timestamp and corrected gyro xyz.
The independent synthetic sweep selects the smallest tested q meeting the
specified .5/1/2/3 Hz rate gain/phase targets, verifies the binary default, and
reports rate AND alpha frequency response. Source matching, gaps, common masks,
edge exclusion, input/binary/script hashes and full traces remain inspectable.
SG21 residual includes true high-frequency motion/vibration, not sensor-only
noise. Bias-corrected gyro is not independent truth.

In sourced Humble, `runtime_replay.py --input /path/imu-quality-input.npz
--executable /path/yaw_rate --output /path/new-evidence` runs a standalone 20 s
1x receive-scheduled ROS replay in localhost domain195. Source stamps share one
offset preserving recorded source/receipt age. Actual scheduling/DDS adds jitter.
Checks include monotonic source subset, matching rate/alpha stamps, finite values
and independent graph. NPZ timestamps are separate int64 arrays to retain epoch
nanoseconds. The recorded P99 is the maximum reported rolling callback window
P99, not whole-run or hardware latency. No vehicle/controller process starts.

Package CTest adds `yaw_rate_core` and `yaw_rate_runtime` (domain194), including
source stamps, covariance semantics, graph independence, invalid input, outage,
gap recovery, clock epochs, launch opt-in and recording capture.
