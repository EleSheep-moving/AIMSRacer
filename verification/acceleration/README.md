# Longitudinal acceleration estimation

[简体中文](README.zh-CN.md)

This directory contains the **offline, causal comparison** and a separate ROS/DDS runtime replay harness. Offline replay consumes archived wheel speed, corrected rear-axle IMU specific force and optional LIO attitude, producing scalar estimates and evidence. The installed wheel + IMU runtime lives in `aims_racer_system`, uses the same observer core, and is opt-in through `longitudinal_acceleration:=true` (default `false`). See the [runtime operation guide](../../docs/operations/longitudinal-acceleration.md).

## Files

| File | Responsibility |
|---|---|
| [observer.py](observer.py) | Offline compatibility import of the [canonical installed core](../../src/aims_racer_system/aims_racer_system/longitudinal_observer.py): integer source times, immutable emitted estimates and delayed historical updates |
| [ROS wrapper](../../src/aims_racer_system/aims_racer_system/longitudinal_acceleration.py) | Dedicated wheel + IMU node, acceleration/status messages, initialization and source-age diagnostics |
| [runtime_replay.py](runtime_replay.py) | Actual DDS delivery and receipt-order oracle comparison using the ROS wrapper, without vehicle drivers |
| [replay.py](replay.py) | Hash-verified input loading, receipt-order replay, training-only parameter selection |
| [evaluate.py](evaluate.py) | Separate offline references, metrics, plots and artifact-only plot regeneration |
| [test_observer.py](test_observer.py) | Numerical, clock, history, nuisance and synthetic-truth tests |
| [test_pipeline.py](test_pipeline.py) | Whole-replay causality, receipt holding, reference gaps and onset metric regressions |
| [Implementation plan](../../docs/plans/2026-10-12-longitudinal-acc-observer.md) | Approved scope and task checklist |
| [Comparison report](../../docs/reports/2026-10-12-longitudinal-acc-observer.md) | Results, interpretation and remaining validation |

## Model and timing

Wheel-only state is `[v,a]`. Fusion state is `[v,a,d]`:

```text
dv/dt = a
da/dt = white jerk
dd/dt = random walk
wheel speed = v + noise
rear specific force x = a + d + noise
optional historical apparent-gravity x (-g_body,x) ≈ d + noise
```

Specific force is `f = a_inertial − g`, so restoring inertial acceleration uses `a_inertial = f + g` in the same frame. In a z-up world `g = [0,0,−9.80665] m/s²`; resting accelerometers read approximately +9.81 m/s² in z. Observer state `a` estimates `d(body_vx)/dt`, not the full inertial acceleration vector. `d` includes `−g_body,x`, accelerometer bias and `(ω × v)_x`; the last observation is an approximate prior, not an exact measurement of the whole disturbance. The recordings use identity Livox/base rotation. A different mounting needs an explicit rotation before this scalar input construction.

Each measurement retains its integer source stamp. Only after its recorded receipt can it enter the observer. A late observation is inserted into bounded historical measurements and the suffix is replayed. Current scalar estimates are predicted to the IMU receipt time. Previously emitted estimates never change. **No LIO pose or attitude is extrapolated, and latest IMU output does not wait for LIO.**

History retains at most 2 seconds and 4,096 measurements plus one posterior anchor. `count_trimmed`, `too_old` and numerical failure counters expose admission/retention effects. Snapshot prediction continues through input gaps; covariance grows. There is no new acceleration-to-motor path and no production EKF configuration change.

## Reproduce the offline comparison

Dependencies: Python 3.10, NumPy, SciPy, Matplotlib and pytest. The existing desktop image is `aimsracer-mpcc:main-dev-20261011`; the report records its image ID. On NX, use existing system Python. The offline comparison needs no ROS environment; the separate runtime replay below requires sourced ROS and the installed package.

```bash
python3 -m pytest -p no:cacheprovider verification/acceleration -q
python3 verification/acceleration/replay.py \
  --inputs /path/to/2026-10-11/longitudinal-acceleration \
  --forces /path/to/2026-10-11/imu-source-time-alignment \
  --output /path/to/new-evidence --jobs 4
```

The input loader requires eight completed field runs, matching extraction SHA256 values, corrected-force replay hashes and source-time metadata. It retains 10 seconds before `RUNNING` and 3 seconds after `COMPLETE`. Corrected force is the frozen production-core replay from the source-time adapter investigation; it is not recalculated using estimated LIO orientation.

To reproduce selected settings without tuning:

```bash
OPENBLAS_NUM_THREADS=1 python3 verification/acceleration/replay.py \
  --inputs /path/to/2026-10-11/longitudinal-acceleration \
  --forces /path/to/2026-10-11/imu-source-time-alignment \
  --output /path/to/nx-evidence \
  --selected-configs /path/to/selected-configs.json
python3 verification/acceleration/evaluate.py --evidence /path/to/nx-evidence
```

Outputs include `inputs.json`, input metadata hashes, all training trials, frozen settings, per-run `metrics.csv`, timestamped estimates and English 16:9 PNG plots. Estimator times are floating elapsed seconds relative to the stored integer `origin_ns`, preserving small source/receipt differences. JSON metric files use Python's `NaN` for unavailable metrics; the CSV also exposes missing values. They should not be silently converted to zero.

## Separate ROS runtime replay

Build/install `aims_racer_system`, then source ROS 2 Humble and `install/setup.bash`. This harness starts the observer wrapper and input/output probes only. It publishes archived wheel/force events approximately in real time, maps integer source times into the current ROS epoch, and checks emitted estimates against the canonical observer using actual callback order and each output timestamp. Run in its required isolated localhost domain:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 \
  python3 verification/acceleration/runtime_replay.py \
  --inputs /path/to/2026-10-11/longitudinal-acceleration \
  --forces /path/to/2026-10-11/imu-source-time-alignment \
  --output /path/to/new-runtime-evidence \
  --run 193524_v35_n15_accel1_cap4_lap02
```

It requires the same frozen input hashes as offline replay and writes `runtime-replay.json`, `outputs.jsonl` and `status.json`. Checks include DDS input/output counts, frame/covariance semantics, oracle equality, timestamp ages, callback costs and absence of `/drive` or `/tf` publishers in that isolated graph. These are wrapper/delivery checks; they do not establish physical acceleration accuracy, hard deadlines, localization joint load or field acceptance. The installed node has no LIO input, navigation EKF fusion or vehicle-command path. Age warnings only report data freshness, and published acceleration covariance is uncalibrated model uncertainty.

## Evidence limits

The two V35 runs are excluded from tuning. Wheel acceleration references use a centered 165 ms Savitzky–Golay derivative and zero-phase 3 Hz filter. Future wheel values are used **only for scoring**, never as observer inputs. These share the wheel sensor and cannot prove ground-truth acceleration accuracy or no wheel slip. Onset offsets are relative to that noncausal proxy, not measured hardware latency. LIO shares the MID360 IMU and is not an independent attitude truth source. Reported NX timings are serial offline computations, not concurrent FAST-LIO/EKF/NDT load or field acceptance.
