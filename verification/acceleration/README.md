# Longitudinal acceleration estimation

[简体中文](README.zh-CN.md)

This is an **offline, causal comparison**, isolated on `feat/longitudinal-acc-observer`. It consumes archived wheel speed, corrected rear-axle IMU specific force and optional LIO attitude. It produces scalar velocity/acceleration estimates and evidence. It has no ROS publishers, launch files or vehicle-command interfaces.

## Files

| File | Responsibility |
|---|---|
| [observer.py](observer.py) | Kalman observer, integer source times, immutable emitted estimates and delayed historical updates |
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
optional historical LIO gravity x ≈ d + noise
```

`a` estimates the derivative of body longitudinal speed. `d` includes body gravity, accelerometer bias and `(ω × v)x`; the last observation is an approximate prior, not an exact measurement of the whole disturbance. The recordings use identity Livox/base rotation. A different mounting needs an explicit rotation before this scalar input construction.

Each measurement retains its integer source stamp. Only after its recorded receipt can it enter the observer. A late observation is inserted into bounded historical measurements and the suffix is replayed. Current scalar estimates are predicted to the IMU receipt time. Previously emitted estimates never change. **No LIO pose or attitude is extrapolated, and latest IMU output does not wait for LIO.**

History retains at most 2 seconds and 4,096 measurements plus one posterior anchor. `count_trimmed`, `too_old` and numerical failure counters expose admission/retention effects. Snapshot prediction continues through input gaps; covariance grows. There is no new acceleration-to-motor path and no production EKF configuration change.

## Reproduce

Dependencies: Python 3.10, NumPy, SciPy, Matplotlib and pytest. The existing desktop image is `aimsracer-mpcc:main-dev-20261011`; the report records its image ID. On NX, use existing system Python. No ROS environment is required.

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

## Evidence limits

The two V35 runs are excluded from tuning. Wheel acceleration references use a centered 165 ms Savitzky–Golay derivative and zero-phase 3 Hz filter. Future wheel values are used **only for scoring**, never as observer inputs. These share the wheel sensor and cannot prove ground-truth acceleration accuracy or no wheel slip. Onset offsets are relative to that noncausal proxy, not measured hardware latency. LIO shares the MID360 IMU and is not an independent attitude truth source. Reported NX timings are serial offline computations, not concurrent FAST-LIO/EKF/NDT load or field acceptance.
