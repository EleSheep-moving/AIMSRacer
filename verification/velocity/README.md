# Independent velocity verification

[简体中文](README.zh-CN.md) · [Operation](../../docs/operations/velocity.md) · [中文质量报告](../../docs/reports/2026-10-15-vx-quality.zh-CN.md)

Production code is C++ (`velocity.hpp`, `velocity.cpp`, `velocity_node.cpp`). `observer.py` is an offline numerical reference, never a long-running vehicle node. No wheel input exists in the observer API. Optional wheel plotting happens after configuration freezing.

Tools:

| File | Purpose |
|---|---|
| `extract.py` | Read original ROS2 SQLite CDR in receipt/row order; retain full 3D values, source/receipt stamps, all covariances, frames and selected-CDR/NPZ hashes |
| `observer.py`, `test_observer.py` | Nine-state reference and deterministic fault/initialization regressions |
| `synthetic.py` | Fixed-seed analytic 3D truth, nine predeclared noise candidates, gain/phase/RMSE gating and frozen configuration |
| `evaluate.py` | B/C/MPCC replay in original receipt order and input quality summaries; does not load wheel arrays |
| `native_acceptance.py`, `compare.py` | Native/reference synthetic and full-event parity; exact discrete states, continuous tolerance 1e−8 |
| `ros_replay.py` | Full 1× selected-sensor ROS transport replay; a common source/receipt time shift preserves original ages; compare the entire expected output sequence |
| `sensitivity.py` | Frozen-model ±20 ms LIO timestamp and mounting/lever perturbations; differences are not truth errors |
| `report.py` | Standalone figures for source-time/arrival-time LIO baselines and optional previously computed pure ax/wheel diagnostics |

Use Python3 with NumPy, SciPy, matplotlib and pytest; extraction/ROS replay also require ROS Humble Python messages. Production build dependencies are unchanged. Example from the repository root:

```bash
OPENBLAS_NUM_THREADS=1 python3 -m pytest -q verification/velocity/test_observer.py
OPENBLAS_NUM_THREADS=1 python3 verification/velocity/synthetic.py --output /new/external/synthetic
python3 verification/velocity/extract.py --help
python3 verification/velocity/evaluate.py --help
```

In a sourced ROS Humble shell, build/test `aims_racer_system` using the workspace's existing CMake/colcon workflow. New CTest targets are `velocity_core_test` (eleven native regressions) and `velocity_runtime` (three actual ROS/launch contracts). `velocity_trace` is a test executable, not a vehicle node.

A complete native field replay:

```bash
/path/to/build/velocity_trace /external/offline/mpcc/events.txt > /external/native-mpcc.csv
python3 verification/velocity/compare.py /external/offline/mpcc/trace.csv /external/native-mpcc.csv
```

Use a separate ROS domain with no vehicle stack for the full transport replay:

```bash
ROS_DOMAIN_ID=199 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 \
python3 verification/velocity/ros_replay.py \
  --input /external/inputs/mpcc/input.npz \
  --expected /external/native-mpcc.csv \
  --executable /path/to/build/vx_observer \
  --output /new/external/ros-replay
```

The replay publishes only corrected IMU and raw LIO pose. It retains original receipt order, intervals, source timestamps and LIO lag after a common wall-clock shift. It records all sent events, numerical outputs, diagnostics, schedule lateness and sampled observer RSS. It asserts the complete expected sequence, not a count-only or short-window comparison. Arrival-order differences can affect numerical corrections and must be reported, not hidden by rewriting stamps.

Artifacts live outside Git under `aimsracer-data/experiments/vx-observer-20261015/`. The dated report identifies the accepted runs, input hashes, limits, completed NX software acceptance and the outstanding Vicon gate. A source build or synthetic pass does not establish measured vehicle speed accuracy.

When NX connectivity is restored, run `resume_nx.sh /new/external/nx-results` with `VELOCITY_PYTHON` set to a Python with NumPy. It copies into `/home/aims/aimsracer-vx-observer-20261015`, builds `build-final`/`install-final`, runs the entire package suite and full B/C/MPCC native parity, then the full 1× MPCC ROS replay. It also computes original bag hashes. It never sources or replaces the vehicle installation. Its complete execution passed on NX on 2026-10-16: 27/27 CTest groups, B/C/MPCC exact discrete parity with maximum Vx difference 1.13e-14 m/s, and all 42,255 expected outputs from the full 1× MPCC replay. Evidence is linked from `accepted/nx`.
