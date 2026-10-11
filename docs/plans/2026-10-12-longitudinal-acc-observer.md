# Longitudinal acceleration observer implementation plan

**Goal:** Compare causal wheel-speed acceleration estimation, wheel/IMU fusion and delayed source-time LIO gravity constraints on the same eight completed field recordings. Produce reproducible numerical evidence before any runtime integration.

**Authorization:** The user approved starting the three-way estimation comparison and explicitly excluded the acceleration-to-motor execution chain. Existing main/NX rear-adapter fixes remain untouched. This experiment is on `feat/longitudinal-acc-observer` in the established `aimsracer-data/code` layout.

**Architecture:** A pure NumPy observer uses `[v,a]` for wheel-only and `[v,a,d]` for force fusion, where `a=d(vx)/dt` and `d` is effective gravity/bias/neglected rotation disturbance. White jerk drives acceleration; disturbance is a random walk. Optional LIO measures historical gravity as a noisy constraint on d, with unmodelled accelerometer bias and rotation explicitly acknowledged. A bounded fixed-lag measurement history inserts delayed source-time observations and replays to current state; previous emitted estimates are immutable. No LIO pose prediction, driving publishers or production estimator changes.

**Evidence:** Original SHA-verified NPZ streams from 2026-10-11 and corrected-force replay from the rear-adapter fix. Events are consumed in original bag receipt order, with integer source stamps. Output is predicted longitudinal state at current bag receipt time, not a newly acquired measurement. VESC stamps receipt rather than hardware acquisition, so cross-sensor timing uncertainty remains.

**Validation:** Synthetic truth tests cover constant acceleration, finite covariance, measurement gaps, tilted force, delayed observations, history retention and no lookahead. Field metrics include stationary bias/noise, >3 Hz residual proxy, causal velocity forecasts at +0.1/+0.2 s and onset lag versus explicitly noncausal wheel reference. First six runs select settings; the two V35 recordings are held out. Wheel-derived references share the baseline sensor and cannot establish independent vehicle acceleration accuracy. Actual phase acceleration has lateral rotation terms; the scalar model must report that limit.

**Tech stack:** Python 3.10, NumPy, SciPy, Matplotlib in the existing pinned desktop Docker image. NX only runs offline numerical checks and timing; no ROS or vehicle drivers.

## Tasks

- [ ] 1. Implement and test `verification/acceleration/observer.py` and `test_observer.py`. Use immutable configuration, scalar Kalman measurements, Joseph covariance updates, nonfinite rejection and fixed-lag replay. Test before implementation and save failing/passing logs.
- [ ] 2. Implement `verification/acceleration/replay.py`: verify raw input hashes, preserve source stamps, reconstruct corrected force provenance, define training/held-out roles and emit current-time estimates in receipt order.
- [ ] 3. Implement `verification/acceleration/evaluate.py`: construct offline proxies separately from causal inputs, score per-run noise/forecast/onset/residual tradeoffs, select parameters using training runs only and report held-out results without retuning.
- [ ] 4. Generate standalone English 16:9 plots and bilingual reports in `aimsracer-data/sessions/2026-10-12/longitudinal-acc-observer`. Include all settings, covariances, clock limits and original source hashes. Explicitly state whether IMU actually improves the baseline.
- [ ] 5. Run independent spec and code-quality review. Resolve findings, repeat only affected verification and commit this experiment branch. Preserve main/NX production source.
- [ ] 6. Run selected offline observer configurations and timing on NX if reachable; mark target verification separately. Document remaining field acceptance and integration work. No live driving or actuator work.

## Commands

Run unit tests in the existing image with the experiment worktree mounted at `/repo`:

```bash
python3 -m pytest -p no:cacheprovider /repo/verification/acceleration/test_observer.py -q
python3 /repo/verification/acceleration/replay.py --inputs /data/sessions/2026-10-11/longitudinal-acceleration --forces /data/sessions/2026-10-11/imu-source-time-alignment --output /data/sessions/2026-10-12/longitudinal-acc-observer
python3 /repo/verification/acceleration/evaluate.py --evidence /data/sessions/2026-10-12/longitudinal-acc-observer
```

Expected verification is numerical and methodological consistency, not a predetermined winner or an unmeasured closed-loop PASS.
