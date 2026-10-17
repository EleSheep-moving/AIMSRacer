# 2026-10-15 estimator replay: LIO covariance and IMU FIR

[中文报告](2026-10-15-estimator-replay-comparison.zh-CN.md)

Two bounded experiments did not justify changing the production defaults. Increasing LIO longitudinal-velocity weight in the navigation EKF produced little typical change and no clear response gain on two calibration recordings. A causal five-tap IMU FIR reduced longitudinal-observer hold fluctuation, with an added sampling delay and mixed prediction results. These are recorded-data results, without new closed-loop or independent ground-truth acceptance.

This report adds the covariance-only and FIR experiments to the [earlier filter-tuning study](2026-10-14-longitudinal-filter-tuning.md). The **navigation EKF velocity output** and the **independent wheel/IMU longitudinal acceleration observer** are separate estimators. The latter has its own [parameters](../../src/aims_racer_system/params/longitudinal_acceleration.yaml) and [launch](../../src/aims_racer_system/launch/longitudinal_acceleration.launch.py); [race launch](../../src/aims_racer_system/launch/race.launch.py) enables it only through the optional `longitudinal_acceleration` flag, default `false`. Navigation [EKF acceleration publication](../../src/aims_racer_system/params/ekf_rear.yaml) remains `false`. This describes repository configuration, not a live-node inventory.

## Navigation EKF: changing LIO vx covariance

Two Oct 13 calibration bags, identified as `215303` and `222944`, were replayed on NX with `robot_localization` 3.5.4, at 1× speed. Each replay fed three EKFs the same sensor stream. Only LIO `twist.covariance[0]` changed: variance floors 0.04 / 0.02 / 0.01 gave actual rear-axle input variances 0.040009 / 0.020009 / 0.010009. Wheel variance 0.04, Q(vx) 0.4 and other saved settings stayed fixed. The retained windows were 220–430 s / 134–345 s, with at least 15 s warmup before displayed events.

There were **38 previously fixed events: 18 acceleration and 20 braking**, selected using absolute peaks ≥4 m/s² of the offline centered 0.21 s wheel-speed derivative. Selection did not depend on the candidates.

| Run | LIO floor | EKF−wheel RMS (m/s) | EKF−recorded EKF RMS (m/s) | Full-window fitted lag (ms) |
|---|---:|---:|---:|---:|
| 215303 | 0.04 | 0.04698 | 0.00438 | 30 |
| 215303 | 0.02 | 0.04704 | 0.00442 | 30 |
| 215303 | 0.01 | 0.04716 | 0.00455 | 30 |
| 222944 | 0.04 | 0.04650 | 0.00484 | 30 |
| 222944 | 0.02 | 0.04656 | 0.00485 | 30 |
| 222944 | 0.01 | 0.04663 | 0.00493 | 30 |

Source: [portable run table](assets/2026-10-15-estimator-replay/ekf-run-metrics.csv) and [summary](assets/2026-10-15-estimator-replay/ekf-summary.json). Compared with the fresh 0.04 replay, the 0.01 candidate changed vx by **0.001207 / 0.001125 m/s RMS**, with absolute P95 **0.001766 / 0.001760 m/s**. These typical changes are smaller than the baseline-to-recorded discrepancies above. Maximum instantaneous differences were **0.07375 / 0.07171 m/s**; effects were not zero everywhere. Parallel execution, simulated clock and callback ordering can contribute to those differences.

The fit is `EKF(t) ≈ gain × wheel(t−shift) + bias`, on a 5 ms source-time grid, searching −250…250 ms in 5 ms steps with common valid samples across shifts. Positive shift means EKF trails wheel; plotted curves were not shifted. Neither 30 ms nor its event-level counterpart is hardware delay, compute time or latency P95. Wheel speed is a fused input, not independent ground speed; approximately 10 Hz LIO source timestamps do not establish when measurements became available to the EKF. These calibration results cannot establish an improvement over earlier V35 tests with different excitation.

The archived checks report equal LIO fanout counts, positive-semidefinite input covariances, finite outputs, unchanged source-bag size/mtime and unchanged production defaults. Input capture fractions ranged from **99.22% to 100%**: reproduction was not exact. The isolated replay omitted driver/control command playback; TF and acceleration publication were disabled for the trial. A startup clock rollback cleared each TF buffer once in the second run; the retained events followed warmup. The NX archive records 39 verified hashes and a successful hash check.

## Longitudinal observer: five-tap IMU FIR

This experiment reused only the captured TUM FIR coefficients, not the complete TUM estimator. The captured source commit is `14ba6285d1cb8d0cd847a69af4e1981015d48b8e`; upstream file paths, hashes and coefficients are retained in the [portable FIR summary](assets/2026-10-15-estimator-replay/fir-summary.json). No third-party source implementation is bundled here.

The causal filter is `y[k] = Σ b[i] x[k−i]`, with coefficients `[0.0772218376, 0.2452233899, 0.3425608620, 0.2452233899, 0.0772218376]`, applied to rear-axle IMU longitudinal specific force before the existing observer update. Their unnormalized sum is **0.987451317**. At nominal 200 Hz, the symmetric filter has **10 ms group delay**; measured coefficient-weighted source sample age was P50 **9.97–9.98 ms**, P95 **10.60–10.68 ms** on the two calibration bags. These sample ages are not a measured physical sensor delay. Source/receipt timestamps were retained without advancing outputs. Four valid samples fill startup history; invalid input or a >30 ms gap resets it, and non-increasing source timestamps are skipped. This startup handling differs from the captured upstream zero-filled FIFO.

The previous `raw_selected` candidate used jerk PSD 30, disturbance PSD 0.01, wheel variance 0.0025 and force variance 16. The production `raw_baseline` used 30 / 0.1 / 0.01 / 16. `default_fir` adds FIR to the latter; `tuned_fir_r16` adds FIR to the former. The new search varied only force variance 16/8/4/2/1 on **21:53 training**, then checked **22:29 and six held-out MPCC runs**. R=16 passed the prior offline noise/mean constraints. R=8 scored slightly better on training forecasts but its maximum absolute hold mean **0.10658 m/s²** exceeded the prior 0.1 ranking bound. It remains a diagnostic candidate; the bound is not a runtime safety gate. FIR correlations prevent simply scaling observation variance by the fluctuation reduction.

| Run | Method | Hold output σ (m/s²) | 200 ms high-speed forecast RMSE (m/s) | Accel/brake alignment (ms) |
|---|---|---:|---:|---:|
| 215303 | raw_baseline | 0.1698 | 0.1786 | 60 / 70 |
| 215303 | default_fir | 0.1443 | 0.1805 | 60 / 70 |
| 215303 | raw_selected | 0.1662 | 0.1732 | 50 / 50 |
| 215303 | tuned_fir_r16 | 0.1393 | 0.1730 | 50 / 50 |
| 222944 | raw_baseline | 0.1695 | 0.1827 | 60 / 60 |
| 222944 | default_fir | 0.1402 | 0.1856 | 60 / 70 |
| 222944 | raw_selected | 0.1566 | 0.1748 | 50 / 50 |
| 222944 | tuned_fir_r16 | 0.1276 | 0.1755 | 50 / 50 |

Source: [calibration table, including R=8](assets/2026-10-15-estimator-replay/fir-calibration-metrics.csv). Matched 4–5 m/s hold input fluctuation fell from **1.1767→0.7260 / 1.2871→0.7929 m/s²**, about 38%. With default observer parameters, output fluctuation fell about 15.0–17.3%, while forecast RMSE increased and event alignment did not improve. Against `raw_selected`, R=16 reduced output fluctuation about 16–19%; the validation forecast changed **0.174793→0.175512 m/s**. Smoother output did not imply better prediction.

For held-out MPCC turning samples (`abs(yaw_rate)>0.8 rad/s`), all six R=16 200 ms forecast RMSEs increased **0.80–1.93%** relative to `raw_selected`:

| Run ID | Samples | raw_selected → tuned_fir_r16 RMSE (m/s) | Increase |
|---|---:|---:|---:|
| 134131 | 74 | 0.07019 → 0.07155 | 1.93% |
| 135222 | 353 | 0.07926 → 0.07989 | 0.80% |
| 135509 | 452 | 0.07844 → 0.07915 | 0.91% |
| 135727 | 536 | 0.08719 → 0.08796 | 0.87% |
| 141211 | 473 | 0.09863 → 0.09949 | 0.87% |
| 142308 | 480 | 0.12485 → 0.12620 | 1.09% |

Source: [full run identifiers and precise values](assets/2026-10-15-estimator-replay/fir-mpcc-turning.csv). Forecast is `v_wheel(t)+0.2 a_est(t)` versus `v_wheel(t+0.2)` on receipt time. The calibration high-speed cohort uses current speed ≥3 m/s and the preceding straight/moving mask, with **4,202 / 4,195 shared samples**. Hold σ pools within-segment demeaned variance weighted by sample count. Event alignment uses correlation on a 10 ms grid against a centered 0.21 s Savitzky–Golay wheel derivative that uses future samples. It is an offline proxy, not independent acceleration truth, physical delay, threshold crossing or a control latency measurement. Scene fluctuation includes vibration and actual motion; neither FIR nor this scalar model resolves gravity projection or wheel slip.

Archived verification covers **43 run/method combinations**, common prediction counts, input/kernel hashes, independent FIR convolution/gain/startup checks and no `numerical_failure`/`too_old` in filtered replays. NX has a checked evidence copy (96 verified hashes), not an independently executed or self-contained NX replay. Neither experiment changed default parameters, enabled navigation EKF acceleration, deployed FIR to the control chain, or established closed-loop benefit. No merge or deployment is claimed.

## Evidence and reproduction boundary

The six linked JSON/CSV files are small, portable summary exports. They contain no raw bags, traces or third-party source copies. [Source manifest](assets/2026-10-15-estimator-replay/source-manifest.json) records SHA256 of the exact source files inspected for this export; archived verification records are preserved as historical checks, not presented as a new replay in this documentation change.

Full local artifacts live outside this repository, relative to an `aimsracer-data` root:

- `sessions/2026-10-15/calibration-ekf-lio-vx-comparison/`: `replay_lio_r.py`, per-run saved YAML/logs/streams, `plot_comparison.py`, event tables and full figures.
- `sessions/2026-10-14/calibration-acceleration-peaks/tum-fir-trial/`: `fir_replay.py`, `study.py`, `selection.json`, filtered/replayed streams and captured upstream provenance. Those scripts depend on preceding extraction/review artifacts.

These are source locators, not repository-relative download links. Recomputing the studies requires those external inputs and their recorded dependencies; the published summary alone supports checking reported values and provenance, not replaying the full experiment.
