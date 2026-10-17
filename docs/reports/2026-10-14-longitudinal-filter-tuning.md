# 2026-10-14 longitudinal filter tuning and IMU propagation

[Full report in Chinese](2026-10-14-longitudinal-filter-tuning.zh-CN.md)

The existing linear Kalman observer with adjusted parameters gave the best trade-off in these offline comparisons. On the 22:29 validation run, straight high-speed 200 ms forecast RMSE decreased from 0.182728 to 0.174793 m/s (4.34%), and pooled 4–5 m/s hold fluctuation decreased from 0.169468 to 0.156627 m/s². Strong acceleration/braking alignment medians changed from 60/60 to 50/50 ms, relative to an offline wheel-speed derivative reference.

A separate IMU-driven velocity predictor with slower wheel corrections was implemented and tested, including joint parameter tuning. Its 50 Hz tuned setting produced 0.185177 m/s forecast RMSE and 0.264786 m/s² hold fluctuation, with 50/80 ms acceleration/braking alignment medians. None of the searched inertial configurations satisfied all training noise and mean constraints. This finding concerns the tested scalar model without independent attitude/gravity compensation, not all inertial fusion designs.

The KF candidate keeps jerk PSD 30 and force variance 16, changes disturbance PSD 0.1→0.01 and wheel variance 0.01→0.0025. The optional configuration（recorded workspace path：`src/aims_racer_system/params/longitudinal_acceleration_tuned_20261014.yaml`） is saved; the default, running nodes, navigation EKF and NX were not changed. The independent launch accepts a `config` argument for a later observation trial.

Parameters were selected on 21:53, then checked on 22:29 and six afternoon MPCC runs. Turning results and the LIO acceleration reference show some regressions; the candidate is not universally better. The supplemental slow-only restriction used training scores after the unrestricted validation had already been inspected. The full report retains this selection limitation.

Independent calculations verified forecast counts, shared cohorts and receipt-time availability for 40 run/method combinations. Existing numerical/replay tests passed (27 tests), and the new inertial kernel passed synthetic and independent matrix-oracle checks. These are offline numerical/proxy results, without new ROS/DDS runtime, deployment, closed-loop acceptance or independent acceleration ground truth.
