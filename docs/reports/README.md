# Experiment reports

These are dated evidence, not current operating instructions. Keep their sample
conditions, source assumptions and limitations with the results. Current behavior
is maintained in [architecture](../architecture.md) and outstanding vehicle work
in the [checklist](../operations/vehicle-checklist.md).

| Report | Scope |
| --- | --- |
| [2026-10-10 main 场地栈整合](2026-10-10-main-field-stack-integration.md) | 实跑源码/参数核对、纯 native 安装入口、默认 v35 bundle 与软件验证边界 |
| [2026-10-10 field review and lap-time priorities](2026-10-10-mpcc-field-review.md) | Latest NX source audit, all field sessions, raw-bag completion checks, solver timing, speed-profile saturation and primary racing-source comparison |
| [2026-10-10 speed boundaries and clock ordering](2026-10-10-mpcc-speed-gates.md) | Field interventions, speed overshoot rejection, clock fix and progressively faster configurations |
| [2026-10-10 independent acceleration experiment](2026-10-10-mpcc-independent-acceleration.md) | Explicit v2 combined-envelope removal with retained actuator bounds |
| [2026-09-20 engineering review](2026-09-20-engineering-review.md) | Synthetic estimator diagnostics and controller limitations |
| [2026-09-21 frame check](2026-09-21-frame-check.md) | Stationary powered-vehicle frame/latency observations with the earlier local FAST-LIO modification |
| [2026-09-28 vehicle response](2026-09-28-vehicle-response.md) | Bag-derived speed onset, steering command path, IMU yaw response and MPCC single-lag fit |
| [2026-09-28 speed-mode calibration](2026-09-28-speed-mode-calibration.md) | New speed-only bag: speed scale, command-to-motion timing, steering lag and LIO age |
| [2026-09-28 map reference](2026-09-28-map-reference.md) | Closed-lap extraction and matching to saved PGO map poses |
| [2026-10-04 premature plan expiry](2026-10-04-plan-expiry.md) | Historical 750 ms / 10 cm configuration; future forecast versus actual plan expiry |
| [2026-10-04 LIO delay and EKF yaw](2026-10-04-lio-delay.md) | Incident latency, two-second EKF replay, rotation Jacobian and matching-cost evidence; cumulative backlog remains unexplained |
| [2026-10-05 LIO worker and catch-up](2026-10-05-fastlio-thread.md) | Joined worker, preserved IMU history, deskew boundaries and isolated desktop replay acceptance |
| [2026-10-05 FAST-LIO + NDT integration](2026-10-05-fastlio-ndt-integration.md) | Historical online-crop integration and subsequent source-time health / hold validation; read target audit for current target policy |
| [2026-10-05 NDT target audit](2026-10-05-ndt-target-audit.md) | Upstream Jetson guidance, fixed full target, map-load warm-up and NX A/B replay evidence |
| [2026-10-05 field test](2026-10-05-field-analysis/field-report.md) | First automatic failure, completed low-speed lap, recording gap and original diagnostics |
| [2026-10-06 analysis handoff](2026-10-06-analysis-handoff.md) | Start here for last night's field analysis, today's changes, evidence locations and branch docs merge notes |
| [2026-10-06 EKF speed response](2026-10-06-ekf-speed-response.md) | NX actual EKF replay, Q(vx) correction, response/smoothness trade-off and held-out data |
| [2026-10-06 MPCC weights and oscillation](2026-10-06-mpcc-weight-response.md) | Field checks and single-variable numerical experiments; weight candidates remain undeployed |
| [2026-10-06 iteration budget](2026-10-06-iteration-budget.md) | Offline 30/60/100 iteration comparison and fixed-initial-state constraint diagnosis |
| [2026-10-06 shared gyro bias correction](2026-10-06-gyro-bias-correction.md) | Shared rear-adapter correction, old-bag replay, actual NX stationary hardware capture and regression evidence |
| [2026-10-06 NX workspace synchronization](2026-10-06-nx-workspace-sync.md) | Preserved NX regression sources, current C++ test interfaces, branch-specific synthetic startup and desktop/NX/GitHub synchronization |

Future reports should identify source/configuration versions, hardware, workload,
measurement definitions, sample count and evidence location. Generated local
artifacts must be identified as unavailable from a fresh checkout when applicable.
