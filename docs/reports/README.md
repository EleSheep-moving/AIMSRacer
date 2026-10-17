<a id="实验报告"></a>
# Experiment reports

**English** · [简体中文](README.zh-CN.md)

These are dated evidence, not current operating instructions. Keep their sample
conditions, source assumptions and limitations with the results. Current behavior
is maintained in [architecture](../architecture.md) and outstanding vehicle work
in the [checklist](../operations/vehicle-checklist.md).

| Report | Scope |
| --- | --- |
| [Selected recent results (start here)](latest-results.md) | State outputs, mocap, calibration and localization evidence; detailed recent reports in Chinese |
| [2026-10-18 VY 质量与图表](2026-10-18-vy-quality.zh-CN.md) | 短静态、共享传感器动态一致性、后轴／质心比较与待验证项 |
| [2026-10-18 独立估计器 NX 部署](2026-10-18-state-observer-nx-deployment.zh-CN.md) | ARM 安装、回放输出、参考点恒等式、HI13 退役 |
| [2026-10-18 动捕校对工具](2026-10-18-mocap-validation.zh-CN.md) | 状态／FastLIO／执行器三项工具、测试、时间边界与配置 |
| [2026-10-17 五量计算与发布策略](2026-10-17-state-observer-latency-policy.zh-CN.md) | 三维惯导、位置约束、源时间重放和警告优先 |
| [2026-10-17 双参考点首次验证](2026-10-17-dual-reference-state-validation.zh-CN.md) | 首次后轴／质心五量软件证据；严格停发是历史策略 |
| [2026-10-17 五量质量历史快照](2026-10-17-state-output-quality.zh-CN.md) | 新双参考点实现前的统计、PSD、计算与精度边界 |
| [2026-10-17 Vicon 比较准备](2026-10-17-vicon-comparison-preparation.zh-CN.md) | 独立参考、时间、外参与求导验证；后续工具见 18 日 |
| [2026-10-17 B/C 当前模型选型](2026-10-17-current-affine-selection.zh-CN.md) | 未补偿带截距、B 十档／C 九档与有限支持域 |
| [2026-10-17 FastLIO 加速度比例](2026-10-17-fastlio-accel-scale.zh-CN.md) | 完整多圈包缩放 A/B、重复性、计算时间与非真值边界 |
| [2026-10-17 NDT 5 Hz 部署](2026-10-17-ndt-5hz.zh-CN.md) | CPU 与观测新鲜度折中、实际加载安装 |
| [2026-10-17 NX 定位 CPU](2026-10-17-localization-cpu.zh-CN.md) | 线程与 OpenMP 等待策略实测；调整频率前历史快照 |
| [2026-10-16 原生标定链](2026-10-16-native-calibration.zh-CN.md) | 实际标定安装、C++ 状态机／PP／监控与隔离测试 |
| [2026-10-15 多圈 MPCC](2026-10-15-mpcc-multilap-review.zh-CN.md) | 15 完整圈、跟踪偏差、原生计划与命令计时 |
| [2026-10-15 电流响应辨识](2026-10-15-current-response-identification.zh-CN.md) | 覆盖、轮速模型、留出与打滑；当前系数见 17 日 |
| [2026-10-15 估计器回放对照](2026-10-15-estimator-replay-comparison.zh-CN.md) | 协方差／FIR 负结果与保留生产默认值 |
| [2026-10-15 原生状态节点](2026-10-15-native-state-nodes.zh-CN.md) | 传感器适配与观测器的 C++ 迁移软件证据 |
| [2026-10-14 EKF 分量响应](2026-10-14-ekf-component-latency.zh-CN.md) | 位置／yaw／速度波形响应与时间口径 |
| [2026-10-14 IMU 转向辨识](2026-10-14-imu-steering-model.zh-CN.md) | yaw-rate 留出预测、τ/K 与闭环证据边界 |
| [2026-10-13 恒速电流／电压](2026-10-13-steady-current-voltage-model.zh-CN.md) | Stage A 锚点、avg_iq 与电压能力分开 |
| [2026-10-13 VESC native command](2026-10-13-vesc-native-command.md) | Native message, central/driver/MPCC migration and software-only PTY acceptance |
| [2026-10-12 acceleration runtime](2026-10-12-longitudinal-acc-runtime.md) | Opt-in installed observer, desktop/NX tests and real-time DDS replay limits |
| [2026-10-12 acceleration comparison](2026-10-12-longitudinal-acc-observer.md) | Eight recorded runs, causal wheel/IMU observers and held-out forecast evidence |
| [2026-10-11 rear IMU source time](2026-10-11-rear-imu-source-time.md) | Independent current force and source-stamped LIO rear odometry |
| [2026-10-10 main field-stack integration](2026-10-10-main-field-stack-integration.md) | Field-run source/parameter audit, native-only installation entry, default v35 bundle and software-verification limits |
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
