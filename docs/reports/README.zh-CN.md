<a id="experiment-reports"></a>
# 实验报告

[仓库内附件总索引 / Evidence index](assets/README.zh-CN.md)

[English](README.md) · **简体中文**

这些是注明日期的证据，不是当前操作说明。应随结果保留其采样条件、源码假设与局限。当前行为见[架构](../architecture.zh-CN.md)，尚待实车完成的工作见[检查清单](../operations/vehicle-checklist.zh-CN.md)。

| 报告 | 范围 |
| --- | --- |
| [电流与横向加速度标定入口](../operations/calibration.zh-CN.md) | 现行电流协议、AY 动捕校对及横向附着历史状态 |
| [横向附着旧教程](2026-10-10-calibration-guide-archive.md#historical-lateral-grip) | 固定半径逐级升速设计归档；旧脚本非当前可用入口 |
| [近期实验精选（先读）](latest-results.zh-CN.md) | 四组重要结论、版本范围、独立精度边界与现场入口 |
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
| [2026-10-15 VX 单分量质量](2026-10-15-vx-quality.zh-CN.md) | 历史 VX 计算、回放证据；当前双参考点输出另见 18 日报告 |
| [2026-10-15 车体加速度质量](2026-10-15-body-acceleration-quality.zh-CN.md) | AX／三维补偿、静态 Allan、运行 PSD 与动态精度限制 |
| [2026-10-15 平面加速度质量](2026-10-15-planar-acceleration-quality.zh-CN.md) | AY、杆臂补偿、相位与异常时间缺陷的历史记录 |
| [2026-10-15 yaw rate 质量](2026-10-15-yaw-rate-quality.zh-CN.md) | gyro 滤波、静态／频域和相对响应 |
| [2026-10-14 纵向滤波对照](2026-10-14-longitudinal-filter-tuning.zh-CN.md) | 数据/窗口口径与滤波响应；不是默认参数修改授权 |
| [2026-10-13 VESC 新控制消息](2026-10-13-vesc-native-command.md) | 中控/驱动/MPCC 迁移、单调时钟保护及仅虚拟串口的软件验收 |
| [2026-10-12 加速度实时节点](2026-10-12-longitudinal-acc-runtime.zh-CN.md) | 可选安装节点、主机/NX 测试及实时 DDS 回放边界 |
| [2026-10-12 加速度对照](2026-10-12-longitudinal-acc-observer.zh-CN.md) | 八次录包、因果轮速/IMU 估计器与保留包预测证据 |
| [2026-10-11 后轴 IMU 源时间](2026-10-11-rear-imu-source-time.zh-CN.md) | 独立实时比力与保留 LIO 源时间的后轴里程计 |
| [2026-10-10 main 场地栈整合](2026-10-10-main-field-stack-integration.md) | 实跑源码/参数核对、纯 native 安装入口、默认 v35 bundle 与软件验证边界 |
| [2026-10-10 场地复盘与圈时优先项](2026-10-10-mpcc-field-review.md) | 最新 NX 源码核查、全部场地 session、原始 bag 完成检查、求解时序、速度规划饱和与竞速原始源码比较 |
| [2026-10-10 速度边界与时钟顺序](2026-10-10-mpcc-speed-gates.md) | 场地干预、速度超调拒绝、时钟修复和逐步提速配置 |
| [2026-10-10 独立加速度实验](2026-10-10-mpcc-independent-acceleration.md) | 显式移除 v2 组合包络，保留执行器边界 |
| [2026-09-20 工程审查](2026-09-20-engineering-review.md) | 合成估计器诊断与控制器局限 |
| [2026-09-21 坐标系检查](2026-09-21-frame-check.md) | 通电静止车辆的坐标系/延迟观察，采用较早的本地 FAST-LIO 修改 |
| [2026-09-28 车辆响应](2026-09-28-vehicle-response.md) | 从 bag 提取的速度起始、转向命令链、IMU 横摆响应和 MPCC 单滞后拟合 |
| [2026-09-28 速度模式标定](2026-09-28-speed-mode-calibration.md) | 新的仅速度 bag：速度尺度、命令到运动时序、转向滞后与 LIO 年龄 |
| [2026-09-28 地图参考](2026-09-28-map-reference.md) | 闭圈提取并匹配已保存的 PGO 地图位姿 |
| [2026-10-04 计划提前过期](2026-10-04-plan-expiry.md) | 历史 750 ms / 10 cm 配置；未来预测与实际计划过期的区别 |
| [2026-10-04 LIO 延迟与 EKF 航向](2026-10-04-lio-delay.md) | 故障延迟、两秒 EKF 回放、旋转 Jacobian 与匹配代价证据；累计积压仍未解释 |
| [2026-10-05 LIO 工作线程与追赶](2026-10-05-fastlio-thread.md) | joined 工作线程、完整 IMU 历史、去畸变边界与隔离桌面回放验收 |
| [2026-10-05 FAST-LIO + NDT 整合](2026-10-05-fastlio-ndt-integration.md) | 历史在线裁剪整合及后续源时间健康/hold 验证；当前 target 策略见 target 核查 |
| [2026-10-05 NDT target 核查](2026-10-05-ndt-target-audit.md) | 上游 Jetson 指南、固定完整 target、地图加载预热与 NX A/B 回放证据 |
| [2026-10-05 场地试验](2026-10-05-field-analysis/field-report.md) | 首次自动失败、完整低速圈、录制缺口与原始诊断 |
| [2026-10-06 分析交接](2026-10-06-analysis-handoff.md) | 昨晚场地分析、今日修改、证据位置与分支文档合并说明的阅读入口 |
| [2026-10-06 EKF 速度响应](2026-10-06-ekf-speed-response.md) | NX 实际 EKF 回放、Q(vx) 修正、响应/平滑取舍与独立数据 |
| [2026-10-06 MPCC 权重与摆动](2026-10-06-mpcc-weight-response.md) | 场地核查与单变量数值实验；权重候选尚未部署 |
| [2026-10-06 迭代预算](2026-10-06-iteration-budget.md) | 离线 30/60/100 次迭代比较与固定初值约束诊断 |
| [2026-10-06 共享 gyro 零偏修正](2026-10-06-gyro-bias-correction.md) | 共享后轴适配器修正、旧 bag 回放、NX 真实静止硬件采集与回归证据 |
| [2026-10-06 NX 工作目录同步](2026-10-06-nx-workspace-sync.md) | 保留 NX 回归源码、当前 C++ 测试接口、分支专属合成启动与桌面/NX/GitHub 同步 |

后续报告应标明源码/配置版本、硬件、工作负载、测量定义、样本量与证据位置。若生成的本地产物无法从全新 checkout 获得，应明确说明。
