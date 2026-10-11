<a id="experiment-reports"></a>
# 实验报告

[English](README.md) · **简体中文**

这些是注明日期的证据，不是当前操作说明。应随结果保留其采样条件、源码假设与局限。当前行为见[架构](../architecture.zh-CN.md)，尚待实车完成的工作见[检查清单](../operations/vehicle-checklist.zh-CN.md)。

| 报告 | 范围 |
| --- | --- |
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
