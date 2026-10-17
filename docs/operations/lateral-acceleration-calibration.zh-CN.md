# 横向加速度标定、AY 校对与横向附着

[English](lateral-acceleration-calibration.md) · [标定总入口](calibration.zh-CN.md) · [项目首页](../../README.zh-CN.md#calibration)

## 当前状态

“横向加速度标定”涉及三个不同环节：

| 环节 | 要确认的量 | 当前状态 |
|---|---|---|
| IMU 与安装标定 | 偏置、比例、轴向、IMU 到后轴／质心的外参 | 完整六姿态与三维外参未完成；IMU 比质心高约 3 cm，高差仍为估计值 |
| AY 输出质量校对 | 后轴／质心惯性加速度的 bias、动态误差、覆盖率与响应 | 独立五量已部署，可发布；Vicon 采集分析工具已准备，独立实车动态验收待测 |
| 横向附着／侧滑标定 | 左右转弯能力、侧滑临界与路面／轮胎附着 | 固定半径逐级升速流程仅有历史教程；旧自动脚本不是当前可用工具，不能认定附着上限已标定 |

[五量 NX 部署](../reports/2026-10-18-state-observer-nx-deployment.zh-CN.md)证明安装与输出链可用；[历史 AY 质量报告](../reports/2026-10-15-planar-acceleration-quality.zh-CN.md)记录单分量实现的相位和时间支持问题，两者不能替代独立动态误差测量。本页只整理现有用法和状态，没有恢复横向自动驾驶程序。

## 选择正确的 AY

当前独立后轴与质心加速度话题为：

- `/state_estimation/rear_axle/acceleration`：`base_link`，`linear.y` 为后轴 AY。
- `/state_estimation/center_of_mass/acceleration`：`center_of_mass`，`linear.y` 为质心 AY。

单位 m/s²，x 前、y 左、z 上。它们是惯性加速度在车体系的投影，经过单位、姿态／重力和刚体参考点处理；不能直接把原始 IMU y 比力改名为质心 AY。输入、补偿和滤波见[五量指南](state-observer.zh-CN.md)。

同点平面运动满足 `a_y=dot(v_y)+v_x*r`。只有横向速度变化项足够小时，`v_x*r` 才能作为相应近似；它不是独立真值。控制器的 `lateral_accel_limit` 是速度规划配置，也不是实测轮胎上限。

## 现有动捕校对流程

1. 在[动捕配置](mocap-validation.zh-CN.md)填写 Vicon 接入、时钟映射和标记到车体／后轴／质心的外参。不能用曲线最佳平移替代独立对时。
2. 同时录制 corrected IMU、两参考点五量与状态、raw/rear LIO、导航 EKF、轮速、最终命令和 VESC 反馈。动作沿用现场已有遥控或程序；校对工具不发布驾驶指令。
3. 用 `extract` 和 `analyze --task state`，或 `--task all` 生成报告。由标记位姿先换算目标点，再求世界位置二阶导数，旋转到车体系作为 AY 参考。
4. 比较 0.08／0.12／0.18 s 求导窗，主窗 0.12 s；主误差不平移、不去偏。静止、直线、转弯、S 弯分别统计，遮挡、缺口和支持不足保留可见。
5. 分开看 AY 精度、源时间年龄、波形偏移与命令到整车响应。中心求导只用于离线精度参考，不用于物理起点判断；本版没有预设实车合格线。

工具与测试见[交付报告](../reports/2026-10-18-mocap-validation.zh-CN.md)，[合成示例](../reports/assets/2026-10-18-mocap-validation/demo/report.zh-CN.md)仅演示输出，不是实车标定结果。

<a id="lateral-grip"></a>
## 横向附着标定：历史入口与限制

[历史中文教程](../reports/2026-10-10-calibration-guide-archive.md#historical-lateral-grip)和[英文教程](../reports/2026-10-10-calibration-guide-en-archive.md#historical-lateral-grip)保存固定半径圆、左右方向、逐级升速的设计，以及历史结果文件说明。

2026-10-18 核查：当前 NX 主安装没有 `lateral_grip_calib` 可用 launch／可执行入口。源码树残留的 Python 字节码缓存不等于可部署工具。旧教程中出现的自动启动命令是历史记录，不能作为已验证的当前流程。

在独立 AY／VX／VY／yaw rate、几何与同步尚未验证前，不根据 IMU 峰值或 `v_x*r` 直接填写新的 `mu_safe` 或横向加速度上限。当前没有新增横向附着系数，也没有因这些报告改动 MPCC 速度规划或约束。

[电流标定入口](calibration.zh-CN.md#current-calibration) · [仓库内附件索引](../reports/assets/README.zh-CN.md)
