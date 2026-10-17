# 近期实验精选与当前结论（截至 2026-10-18）

[电流标定总入口](../operations/calibration.zh-CN.md) · [横向加速度标定／AY 校对](../operations/lateral-acceleration-calibration.zh-CN.md)：包括现行流程、已完成结果与历史横向附着入口状态。

[附件总索引](assets/README.zh-CN.md)：所有已上传附件均可在仓库内浏览；来源记录中的原始路径不作为附件链接。

[English](latest-results.md) · [完整报告索引](README.zh-CN.md) · [文档首页](../README.zh-CN.md)

优先读下面四组，详细历史留在日期化报告中。本次整理汇集主机报告、NX 安装记录和外部实验结论，保留关键图表、指标与来源哈希；没有重新实验或调整车辆参数。

**本次提交只发布文档及小型证据，不提交工作区中尚未合入 main 的估计器、动捕、标定或定位实现。** “已部署”指报告注明日期、路径和哈希的 NX 安装快照；不表示全新 clone 已含对应源码。操作命令应在报告对应的已部署环境执行。原始 bag、完整重放数组、地图、bundle 和构建环境仍在外部 `aimsracer-data`，不会随文档上传。

## 1. 当前最关心：后轴／质心五量与 VY

| 优先读 | 核心结论 | 证据能支持到哪里 |
|---|---|---|
| [五量操作与计算链](../operations/state-observer.zh-CN.md) | 仅 corrected IMU＋raw LIO 位姿；不读取导航 EKF、轮速或 LIO twist；后轴和质心均发布 VX/VY/AX/AY/yaw rate | 独立于 Robot Localization 的输出分支；LIO 共享 IMU，不能当独立真值。MPCC 当前仍读取导航 VX，尚未切换。 |
| [VY 实包图表](2026-10-18-vy-quality.zh-CN.md) | 短静态后轴均值约 0.000650 m/s、std 0.002490 m/s；相对 LIO 跟随比导航 EKF 更及时，但快速波动和峰值更大 | 历史 bag 的新实现回放与共享传感器一致性；尚无 Vicon 动态精度、物理延迟或控制适用性验收。 |
| [NX 实际部署](2026-10-18-state-observer-nx-deployment.zh-CN.md) | ARM C++ 安装、30 组 CTest、约 222 s 实包回放；参考点换算最大残差约 8.64e−16 m/s；HI13 已退役 | 安装路径、ELF、参数及输出接口可复核；不是重新实车行驶。 |
| [低延迟与警告优先策略](2026-10-17-state-observer-latency-policy.zh-CN.md) | 位置创新经九维协方差约束速度与偏置，晚到 LIO 在历史源时刻更新并重放；年龄异常尽量警告 | 不把已发布滤波 AY/r 再接回速度积分；不能将曲线最佳平移当固定物理延迟。 |

IMU→质心名义偏移为 `[-0.09,-0.03,-0.03] m`；z 只是约 3 cm 的估计。AX/AY 动态相位、外参和 VY 峰值均待独立验证。

[首次双参考点软件验证](2026-10-17-dual-reference-state-validation.zh-CN.md)保留早期严格停发策略快照；[五量实现前质量审查](2026-10-17-state-output-quality.zh-CN.md)及各单分量报告保留历史缺陷和 PSD，不当作当前发布状态。

## 2. 下一次动捕：精度、时间与执行器响应

| 优先读 | 已准备 | 现场仍需确定 |
|---|---|---|
| [动捕工具交付](2026-10-18-mocap-validation.zh-CN.md) | C++ 时间采集；状态/FastLIO/执行器三项分析；固定 VRPN 源码；主机与 NX 各 51 项新测试＋19 项兼容测试 | Vicon 服务器/刚体、相机与 VRPN 时间语义、对时和标记外参。软件测试不等于现场精度通过。 |
| [现场配置和统一命令](../operations/mocap-validation.zh-CN.md) | inspect、record、extract、analyze、demo；仅采集分析；Ctrl-C 落盘 | 先填配置；动作沿用遥控、现有标定或 MPCC，不增加控制发布者。 |
| [可浏览的合成示例](assets/2026-10-18-mocap-validation/demo/report.zh-CN.md) | 三项中文报告、状态曲线、FastLIO 图、执行器事件与 CSV/JSON | **合成数据，仅演示工具输出**；没有 S 弯的示例会明确标为缺失。 |

源时间年龄、波形偏移、命令到 ERPM／整车运动的响应分别报告。无实际舵角反馈，不能独立认定纯舵机机械延迟。主误差不平移、不去偏；中心求导仅用于精度参考，不用于判定物理响应起点。

## 3. B/C 电流模型与现场标定

| 优先读 | 当前结果 | 使用限制 |
|---|---|---|
| [当前区间电流模型选型](2026-10-17-current-affine-selection.zh-CN.md) | 未补偿、带截距：B `a_bar=0.105147·I_bar−1.370669`，C `a_bar=0.129365·I_bar−1.250321`；留一档 RMSE 0.04186 / 0.04959 m/s² | 1–5 m/s 的筛选区间描述；B 10 档、C 9 档。不是瞬时加速度精度或已部署电流反解器，不能再先扣 basic。 |
| [电流响应与覆盖](2026-10-15-current-response-identification.zh-CN.md) | 保存最初采集覆盖、轮速响应、打滑与短平台问题 | 后续选型以 17 日报告为准；实测轮速加速度不等于无滑移车体加速度。 |
| [原生标定实际启动链](2026-10-16-native-calibration.zh-CN.md) | 持续节点、状态机、PP 与监控为 C++；核对实际标定 install | 与车辆主安装是不同工作区；部署软件验证和实车采集完成分开。 |
| [完整恒流端点协议](../operations/full-step-current.zh-CN.md) | 实测轮速端点、实际 Iq 交接、原有 inspect/start/stop | 历史采集协议说明；本次整理不启动或启用车辆。 |

## 4. MPCC、定位与延迟的关键结果

| 优先读 | 发现 | 结论边界 |
|---|---|---|
| [15 日 MPCC 连续圈](2026-10-15-mpcc-multilap-review.zh-CN.md) | 15 完整圈，原生速度均值/峰值 3.058/4.671 m/s，诊断横向误差 RMS 9.866 cm；重复弯外偏差仍存在 | 定位轨迹不是独立真值；顺序调参不构成同工况因果试验。 |
| [转向／横摆模型](2026-10-14-imu-steering-model.zh-CN.md) | 六次整跑留出 yaw-rate RMSE 0.167182→0.069622 rad/s | 改善离线横摆预测不证明闭环跟踪改善。 |
| [EKF 分量响应](2026-10-14-ekf-component-latency.zh-CN.md)与[滤波回放对照](2026-10-15-estimator-replay-comparison.zh-CN.md) | 响应和平滑存在权衡，未支持改变生产滤波默认值 | 源时间波形偏移不是独立物理延迟，也不预设 Robot Localization 正确。 |
| [FastLIO 加速度比例完整回放](2026-10-17-fastlio-accel-scale.zh-CN.md) | x9.81 相对 x10 的 RUNNING 位置差 P95 约 9.976 mm、VX 差 P95 0.02551 m/s；没有提速收益 | 同一实现的缩放策略比较，不是独立定位误差或多算法公平基准。 |
| [NX 定位 CPU 诊断](2026-10-17-localization-cpu.zh-CN.md)与[NDT 5 Hz 部署](2026-10-17-ndt-5hz.zh-CN.md) | 单线程省少量 CPU 但增加尾部耗时；后续选择 5 Hz，历史回放约省 35% NDT CPU，观测年龄 P95 223→308 ms | 5 Hz 是 CPU/新鲜度折中，不是单帧加速；隔离短回放不代表整车并发负载。 |

## 查证与维护

- 先读当前结果，再沿报告进入仓库内 PNG、CSV、JSON 和安装回执。原始实验内容与历史口径不追改。
- [本次整理来源清单](assets/2026-10-18-documentation/source-manifest.json)记录导入文件的来源路径与整理前哈希。正文将仓库外文件明确写成外部路径，避免 GitHub 上的悬空证据链接。
- 新实验应新增日期化报告并更新本页，不能用后续结果悄悄覆盖原始统计。
