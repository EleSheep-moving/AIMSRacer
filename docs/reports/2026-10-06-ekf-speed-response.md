# NX EKF 纵向速度额外响应滞后调查与修正

日期：2026-10-06。执行位置：Orin NX。仓库：`/home/aims/AIMSRacer-fastlio-ndt`，分支 `feat/fastlio-ndt-mpcc`，基线提交 `0a5b9eddb0e6294f30decf4a6b4803fd11e2d631`。

## 结论与修改

实际 ROS EKF 同数据回放表明，当前纵向速度过程噪声设置过于保守，是额外响应滞后的一个主要来源。将 `src/aims_racer_system/params/ekf_rear.yaml` 的 `process_noise_covariance` 中 vx 对角元素（15×15 矩阵索引 96）从 **0.025 调至 0.4**，能够显著缩短相对轮速的响应滞后。

这是本次唯一的配置语义变化。轮速和 LIO 的观测协方差、融合项、200 Hz 输出频率、历史回放、TF 设置均保持原值。修改直接位于 NX 当前分支，尚未提交或推送。

| 数据段 | 原配置相对轮速滞后 | 新配置相对轮速滞后 | 验证类型 |
|---|---:|---:|---|
| 10 月 5 日遥控段 | 125 ms | 45 ms | 重建后的实际安装配置 |
| 10 月 5 日自动段 | 110 ms | 40 ms | 重建后的实际安装配置 |
| 9 月 28 日标定包前约 166 s | 135 ms | 45 ms | 独立数据段、同参数候选配置 |

这里的“滞后”是按消息源时间拟合得到的相对速度相位偏移，**不是固定纯延迟，也不是相对真实地速的总延迟**。首次参数筛选中遥控段得到 35 ms，安装配置复测得到 45 ms，报告保留这约 10 ms 的重复运行差异。该差异的具体来源未单独识别。

## 调查依据

NX 安装 `robot_localization` 为 `3.5.4-1jammy.20260605.164056`。本次对照其固定版本源码和官方文档：

- [状态估计参数说明](https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/state_estimation_nodes.rst)
- [EKF 预测与更新实现](https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/ekf.cpp)
- [融合配置建议](https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/configuring_robot_localization.rst)

预测协方差加入 `dt * Q`；更新增益为 `K = P Hᵀ (H P Hᵀ + R)⁻¹`。过小的速度过程噪声会让滤波器过度相信预测，在真实速度变化时更慢地跟随观测。官方文档也明确指出，提高过程噪声相对观测噪声的比例可加快对观测的收敛。

当前链路融合 LIO 位姿/速度、轮速 vx、IMU yaw rate；未直接融合前向加速度，`use_control` 为 false。模型仍包含推断出的加速度状态，不能简单称为严格匀速模型。现有轮速 vx 观测方差为 0.04；LIO 适配器线速度方差下限为 0.04。

此前场地 EKF 消息年龄 P95 约 2.9 ms，和约 100 ms 的速度响应偏移并不相同。历史观测回放、预测至当前时刻解决时间对齐；它们无法单独消除由预测/观测权衡造成的响应滞后。

比较过的路径：

| 方案 | 判断 |
|---|---|
| 增大 vx 过程噪声 | 单变量可复现改善，选用 |
| 减小轮速观测方差 | 会提高对轮速的信任，尚缺轮滑和低速误差的独立标定，暂不采用 |
| 融合 IMU 前向加速度或命令预测 | 需要重力、姿态、偏置或车辆响应标定，超出本次最小修正 |
| 仅提高 ax 过程噪声至 0.1 | 遥控段仍约 95 ms，不能在两段同时达标 |

## 数据、方法与参数筛选

原始数据均保留：

1. `/home/aims/aimsracer-data/sessions/2026-10-05/225646-field-ndt/bag/bag_0.db3`，录制已关闭。遥控分析窗口约 50.32 s，自动分析窗口约 10.48 s。
2. `/home/aims/aimsracer-data/sessions/2026-09-28/calibration-005010/bag/mpcc-calib-20260928-005010_0.db3`。本次只回放前约 166 s，并非完整标定包。

执行实际 `robot_localization/ekf_node`，使用 1× 回放、200 Hz 仿真时钟。测试进程仅使用 `ROS_DOMAIN_ID=94`、`ROS_LOCALHOST_ONLY=1`；回归测试使用域 95。只回放轮速、LIO、IMU、静态 TF 和供对照捕获的已录制 filtered odom。已录制 filtered odom 不作为滤波输入；未回放任何驱动或舵机命令。

比较按消息源时间建立 10 ms 网格，在 -100 至 250 ms 范围内以 5 ms 步长搜索偏移，并拟合增益和偏置。表中的主指标使用轮速大于 0.35 m/s 的样本，避免混用其他移动阈值。基线回放速度与已录制 EKF 的 RMS 差异约 0.00085 m/s（遥控）、0.00138 m/s（自动），支持基线重现。

| Q(vx) | 遥控滞后 | 自动滞后 |
|---:|---:|---:|
| 0.025 | 125 ms | 110 ms |
| 0.1 | 75 ms | 75 ms |
| 0.2 | 55 ms | 55 ms |
| 0.4 | 35 ms | 40 ms |
| 0.8 | 25 ms | 30 ms |

0.4 是本次测试中满足两个场地段滞后 ≤50 ms 的最小 vx 候选值。0.8 的改善进一步减小，但自动段速度增量 RMS 已超过基线两倍，因此未选择。

## 重建后实际安装配置复测

| 指标 | 遥控 原→新 | 自动 原→新 |
|---|---:|---:|
| 相对轮速滞后 | 125→45 ms | 110→40 ms |
| 同时间轮速差 RMS | 0.02253→0.00881 m/s | 0.03634→0.01456 m/s |
| 同时间 LIO 速度差 RMS | 0.03383→0.02295 m/s | 0.04994→0.03650 m/s |
| 10 ms 速度增量 RMS | 0.00200→0.00242 m/s | 0.00356→0.00597 m/s |
| 位置相对 LIO 差 P95 | 0.97→0.82 cm | 2.75→2.50 cm |
| 航向相对 LIO 差 P95 | 约 0.568°→0.568° | 约 0.793°→0.793° |
| 输出频率 | 约 200 Hz | 约 199.7 Hz |

代价：速度输出更少平滑。10 ms 增量 RMS 遥控增加约 21%，自动增加约 68%。增量同时包含真实加减速，不能直接等同于噪声。位置/航向差以输入 LIO 为参考，不是独立真值误差。

独立标定数据段中，轮速差 RMS 由 0.12229 降至 0.04291 m/s；位置相对 LIO 差 P95 由 5.14 降至 2.83 cm；10 ms 增量 RMS 增加约 17%。

[响应曲线](assets/2026-10-06/ekf-speed-response.png)

图片和完整机器可读结果位于下述证据目录；仓库报告的图片请从该目录查看。

## 已完成验证

- YAML 解析确认唯一语义变化为 `process_noise_covariance[96]`。
- `aims_racer_system` 重建成功，使用现有 `log/fastlio-ndt` build/install。
- 安装后的 `ekf_rear.yaml` 链接到修改后的源文件，Q(vx)=0.4，SHA256 为 `eafdface37e679b94e5e30edf8f17ead001eb2066a008839489247a3b6e821d5`。
- 原配置响应验收失败，新配置通过；重建后实际安装配置再次通过两段共 16 项检查。
- 回放验收检查：滞后 ≤50 ms 且至少减半；相对 LIO 位置 P95 不增加超过 2 cm、航向 P95 不增加超过 0.1°；10 ms 速度增量 RMS 不超过基线两倍；LIO 速度差 RMS 不增加超过 0.005 m/s；输出 ≥190 Hz；结果有限。
- `aims_racer_system` 的 5 个 CTest 项目全部通过：rear_axle_force、rear_axle_contract、nav_map_argument、localization_policy、monitor_health_delivery。
- 实际安装配置回放开启 TF，捕获到预期 `odom → base_link` 边；对照 EKF 设置 `publish_tf=false`。ROS 图仍可能列出其 TF 发布端点，不能由端点数量推断重复广播。此项不等同于完整实车 TF 验证。

这些门槛用于本次工程候选筛选，并非真实定位精度或实车闭环安全证明。没有执行实车运动，没有启动 MPCC。

## 证据与复现

全部脚本、生成配置、日志、捕获数组、原配置备份和官方源码快照：

`/home/aims/aimsracer-data/sessions/2026-10-06/ekf-vx-response`

重点文件：`replay.py`、`compare.py`、`acceptance.py`、`comparison.json`、`acceptance-baseline.json`、`acceptance-vx_Q_0p4.json`、`acceptance-deployed.json`，以及 `installed-manual/metadata.json`、`installed-auto/metadata.json`。

复现实际安装配置遥控段（新 label 防止覆盖旧结果）：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/aims/aimsracer-data/sessions/2026-10-06/ekf-vx-response/replay.py   --bag /home/aims/aimsracer-data/sessions/2026-10-05/225646-field-ndt/bag/bag_0.db3   --offset 390 --duration 66 --label repeat-installed-manual --variants baseline,deployed
```

自动段使用 offset 568、duration 30、新 label。标定段使用第二个 bag、offset 0、duration 166、variants baseline,vx_Q_0p4。脚本结束会清理其启动的 EKF 和播放器。

## 实车后续与回退

下一次启动现有集成链路会使用新配置；已运行的 EKF 需要重新启动才能加载 YAML。建议首先比较同样的遥控加减速段，再观察低速停车、轮滑、定位创新、速度平滑程度及闭环控制。当前数据不能证明车辆扭动已经解决，也不能把残余 40–45 ms 全部写入一个固定控制延迟。

轮速时间戳是驱动接收时间，缺少独立地速真值；未验证更高速或打滑工况。0.4 是已有数据支持的候选值，保留继续标定空间。

原配置备份：`/home/aims/aimsracer-data/sessions/2026-10-06/ekf-vx-response/baseline/ekf_rear.yaml`。如需回退，恢复该文件到仓库同名源路径并重启 EKF；当前为 symlink-install，安装路径指向源配置。
