# IMU 与延迟 LIO 的车体纵向加速度观测

> 兼容单分量接口说明：当前推荐使用[后轴／质心统一五量节点](state-observer.zh-CN.md)。对应源码保留在 `src/aims_racer_system/`，按当前构建流程生成。


[English](body-acceleration.md) · **简体中文** · 实现计划（记录中的工作区路径：`docs/plans/2026-10-15-body-acceleration.md`）

`body_acceleration` 是独立 C++ 观测节点。首版输出后轴 `base_link` 处、沿车体前向投影的**惯性加速度**，用于录包和动态分析。它不订阅轮速、导航 EKF 或电流，不发布控制命令或 TF。安装几何和加速度计标定默认尚未确认；软件验证不等于动态绝对精度或实车控制验收。

## 输入输出

| 话题 | 含义 |
| --- | --- |
| `/livox/imu_bias_corrected` | 已减启动 gyro 零偏的原始 Livox IMU；加速度为 g，角速度为 rad/s，`livox_frame` |
| `/rear_axle/lio_odom` | `odom` / `base_link` 姿态与源时间；不使用速度字段 |
| `/state_estimation/body_acceleration` | `AccelWithCovarianceStamped`；仅 `accel.accel.linear.x` 有效，m/s²，`base_link`，消息头是对应 IMU 源时刻 |
| `/state_estimation/body_acceleration/status` | 10 Hz `DiagnosticArray`：有效性、源/接收年龄、标定标志、回放与拒绝计数、回调耗时 |
| `/state_estimation/body_acceleration/debug` | 10 Hz 分级比力、重力、角速度、角加速度、杠杆项和滤波前后纵向加速度；源时刻在 `source_stamp_ns` |

其他加速度/角加速度分量的零占位没有物理零值含义，协方差对角设为 `1e6`；检查 `x_valid/y_valid/z_valid/angular_valid`。首版数值协方差是**未独立标定的模型不确定度**，不是精度保证。原始 Livox 全零协方差表示未知。

新的 `a_x^B` 与旧节点 `/state_estimation/longitudinal_acceleration` 的 `d(body_vx)/dt` 不同：

\[
a^B=\dot v^B+\omega^B\times v^B.
\]

平面条件下 `a_x^B = d(v_x^B)/dt - r v_y^B`。转弯时不能把轮速导数当作独立加速度真值。旧轮速融合节点及其话题继续保留，见[原操作页](longitudinal-acceleration.zh-CN.md)。

## 时间与补偿

先完成单位和安装转换，再计算

\[
a_{\rm rear}^B=f_{\rm IMU}^B-b_a^B+g^B
-\alpha^B\times\ell-\omega^B\times(\omega^B\times\ell).
\]

`livox_translation` 是后轴到 IMU 的向量；默认 `[0.30, 0, 0.03]` m 是已有近似几何，不是质心位置。角加速度来自因果时间窗回归，其噪声和跟随都进入补偿结果。固定加速度偏置默认为零，不能用一次静止姿态同时辨识零偏、比例和安装倾斜。

LIO 姿态作为源时间的倾斜锚点，之后用三轴 gyro 传播。迟到的 LIO 在历史时刻更新，并重放之后的姿态、补偿和纵向滤波状态；已经发布的历史输出保持原样。输出不等待下一帧 LIO，也不把“最近收到”的旧姿态直接当成当前姿态。历史有时间和条数上限；gyro 缺口、过旧锚点及无效输入均有诊断。

LIO 与本节点共享物理 IMU；它们吻合只能说明一致性，不能说明独立绝对精度。节点不使用二维导航 EKF 的 roll/pitch。

实时状态和 debug 分别给出 `alpha_window_span_s`、`alpha_sample_center_age_s`、`estimate_lio_anchor_age_s` 和 `latest_received_lio_source_age_s`。前两者表示 gyro 回归的样本时间支持，不是所有频率通用的相位延迟；后两者区分“当前 IMU 估计实际使用的历史锚点”和“最新收到的 LIO”。不能只看一个消息头就认为所有分量同样新鲜。

## 启动与录制

在现有 IMU 零偏和 LIO 输入已经运行时，可独立启动：

```bash
ros2 launch aims_racer_system body_acceleration.launch.py
```

只启动观测器，不启动传感器、车辆或控制器。可指定 `config:=/absolute/body_acceleration.yaml`、`geometry_config:=/absolute/rear_axle_geometry.yaml`；回放时使用 `use_sim_time:=true`。

本轮 NX 软件验证安装在独立目录 `/home/aims/aimsracer-body-observer-20261015`，未替换原主工作区安装。使用该版本时，在新终端先 source 原工作区，再叠加其 `install/local_setup.bash`：

```bash
source /home/aims/AIMSRacer/install/setup.bash
source /home/aims/aimsracer-body-observer-20261015/install/local_setup.bash
ros2 launch aims_racer_system body_acceleration.launch.py
```

现有 vehicle/race 启动入口支持显式参数：

```text
body_acceleration:=true record:=true
```

默认 `body_acceleration:=false`。启用录制会保存新话题和对应配置；其他车辆参数按原启动流程提供。不要同时启动两个相同输出的观测器。

```bash
ros2 topic echo /state_estimation/body_acceleration/status --once
ros2 topic echo /state_estimation/body_acceleration --once
```

数值输出只在有效 IMU、连续 gyro 和可用姿态条件下发布。断流期间状态消息继续发布；ROS 时钟倒退会重置历史。状态中的 `imu_age_s/lio_age_s` 是源时间年龄，`*_receipt_age_s` 是距最近接收的时间；两者都不等于运动响应延迟。

## 标定、验证与后续动捕

配置位于 `params/body_acceleration.yaml`，几何来自共享的 `params/rear_axle_geometry.yaml`。`force_variance/gyro_variance/tilt_std_rad` 是初版模型假设；`extrinsics_calibrated/accelerometer_calibrated` 默认 false。改变安装、传感器或滤波前应重新核对输入质量。

离线工具（记录中的工作区路径：`verification/body_acceleration/README.md`）提供原包只读提取、逐轴统计、频谱、Allan 分析、静态标定和原生到达顺序回放。长静止、温漂、多姿态和新场地实验尚需实际采集；短静止段不能代替长时间验证。

后续动捕输入保留采样/接收时间、时钟映射和刚体外参。参考加速度只在离线评估时求导，明确带宽和边界处理；不会输入在线观测器。验证顺序为合成物理真值、延迟/缺口故障、真实录包、NX 隔离 DDS，最后再做现场与动捕验证。

## 电流与横向加速度标定

[电流标定 A/B/C 与当前离线模型](calibration.zh-CN.md) · [横向加速度 AY 校对及附着标定状态](lateral-acceleration-calibration.zh-CN.md)。区间电流拟合与轮胎横向能力分别验证，不由既有系数直接生成新的控制前馈或横向加速度上限。
