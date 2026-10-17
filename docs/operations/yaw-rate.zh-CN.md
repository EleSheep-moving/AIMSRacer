# 独立车体 yaw rate 与角加速度观测

> 2026-10-18 整理范围：本页记录已部署工作区的用法；这次文档提交不包含对应的新运行时代码，命令须在已部署环境执行。当前后轴／质心五量以[统一状态指南](state-observer.zh-CN.md)为准。 本页为较早的单分量分支说明，历史数值与局限保留。


[English](yaw-rate.md) · **简体中文** · [验证报告](../reports/2026-10-15-yaw-rate-quality.zh-CN.md)

`yaw_rate` 是独立 C++17 观测节点，输入 `/livox/imu_bias_corrected` 的三轴角速度。共享安装旋转换算到 `base_link` 后，用 `[r, alpha_z]` 运动学卡尔曼滤波器估计车体 z 轴角速度及其变化率。无轮速、LIO、导航 EKF、电流或转向指令订阅；不发布控制命令或 TF。上游启动零偏只扣一次，首版不估计在线零偏。

## 定义与消息

`r=omega_body.z`，单位 rad/s；`alpha_z=dr/dt`，单位 rad/s²。一般 roll/pitch 下 `r` 不等于欧拉 yaw 导数。角速度不需要位置杆臂补偿，但需要安装旋转。当前节点不替换车体纵向加速度中已有的 gyro/角加速度处理。

| 话题 | 类型和有效字段 |
| --- | --- |
| `/livox/imu_bias_corrected` | 输入 `sensor_msgs/Imu`；gyro rad/s，`livox_frame` |
| `/state_estimation/yaw_rate` | `sensor_msgs/Imu`，仅 `angular_velocity.z` 有效，`base_link` |
| `/state_estimation/yaw_acceleration` | `AccelWithCovarianceStamped`，仅 `accel.accel.angular.z` 有效 |
| `/state_estimation/yaw_rate/status` | 10 Hz `DiagnosticArray`，输入年龄、有效性、计数、状态和回调窗口耗时 |

两种数值输出使用相同 IMU 源时间。rate 消息的 orientation/linear_acceleration covariance 首元素为 -1，表示不可用；angular x/y covariance 对角为 1e6。角加速度的其他五个分量 covariance 对角同样为 1e6。零占位没有物理零值含义，不能把输出当成完整 IMU。协方差仅为给定 q/R 及上游偏置假设下的条件模型协方差，未独立标定，不含对真实零偏误差的保证。

## 模型与时间

`F=[[1,dt],[0,1]]`，观测 `z=r+noise`，`Q=q*[[dt³/3,dt²/2],[dt²/2,dt]]`；采用 Joseph 协方差更新。默认 `process_psd=4`、`measurement_variance=1e-4`、`initial_alpha_variance=100`。这些是有效模型参数，不是已识别的硬件噪声。合成扫频显示 5–10 Hz 有增益抬升，不能把滤波器描述为所有频率都衰减的普通低通。

节点按 IMU 源时间推进，不预测到接收时刻，不等待 LIO。重复／倒退源时间被拒绝；大于 30 ms 的源间隔重置 r 与 alpha 状态，连续三个有效样本后恢复发布。超过 50 ms 的输入源年龄、未来时间、错误 frame、不可用 gyro、非有限数值都被拒绝。断流不发布虚构零值，状态仍继续更新。ROS 时钟回退清空状态，开始新时钟 epoch；输出时间严格递增、不可重复的契约限于同一 epoch。

状态中的 `imu_source_age_s` 与 `imu_receipt_age_s` 分别是估计所用源年龄、距最新输入回调的时间，不是运动响应延迟。`callback_window_p99_ms` 是最多 512 次回调的窗口统计，包含数值发布，不是整段 P99。所有本节点模型／话题参数启动后只读；修改需重启节点。

## 启动与录制

已有 IMU 零偏链路运行后，在 source 对应安装的终端独立启动：

```bash
ros2 launch aims_racer_system yaw_rate.launch.py
```

只启动观测器。可提供 `config:=/absolute/yaw_rate.yaml`、`geometry_config:=/absolute/rear_axle_geometry.yaml`；ROS bag 回放使用 `use_sim_time:=true`。安装旋转来自共享 `livox_rpy`，与既有加速度节点保持一致。

vehicle/race 入口支持 `yaw_rate:=true`，默认 false；`record:=true` 录制 rate、角加速度、状态及源 IMU，并保存参数。其他驾驶配置按原流程传入。

```bash
ros2 topic echo /state_estimation/yaw_rate/status --once
ros2 topic echo /state_estimation/yaw_rate --once
```

首轮 NX 软件安装是 `/home/aims/aimsracer-yaw-observer-20261015/install`，与主工作区分离。该目录使用直接 CMake 安装，需 source 包级 setup：

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-yaw-observer-20261015/install/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system yaw_rate.launch.py
```

当前输出用于观测和录包；实包与 EKF 的一致性不构成独立动态真值。B/C 包中已有 gyro 零偏残余的证据，不能通过进一步平滑消除或宣称已在线辨识。后续若用 LIO/外部姿态估计慢偏置，需单独验证共享 IMU 相关性、延迟和可观性。
