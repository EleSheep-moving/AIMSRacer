# 后轴 IMU 与 LIO 源时间修正

**简体中文** · [English](2026-10-11-rear-imu-source-time.md)

## 时间约定

- `/rear_axle/lio_odom`：保留 LIO 扫描结束时间戳。源时刻 10.000 s 的 LIO 使用该时刻的 gyro，即使在 10.170 s 才收到。精确匹配或在前后 IMU 测量之间插值，不外推位姿或 gyro。插值的 IMU 源时间间隔最多 30 ms，检查数据连续性；若 LIO 回调先到，短队列等待后一条 IMU 覆盖。
- `/rear_axle/imu`：每条最新校正 IMU 到达后立即处理发布，保留原时间戳。只做单位、固定安装坐标旋转与旋转杠杆补偿。不订阅 LIO、不预测姿态、不再检查 250 ms 姿态年龄、不新增对齐 IMU 话题。
- IMU 输出仍是含重力的比力，姿态不可用（`orientation_covariance[0] = -1`）。比力是否可用取决于输入、协方差及角加速度估计需要的 gyro 历史。启动或历史中断后仍可能短暂不可用；此时加速度默认零值不能当成测量。

当前二维 EKF 继续只融合 LIO、轮速 vx 与 IMU yaw rate，加速度和姿态融合保持关闭。既有 `predict_to_current_time=true` 保留，它预测融合状态，不修改上游 LIO 观测。FAST-LIO 内部 IMU 传播、点云去畸变、坐标系和 scan-end 发布均未修改。

## 原因与修复

旧 IMU 适配器要求 LIO 姿态足够新，并用 gyro 传播到最新 IMU 时刻。这份姿态可用于去重力，但杠杆补偿本身不需要它：

`f_rear = R_mount f_imu - alpha × r - omega × (omega × r)`。

原先的耦合会因 LIO 晚到而停用本来可用的比力。现已解除这项依赖。离线去重力需要按加速度源时间对齐 LIO 姿态，并注明插值方法；此前报告使用传播姿态得到的指标，不能直接视作新时间对齐方法的实测结果。

## 录制输入回放

2026-10-10 的 8 个完整圈，通过实际 C++ 核心进行数值回放。原始 Livox 加速度按精确整数纳秒时间戳匹配已录制的后轴 gyro（这些包安装旋转为零，后轴 gyro 对应零偏修正后的输入）。活动窗口为首次 enabled RUNNING 至首次 COMPLETE；新旧版本均用后轴消息 bag 接收时间选窗口。

| 包 | 原比力不可用比例 | 修正后比力不可用比例 |
| --- | ---: | ---: |
| 175129_v10_lap01 | 2.1836% | 0.0000% |
| 185351_v12_n15_independent_lap01 | 5.8445% | 0.0307% |
| 190404_v15_n15_cap4_lap01 | 1.4318% | 0.0000% |
| 190848_v18_n15_accel1_cap4_lap01 | 3.4767% | 0.0589% |
| 192020_v22_n15_accel1_cap4_lap02_clockfix | 5.8283% | 0.0209% |
| 192725_v26_n15_accel1_cap4_lap01 | 4.6814% | 0.0217% |
| 192955_v35_n15_accel1_cap4_lap01 | 5.8814% | 0.0158% |
| 193524_v35_n15_accel1_cap4_lap02 | 1.9676% | 0.0000% |

这些是匹配录制输入的数值结果，不是在线 ROS 吞吐实测，也不是新的实车试验。提取流中的缺失输入及真实回调顺序不能完全重建。剩余不可用消息来自角加速度所需历史不足。比力协方差保留传感器与杠杆补偿的不确定度，删除原来的重力/姿态经验膨胀项；这尚不是下游加速度协方差的实测标定。

## 验证与证据

- 修复前回归复现：没有 LIO 就无比力，以及 gyro 向前传播姿态。
- ROS 回归复现旧时间选择偏差：选到 0.397 rad/s，而源时刻应为 0.400 rad/s。
- 本机包构建与测试：`colcon test-result` **75 项，0 错误，0 失败**。ROS 测试验证纳秒时间戳保留、位姿不变、源时间速度补偿，以及 LIO 到达前持续提供最新 IMU。
- NX 包构建和测试也通过：**75 项，0 错误，0 失败**。没有启动车辆驱动或发布驾驶指令，ROS 回归节点运行在隔离的 domain 188。

证据：`~/aimsracer-data/sessions/2026-10-11/imu-source-time-alignment/`，包括失败回归记录、`test-result.txt`、`replay_metrics.csv`、回放源码和清单。原录包及既有提取数据保留。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select aims_racer_system --event-handlers console_direct+
colcon test-result --verbose
```

[架构](../architecture.zh-CN.md#ekf-与-lio-时间) · [系统包](../../src/aims_racer_system/README.zh-CN.md)
