# VESC 车辆控制接口

[English](vesc-command.md) · **简体中文**

`/vesc/command` 使用 `vesc_msgs/msg/VescCommand`。RC 选择器以 100 Hz 发布；VESC core 与 IMU 也按 100 Hz 轮询，串口为 921600 baud。MPCC 保持 50 Hz 的 `/drive` 速度提议，并读取选择后的命令维护输入历史。

| control_mode | 使用的目标字段 | 单位 / 含义 |
| --- | --- | --- |
| 0 MODE_SPEED | target_speed_mps | 有符号 m/s，前进为正 |
| 1 MODE_CURRENT | target_current | 有符号电机电流，A |
| 2 MODE_DUTY | target_duty | 占空比 [-1, 1] |
| 3 MODE_BRAKE_CURRENT | target_brake_current | 非负制动电流幅值，A |
| 4 MODE_ACCELERATION | target_acceleration_mps2 | 保留的有符号车体纵向速度导数；当前始终拒绝 |

各模式的 `steering_angle` 都以 rad 表示，左转为正。Header 是中控发布时的 ROS 时间，frame_id 必须为 `base_link`。所有字段必须有限，未选中的目标字段必须严格为零。可靠 KeepLast(1) 和 100 ms DDS lifespan 使整条待执行命令一起替换，包括模式及转向。

驱动和 MPCC 检查 Header 及可用的 DDS 源时间，过期消息不刷新驱动的单调时钟 watchdog。无效消息发送字面值零 ERPM，并保留转向；100 ms watchdog 超时也保留转向。中控 RC 锁定、RC 超时或命令源超时则发布零速度并回正。驱动启动后，在收到有效命令前不发送控制命令。

## 单位转换与限幅

当前标定：`ERPM = 3465 × target_speed_mps + 0`；`servo = −0.5137 × steering_angle + 0.506`，限幅到 `[0.264, 0.750]`。非零速度按配置的电气 RPM 范围 `[-40000, 40000]` 限幅。零速度及安全停止始终发送零 ERPM，不受标定 offset 或最小限幅影响。电机和 servo packet 在同一串口写锁内成对发送。

`/sensors/servo_position_command` 是用于里程计的舵机命令回显，不是转角传感器测量。默认轮速方差仍为 `0.04 (m/s)²`。

RC 保留速度、电流、占空比档位，以及限幅、方向、锁定和优先级。整车 CH10 的手动电流请求范围仍为 **3–100 A**；驱动 `vesc.yaml` 的最终电流限幅为 **−100 至 +100 A**。这两个值分别限制请求与实际下发，不能把 100 A 请求当作 100 A 实际电流。驱动子模块自带的 `vesc.yaml` 和 `vesc_config.yaml` 也统一为 ±100 A。这些 ROS 限幅针对电流模式目标；速度模式 PID 的电流限制仍由 VESC 固件配置。

`current_startup_enabled=true` 保留 40 A 起步辅助（`startup_duty_current=40`、`startup_timeout_ms=100`）：请求电流绝对值小于 0.1 A 时发送零；否则，使用同符号起步电流，直到新鲜遥测确认绝对 ERPM 大于 1 并持续 100 ms。最终仍受驱动电流限幅。显式关闭辅助后直接下发请求电流。录制的 `target_current` 是请求值，起步期间可能不同于串口目标；参数快照保留辅助设置。

车辆和独立驱动默认 `brake_min=brake_max=0`，禁用非零制动。协议通路保留。加速度到电流控制留给后续 VESC ROS 模块实现，需要先完成辨识及限幅；当前没有比例换算占位实现。

## MPCC 与标定

导航输入继续使用 `/drive`（`AckermannDriveStamped`，jerk=0）。MPCC 只读取 MODE_SPEED，直接使用 m/s 和 rad。若速度、转角与 100 ms 内自身近期提议相差均小于 1e-6，则在预测历史内部恢复加速度及转向角速度元数据。非速度命令不进入速度历史，并通过既有故障流程撤销正在运行的自动控制。

标定输入改为 `/calib/vesc_command`，使用相同消息，要求新鲜源时间戳、RC 待命/重新待命和源 TTL。旧 `/ackermann_cmd`、Float64 电机/舵机控制订阅及 `ackermann_to_vesc_node` 已移除；没有 `/vesc/applied_command`。历史录包保留原 schema，新录包记录 native 命令、控制权限、`/drive` 和遥测。

本次迁移验收只使用隔离 DDS、合成发布者及 PTY VESC 模拟器，不打开车辆串口或发送实机指令。实机测试需车主在场并明确指示。见[迁移验收报告](reports/2026-10-13-vesc-native-command.md)。
