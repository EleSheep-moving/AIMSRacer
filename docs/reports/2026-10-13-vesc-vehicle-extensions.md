# 保留 VESC 车辆扩展与 100 Hz 验证

VESC 子模块 `2c242384d302791449f90e4932ef267d3f5a0759` 在上游 100 Hz 收发实现中保留 AIMSRacer 现有车辆功能。比对替换前 AIMSRacer `480dac58` 的所有 VESC 文件，转换器、里程计与 launch 标定恢复原实现；功能差异集中在 100 Hz 参数、串口收发修复、线程共享状态、测试和文档。车辆 `params/vesc.yaml` 与原配置逐字比对一致，只增加 `baud_rate: 921600` 和 `telemetry_rate_hz: 100.0`。

## 保留的行为

| 功能 | 参数及行为 |
|---|---|
| VESC 电机指令 watchdog | `command_watchdog_enabled: true`、`command_timeout_sec: 0.10`、`watchdog_safe_erpm: 0.0`；开始接收指令后，超时发送零速。速度、电流、制动、占空比和位置输入均刷新计时；舵机输入不刷新电机计时。 |
| Ackermann 转换器 watchdog | `ackermann_watchdog_enabled: true`、`ackermann_timeout_sec: 0.10`；开始接收后，超时发布零速，100 Hz 检查。 |
| 输入恢复 | 两层 watchdog 在收到新的输入后恢复正常下发。首次指令前不主动发送 watchdog 零速。 |
| 轮速里程计方差 | `longitudinal_velocity_variance: 0.04`（(m/s)²）写入 `twist.covariance[0]`，支持有限正数覆盖；两个里程计回调均保留该扩展。 |
| 舵机限位 | 转换器和驱动均按 0.264–0.750 裁剪；保留 -0.5137 gain、0.506 offset 和 0.36 m 轴距标定。 |
| 电流启动 | 保留 `startup_duty_current` 默认 40 A 的可配置行为。 |

现有车辆选择器、MPCC、导航 EKF、后轴适配、TF 分工和车辆参数沿用原框架。VESC 遥测和车辆下发链保持 100 Hz。

## 验证

- `vesc_driver`、`vesc_ackermann`、`aims_racer_system` 编译通过，安装参数与源码一致。
- `driver_link` CTest 两个场景通过（50/100 Hz）：已知 ERPM/电流解析、分段回复、CRC、100 Hz 电机/舵机帧、五种电机模式下的超时和恢复，以及舵机持续消息时的电机超时。模拟驱动测试显式提供操作限值，验证实际非零目标和零速切换。
- `vehicle_extensions` CTest 两个场景通过：转换器首次指令前行为、100 Hz 转发、0.10 s 输入超时、恢复、舵机上下边界、40 A 启动电流、默认 0.04 和自定义 0.09 方差、原 odom/base_link frame。
- 独立 domain 的完整 PTY 链路：core、两类 IMU、wheel odom、ackermann、电机/舵机命令、servo echo 和串口查询/下发帧均 **100.0 Hz**，无帧错误。
- 实机只查询遥测：10 s 窗口 core **99.9 Hz**，两类 IMU **100.0 Hz**；19.8 V、ERPM 0、故障码 0。指令与 watchdog 测试使用模拟串口，未进行实车运动测试。

原始结果和日志保存在 `/home/aims/aimsracer-data/sessions/2026-10-13/vesc-vehicle-extensions/`。
