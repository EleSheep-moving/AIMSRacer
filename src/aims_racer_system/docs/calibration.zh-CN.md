<a id="calibration-materials-and-current-workflow"></a>
# 校准资料与当前流程

**简体中文** · [English](calibration.md)

已有 A/B/C 电流采集与离线模型；独立车体精度和闭环加速度控制接口仍需单独验收。当前车辆操作使用[main 部署](../../../docs/deployment/README.zh-CN.md)、[车辆启动](../../../docs/operations/bringup.zh-CN.md)、[已知地图 MPCC](../../../docs/operations/known-map-mpcc.zh-CN.md)与[录制](../../../docs/operations/recording.zh-CN.md)。

原来的自动采集、Pure Pursuit、纵向/横向校准教程已保留为[2026-10-10 中文归档](../../../docs/reports/2026-10-10-calibration-guide-archive.md)和[英文归档](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md)。其中脚本和 launch 属于旧源码布局，部分已移除；该教程不作为当前运行命令。

已有响应证据见[2026-09-28 speed-mode 分析](../../../docs/reports/2026-09-28-speed-mode-calibration.md)，后期实际行驶见[2026-10-10 外场复盘](../../../docs/reports/2026-10-10-mpcc-field-review.md)。模型转向滞后与命令到 yaw 响应的条件按对应记录解释，未提供实际转角传感器测量。

当前标定入口为 `/calib/vesc_command`（`vesc_msgs/msg/VescCommand`）。发布者需使用当前 ROS 时间及 `base_link`，每条命令仅填写所选模式目标；RC 解锁并进入标定后必须收到新命令。模式 4 暂不实现，默认非零制动禁用。历史教程的旧 Ackermann 标定消息不能直接 remap 接入。详见 [新控制接口](../../../docs/vesc-command.md)。

## 当前标定入口

[电流标定 A/B/C](../../../docs/operations/calibration.zh-CN.md#current-calibration) · [横向加速度标定／AY 校对](../../../docs/operations/lateral-acceleration-calibration.zh-CN.md) · [动捕工具](../../../docs/operations/mocap-validation.zh-CN.md)。这些入口区分已部署采集、离线结果和历史横向附着脚本；不会把旧教程当成当前可执行流程。
