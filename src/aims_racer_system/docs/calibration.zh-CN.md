<a id="calibration-materials-and-current-workflow"></a>
# 校准资料与当前流程

**简体中文** · [English](calibration.md)

当前纵向辨识由独立 `aims_calibration` 包负责，包括恒速、正电流加速和负电流减速，可选增强 PP 或遥控转向。浏览器绘制地图参考并标记直线试验区间后，按[编辑器、PP 与标定指南](../../../docs/operations/pp-calibration.zh-CN.md)执行。加速度目标执行链仍是另一项工作。当前车辆操作使用[main 部署](../../../docs/deployment/README.zh-CN.md)、[车辆启动](../../../docs/operations/bringup.zh-CN.md)、[已知地图 MPCC](../../../docs/operations/known-map-mpcc.zh-CN.md)与[录制](../../../docs/operations/recording.zh-CN.md)。

原来的自动采集、Pure Pursuit、纵向/横向校准教程已保留为[2026-10-10 中文归档](../../../docs/reports/2026-10-10-calibration-guide-archive.md)和[英文归档](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md)。增强 PP 与纵向标定已拆入当前独立包；归档中的旧命令和直接绕过接口的 remap 不作为当前启动指令。本版不恢复横向抓地标定。

已有响应证据见[2026-09-28 speed-mode 分析](../../../docs/reports/2026-09-28-speed-mode-calibration.md)，后期实际行驶见[2026-10-10 外场复盘](../../../docs/reports/2026-10-10-mpcc-field-review.md)。模型转向滞后与命令到 yaw 响应的条件按对应记录解释，未提供实际转角传感器测量。

当前标定入口为 `/calib/vesc_command`（`vesc_msgs/msg/VescCommand`）。发布者需使用当前 ROS 时间及 `base_link`，每条命令仅填写所选模式目标；RC 解锁并进入标定后必须收到新命令。模式 4 暂不实现，默认非零制动禁用。历史教程的旧 Ackermann 标定消息不能直接 remap 接入。详见 [新控制接口](../../../docs/vesc-command.md)。
