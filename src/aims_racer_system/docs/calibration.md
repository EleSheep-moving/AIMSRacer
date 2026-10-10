# 校准资料与当前流程

完整纵向辨识按后续加速度控制接口计划进行，当前试跑可继续分析已有 bag 中的指令和实测响应。当前车辆操作使用[main 部署](../../../docs/deployment/README.md)、[车辆启动](../../../docs/operations/bringup.md)、[已知地图 MPCC](../../../docs/operations/known-map-mpcc.md)与[录制](../../../docs/operations/recording.md)。

原来的自动采集、Pure Pursuit、纵向/横向校准教程已保留为[2026-10-10 中文归档](../../../docs/reports/2026-10-10-calibration-guide-archive.md)和[英文归档](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md)。其中脚本和 launch 属于旧源码布局，部分已移除；该教程不作为当前运行命令。

已有响应证据见[2026-09-28 speed-mode 分析](../../../docs/reports/2026-09-28-speed-mode-calibration.md)，后期实际行驶见[2026-10-10 外场复盘](../../../docs/reports/2026-10-10-mpcc-field-review.md)。模型转向滞后与命令到 yaw 响应的条件按对应记录解释，未提供实际转角传感器测量。
