# 当前标定源码与操作入口

[English](calibration.md) · [简体中文](calibration.zh-CN.md)

纵向电流、自动横向标定与动捕校对源码统一在 [src/calibration](../../calibration/README.zh-CN.md)，在线状态机／PP 和采集节点使用 C++。现场加载本工作区 `install/setup.bash`。

- [纵向 A/B/C 恒流协议](../../../docs/operations/full-step-current.zh-CN.md)：准备基准、地图与参考，启动和显式启用分开。
- [轮速电流离线拟合／查询](../../calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md)：输出离线模型，不替换控制器。
- [自动横向附着标定](../../calibration/lateral_grip/README.zh-CN.md)：左右转逐级升速，与纵向共享指令通道，不同时运行。
- [动捕校对](../../../docs/operations/mocap-validation.zh-CN.md)：五量／FastLIO／执行器三项，只采集分析。
- [参考轨迹与标定区域编辑](../../../tools/reference/editor/README.zh-CN.md)：离线编辑与导出。

标定命令使用 `/calib/vesc_command`（`vesc_msgs/msg/VescCommand`），最终命令由现有选择器发布到 `/vesc/command`。状态与权限见[当前接口](../../../docs/vesc-command.zh-CN.md)。
