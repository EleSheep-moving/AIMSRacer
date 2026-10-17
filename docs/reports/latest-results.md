# 当前功能与验收状态（2026-10-18）

[操作指南](../README.zh-CN.md) · [相关报告](README.zh-CN.md) · [附件](assets/README.zh-CN.md)

本仓库已包含独立五量估计器、纵横向标定和动捕校对源码。原报告的安装哈希与数值对应其注明的快照；新机器需按当前源码重新构建，不直接套用旧工作区命令。

| 当前功能 | 操作／源码 | 当前可用程度 |
|---|---|---|
| 后轴与质心 VX/VY/AX/AY/yaw rate | [状态指南](../operations/state-observer.zh-CN.md)、[现场脚本](../../src/aims_racer_system/field/README.zh-CN.md) | C++ 实现与两端软件验证完成；[VY 图表](2026-10-18-vy-quality.zh-CN.md)来自历史实包回放，共享 IMU 一致性不是独立精度。Vicon 动态验收待测。 |
| 纵向 A/B/C 电流标定 | [完整协议](../operations/full-step-current.zh-CN.md)、[源码](../../src/calibration/longitudinal_current/README.zh-CN.md) | 现有采集与 PP 使用 C++；当前 B/C 系数及支持域见[模型选型](2026-10-17-current-affine-selection.zh-CN.md)。离线轮速模型不自动接入控制器。 |
| 左右转逐级升速横向试验 | [lateral_grip](../../src/calibration/lateral_grip/README.zh-CN.md) | 自动流程与 AX/AY 记录可用；实际摩擦上限仍待测，固定转角不保证固定半径。 |
| 状态／FastLIO／执行器动捕校对 | [配置与命令](../operations/mocap-validation.zh-CN.md)、[软件验证](2026-10-18-mocap-validation.zh-CN.md) | C++ 采集与 Python 离线分析可用；Vicon 地址、刚体、对时和外参需现场核实。 |
| 参考轨迹与标定区域编辑 | [编辑器](../../tools/reference/editor/README.zh-CN.md) | 离线编辑与导出；不自动改变车辆使用的参考或 bundle。 |

MPCC 仍使用现有导航输入，包括 VX；本次没有切换到独立估计器。质心高度约 −0.03 m 是估计值。消息源时间年龄、波形偏移和物理响应分别报告，软件通过不等于实车精度通过。

本轮 [源码归并记录](2026-10-18-code-consolidation-audit.zh-CN.md)包含文件范围和验证；CPU 采集脚本不纳入发布。旧 Ood、已退役运行时和被替代教程不再作为当前导航。
