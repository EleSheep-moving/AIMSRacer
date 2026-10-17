# 2026-10-18 工具归并与主仓库发布

本次将三类工具及其已经实现的依赖纳入主仓库：轮速电流离线拟合／查询、参考轨迹与标定区域编辑器、独立状态估计器／Vicon 现场脚本。CPU 采集工具按用户要求退出两端源码目录，NX 快捷入口恢复原会话脚本；该工具不在本次提交中。

## 源码与使用入口

| 功能 | 当前仓库入口 | 行为 |
|---|---|---|
| 轮速电流模型拟合／查询 | [longitudinal_current](../../src/calibration/longitudinal_current/README.zh-CN.md)、[模型说明](../../src/calibration/longitudinal_current/docs/wheel-current-model.zh-CN.md) | 离线拟合与标准库查询；安装 `wheel-current-fit`、`wheel-current-query`，不自动替换当前选用的 B/C 模型 |
| 参考与标定区域编辑 | [tools/reference/editor](../../tools/reference/editor/README.zh-CN.md) | 本地网页编辑与数值导出；Three.js 来源与许可证一并保存；不自动切换车辆参考 |
| 估计器／Vicon 现场入口 | [aims_racer_system/field](../../src/aims_racer_system/field/README.zh-CN.md) | Bash／Zsh 环境加载，独立估计器启动和 Vicon 兼容命令；现场配置留在用户配置目录 |

`main` 原先仅有这些功能的部分报告。为使上述命令在源码层面完整，本次同时纳入此前已验证的 `src/calibration/{longitudinal_current,lateral_grip,mocap_validation}`、`src/aims_pp`、C++ 五量节点／适配／launch／测试及 `verification/vicon_state` 等离线验证依赖。固定 VRPN 源码、版本清单与许可证随动捕目录保存。

RC 选择器增加 `/control/calibration_enabled` 状态发布，供标定工具核对授权；不改变命令选择算法。参考 `path.py` 仅补充 `source=recorded` 元数据。没有合入实验版原生 MPCC 控制算法、车辆调参、NDT 补丁或频率修改；MPCC 仍读取原导航状态，包括 VX。

## 文档维护

项目首页、文档首页、包说明、标定指南和参考工具入口均指向当前源码与构建流程。Ood／已退役运行时、旧 Python 横向教程、早期单分量观测器不再作为当前索引；历史文件和原始报告保持原位。报告索引只保留当前功能、模型与验收直接需要的材料。

新的 [当前功能清单](latest-results.zh-CN.md)区分软件可用与尚未完成的实车验收。旧报告中的安装哈希和测量值仍对应当时快照，不代表新 clone 的编译产物。关键附件继续用仓库内相对链接，不依赖旁边的外部数据目录。

## 验证与限制

先前两端回归：轮速模型／拟合／CLI 各 77 项，编辑器后端与导出各 21 项；主机另有 1 项 Chromium 页面交互检查。两端安装命令帮助、Bash／Zsh 入口及站点配置路径已核对。对应 [源码 SHA256 与验证摘要](assets/2026-10-18-code-consolidation-audit/receipt.json)仅保留这三类工具，不包含 CPU 采集工具。

NX 系统 SciPy 缺少 `isotonic_regression`。拟合需按模型说明显式使用独立分析环境的 Python；查询仅依赖标准库。梯形积分使用 `scipy.integrate.trapezoid` 兼容 NumPy 1／2，不全局升级 ROS Python 依赖。

轮速电流模型描述轮速导数与实测 `avg_iq`，不能作为独立车体 AX 真值。五量仍待 Vicon 动态精度验收；质心高度约 −0.03 m 是估计值。横向试验的软件验证不代表已经测得摩擦极限。所有本次验证都不启动车辆动作。

此前纵横向包两端安装／验证快照见[归并记录](2026-10-18-calibration-unification.zh-CN.md)。本次提交前另在基于 `origin/main` 的隔离发布工作区重新构建和回归，结果随本报告附件保存。完整日志留在数据目录 `maintenance/2026-10-18-main-tools-publication/`，阅读本页不依赖该外部目录。

## 发布工作区复验

以 `origin/main` 为基线隔离构建：八个工具／相关包构建通过，系统包另行构建通过。环境复用现有 ROS／驱动 underlay；未重建无改动的 MPCC 和 RC 接收器，也未启动完整车辆图。

- 离线电流模型 77 项通过；编辑器及导出共 22 项通过，含真实 Chromium 交互。
- `aims_pp` 71、`aims_calibration` 430、`aims_lateral_grip` 18、`aims_mocap_validation` 52 个 colcon 结果条目均零错误、零失败、零跳过；计数含测试包装层，不是独立实车样本数。
- 系统包完整回归 222 个 colcon 结果条目通过，零错误、零失败、零跳过；包含独立五量、源时间／DDS、launch 与标定布局检查。
- RC 选择器核心、消息管线及 VESC 虚拟串口链三组功能测试通过。其常规 lint 聚合检查存在旧格式／manifest 问题，未将整仓 lint 声称为全通过；本轮不做无关格式重写。固定 VRPN 的上游空白保持原样。
- 修复系统包测试夹具写死旧 `src/aims_calibration` 路径的问题，改用当前 `calibration/longitudinal_current` 源码布局；不改变生产估计／控制算法。
- 修改及新增的自有 Markdown 本地文件链接均可解析，没有指向外部 `aimsracer-data` 的附件超链接。

NX 保留已有场地扩展，例如 race 的 `target_speed` 透传，以及本地车辆参数和 NDT 设置；这些无关改动不纳入本次提交，文件同步不覆盖它们。
