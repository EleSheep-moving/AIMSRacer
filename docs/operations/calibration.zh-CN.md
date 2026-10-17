# 电流与横向加速度标定入口

[English](calibration.md) · [项目首页](../../README.zh-CN.md#calibration) · [附件总索引](../reports/assets/README.zh-CN.md)

这里汇总电流标定、横向加速度测量校对和横向附着标定，按实际完成状态导航。三类源码统一在 `src/calibration/`，软件部署与实车附着验收分别说明。

| 要做的工作 | 操作入口 | 结果与状态 |
|---|---|---|
| 电流标定 A：恒速保持电流 | [A/B/C 环境与完整恒流协议](full-step-current.zh-CN.md) | [恒速 Iq／电压报告](../reports/2026-10-13-steady-current-voltage-model.zh-CN.md)；已有实车采集 |
| 电流标定 B/C：加速、减速区间响应 | [完整恒流采集](full-step-current.zh-CN.md) | [当前模型选型](../reports/2026-10-17-current-affine-selection.zh-CN.md)、[采集覆盖](../reports/2026-10-15-current-response-identification.zh-CN.md) |
| 横向加速度 AY 的单位、参考点和动态质量 | [横向加速度标定与校对](lateral-acceleration-calibration.zh-CN.md) | 后轴／质心输出和离线工具已提供，独立动态精度仍待动捕测量 |
| 横向附着、侧滑临界及最大 AY | [横向标定状态](lateral-acceleration-calibration.zh-CN.md#lateral-grip) | 已恢复 C++ 固定名义转角／左右逐级升速工具；实车附着上限待测 |
| 用动捕比较估计器和执行器 | [统一采集与分析](mocap-validation.zh-CN.md) | [交付及测试记录](../reports/2026-10-18-mocap-validation.zh-CN.md)；现场时钟与标记外参尚需填写 |

<a id="current-calibration"></a>
## 电流标定：从哪里开始

当前源码统一为 [`src/calibration/`](../../src/calibration/README.zh-CN.md)：纵向 `longitudinal_current`、横向 `lateral_grip`、动捕 `mocap_validation`。NX 现场环境统一加载 `~/AIMSRacer/install/setup.bash`；旧实验工作区保留作回退。纵向短入口为：

```bash
# B；C 使用独立会话，将 B 换成 C
~/run_calibration_full_step.sh B
```

同一标定环境的另一终端使用 `inspect`、`start`、`stop`，具体环境加载、冻结 A 基准、地图／参考和录包步骤以[完整操作说明](full-step-current.zh-CN.md)为准。启动会话与启用动作分开，默认 `auto_start=false`；本次文档更新没有执行这些命令。[原生安装核验](../reports/2026-10-16-native-calibration.zh-CN.md)说明了实际加载路径。

输入统一使用实测有符号 `avg_iq`。当前选型是 B/C 分别拟合的区间平均模型 `a_bar=b+k·mean(avg_iq)`，不再额外扣 basic。B/C 由采集阶段区分，不能仅按 Iq 的正负切换；系数、筛选范围和留出验证均见[当前模型报告](../reports/2026-10-17-current-affine-selection.zh-CN.md)。

A 的保持电流、B/C 的轮速响应、车体加速度真值和已部署的加速度控制接口是不同成果。现有小 RMSE 不代表瞬时车体加速度或高电流滑移区已验收。

<a id="lateral-calibration"></a>
## 横向加速度标定：从哪里开始

先读[横向加速度指南](lateral-acceleration-calibration.zh-CN.md)，确认参考点、AX/AY 的物理定义、安装外参和时间基准，再使用[动捕工具](mocap-validation.zh-CN.md)进行精度与响应校对。AY 输出可计算不等于传感器标定、动态精度或轮胎附着极限已验证。

## 文档与证据

- [近期实验精选](../reports/latest-results.zh-CN.md)：当前结论及日期化证据。
- [后轴／质心五量指南](state-observer.zh-CN.md)：计算链、话题、状态和几何。
- [仓库内附件总索引](../reports/assets/README.zh-CN.md)：图表、CSV、JSON、安装回执均可在仓库内阅读。原始包路径仅保留作溯源，不是阅读依赖。
