# 文档索引

[English](README.md) · **简体中文** · [项目首页](../README.zh-CN.md)

当前 **FAST-LIO2 + 后轴 EKF + NDT + 原生 MPCC** 栈的资料导航。首页和本索引提供双语，详细指南保留表中列出的语言。

## 推荐阅读顺序

1. **新机部署：** [系统依赖](deployment/README.md) → [目标平台](deployment/orin.md) → [车辆启动](operations/bringup.md)。
2. **首次已知地图运行：** [坐标系与话题](architecture.md) → [地图参考准备](../src/controller/docs/usage.md) → [初始化与启用](operations/known-map-mpcc.md)。
3. **控制器开发：** [实现](../src/controller/docs/implementation.md) → [原生运行时](../src/aims_mpcc_rt/README.md) → [回归与重放](../verification/README.md)。
4. **分析一次运行：** [录制](operations/recording.md) → [外场复盘](reports/2026-10-10-mpcc-field-review.md) → [日期化报告](reports/README.md)。

## 安装与硬件

| 指南 | 内容 | 语言 |
| --- | --- | --- |
| [部署指南](deployment/README.md) | 系统包、固定源码、SDK 准备、并发受限构建与 bundle 数据 | 中文 |
| [Orin NX](deployment/orin.md) | 车辆设备、MID360 网络与原生部署 | 中文 |
| [x86 / NUC](deployment/nuc.md) | 同一源码栈的 x86 构建与目标平台 bundle 编译 | 中文 |
| [设备规则](../rules/README.md) | 稳定设备名与串口占用 | 英文 |
| [依赖清单](../dependencies/manifest.json) | 精确上游提交、补丁与 acados 架构设置 | JSON |

## 车辆操作与数据

| 指南 | 内容 | 语言 |
| --- | --- | --- |
| [启动指南](operations/bringup.md) | 选择 vehicle、mapping、race 或单独控制器入口 | 中文 |
| [已知地图 MPCC](operations/known-map-mpcc.md) | 地图/bundle 身份、后轴初值、启用、停止与连续圈 | 中文 |
| [录制指南](operations/recording.md) | launch 录包、runtime 日志、QoS、会话目录与计时口径 | 中文 |
| [车辆检查](operations/vehicle-checklist.md) | 设备、定位、权限与运行检查 | 中文 |
| [参考与 bundle 流程](../src/controller/docs/usage.md) | map 参考、速度规划、导出与目标 CPU 编译 | 中文 |
| [地图/参考工具](../tools/reference/README.md) | 保存 PGO 地图，恢复与地图对应的闭环参考 | 中文 |
| [校准资料](../src/aims_racer_system/docs/calibration.md) · [英文入口](../src/aims_racer_system/docs/calibration.en.md) | 已有响应证据、当前流程与采集指南归档 | 中文 / 英文 |

地图、参考输入、生成 bundle 和原始 bag 属于外部数据。clone 仓库提供源码与报告，不包含全部实录数据或可直接运行的车辆 bundle。

## 架构、控制与诊断

| 指南 | 内容 | 语言 |
| --- | --- | --- |
| [系统架构](architecture.md) | TF 归属、后轴坐标、话题、时间戳与估计/控制链 | 中文 |
| [MPCC 实现](../src/controller/docs/implementation.md) | 车辆模型、成本、几何、速度规划与执行历史 | 中文 |
| [原生控制器](../src/aims_mpcc_rt/README.md) | C++ 运行时、固定 bundle、launch 参数与诊断 | 中文 |
| [离线参考工具](../src/controller/README.md) | `aims_mpcc` 包及其离线职责 | 中文 |
| [定位 monitor](localization_monitor.md) | 可信锚点、新鲜度、epoch 与后台质量诊断 | 中文 |
| [RC 选择器](../src/ackermann_mux/README.md) | RC 通道、手动/自主仲裁、命令边界与超时 | 英文 |
| [车辆包](../src/aims_racer_system/README.md) | launch 组合与传感器/估计适配 | 中文 |
| [已安装系统助手](../src/aims_racer_system/scripts/README.md) | gyro 修正、后轴转换与 NDT 初始化 | 中文 |

## 验证与实测证据

| 资料 | 说明 | 语言 |
| --- | --- | --- |
| [开发验证](../verification/README.md) | 当前回归/重放工具、测试位置与所需 bundle 输入 | 中文 |
| [main 栈整合](reports/2026-10-10-main-field-stack-integration.md) | 外场默认配置一致性、NX 构建、实际安装来源与软件检查 | 中文 |
| [最新外场复盘](reports/2026-10-10-mpcc-field-review.md) | 分次运动时间、速度、跟踪、求解耗时与联合 CPU 负载 | 中文 |
| [全部日期化报告](reports/README.md) | 历史 LIO、EKF、gyro 零偏、车辆响应、控制与定位证据 | 中英混合 |

外场数据保留车辆、地图、配置与计时定义。软件检查和实际行驶分别报告。

## 历史设计资料

以下文档记录早期分支与已退役的运行时设计：

- [求解诊断与热启动](../src/controller/docs/solver-diagnostics.md)：早期 Python/IPOPT 控制器。
- [后端对比](../src/controller/docs/backend-comparison.md)：早期 IPOPT/acados/QP 实验接口。
- [NX 优化](../src/controller/docs/nx-optimization.md)：早期 Python 运行时优化。

旧操作指南通过[日期化报告](reports/README.md)查阅。当前 main 操作使用上面的安装与车辆运行指南。

## 归属与许可

[模型 NOTICE](../src/controller/NOTICE.md) · [原生运行时 NOTICE](../src/aims_mpcc_rt/NOTICE.md) · [固定依赖来源](../dependencies/manifest.json)

[返回项目首页](../README.zh-CN.md)
