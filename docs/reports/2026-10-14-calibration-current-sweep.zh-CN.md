# B/C 标定软件验证

[English](2026-10-14-calibration-current-sweep.md) · **简体中文**

## 范围与版本

分支 `feat/calibration-current-sweep` 以 NX 最新 A 工作目录 `867dad3` 为基础，
八项外场补丁已保存在 `564d85c`。A 采集设置、profile 与隐式 v1 格式保留。
B/C 继续使用 native 选择器与驱动链，冻结基于指定基准的计划，输出 v2 证据。
A 原源码、编译环境和录包未替换；最终同步版本和文件哈希记录在 NX 验证目录。

## 验证结果

| 检查 | 结果 | 能说明什么 |
| --- | --- | --- |
| 主机纯 Python 标定测试 | 146 通过 | 计划、状态机、CLI、证据、包与配置契约 |
| NX 完整标定与系统标定 launch 测试 | 195 通过，63.35 秒 | 包含 ROS 适配器与已安装 native 链测试 |
| NX 编译 | PP、标定、系统三个包成功 | 整车已有编译依赖由下层环境提供 |
| 修改指南的文件链接 | 141 项，无缺失 | 中英操作文档、索引与包指南 |
| NX 原 A 工作目录待提交补丁 | 字节完全一致 | SHA256 `7c1a629fc551ee52dd9fbd1a4822dd42c18bd671bbaee557298aa5d6910a98a6` |

NX 证据目录：
`/home/aims/aimsracer-data/verification/calibration-current-sweep/nx-final-20261013-v1byGl/`。
保存 `build.log`、`tests.log`、`tests.xml`、测试 JSON/CSV 与 ROS 日志。
ROS 使用隔离域和 localhost，驱动测试只打开 `/dev/pts` 虚拟 VESC，未启用实车实验。

新增四项已安装链路测试覆盖 B 成功、B 两个区间的电流能力跳档、C 104 mA／
零电流／负电流，以及 C 正电流失败后仍继续滑行和制动。原七项串口测试继续覆盖
A、显式正负电流、权限撤销以及 CLI 启动、停止和中止。
遥测由脚本产生，用于验证编码与状态逻辑，不代表车辆动力学。

回归修复涉及日志溢出与最新失败检查点、COMPLETE 后的故障、基准与计划来源、
遥控转向的车辆身份、初始化位姿参数、启动前实际跟踪配置检查、时间戳浮点精度、
以及避免在 native ROS 回调内抛出 Ctrl+C 异常。中止问题先复现两次，修复后
独立串口测试及最终完整测试均通过。

## 选用的真实 A 结果

源结果：
`/home/aims/aimsracer-data/sessions/2026-10-13/222944_118387098-pp-speed-hold-1to5-repeat-battery/20261013T142946Z-1849a491/trials.json`。
九档均通过离线验收。派生基准位于：
`/home/aims/aimsracer-data/calibration/baselines/20261013-repeat-battery.json`，哈希为
`e2e367319a67e84a4161e46b1d2936a0c1abc06146db6a6d714f5b90afa7ad62`。
电压仅取 A 实际纳入的分段样本，均值约从 16.2 降到 15.9 V；
源车辆哈希与仓库内专用车辆配置相同。

已安装离线命令生成 **B 145 项、C 184 项**，旁边保留
`20261013-B-preview.json` 与 `20261013-C-preview.json`。
这些是能力判断跳档之前的计划数量，不是已完成的 B/C 数据。

## 尚待验收

实车小电流和外场 B/C 验收尚未进行。当前软件证据不证明停车距离、实际电流
准确性、加速度响应或大电流能力。实车流程见 [B/C 操作指南](../operations/current-sweep.zh-CN.md)。
