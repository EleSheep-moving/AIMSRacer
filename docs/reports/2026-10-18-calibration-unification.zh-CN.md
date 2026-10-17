# 纵向电流、横向附着和动捕工具统一

主机与 NX 的源码和主安装已经统一。源码入口是 [`src/calibration/`](../../src/calibration/README.zh-CN.md)，现场环境加载各自 `AIMSRacer/install/setup.bash`。本文保留最初源码与安装归并时的验证快照；随后纳入主仓库的范围和验证见[当前源码发布记录](2026-10-18-code-consolidation-audit.zh-CN.md)。

| 目录 | ROS 包名 | 当前功能 |
|---|---|---|
| [longitudinal_current](../../src/calibration/longitudinal_current/README.zh-CN.md) | `aims_calibration` | A 恒速电流、B 加速、C 减速／制动；原生 C++ 状态机和操作监控 |
| [lateral_grip](../../src/calibration/lateral_grip/README.zh-CN.md) | `aims_lateral_grip` | 恢复 C++ 自动左右转、逐级升速、稳速采样和换向前实测停稳 |
| [mocap_validation](../../src/calibration/mocap_validation/README.zh-CN.md) | `aims_mocap_validation`、`vrpn`、`vrpn_mocap` | 动捕五量、FastLIO 和命令响应的只读采集分析 |

共用 PP 仍在 `src/aims_pp/`。旧 `src/aims_calibration` 和 `src/mocap_validation` 不再作为重复源码存在。旧实验工作区、迁移前源码、安装和现场快捷脚本已留存回退副本。

## 保留和恢复的行为

纵向四个 C++ 文件 `native_engine.cpp`、`native_node.cpp`、`native_runtime.cpp`、`native_operator.cpp` 与迁移前逐字节一致。变更限于目录、测试资源相对路径、文档和源码内现场快捷入口；没有修改 A/B/C 电流扫描或终止判据。

合并了原 NX 标定工作区已经使用的选择器状态发布：`/control/calibration_enabled` 仅在选中有效标定请求时为 true。新增发布代码不改变指令选择逻辑。回归检查真实选择器的状态 true／过期后 false，以及驱动虚拟串口链。

横向工具使用固定名义转角 `atan(wheelbase/R)`，不保证实车保持名义半径。速度请求渐进上升，达到实际目标、稳定后才累计采样；达不到目标时记录未完成并超时退出。左右换向前要求连续新鲜轮速确认停稳。没有新增弯中联合加减速试验，也不自动写入 `mu_safe` 或 MPCC 限值。

纵向启用服务保留 `/calib/enable`，横向使用 `/calib/lateral/enable`，避免误启另一种试验。二者共用 `/calib/vesc_command`，不能同时运行；选择器仍独占 `/vesc/command`。必要权限、控制数据新鲜度、最终命令一致性、唯一发布者、驱动起步电流替换关闭及日志写入有效是驾驶前提。退出时 0 A 是滑行请求，不代表物理停车。独立估计量质量异常只记录 WARN，保留原始数值及源时间。

横向输出默认保存到 `$HOME/aimsracer-data/sessions/lateral-grip/`；可显式配置。G-G 使用同一条后轴／质心加速度消息中的 AX/AY，异步 VX 与 yaw rate 只在有界源时间差内作近似对照，保留匹配时间差。固定转角的半径偏差和 `VX*r` 都不单独构成轮胎打滑或真实摩擦系数证据。

## 源码一致性

两端 `src/calibration/` 均为 1,418 个文件，目录 SHA256 为 `b80e0f5d8ca8ffc13c681922c84eac2b0acf40bc65181d8400751e3032964ec7`；`src/aims_pp/` 均为 31 个文件，目录 SHA256 为 `7b8cc10853ef5cd126816e2aebfac8970c81e12fe2a01232918563850d5d5b7a`。选择器源码 SHA256 同为 `033c66b84412ea73f3d6bcae65c3bff9541bf5673f062c01e41ca80d0cf4ddd0`。

目录哈希算法：排除 `.git`、`__pycache__`、`.pytest_cache` 和 `.egg-info`，以相对路径到文件 SHA256 的字典执行 Python `json.dumps(rows, sort_keys=True).encode()`，再计算 SHA256。它核验源码一致性；x86 与 ARM 的二进制各自构建，不要求二进制哈希相同。

## 两端验证

| 验证 | 主机 x86 Docker | NX ARM |
|---|---|---|
| 纵向、PP、动捕迁移后完整测试 | colcon 496 tests，0 errors／failures／skips | 同左 |
| 横向 core／写入失败／离线分析／ROS 进程测试 | colcon 18 tests，0 errors／failures／skips | 原隔离构建同样通过；主安装再次运行强化进程测试 |
| 主安装横向强化故障场景 | 12 passed | 12 passed |
| 主安装真实选择器／驱动 PTY | 3 passed | 3 passed |
| 主安装纵向 B/C 冻结配置启动 | 2 passed | 2 passed |
| launch 参数与旧 Vicon 兼容入口 | 已核验 | 已核验 launch 和实际包路径 |

colcon 数字包含 CTest／pytest 包装层计数，不是独立实车样本数。横向故障测试已修正清理阶段撤权可能掩盖具体故障的断言：所有非权限故障在权限保持有效时先验证故障原因和退出，随后才清理。冻结源时间的测试确认在新鲜度尚未超时前检测异常。全部 ROS 场景为 localhost 隔离 domain，驱动仅接虚拟串口。

实际加载核验：主机 Docker 三个工具均来自 `/work/install/`，NX 均来自 `/home/aims/AIMSRacer/install/`；纵向运行时的 `libaims_pp_runtime.so` 也来自该主安装。NX 现有状态估计系统的可执行文件哈希与迁移前一致，兼容横向 launch 为新增资源。

## 现场入口

NX 原短命令仍可用，但现在指向主源码内的入口：

```bash
source ~/calibration_full_step_env.bash
~/run_calibration_full_step.sh B  # 或 C；只启动会话，不自动启用
ros2 run aims_calibration inspect
```

地图、参考和冻结 A 基准保存在 `~/.config/aimsracer/calibration-site.bash`，沿用原现场值。其他终端同样加载该环境；旧终端不能用旧环境推断新版本已经加载。

统一工具操作及横向配置见[源码总入口](../../src/calibration/README.zh-CN.md)。本次没有启动实车动作，没有完成动捕独立精度验收，也没有测得摩擦边界。

原始构建／测试／部署日志与校验清单的本机归档位置为 `aimsracer-data/experiments/calibration-unification-20261018/` 和 `aimsracer-data/maintenance/2026-10-18-calibration-unification/`；上述数值、方法和状态已在本报告列明，阅读本报告不依赖外部目录。

后续补齐的离线模型、编辑器与现场脚本，以及更新后的源码哈希，见[遗漏代码归并审计](2026-10-18-code-consolidation-audit.zh-CN.md)。
