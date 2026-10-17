# 标定实际启动链的 C++ 迁移与 NX 安装核验

2026-10-16，补齐了此前只部署到主工作区的 C++ 迁移。标定状态机、嵌入的 PP 跟踪、独立 PP 节点和持续操作监控现在位于本仓库 `src/aims_calibration`、`src/aims_pp`；NX 原标定工作区也已更新。安装后的隔离 ROS 测试通过，未启动实车整车会话或发出实车驾驶指令。

## 实际入口

此前 `~/calibration_full_step_env.bash` 加载的是 `/home/aims/aimsracer-data/code/calibration-full-step/install/setup.bash`，该目录保留旧 Python 传感器节点；主工作区的 C++ 程序不会自动替换另一个 workspace。

现已核对该实际安装目录中的八个入口为 ARM aarch64 ELF：

| 包 | 原生入口 | 作用 |
| --- | --- | --- |
| `aims_racer_system` | `livox_gyro_bias`、`lio_to_rear_axle`、`longitudinal_acceleration` | 零偏修正、后轴转换和独立观测器 |
| `aims_calibration` | `longitudinal_calibration` | 50 Hz 状态机、PP 转向、看门狗、后台证据写入 |
| `aims_calibration` | `inspect`、`start`、`stop` | 检查、显式启用、持续监控和停用 |
| `aims_pp` | `pp_node` | 独立 PP 节点，默认停用 |

标定进程内部使用原生 PP 库，不额外启动独立 `pp_node` 来争用命令发布。`imu_to_rear_axle` 原本已是 C++。旧三个传感器 `.py` 包装程序不再出现在更新后的安装前缀中。

ROS launch 和录包工具继续使用原来的 ROS 入口。冻结计划、参数、参考与文件身份在启动时由一次性 Python 准备程序读取；该进程退出后才启动原生持续运行。`baseline`、`plan`、`results` 与数值参考保留 Python；`start` 的持续监控已是 C++，完成后的离线结果验收可短暂调用 Python。

源码：标定状态机（记录中的工作区路径：`src/aims_calibration/src/native_engine.cpp`）、ROS 适配与类型化参数（记录中的工作区路径：`src/aims_calibration/src/native_node.cpp`）、操作监控（记录中的工作区路径：`src/aims_calibration/src/native_operator.cpp`）、PP 库（记录中的工作区路径：`src/aims_pp/src/runtime.cpp`）、标定启动（记录中的工作区路径：`src/aims_racer_system/launch/calibration.launch.py`）。

## 保留的现场行为

迁移以 NX 当前现场修改为基准，先归档原始源码与差异，再实施移植。A 的连续/累计采样、旧 interval 扫描、B/C full-step、实际 `avg_iq` 交接、实测轮速端点、稳态窗口、重试及 partial 记录继续使用原有规则。

C 保留首次新鲜轮速达到或超过起点即请求切流的策略，未恢复起始稳速等待；当前 C profile 的 `protection_reverse_guard_enabled: false` 也被保留。未增加额外的实测轮速运行上限。遥控权限、源时间与接收年龄、独占发布者、定位健康、地图身份、时钟和日志故障仍可中止采集。

原 profile 的 `currents: []` 先转换成明确类型的原生参数，原启动命令可以继续使用。配置在运行中固定，除了 `use_sim_time`，参数服务拒绝更改；更换车辆、参考或试验配置需要重启。

## 软件验证

| 层级 | 结果 | 证据 |
| --- | --- | --- |
| 本机 Humble 标定注册测试 | 17 组通过 | [CTest](assets/2026-10-16-native-calibration/calibration-registered-ctest.log) |
| Python/C++ 状态机对照 | 113 条轨迹、6,382 次公开接口事件通过 | [控制台](assets/2026-10-16-native-calibration/native_engine_parity_console.log) |
| 本机 PP 与主仓库启动契约 | PP 70 项、启动 29 项通过 | [PP](assets/2026-10-16-native-calibration/pp-native-tests.txt)、[启动](assets/2026-10-16-native-calibration/main-calibration-launch-tests.log) |
| NX 软件测试工具汇总 | 583 项，0 错误、0 失败、0 跳过 | [汇总](assets/2026-10-16-native-calibration/nx-test-result-final.log) |
| 更新后实际标定安装目录 | 标定/操作进程 15 项、独立 PP 进程 5 项通过 | [标定](assets/2026-10-16-native-calibration/deployed-calibration-process-tests.log)、[PP](assets/2026-10-16-native-calibration/deployed-pp-process-tests.log) |
| 实际安装与源码 | 8 个 ARM 原生入口、190 个源码文件哈希核对一致 | [入口](assets/2026-10-16-native-calibration/deployed-native-install.json)、[源码](assets/2026-10-16-native-calibration/deployed-source-verification.json) |

这些数字包含 CTest 包装、Python 参考测试和进程测试等不同层级，不能相加为独立场景总数。对照程序明确排除了 32 个直接修改 Python 内部状态或没有接口事件的用例；新增公开接口轨迹覆盖 UNIX 大时间戳、C 首次轮速触发和反向保护开关。时间戳比较使用零相对容差和最多四个 ULP 的绝对容差，并检查整数、布尔类型及完整事件数。

原生进程测试覆盖默认停用、启用预检、原始 B/C YAML、合格定位下的 C 切流、源/接收时间失效、冻时钟、重复发布者、队列/磁盘失败、操作员中断和 A 结果离线验收。NX 测试适配了缺少 `/proc/.../children` 的内核，通过 PPid 检查持续子进程；模拟时钟测试保留因果传感器时间与真实命令回显时间，避免跨话题传输顺序造成错误的测试输入。

审查发现并修复了每次定时回调复制累计样本的问题，运行状态现在通过有界 `status()` 读取，完整结果只在提交时复制。20,000 点合成采样检查属于软件开销验证，不能解释成 NX 实时或实车延迟结论。

## 安装、运行与回退

NX 先在 `/home/aims/aimsracer-data/code/calibration-native-20261016` 编译三个包并验证，然后安装已验证的 ARM 包到原 `calibration-full-step/install`，调整安装前缀的环境脚本。实际安装再次通过原生进程测试，动态 PP 库也解析到原标定工作区。[构建](assets/2026-10-16-native-calibration/nx-stage-build.log)、[动态库路径](assets/2026-10-16-native-calibration/deployed-library-resolution.txt)、[验证清单](assets/2026-10-16-native-calibration/validation-summary.json)。

原脚本和现场地图、轨迹、冻结基准保持原来的路径。B 仍使用：

```bash
~/run_calibration_full_step.sh B
```

另一个终端使用：

```bash
source ~/calibration_full_step_env.bash
ros2 run aims_calibration inspect
ros2 run aims_calibration start
```

C 使用新会话，将第一个命令的 `B` 改为 `C`。仍需现场确认定位、轨迹和遥控状态后显式启用；软件验证不代替这一步。完整操作见[恒电流采集](../operations/full-step-current.zh-CN.md)。

NX 回退归档为 `/home/aims/aimsracer-native-calibration-20261016/backup-before-deploy.tgz`，原三个安装前缀还保存在同目录的 `install-before-deploy/`。车辆停稳、结束当前会话后，可将归档解压回 `/home/aims/aimsracer-data/code/calibration-full-step` 并重新加载环境恢复此前源码与安装。完整本机证据位于 `/home/elesheep/aimsracer-data/experiments/native-calibration-port-20261016/`；仓库只保留便携摘要和日志。

[返回电流标定入口](../operations/calibration.zh-CN.md) · [横向加速度标定与校对](../operations/lateral-acceleration-calibration.zh-CN.md)。本报告的日期、数值与证据边界保持原口径。
