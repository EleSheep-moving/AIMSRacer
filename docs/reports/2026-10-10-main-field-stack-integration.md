# 2026-10-10 场地版本整合到 main

## 基准与范围

按用户要求，以最后两次 V35 外场运行的版本为基准整理入口、安装内容与文档。原代码 `5ad071c`，FAST-LIO 子模块 `afe5f4c`；NDT、ndt_omp 与 acados 的完整固定版本及两份原样补丁见 [依赖清单](../../dependencies/manifest.json)。原始场地结果见[外场复盘](2026-10-10-mpcc-field-review.md)。

直接核对 NX 当日启动记录和已安装 launch：18:02 启动的是 `base_orin_livox_bringup_v2`，无命令行参数覆盖，后期 V35 沿用该 base/NDT。重组后 launch 的 Node、namespace、参数文件、remap 和后轴 include 参数相同；节点顺序、恒等 remap 和控制台输出属于整理。CH10 手动电流映射为 3–100 A，油门死区 50；VESC 驱动配置的 ±80 A 范围是另一层参数，均保留。建图入口的 20 A 不作为这次实跑基准。

控制模型、core/execution/history/output、RC 选择器、后轴与 gyro/EKF 链路保留实跑源码/数值。包括 NX 的约束处理、ROS 时钟排队、同 epoch 锚点更新、FAST-LIO IMU 连续性修复和两份 NDT 补丁。默认车辆 YAML 与 V35 bundle 的全部展开配置相同：N15、dt=.1、RTI2、目标3.5、上限4、加速/制动1、曲率规划ay1、组合椭圆关闭；request20Hz、预算50ms、lead20ms，输出周期保持20ms。

## 当前入口与安装

- `vehicle.launch.py`：Livox、FAST-LIO、后轴适配、gyro、EKF、RC/VESC。
- `mapping.launch.py`：建图 TF 安排与 PGO，支持 `record`。
- `race.launch.py`：整车、保存地图 NDT、native MPCC，支持 `record`、显式 `initial_pose`、`auto_start`、`repeat_laps`。
- `aims_mpcc_rt/mpcc.launch.py`：单独 native 控制器；bundle 决定车辆配置、参考和时域。

默认 `record=false`、`auto_start=false`、`repeat_laps=false`。自动启用是一次性的可选 native 启动行为：复用原 enable 判断，停止/完成/故障后不自动再启用；已见可信锚点后发生 epoch 故障，会取消待启用状态。初值 CLI 在原60秒 deadline内等待 NDT ACTIVE，避免并行启动时读取到暂时 INACTIVE 就退出。

生产安装只含当前控制与适配入口；Python 控制包仅有 `record_path`、`prepare_path`，RC 包仅安装 `joystick_control_v2`。旧 IPOPT/QP/Python 驾驶入口、旧 Nav/ICP launch、shadow、监督包装与实验资格脚本不进入生产安装。当前 replay 工具在 [verification](../../verification/README.md)，准备工具在 `tools/`；旧操作教程保存为日期归档，Git 保留旧实现，bag 和外部实验数据没有删除。

## 桌面验证

环境：ROS Humble Docker，新源码工作区、仅 ROS 系统底层环境、重新准备的固定依赖与 acados。没有启动真实整车图。

| 检查 | 结果 |
|---|---|
| 整车 selected 依赖闭包完整构建 | 18 packages finished |
| 控制器离线回归，新 acados source/install | 410 passed，无跳过 |
| native CTest，含14个一次启动场景 | 45/45 passed |
| 后轴、gyro、定位协议、初始化与 launch CTest | 11/11 passed |
| 离线 exporter/分数间隔、依赖准备和 launch等 Python 检查 | 70 passed |
| 安装后的错误启动参数与 RC 合成输入 | 9 passed |
| 安装后的公开 launch `--show-args`、可执行入口核对 | 通过；不启动节点 |

以上集合存在覆盖重叠，不相加作为总测试数。新 default exporter 未额外传 horizon，产物确认 N15/.1；新 config 与 V35 全等，输入参考 CSV/metadata 字节相同，全部 OCP 维度、成本缩放和上下界相同。桌面重新拟合的参考多项式系数最大差约2.31e-10，规划速度最大差约5.35e-12 m/s，源于桌面数值库与 NX 环境差别；不是轨迹/权重调参。两个离线 C kernel 也包含于 bundle 源码归档。

完整构建补齐了干净环境缺少的 PCL、GTSAM、udp_msgs 等依赖；新安装布局补齐了实际 CMake 生成的 acados link metadata 和 SDK 默认 Tera。初次缺依赖与无效测试 domain 的失败日志均保留，没有当作成功或隐藏。

## 部署与证据

当前启动方式见[启动指南](../operations/bringup.md)。NX 以 `~/AIMSRacer` main 和该目录的标准 `install` 为生产环境；重建前保存旧 build/install，保留既有外场 worktree 与数据用于回退。地图与 bundle 在源码外，分别显式选择，不能用源码 clone 假定它们存在。

证据目录：本机 `~/aimsracer-data/sessions/2026-10-10/mpcc-main-integration/`，包含完整依赖审计、实际 launch 字典、源码/补丁 SHA、原始测试及构建日志、bundle 配置对照。NX 部署与 ARM 验证结果写在同日期证据目录；本文桌面通过结果不代表新组合 launch/可选自动启用已经重新完成外场闭环验收。
