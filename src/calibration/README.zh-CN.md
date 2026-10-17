# 标定与动捕校对

本目录是主机和 NX 的共同源码入口。ROS 包名与源码文件夹名分别列出，既区分用途，也保留已有标定命令。

| 源码目录 | ROS 包 | 用途 |
|---|---|---|
| [longitudinal_current](longitudinal_current/README.zh-CN.md) | `aims_calibration` | 纵向电流：A 恒速基础电流、B 加速、C 减速／制动 |
| [lateral_grip](lateral_grip/README.zh-CN.md) | `aims_lateral_grip` | 固定名义转角、左右转、逐级升速的自动横向标定 |
| [mocap_validation](mocap_validation/README.zh-CN.md) | `aims_mocap_validation`、`vrpn`、`vrpn_mocap` | 只采集分析：五量、FastLIO、命令响应的动捕校对 |

纵向标定共用的 PP 库仍位于 `src/aims_pp/`。三个工具的日志、录包、现场配置和分析结果放在 `aimsracer-data/sessions/`，源码不依赖某个历史实验工作区。历史实验工作区保留作回退和溯源，不是新的源码入口。

## 构建和终端环境

在 ROS Humble 环境中，工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
src/calibration/build.sh
source install/setup.bash
colcon test --packages-select aims_calibration aims_lateral_grip aims_mocap_validation
colcon test-result --verbose
```

主机没有原生 ROS 时，在 Docker 内构建。`build.sh` 包含固定 VRPN 的构建兼容参数；上游源码和许可证保持原样。现场每个终端使用同一份 `install/setup.bash`，可用以下命令确认加载来源：

```bash
ros2 pkg prefix aims_calibration
ros2 pkg prefix aims_lateral_grip
ros2 pkg prefix aims_mocap_validation
```

## 统一操作入口

```bash
# 纵向：已有整车标定会话时，只检查／启用／停止，不再启动第二套车辆链
src/calibration/run.sh current inspect
src/calibration/run.sh current start
src/calibration/run.sh current stop

# 横向：launch 仅启动标定节点；先按横向说明检查现有车辆链和物理限值
src/calibration/run.sh lateral launch --show-args
src/calibration/run.sh lateral inspect
src/calibration/run.sh lateral stop

# 动捕：只读采集与分析
src/calibration/run.sh mocap inspect --config /absolute/path/config.json
```

`current launch` 转入整车 `calibration.launch.py`，需要显式填写场地参数；`lateral launch` 不启动第二个整车链。纵横向标定共享同一标定指令通道，不能同时运行。纵向启用服务为 `/calib/enable`，横向为 `/calib/lateral/enable`，避免误启另一种试验。节点启动与人工启用分开；录包和动捕工具不发布车辆控制指令。具体参数与操作步骤以各子目录说明为准。

## 测量的含义

横向模块记录后轴、质心五量和各自源时间；以同一条加速度消息的 AX／AY 作 G-G 图。`VX × yaw rate` 只作为近似对照。固定转角不保证实车保持固定半径；实际半径偏差也不能单独证明轮胎打滑。左右横向工况只是完整纵横向包络的一部分，不自动生成控制器附着上限。

[项目标定指南](../../docs/operations/calibration.zh-CN.md) · [纵向完整电流协议](../../docs/operations/full-step-current.zh-CN.md) · [动捕指南](../../docs/operations/mocap-validation.zh-CN.md)

纵向目录还包含 [轮速—电流离线拟合／查询](longitudinal_current/docs/wheel-current-model.zh-CN.md)，通过 `current wheel-current-fit|wheel-current-query` 调用；这些命令不接管车辆控制。
