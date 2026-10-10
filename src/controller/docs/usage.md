# Native MPCC 使用

当前在线控制由 `aims_mpcc_rt/mpcc_rt_node` 执行。`aims_mpcc` 仅提供参考录制与准备，模型和数值工具用于离线生成。整车操作见[启动](../../../docs/operations/bringup.md)、[地图初始化](../../../docs/operations/known-map-mpcc.md)、[录制](../../../docs/operations/recording.md)。

## 当前实跑配置

默认 [vehicle.yaml](../config/vehicle.yaml)为 2026-10-10 v35 参数：N15/dt=.1（1.5 s 预览）、RTI 至多两次、20 Hz 请求上限、50 Hz 命令。巡航 3.5、硬上限 4、加速/制动 1、曲率规划 ay=1，单位分别为 m/s、m/s²。后轴 footprint 前 0.52、后 0.10、半宽 0.16 m；转向滞后 0.08 s 是现有车辆 yaw 响应的模型近似。

`enforce_corridor=false` 和 `combined_accel_constraint_enabled=false` 是本次实跑条件。物理速度/加减速/转角/转向速率、有限数和输入/定位资格仍检查；规划 ay 不提供运行时轮胎约束保证。规划峰值约 2.99、实测约 2.89 m/s。[外场复盘](../../../docs/reports/2026-10-10-mpcc-field-review.md)说明运动窗口和 COMPLETE 的不同计时口径。

## 准备参考

当前部署 bundle 已包含参考，正常运行无需重复准备。新参考先录制：

```bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
```

手动前进一圈并保留少量重叠。默认录 `/odometry/filtered` 的 `odom/base_link`；建图可改 `odom_topic` 为 `/rear_axle/lio_odom`。`odom` CSV 与当前会话原点绑定。要在重新启动后复用，先离线关联保存地图的 pose，形成 `map/base_link` CSV。历史关联例子见[地图参考报告](../../../docs/reports/2026-09-28-map-reference.md)。 保留的恢复脚本与输入说明见 [tools/reference](../../../tools/reference/README.md)。

```bash
ros2 run aims_mpcc prepare_path /absolute/path/to/closed_map.csv \
  "$HOME/aimsracer-data/reference-new" \
  --vehicle-config src/controller/config/vehicle.yaml \
  --left-width 0.5 --right-width 0.5 \
  --map-file "$HOME/maps/20260928_010503/map.pcd"
```

`--map-file` 为 map-frame CSV 绑定 PCD SHA-256，不执行坐标转换。左右宽度必须反映确认过的空间，示例值 0.5 m 是课程模型。`--start-time`、`--end-time` 可截取一个闭环。准备过程拒绝无效闭合、倒车、不连续、交叉、曲率和坐标/几何不匹配，保留原始 CSV，输出 `path.csv`、`metadata.json`、`raw.csv`；输出目录必须新建。

周期 quintic 样条描述固定几何；速度由曲率与周期加速/制动传播规划，不照抄驾驶员 CSV 速度。当前不接收在线轨迹 topic，也未实现按位置速度区域或在线赛线优化。

## 离线生成与目标平台构建

先按[源码准备](../../../docs/operations/bringup.md)安装固定 acados，离线 Python 环境需有 CasADi 3.7.2、NumPy、SciPy、PyYAML 和固定源码的 acados_template。在 workspace 根目录设置：

```bash
export ROOT="$PWD"
export ACADOS_SOURCE_DIR="$ROOT/dependencies/work/acados/source"
export ACADOS_INSTALL_DIR="$ROOT/dependencies/work/acados/install"
export LD_LIBRARY_PATH="$ACADOS_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROOT/src/controller:$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
python3 src/aims_mpcc_rt/scripts/export_bundle.py \
  --config src/controller/config/vehicle.yaml \
  --reference "$HOME/aimsracer-data/reference-new" \
  --output "$HOME/aimsracer-data/bundles/reference-new-v35" \
  --horizon 15 --dt 0.1 --source-only
python3 src/aims_mpcc_rt/scripts/build_bundle.py \
  --bundle "$HOME/aimsracer-data/bundles/reference-new-v35" \
  --acados-install "$ACADOS_INSTALL_DIR"
```

`--source-only` 生成固定 C 和模型数据，后一步针对当前 CPU 编译。x86 生成的完整 source bundle 可复制到 NX，再在 NX 执行 `build_bundle.py`；不能把 x86 共享库当作 NX 产物。完成目标构建后保留整个 bundle，包含 `input_reference/`、`input_config.yaml`、`manifest.json`、`native_manifest.json`、生成 C 和共享库。改变 config、reference 或生成器时离线创建新 bundle。

在线节点验证哈希、ABI、平台及加载库，不在线生成或编译。构建证据与实车效果分别记录；软件回归放在 `verification/`。

## 在线启动参数

推荐整车入口：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

| race 参数 | 默认 / 含义 |
| --- | --- |
| `map_file` | 必填，准确的不可变 PCD |
| `artifact_directory` | 必填，已在目标平台构建的完整 bundle |
| `initial_pose` | 空；可填 map 中 base_link 的 `x y z roll pitch yaw`，角度 rad |
| `auto_start` | false；等待同一组 enable 条件后启动一次 |
| `repeat_laps` | false；单圈停车，true 连续圈 |
| `record` | false；true 自动 bag 与 runtime |
| `session_directory` | 空；自动日期会话目录，指定时须新目录 |
| `log_directory` | 空；只记录 native runtime；record=true 选择会话目录 |

需要与现有车辆/定位图连接时，单独 native 入口为：

```bash
ros2 launch aims_mpcc_rt mpcc.launch.py \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010"
```

native launch 从 bundle 派生 `input_reference/` 和 `input_config.yaml`，horizon/dt 来自 manifest。无需重复指定路径、车辆配置或时域。native 入口另有 `auto_start_timeout=60`、`solve_frequency=20` Hz、`solver_timeout=.05` s、`handover_delay=.02` s、`odom_topic=/odometry/filtered`；参数启动后固定。完整交付预算包含 worker 与 delivery 等待，不等于仅 solver 时长。原始 source TTL 为 `0.8×N×dt=1.2 s`，不会因结果丢弃或新交付而续期。

## 初始化、启用与停止

初始化按[已知地图](../../../docs/operations/known-map-mpcc.md)提供 map/base_link pose 或 RViz。`/mpcc/status` 与 `/localization/status` 描述实际资格。手动启用/停止：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

enable 要求 solver ready、新鲜后轴状态、实际转发历史、RC 权限、可信地图锚点、唯一 `/drive` 发布者、车辆静止和参考方向匹配。`auto_start=true` 复用这组条件，在默认 60 s 内成功后只启动一次；显式 stop 取消等待。完成或故障不自动重新启用，fatal 不重启。

`repeat_laps=false` 从实际启用位置计一圈并停车，true 连续圈直到 stop。使用 RC/急停处理现场紧急情况，撤销权限、停稳后关闭 launch。日志内容与计时口径见[录制](../../../docs/operations/recording.md)。
