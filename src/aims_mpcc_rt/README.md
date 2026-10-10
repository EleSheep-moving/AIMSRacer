# Native acados/C++ MPCC

`aims_mpcc_rt` 是当前在线控制包。离线 Python 冻结车辆配置、周期 quintic 参考和曲率速度规划，生成 acados SQP-RTI C；在线 C++ 加载与验证 bundle，处理状态/命令历史、RTI、接管、停车和 ROS 输出。

当前默认实跑配置为 **N15、dt=.1 s、RTI 至多两次、20 Hz 请求上限、50 Hz 命令**；完整交付预算 50 ms、预计接管提前量 20 ms、原始 source TTL 1.2 s。六个物理状态、三个控制与三个辅助历史状态，保留后轴运动学、转向一阶滞后和精确周期参考。当前 `rate_bounded_v2`、独立加减速约束，组合加速度椭圆与 corridor 关闭；规划 ay=1 仍影响曲率速度。

[外场复盘](../../docs/reports/2026-10-10-mpcc-field-review.md)记录了 Orin NX 在真实 FAST-LIO2/EKF/NDT 链路下多档实跑。3.5 m/s 为配置巡航，规划峰值约 2.99、实测约 2.89 m/s；不能据标签声称达到 3.5。运动窗口含起步与制动，不是 flying lap。

## 准备与构建

同一 workspace 构建：

```bash
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

离线 export/build 使用本包 `scripts/export_bundle.py` 与 `scripts/build_bundle.py`，完整参数与环境见 [MPCC 使用](../controller/docs/usage.md#离线生成与目标平台构建)。固定 acados commit 及子模块见[依赖清单](../../dependencies/manifest.json)。目标安装为 `dependencies/work/acados/install`；offline 设置 `ACADOS_SOURCE_DIR` 指向 `source`、`ACADOS_INSTALL_DIR` 指向 `install`，runtime 只需要当前安装的动态库路径。

完整 bundle 必须保留 `input_reference/`、`input_config.yaml`、source/native manifests、生成源码和目标 CPU 库。manifest 绑定配置、参考、生成器、平台和加载依赖哈希。在线启动拒绝不兼容产物，不在线生成编译。任何配置/参考改变都离线创建新 bundle。

## 启动

完整车辆使用：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

连接已运行的车辆/定位图可单独启动：

```bash
ros2 launch aims_mpcc_rt mpcc.launch.py \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010"
```

launch 自动从 bundle 派生 reference/config；N/dt 来自 manifest。默认 disabled、`auto_start=false`、`repeat_laps=false`。可选 `auto_start=true` 在就绪后执行一次 enable，默认等待 60 s；显式 stop 取消等待，成功后不重复启用，fatal 不重启。`repeat_laps=true` 连续圈直到 stop。初始化 pose 和完整参数见[已知地图](../../docs/operations/known-map-mpcc.md)与[使用指南](../controller/docs/usage.md#在线启动参数)。

## 接口与执行边界

| 接口 | 作用 |
| --- | --- |
| `/odometry/filtered` | 后轴 `odom/base_link` 状态 |
| `/ackermann_cmd` | 已转发输入历史 |
| `/control/autonomy_speed_enabled` | RC 自主 speed 权限 |
| `/localization/status`、anchor/map identity | 可信地图资格与对齐 |
| `/drive` | 唯一 native 控制提议，由 RC 选择器执行 |
| `/mpcc/status`、reference/prediction | 状态与显示 |
| `/mpcc/enable` | SetBool 显式启用或停止 |

接管验证原始 source 年龄、实际前缀、新的物理状态与对齐，拒绝候选不延长旧计划寿命。输入有效性、定位资格、命令归属、有限数与物理界限各有责任；更快求解不放宽这些边界。完整执行链见[实现指南](../controller/docs/implementation.md#measurement-time-computation-and-takeover)。

`log_directory` 写 `runtime.csv`；整车 `record=true` 自动将其与 raw/corrected IMU、LIO、EKF/wheel、TF、NDT/MPCC 状态 bag 放在同一会话。worker 与 delivery 时长、接管和输出计时分别保留，详见[录制](../../docs/operations/recording.md)。

软件验证程序位于 `verification/`，不安装为车辆生产入口。历史数值、隔离 ROS、NX 联合负载与实跑报告保留在 [reports](../../docs/reports/README.md)。版权和来源见 [NOTICE.md](NOTICE.md)及 [controller NOTICE](../controller/NOTICE.md)。
