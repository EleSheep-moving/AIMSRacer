# AIMSRacer

PolyU AIMS Lab 的 ROS 2 Humble 无人赛车栈。当前主流程运行于 Orin NX 16 GB，连接 KKPIT ZQR 1/7 底盘、Livox MID360、VESC 和 RadioMaster Pocket ELRS。

当前主流程为 **FAST-LIO2 → 后轴 EKF → 已知地图 NDT → native acados/C++ MPCC → RC 选择器 → VESC**。同一 workspace 完成源码准备、构建和启动。地图、参考、生成 bundle 与录包存放在源码目录外。

## 准备与构建

新机器先克隆 main；已有 checkout 从子模块准备步骤开始。在 workspace 根目录、bash 终端执行；ROS 2 Humble 和系统依赖的安装见[部署指南](docs/deployment/README.md)，设备规则见 [rules](rules/README.md)。

```bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
```

依赖脚本按[固定清单](dependencies/manifest.json)准备 NDT 及已核验补丁、Livox ROS 2、serial 和必要 SDK。acados 安装到 `dependencies/work/acados/install`。生成参考和 bundle 的离线流程见 [MPCC 使用](src/controller/docs/usage.md)。

## 运行已知地图

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

默认不自动启用，单圈结束后停车。通过 [base_link 地图初值或 RViz](docs/operations/known-map-mpcc.md)初始化 NDT，检查定位与 RC 权限，再调用 `/mpcc/enable`。`auto_start:=true` 可在就绪后执行一次启动；显式停止会取消等待，完成或故障后不会自动重新启用。`record:=true` 自动录 bag 和 `runtime.csv`。

手动驾驶启动 `vehicle.launch.py`；建图启动 `mapping.launch.py`。三种入口选择一个，详见[启动](docs/operations/bringup.md)。

## 接口与实跑边界

`base_link` 位于后轴中点。EKF 独占 `odom → base_link`，NDT 在已知地图运行时拥有 `map → odom`；建图时由 PGO 拥有该边。`/drive` 是 MPCC 提议，RC 选择器输出的 `/ackermann_cmd` 才是转发给底盘的命令。[架构](docs/architecture.md)说明坐标、时间戳和命令归属。

默认车辆配置复现 2026-10-10 后期实跑：N15、dt=0.1 s、RTI 至多两次、巡航目标 3.5 m/s、硬上限 4 m/s、加速/制动 1 m/s²、曲率规划 ay=1 m/s²，组合加速度椭圆关闭。规划峰值约 **2.99 m/s**，两轮实测峰值约 **2.89 m/s**；运动窗口含起步与停车，不是 flying lap。配置值不代表实车达到的速度。[外场复盘](docs/reports/2026-10-10-mpcc-field-review.md)保留原始实验条件与计时口径。

| 任务 | 当前指南 |
| --- | --- |
| 启动、录制、停车 | [启动](docs/operations/bringup.md) · [录制](docs/operations/recording.md) · [车辆检查](docs/operations/vehicle-checklist.md) |
| 初始化保存地图 | [已知地图 MPCC](docs/operations/known-map-mpcc.md) |
| 准备参考和 bundle | [MPCC 使用](src/controller/docs/usage.md) |
| 阅读控制器实现 | [实现](src/controller/docs/implementation.md) · [native 包](src/aims_mpcc_rt/README.md) |

系统约定放在 `docs/`，包内说明链接到这些约定。日期化[报告](docs/reports/README.md)是历史证据。版权与来源见各包 `NOTICE.md`、许可证及下列致谢。

## 🙏 Acknowledgement
This project would not be possible without the use of multiple great open-sourced code bases as listed below:

- 🏎️ [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- 🏁 [QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- 🎯 [Original upstream system package](https://github.com/f1tenth/f1tenth_system)
- 📡 [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- 🔀 [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- ⚡ [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- 🗺️ [FAST-LIO2_ROS2 (maintained fork)](https://github.com/EleSheep-moving/FASTLIO2_ROS2.git)

##### 🏛️ Hardware and basic software were developed at PolyU AIMS Lab.
##### 🎓 Currently pursuing MPhil at PolyU AIMS Lab, with ongoing development in progress.
