# AIMSRacer

![AIMSRacer — Orin NX 上的已知地图定位与轨迹跟踪](docs/assets/aimsracer-banner.svg)

<p align="center">
  <a href="README.md">English</a> · <strong>简体中文</strong><br/>
  <a href="#quick-start">快速开始</a> · <a href="#architecture">系统架构</a> ·
  <a href="#defaults">默认配置</a> · <a href="#results">实测结果</a> · <a href="docs/README.zh-CN.md">文档索引</a>
</p>

**PolyU AIMS Lab** 开发的 ROS 2 无人赛车栈。在先验 LiDAR 地图中跟踪提前保存的轨迹，结合连续局部估计、NDT 全局定位和原生 acados/C++ MPCC 控制器。

**当前平台：** Orin NX 16 GB · Ubuntu 22.04 / ROS 2 Humble · KKPIT ZQR 1/7 底盘 · Livox MID360 · VESC · RadioMaster Pocket ELRS。

<a id="navigation"></a>
## 按任务查找

| 我想…… | 从这里开始 |
| --- | --- |
| 安装与构建 | [部署指南](docs/deployment/README.md) · [Orin NX](docs/deployment/orin.md) · [x86 / NUC](docs/deployment/nuc.md) |
| 手动驾驶、建图或跟踪保存轨迹 | [启动指南](docs/operations/bringup.md) · [已知地图初始化](docs/operations/known-map-mpcc.md) |
| 准备地图坐标系参考和求解器 bundle | [参考与 bundle 流程](src/controller/docs/usage.md) · [地图/参考工具](tools/reference/README.md) |
| 录制与分析一次运行 | [录制指南](docs/operations/recording.md) · [运行时诊断](src/aims_mpcc_rt/README.md) |
| 理解或修改控制器 | [MPCC 实现](src/controller/docs/implementation.md) · [原生运行时](src/aims_mpcc_rt/README.md) |
| 查看接口与定位状态 | [坐标系与话题](docs/architecture.md) · [定位健康](docs/localization_monitor.md) |
| 查看实测或执行回归 | [外场结果](docs/reports/2026-10-10-mpcc-field-review.md) · [开发验证](verification/README.md) |
| 浏览所有指南 | **[完整文档索引](docs/README.zh-CN.md)** |

项目首页和文档索引提供中英双语。大部分详细操作指南目前为中文，索引标明各文档的语言。

<a id="architecture"></a>
## 系统如何协作

~~~mermaid
flowchart TB
    L["Livox MID360"] --> F["FAST-LIO2"]
    F --> E["Rear-axle EKF"]
    W["Wheel speed + corrected IMU"] --> E
    F -- "Deskewed scan" --> N["NDT + localization monitor"]
    E -- "Odometry prediction" --> N
    P["Prior map"] --> N
    E -- "Local state" --> C["Native acados MPCC"]
    N -- "Map anchor + health" --> C
    R["Saved map reference"] --> C
    C -- "/drive" --> S["RC selector"]
    S -- "/ackermann_cmd" --> V["VESC"]
    classDef sensor fill:#e0f2fe,stroke:#0284c7,color:#0c4a6e;
    classDef estimation fill:#ecfdf5,stroke:#0f766e,color:#064e3b;
    classDef controller fill:#0f172a,stroke:#38bdf8,color:#f8fafc;
    classDef actuator fill:#fff7ed,stroke:#ea580c,color:#7c2d12;
    classDef data fill:#f1f5f9,stroke:#64748b,color:#334155;
    class L,W sensor;
    class F,E,N estimation;
    class C controller;
    class S,V actuator;
    class P,R data;
~~~

- **FAST-LIO2 + EKF：** 提供连续局部运动与后轴状态。
- **NDT：** 用保存地图提供全局修正，同时监控可信锚点新鲜度与地图身份。
- **MPCC：** 在 `odom` 中预测运动，通过可信对齐关联 `map` 中的保存参考。
- **RC 选择器：** 决定实际转发给车辆的命令。`/drive` 是控制提议，`/ackermann_cmd` 是转发命令。

TF 链为 **`map → odom → base_link → livox_frame`**。已知地图运行时，NDT 拥有 `map → odom`，EKF 拥有 `odom → base_link`；`base_link` 位于**后轴中点**。[架构指南](docs/architecture.md)说明建图模式的归属、时间戳和全部话题。

<a id="quick-start"></a>
## 快速开始

### 1. 准备 workspace

按[部署指南](docs/deployment/README.md)安装系统依赖并配置硬件。

<details>
<summary><strong>首次源码准备与构建</strong></summary>

安装 ROS 2 Humble 和列出的系统依赖后，在 Bash 中执行。已有 checkout 可从子模块步骤开始。

~~~bash
git clone --branch main https://github.com/EleSheep-moving/AIMSRacer.git
cd AIMSRacer
git submodule update --init src/FASTLIO2_ROS2
python3 tools/setup_dependencies.py --workspace .
bash tools/setup_acados.sh --workspace . --jobs 2
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2 -l2"
colcon build --packages-up-to aims_racer_system aims_mpcc_rt --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
~~~

源码与补丁固定在[依赖清单](dependencies/manifest.json)中。Make 并发已限制，适用于 NX 构建；acados 安装在 `dependencies/work/acados/install`。

</details>

### 2. 选择启动入口

每个操作终端加载当前 workspace：

~~~bash
cd "$HOME/AIMSRacer"
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/dependencies/work/acados/install/lib:${LD_LIBRARY_PATH:-}"
~~~

| 用途 | 启动入口 | 内容 |
| --- | --- | --- |
| 手动驾驶 / 传感器检查 | `ros2 launch aims_racer_system vehicle.launch.py` | Livox、FAST-LIO2、后轴 EKF、RC 和 VESC |
| 建图 | `ros2 launch aims_racer_system mapping.launch.py record:=true` | 车辆传感器、PGO 和录制 |
| 已知地图轨迹跟踪 | 下方 `race.launch.py` 命令 | 车辆、NDT、定位 monitor 和原生 MPCC |
| 接入已有车辆/定位图，仅启动控制器 | `aims_mpcc_rt/mpcc.launch.py` | 原生 MPCC |

三个整车入口每次选择一个。控制器单独启动方式见[原生运行时指南](src/aims_mpcc_rt/README.md)。

使用 NX 当前已部署数据启动已知地图运行：

~~~bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
~~~

**必需数据：** 已有 PCD 地图，以及在目标 CPU 上编译的完整求解器 bundle，包含参考与配置。数据保存在源码外，clone 仓库不会得到它们。新数据按[参考/bundle 流程](src/controller/docs/usage.md)准备。

### 3. 初始化、启用与停止

按车辆实际后轴位置与朝向，通过 RViz 或[初始化工具](docs/operations/known-map-mpcc.md)设置 **`map/base_link`** 初值。定位就绪、RC 选择自主 speed 模式后启用：

~~~bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
~~~

请求停止：

~~~bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
~~~

| 可选 race 参数 | 默认 | 行为 |
| --- | --- | --- |
| `record` | `false` | 录制 bag、配置快照与 `runtime.csv` |
| `auto_start` | `false` | 正常就绪条件满足后启用一次 |
| `repeat_laps` | `false` | 单圈停车；`true` 持续运行直到停止 |
| `initial_pose` | 空 | `map/base_link` 中的 `x y z roll pitch yaw`；角度 rad |

显式停止会取消等待中的自动启动。完成或故障后不会自动重新启用。录制数据位于 `~/aimsracer-data/sessions/`，详见[录制指南](docs/operations/recording.md)。

<a id="defaults"></a>
## 当前默认配置

默认参数复现 **2026-10-10 V35 外场配置**。已部署 bundle 决定参考、车辆配置和预测网格。

| 项目 | 数值 |
| --- | --- |
| 车辆模型 | 后轴运动学自行车模型，一阶转向响应 |
| 预测时域 | **15 × 0.1 s = 1.5 s** |
| 求解器 | 生成 C，acados **SQP-RTI / HPIPM**，至多 **2 次 RTI** |
| 求解请求 / 命令发布 | 至多 **20 Hz** / **50 Hz** |
| 完整交付预算 / 接管提前量 | **50 ms** / **20 ms** |
| EKF 发布 | 配置 **200 Hz**；`odom/base_link` |
| 巡航目标 / 速度硬上限 | **3.5 / 4.0 m/s** |
| 加速 / 制动上限 | **1.0 / 1.0 m/s²** |
| 曲率速度规划参数 | **1.0 m/s²** |
| 转向响应时间常数 | **0.08 s** |
| 走廊 / 组合加速度椭圆 | 当前外场配置均关闭 |
| 手动电流映射 | **CH10：3–100 A**；CH3 油门，死区 **50** |

以上为配置频率和限制值。定时发布不会产生新的测量或地图配准。MPCC 当前使用**速度命令**。全部参数和权重见 [vehicle.yaml](src/controller/config/vehicle.yaml)，求解与运行时细节见[实现指南](src/controller/docs/implementation.md)。

<a id="results"></a>
## 实测结果与验证

后期两次 V35 运行在 Orin NX 上**同时运行 FAST-LIO2 + EKF + NDT**。

| 外场指标 | V35 第一次 | V35 第二次 |
| --- | ---: | ---: |
| EKF 实测速度峰值 | 2.887 m/s | 2.885 m/s |
| MPCC worker 完整计算 P95 | 4.001 ms | 2.656 ms |
| 绝对横向误差 P95 | 8.87 cm | 8.89 cm |
| 含起步与制动的运动窗口 | 21.65 s | 22.08 s |

规划速度峰值为 **2.990 m/s**，低于 3.5 m/s 的巡航目标。worker 计算不含交付等待，与求解器单独耗时、命令发布周期分别统计。运动窗口包含静止起步和制动，不是 flying lap 圈时。定义、记录编号和完整证据见[外场复盘](docs/reports/2026-10-10-mpcc-field-review.md)。

**main 栈软件验证：** NX 构建完成18个包；原生控制器24/24、系统适配11/11、RC 2/2、FAST-LIO 输入缓冲1/1、启动检查7/7通过。部署检查使用隔离/remap接口，没有发送车辆命令。[整合与验证记录](docs/reports/2026-10-10-main-field-stack-integration.md)说明范围，包括新组合 launch 与可选自动启动的软件检查。

<a id="repository"></a>
## 仓库目录

| 位置 | 职责 |
| --- | --- |
| [src/aims_racer_system](src/aims_racer_system/README.md) | 整车 launch、估计适配与定位健康 |
| [src/aims_mpcc_rt](src/aims_mpcc_rt/README.md) | 原生 C++ 运行时与生成 bundle 工具 |
| [src/controller](src/controller/README.md) | 离线参考、模型、速度规划与车辆配置 |
| [src/ackermann_mux](src/ackermann_mux/README.md) | RC 选择与实际命令转发 |
| [dependencies/manifest.json](dependencies/manifest.json) · `tools/` | 固定依赖与准备 |
| [tools/reference](tools/reference/README.md) | 保存地图与闭环参考准备 |
| [verification](verification/README.md) | 开发回归、重放与审计 |
| [docs](docs/README.zh-CN.md) | 操作指南、架构与日期化证据 |

地图、生成 bundle 和原始 bag 保存在源码外。在线控制器是 `aims_mpcc_rt`，`aims_mpcc` 提供离线工具。

<a id="credits"></a>
## 致谢与许可证

第三方归属和许可证保存在[模型 NOTICE](src/controller/NOTICE.md)、[原生运行时 NOTICE](src/aims_mpcc_rt/NOTICE.md)及各包中。依赖版本与补丁见[固定清单](dependencies/manifest.json)。

本项目基于以下开源贡献：

- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [QUTMS Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless)
- [Original upstream system package](https://github.com/f1tenth/f1tenth_system)
- [ros2_crsf_receiver](https://github.com/AndreyTulyakov/ros2_crsf_receiver.git)
- [ackermann_mux](https://github.com/z1047941150/ackermann_mux.git)
- [Veddar VESC Interface](https://github.com/f1tenth/vesc)
- [FAST-LIO2_ROS2 maintained fork](https://github.com/EleSheep-moving/FASTLIO2_ROS2.git)

硬件与基础软件由 PolyU AIMS Lab 开发。目前在 PolyU AIMS Lab 攻读 MPhil，项目持续开发中。
