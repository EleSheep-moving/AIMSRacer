# 当前车辆架构

主流程使用一个 ROS 2 Humble workspace。`vehicle.launch.py` 运行传感器、本地估计和底盘；`mapping.launch.py` 运行建图；`race.launch.py` 在 vehicle 上增加已知地图 NDT 和 native acados/C++ MPCC。[启动流程](operations/bringup.md)说明如何选择入口。

## 数据与命令链

```mermaid
flowchart LR
    Livox[MID360 lidar / raw IMU] --> LIO[FAST-LIO2]
    Livox --> Bias[gyro bias correction]
    Bias --> RearIMU[rear axle IMU]
    LIO --> RearLIO[rear axle odometry]
    RearIMU --> EKF[EKF]
    RearLIO --> EKF
    Wheel[VESC wheel speed] --> EKF
    LIO --> NDT[NDT known map]
    EKF --> NDT
    NDT --> Health[localization monitor]
    EKF --> MPCC[native MPCC]
    Health --> MPCC
    Bundle[immutable bundle] --> MPCC
    MPCC --> Drive[/drive]
    Drive --> RC[RC selector]
    RC --> Applied[/ackermann_cmd]
    Applied --> VESC[VESC converter / driver]
    Applied --> MPCC
```

MPCC 发布的是提议。RC 选择器决定是否转发、以何种模式转发；控制器使用实际 `/ackermann_cmd` 历史预测测量到执行期间的运动。电机/servo command 与实际速度、转角不是同一种数据。当前控制接口为 speed 模式，完整纵向辨识按后续加速度接口计划进行。

## 坐标与安装约定

`base_link` 位于后轴中点，x 向前、y 向左、z 向上。`rear_offset=0`，默认 footprint 相对该点前 0.52 m、后 0.10 m、左右 0.16 m。

所有外部 Livox lidar、IMU、LIO body cloud 使用 `livox_frame`。外部安装关系由 [rear_axle_geometry.yaml](../src/aims_racer_system/params/rear_axle_geometry.yaml)统一定义：平移 `[0.30,0,0.03] m`，RPY `[0,0,0] rad`，是当前安装近似。FAST-LIO 内部仍使用自己的 lidar/IMU 外参；后轴适配不再重复组合它。厘米级 lidar/IMU 原点差在外部坐标中近似忽略。

后轴 LIO 适配转换 pose、速度和 covariance；速度包含传感器到后轴的旋转杠杆补偿。后轴 IMU 将加速度从 g 转换为 m/s² 并做杠杆补偿，保留原始 source stamp。gyro bias 修正输出独立 topic，原始 Livox 输入仍保留供 LIO 与录包。

```text
map
 └─ odom
     └─ base_link
         ├─ livox_frame
         └─ base_footprint
```

| TF 边 | race / vehicle | mapping |
| --- | --- | --- |
| `map → odom` | race 的 NDT；vehicle 单独启动不提供 | PGO |
| `odom → base_link` | EKF | 后轴 LIO 适配器 |
| `base_link → livox_frame` | 单一静态安装 TF | 同左 |
| 原始 FAST-LIO TF | 隔离在 `/fastlio2/tf` | 同左 |

VESC 不发布该车辆 TF。PGO 与 NDT 使用不同运行入口，以保持 map 边唯一归属。`base_footprint` 为已有零变换，不是地面投影测量。

## Topics and compensation

以下频率为配置目标或输入驱动的名义值，现场消息率以记录为准。timer 频率不代表新的测量频率。

| Topic | 来源 | 内容 / 时间 / 使用者 |
| --- | --- | --- |
| `/livox/lidar` | Livox | 10 Hz 配置；raw CustomMsg，scan-start stamp 与逐点偏移；LIO、重放 |
| `/livox/imu` | Livox | raw `livox_frame` IMU，约 200 Hz 的实录输入，加速度单位 g；LIO、bias、录制 |
| `/livox/imu_bias_corrected` | gyro bias | 修正 gyro，保留源时间；后轴适配 |
| `/imu/gyro_bias/status` | gyro bias | 校正状态与诊断 |
| `/fastlio2/lio_odom` | LIO | `odom/livox_frame`，processed scan-end stamp；后轴适配、PGO |
| `/fastlio2/body_cloud` | LIO | `livox_frame` processed scan，与 raw LIO stamp 相同；NDT、PGO、显示 |
| `/fastlio2/tf` | LIO | 私有 raw TF，不进入公共 TF 树 |
| `/fastlio2/visualization/world_cloud` | LIO | 可选 raw LIO world 显示，默认关闭；不用于定位控制 |
| `/rear_axle/lio_odom` | 后轴适配 | `odom/base_link`，保留 LIO stamp；EKF |
| `/rear_axle/imu` | 后轴适配 | `base_link` IMU，m/s²，保留 IMU stamp；EKF |
| `/rear_axle/wheel_odom` | VESC | 后轴 wheel vx，约 50 Hz 名义；EKF |
| `/odometry/filtered` | EKF | `odom/base_link`，200 Hz 配置；MPCC、scan-time TF 预测 |
| `/localization/ndt_pose` | NDT | 配准输出 pose |
| `/localization/odom_bridge_pose` | NDT | 连续本地运动与地图锚点桥接 pose |
| `/localization/ndt_status` | NDT | 配准诊断 |
| `/localization/anchor_status` | NDT | protocol-v1 可信锚点、epoch、sequence 和对齐载荷 |
| `/localization/map_sha256` | monitor | 保存 PCD 身份，transient-local |
| `/localization/map_valid` | monitor | 定位资格标志；不是独立精度测量 |
| `/localization/status` | monitor | ready、健康、新鲜度和 map identity；native 控制资格检查 |
| `/drive` | native MPCC | 50 Hz 提议，最多 20 Hz 优化请求；RC 选择执行 |
| `/ackermann_cmd` | RC selector | 已选择的转发命令，200 Hz 配置；VESC 与 MPCC 输入历史 |
| `/control/autonomy_speed_enabled` | RC selector | 自主 speed 模式权限 |
| `/commands/motor/speed` | VESC converter | 目标 ERPM；不是实测车速 |
| `/commands/servo/position` | VESC converter | servo 目标；不是转角测量 |
| `/sensors/core` | VESC driver | 实际 ERPM、电流、电压等 telemetry |
| `/sensors/servo_position_command` | VESC driver | servo 命令 echo |
| `/mpcc/status` | native MPCC | 执行阶段、自动启动、资格、worker/接管/输出诊断 |
| `/mpcc/reference` | native MPCC | bundle 中的固定参考；transient-local |
| `/mpcc/prediction` | native MPCC | 接受候选的预测，用于显示 |

## EKF 与 LIO 时间

EKF 使用后轴 LIO pose/linear velocity、wheel vx 和 IMU yaw rate；不融合 IMU acceleration/orientation，也不融合 wheel 积分 pose 或转向推算 yaw rate。`two_d_mode=true`，延迟测量历史为 2 s，回放后 `predict_to_current_time=true`。LIO 和 IMU 数据相关，不是独立观测源。

FAST-LIO 用单一 worker 处理 scan 和完整 IMU 区间。pending lidar 默认深度 2，过载保留新扫描，传播所需 IMU 保留；原始 IMU reliable 深度 4096。时间回退、非有限值、溢出和 scan-end 覆盖仍检查。新鲜 EKF header 不代表 LIO 无延迟；具体版本与实测见[外场复盘](reports/2026-10-10-mpcc-field-review.md)。

## Persistent reference and local control frames

保存参考在 `map` 中，绑定准确 PCD hash。控制状态和模型传播保持连续 `odom/base_link`；可信 `map ← odom` 对齐用于参考投影、误差与可选边界计算。初值是 **base_link 在 map 中的六维 pose**，详见[已知地图流程](operations/known-map-mpcc.md)。

NDT 的 50 Hz map TF 可持有上次可信修正，不等于 50 Hz 重新配准。monitor 与 native 同时检查 epoch、sequence、源与接收年龄、地图身份及一致的可信对齐载荷。TF 单独新鲜不能授予自主控制。

## Native MPCC 与当前边界

模型有六个物理状态与三个控制，加上历史辅助状态；后轴运动学传播、转向一阶滞后、周期 quintic 参考和曲率速度规划，generated acados SQP-RTI 经 C++ 调用。当前 bundle N15/dt=.1，RTI 至多两次，50 ms 完整交付预算，20 ms 预计接管提前量，原始状态 TTL 为 `0.8×N×dt=1.2 s`。命令以 50 Hz 发布，拒绝新候选不更新旧计划 source epoch。

默认巡航目标 3.5、硬上限 4、加速/制动 1、曲率规划 ay=1；`combined_accel_constraint_enabled=false`、`enforce_corridor=false`。独立输入/状态、有限数、定位和命令归属仍检查。规划 ay 不构成运行时轮胎硬约束；人为参考宽度也不构成真实赛道边界。[实现指南](../src/controller/docs/implementation.md)解释成本、执行历史和认证。

[2026-10-10 实跑](reports/2026-10-10-mpcc-field-review.md)规划峰值 2.99、实测峰值约 2.89 m/s，运动窗口并非 flying lap。这些记录不证明更高抓地能力或其他地图下的运行表现。
