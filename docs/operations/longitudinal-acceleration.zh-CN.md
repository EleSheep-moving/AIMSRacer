# 纵向加速度观测

[English](longitudinal-acceleration.md) · **简体中文**

选定的轮速＋IMU 观测器提供独立纵向加速度估计，用于录制和下游参考。场地验证时显式开启；不改变导航 EKF、TF、MPCC、VESC 或驾驶权限，加速度反馈控制尚未接入。

## 输入与输出

| 话题 | 字段与含义 |
|---|---|
| `/rear_axle/wheel_odom` | 输入车体纵向速度 `twist.twist.linear.x`，child frame 必须是 `base_link` |
| `/rear_axle/imu` | 输入后轴比力 `linear_acceleration.x`，已经是 m/s²、`base_link`；尊重协方差中的不可用标志 |
| `/state_estimation/longitudinal_acceleration` | `geometry_msgs/AccelWithCovarianceStamped`，`accel.accel.linear.x` 估计 `d(body_vx)/dt`，单位 m/s² |
| `/state_estimation/longitudinal_acceleration/status` | `DiagnosticArray`：初始化、源时间年龄、比力有效性、计数及回调耗时 |

同一坐标系下，比力定义为 **f = a − g**。z 轴向上的世界中 `g_world = [0,0,−9.80665] m/s²`，静止加速度计约为 +9.81 m/s²；恢复惯性运动加速度要用 **a = f + g**。后轴适配器只做单位、安装旋转和杆臂换算。观测器通过 `v_wheel≈v`、`f_x≈a+d` 估计 `[v,a,d]`，干扰包含 `−g_body,x`、零偏和 `(ω×v)_x`。它不重建姿态；旋转时，这个车体速度导数与完整惯性加速度向量不同。

输出其他均值分量为零，其他协方差对角线为 1e6，表示没有观测这些分量。covariance[0] 是模型加速度方差，尚未经过场地真值标定。不能将消息当成完整六轴加速度，也不自动输入导航 EKF。

## 启动

在现有 race 命令后添加：

```bash
longitudinal_acceleration:=true record:=true
```

完整示例，地图和 bundle 应与当前场地对应：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  longitudinal_acceleration:=true record:=true
```

实车验证前，vehicle/race 中这个开关默认 false，原有 auto-start 也仍为 false。加速度节点只做观测；正常驾驶沿用既有[启用/停止流程](known-map-mpcc.zh-CN.md)。`record=true` 会录制两个新话题、原有轮速/IMU/LIO/EKF/控制输入，并随其他参数保存 `longitudinal_acceleration.yaml`。

如果车辆程序已运行且没有加速度节点，在另一个已 source 的终端单独启动：

```bash
ros2 launch aims_racer_system longitudinal_acceleration.launch.py
```

只运行一个实例，选择独立或集成启动。独立 launch 不启动传感器、电机驱动、EKF、TF 或控制器。回放 ROS bag 时添加 `use_sim_time:=true`，由回放提供 `/clock`。

## 时间和初始化

回调收到观测后，将精确整数纳秒源时间输入观测器。轮速回调更新状态，只有 IMU 回调触发发布。标量状态预测后，输出 header 使用当前 ROS 估计时刻，不改写上游源时间。IMU 不到达时，不通过定时器编造周期加速度输出。

首次输出需要有效轮速和比力。初始化之后，无效比力被跳过，其默认零值不会融合；IMU 心跳仍可触发模型/轮速估计。源年龄告警只是诊断，不冻结输出。ROS 时间回退时重置状态、历史及初始化。无效坐标系、非有限数据、未来时间戳、历史保留失败都有计数。

独立进程串行处理输入与状态回调，数学核心与离线比较共用。没有额外输出低通滤波、LIO 预测或 LIO 年龄依赖。

## 检查与录制两圈

```bash
ros2 topic echo /state_estimation/longitudinal_acceleration --once
ros2 topic hz /state_estimation/longitudinal_acceleration
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

检查 `initialized=true`、`force_valid`、轮速/IMU/比力源年龄、数值错误和回调耗时。输出频率随 IMU 到达，既有录包中约为 200 Hz，不是严格的 5 ms 截止保证。只看输出 header 年龄看不出传感器年龄，应同时看状态字段。

场地先录静止段，再以低速和常用速度各跑一圈，保留当前控制行为。检查加减速响应、停车回零、源数据到输出的延迟、发布间隙及 FAST-LIO/EKF/NDT 联合负载下的回调成本。轮速近零处理、坡度变化、振动和打滑仍需关注；配置目标速度不等于实测速度。

## 参数和证据

安装参数位于 `params/longitudinal_acceleration.yaml`：jerk PSD 30、干扰 PSD 0.1、轮速方差 0.01、比力方差 16、历史 2 秒/4,096 条。这些是六次训练、两次保留录包选出的有效调参值，运行期 B 节点没有启用重力/LIO 观测。

[离线比较](../reports/2026-10-12-longitudinal-acc-observer.zh-CN.md)建立录制输入上的效果，[验证工具](../../verification/acceleration/README.zh-CN.md)提供复现。安装软件、合成 DDS 回放、车辆联合负载和真实场地驾驶属于不同证据阶段。确认场地响应和数据质量前，加速度输出用于观测。

[启动指南](bringup.zh-CN.md) · [录制](recording.zh-CN.md) · [架构](../architecture.zh-CN.md)
