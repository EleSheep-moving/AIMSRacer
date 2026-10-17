# 后轴与质心：独立 VX、VY、AX、AY、yaw rate 输出

> 本页对应仓库中的 C++ `state_observer`、配置和现场脚本。动捕校对见[统一操作说明](mocap-validation.zh-CN.md)。


新增 C++ `state_observer` 节点同时提供后轴和质心两套观测。VX/VY 来自 corrected IMU 推进、LIO 历史位姿约束的三维速度估计；AX/AY 来自重力恢复和刚体参考点补偿；yaw rate 来自 corrected gyro 运动学滤波。只订阅 `/livox/imu_bias_corrected` 与 raw `/fastlio2/lio_odom`，不读取轮速、导航 EKF、LIO twist、电流或控制指令。

VY 已成为可发布的独立观测器输出，但这里的“独立”指不依赖导航 EKF/轮速输入，**不是独立传感器真值或已通过 Vicon 的精度保证**。LIO 与观测器仍共享 IMU。旧 VX、body/planar acceleration 和 yaw 节点的话题及默认行为保留。

2026-10-17 明确的目标分工：Robot Localization 为 MPCC 提供 X、Y、Yaw；本节点独立提供 VX、VY、AX、AY、yaw rate，优先控制动态延迟。**这是目标接口：当前 `aims_mpcc_rt/src/node.cpp` 仍读取导航 odometry 的 VX，本节点还未接入控制器。** 不应为保留本节点连续输出而调整导航融合配置或控制器接线。

### 计算链说明

平面、同一参考点下满足 `dot(vx)=ax+r*vy`、`dot(vy)=ay-r*vx`。本实现使用它们对应的三维惯性导航形式：IMU 比力和 gyro 推进世界系位置、速度、姿态，LIO 的位置创新经 `[position,velocity,accel_bias]` 九维协方差纠正速度和偏置，LIO 姿态锚定姿态，然后重放到最新 IMU 源时刻；最后转换到两个目标点。**不读取 LIO twist/VY，也不把发布后的滤波 AY、yaw rate 接回速度积分。** 速度传播使用 corrected IMU 与速度核心自身估计的加速度偏置；AX/AY 发布分支目前固定加速度偏置仍取零，r 的发布分支另有滤波，所以不能把三条发布流逐点代入当作严格离散代数恒等式。

所有五量直接由本节点的三条分支产生，均不订阅 Robot Localization 输出。LIO 纠偏仍依赖共享传感器，速度协方差也不是完整独立传感器融合的精度证明。

## 1. 两套话题与有效字段

| 后轴话题 | 质心话题 | 类型和有效字段 |
| --- | --- | --- |
| `/state_estimation/rear_axle/velocity` | `/state_estimation/center_of_mass/velocity` | `TwistWithCovarianceStamped`；linear.x=VX、linear.y=VY，m/s |
| `/state_estimation/rear_axle/acceleration` | `/state_estimation/center_of_mass/acceleration` | `AccelWithCovarianceStamped`；linear.x=AX、linear.y=AY，m/s² |
| `/state_estimation/rear_axle/yaw_rate` | `/state_estimation/center_of_mass/yaw_rate` | `sensor_msgs/Imu`；仅 angular_velocity.z=r，rad/s |
| `/state_estimation/rear_axle/status` | `/state_estimation/center_of_mass/status` | `DiagnosticArray`；10 Hz，五量分别报告有效性/源时间/源年龄/拒绝原因 |

后轴 frame 为 `base_link`，质心 frame 为 `center_of_mass`；两者轴均为 x 前、y 左、z 上。质心只改变参考原点，不把速度旋转到地图系。该节点不发布 TF 或控制命令；不会改导航 EKF 或控制器的参考 frame。

数值消息使用实际处理的 IMU 源时间。每种量在各自分支可计算时发布，两参考点的同类消息源时间一致。默认 `warn_only_quality_gates=true`：输入年龄超阈值时继续发布可计算结果，并报警及保留源年龄；不会把旧数据改成当前时间。尚未初始化/无法计算时不发布假零；停止更新时应结合 status 判断。消息中未观测分量的零值是占位，对角协方差设为 1e6；yaw 消息的 orientation 和 linear_acceleration covariance 首元素为 −1。

## 2. 几何、公式与高度估计

共同安装旋转和后轴→IMU 向量来自 `params/rear_axle_geometry.yaml`，当前约为 `[0.30,0,0.03] m`。IMU→质心向量来自 vehicle_geometry_measurements.yaml（记录中的工作区路径：`src/aims_racer_system/params/vehicle_geometry_measurements.yaml`）：

```text
d_IMU_to_CG_body = [-0.09, -0.03, -0.03] m
```

前两项为用户此前提供，z 是 2026-10-17 补充的“估计 3 cm，具体不清楚”：**高度估计，不确定度未知，外参未完整标定**。因此状态保留 `cg_height_estimated=true`、`geometry_calibrated=false`、`dynamic_accuracy_validated=false`。配置中的 `complete_3d_offset_available=true` 只表示三项名义值已提供；`complete_3d_offset_measured=false` 表示三维外参尚未实测完整。轴距不参与这里的刚体参考点换算。

对任意目标点 P，以 `d=position(P)−position(IMU)`、在车体系表达，计算：

\[
v_P^B=v_{IMU}^B+\omega_B\times d,
\qquad
a_P^B=f_{IMU}^B-b_a^B+g^B+\alpha_B\times d+\omega_B\times(\omega_B\times d).
\]

后轴使用 `d=−rear_to_IMU`，质心直接使用上述 IMU→CG 向量；两点分别完成物理补偿后进入 AX/AY 滤波，**不从后轴已经滤波的加速度二次拼接质心信号**。raw LIO 姿态先根据安装旋转换成车体姿态，再用于加速度的历史锚定。corrected Livox 比力输入仍为 g，转换系数 9.80665；gyro 为 rad/s。

yaw rate 是刚体车体 z 轴角速度，两参考点数值和时间完全相同，只是 frame 标签不同；roll/pitch 不为零时，它不是欧拉 yaw 的简单导数。速度协方差保留 XY 交叉项并加入条件 gyro 杆臂项；它仍未校准共享 IMU/LIO 相关性和实际外参误差。

参考点速度换算使用同一 IMU 源时刻的安装旋转后 gyro；发布的 yaw rate 另经过滤波。角加速度仍由加速度分支自己的三轴回归形成，没有把另一 yaw 节点的滞后 alpha 直接换入补偿链。

## 3. 编译与独立启动

在 ROS Humble 环境编译当前源码并 source 对应安装：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/AIMSRacer
colcon build --packages-select aims_racer_system --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch aims_racer_system state_observer.launch.py
```

最后一条命令只启动观测器。已有 corrected IMU 和 raw LIO 传感器链即可，不需要启动另一套 vehicle/race。Ctrl-C 停止。不要同时启动两个同名 `state_observer` 节点。

查看两点 VY/VX：

```bash
ros2 topic echo /state_estimation/rear_axle/velocity --once
ros2 topic echo /state_estimation/center_of_mass/velocity --once
ros2 topic echo /state_estimation/center_of_mass/acceleration --once
ros2 topic echo /state_estimation/center_of_mass/yaw_rate --once
ros2 topic echo /state_estimation/center_of_mass/status --once
```

独立入口支持 `config:=/absolute/state_observer.yaml`、`geometry_config:=/absolute/rear_axle_geometry.yaml`、`cg_geometry_config:=/absolute/vehicle_geometry_measurements.yaml`、`use_sim_time:=true`。高度未知/非有限时启动失败，不会默认为零。模型与几何参数在启动后只读，修改后需重启。

在已有的 vehicle/race 启动命令上增加：

```text
state_observer:=true record:=true
```

默认仍为 false。`record:=true` 会录制两套八个话题与原始输入，并保存参数快照。父入口支持 `state_observer_config`、`state_geometry_config` 和 `cg_geometry_config`，分别透传到独立入口；所选外部文件额外保存为 `state_observer_selected.yaml`、`state_geometry_selected.yaml`、`cg_geometry_selected.yaml`，避免只留下已被覆盖的路径。

## 4. 如何读状态与已知限制

- `vx_valid/vy_valid`、`ax_valid/ay_valid`、`yaw_rate_valid` 表示相应分支可计算且 IMU 源时间新鲜；不是绝对精度验收标志。默认模式下 LIO 超时可保持它们为 true，同时 `quality_degraded=true`、reason=`lio_stale`、诊断 WARN；源时间过旧时 valid=false，但仍发布原源时间对应的可计算数值。
- `velocity_computable`、`acceleration_computable`、`yaw_rate_computable` 与质量/新鲜度分开：只要该分支数值可计算，允许持续发布；消费者应结合原始时间戳与 status 选择使用策略。
- `*_source_stamp_ns` 与 `*_source_age_s` 表示该分支实际使用的 IMU 源时间和接收时钟年龄；不是物理动态延迟。
- 初始化之后 LIO 缺失或过期时，默认继续 IMU 推进 VX/VY 和姿态、计算 AX/AY，给出 WARN；不会仅因 0.5 s 超时重新做三位姿初始化。速度协方差随 IMU 推进增长；加速度增加随纯惯性续推时间平方增长的保守模型方差，尚未标定。已迟到但仍在 2 s 历史内的 LIO 可纠偏，明显位置/姿态创新仍可被拒绝，但不因此停止 IMU 输出。
- IMU 到达年龄超过 50 ms 只报警并按原源时间计算/发布；它不同于 IMU 源序列本身断流。没有新 IMU 时不重发或重盖时间戳，诊断持续更新并将新鲜度置为无效。
- `warn_only_quality_gates=false` 可恢复新节点原来的严格年龄停发策略；旧 scalar VX 和 body/planar 节点的默认策略不随新节点更改。
- 同一时钟 epoch 内不重复发布旧源时间。晚到 LIO 可以纠正内部历史，不重发已发布过去；ROS 时钟回退会清空历史和发布 epoch。
- 新加速度分支保持 25 ms 因果 gyro 回归，但要求至少 **10 ms** 的时间支持；不足时 `acceleration_reason=gyro_time_support`，不计算不可信角加速度、不发布 AX/AY，支持恢复后继续。该门控只在新入口启用，旧节点默认行为不变。
- 保留必要的数据拒绝/恢复条件：数值非有限/溢出、明确标记测量不可用、错误 frame/非法时间戳、未初始化、源间隔超过 30 ms 后的重新准备，以及无法计算角加速度的窗口。拒绝某条异常 LIO 不等于停止整个估计器；到达晚也不等于源数据缺失。
- 这道门控解决“极小回归跨度仍当作可信计算”的缺口，**没有解决角加速度与当前比力的动态相位不一致**。已有快速换向补偿误差仍需独立验证；质心高度估计也可能影响三维补偿。
- 当前加速度计固定偏置取默认零，未完成六姿态标定；yaw 滤波不在线估计残余 gyro bias。VY 的新发布也不等于已验证的侧滑状态。

随后按用户要求修订的计算链解释、警告优先策略及与 Robot Localization 的对照，见[延迟及发布策略复核](../reports/2026-10-17-state-observer-latency-policy.zh-CN.md)。

计划于 2026-10-19 做 Vicon 测试；MPCC 暂保留原输入。可配置的采集／后轴与质心五量对照程序已准备，见 [Vicon 工具](../../verification/vicon_state/README.zh-CN.md)与[主源码中的现场命令](../../src/aims_racer_system/field/README.zh-CN.md)。在线核心及发布均为 C++，此处 Python 只负责 launch、测试与离线分析。

## 电流与横向加速度标定导航

[电流标定 A/B/C 与当前结果](calibration.zh-CN.md#current-calibration) · [横向加速度 AY 校对及附着标定状态](lateral-acceleration-calibration.zh-CN.md)。源码统一在 `src/calibration/`，横向使用当前 C++ 入口；历史 Python 命令仅供溯源。
