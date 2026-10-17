# 后轴平面加速度与 G 观测

> 2026-10-18 整理范围：本页记录已部署工作区的用法；这次文档提交不包含对应的新运行时代码，命令须在已部署环境执行。当前后轴／质心五量以[统一状态指南](state-observer.zh-CN.md)为准。 本页为较早的单分量分支说明，历史数值与局限保留。


[English](planar-acceleration.md) · **简体中文** · [质量报告](../reports/2026-10-15-planar-acceleration-quality.zh-CN.md) · [Vicon 采集](vicon-observer-validation.zh-CN.md)

独立 C++ 观测节点已扩展为后轴 `ax/ay` 和 G 输出。它用于并行观测、录包与后续验证；尚未接入控制器、侧滑判断或摩擦边界。当前快速转动的杆臂补偿存在时间响应误差：固定后轴合成试验在 3 Hz、yaw rate 幅值 1 rad/s 时，横向虚假输出的绝对值 P95 为 **0.1405g**。`y_valid=true` 仅说明输入和计算有效，不保证动态精度。

## 输入、处理与参考点

输入只有 `/livox/imu_bias_corrected` 的三轴比力和 gyro，以及 `/rear_axle/lio_odom` 的姿态。无轮速、电流、导航 EKF 或独立 yaw 节点订阅。沿用[车体加速度](body-acceleration.zh-CN.md)的源时间历史回放：LIO 在自己的历史时刻约束姿态，再用保存的 gyro 与滤波状态重放到当前 IMU 源时刻。不是用晚到的姿态直接覆盖当前姿态。

车体轴 x 前、y 左、z 上，参考点为 `base_link` 后轴；不是质心。安装旋转来自共享 `livox_rpy`，后轴到 IMU 杆臂当前约 `[0.30,0,0.03]` m。重建为：

```text
a_rear = R_body_from_imu * f_imu - b_accel + g_body
         - alpha × lever - omega × (omega × lever)
```

gyro 使用原有因果 25 ms 三轴线性回归计算角加速度。重建后，独立 `[ax,jx]`、`[ay,jy]` 状态共享相同模型参数、协方差与增益，保证最终 XY 滤波响应一致。默认 `jerk_psd=1e4`、比力有效方差 `0.01`；不新增在线加速度零偏估计。共享增益并不能消除前级重力／角加速度的不同响应，也不能证明实际噪声各向同性。XY 相关性未辨识，输出对角协方差只是条件有效模型。

## 已提供的本车质心几何

用户于 2026-10-15 提供轴距 **L=0.365 m**。按 ROS 车体轴 x 前、y 左、z 上，从 MID-360 内置 IMU 指向质心的向量为 **d=[−0.09,−0.03,dz] m**：质心在 IMU 后方 9 cm、右侧 3 cm。用户另确认 IMU 位于质心上方，所以 **dz<0，距离未知**；不能把未知高度当成零。测量不确定度未提供，原始数据记录在 vehicle_geometry_measurements.yaml（记录中的工作区路径：`src/aims_racer_system/params/vehicle_geometry_measurements.yaml`）。记录日期表示收到测量值的日期，不代表测量发生日期。

从 IMU 点转换到质心的三维关系为 `a_CG=a_IMU+alpha×d+omega×(omega×d)`，其中 a_IMU 已完成安装旋转与重力恢复，但尚未移到后轴。只有严格平面 yaw 运动时，才可仅用已知 x/y 写成：

```text
ax_CG = ax_IMU + 0.03*alpha_z + 0.09*r²
ay_CG = ay_IMU - 0.09*alpha_z + 0.03*r²
```

这些是待实现的参考点转换关系，不是当前输出。完整三维补偿仍缺 dz，roll/pitch 运动时不能默认为零。当前图和话题继续表示后轴；不能把上述 IMU→质心向量直接用于已经移到后轴的输出。现有后轴到 Livox 统一外部原点只是安装近似，尚不能据此推出精确的后轴到质心距离。

当前控制器／VESC 配置仍使用历史轴距 0.36 m，新测量记录与运行配置分开保留。更新控制模型时需要核对已有辨识参数、生成 bundle 和实际安装版本；本次仅记录几何，没有改变运行时参数。

## 输出与有效性

| 话题 | 消息与含义 |
| --- | --- |
| `/state_estimation/planar_acceleration` | `AccelWithCovarianceStamped`；linear x/y 为 m/s²，covariance[0]/[7] 为有效模型方差 |
| `/state_estimation/planar_acceleration/g` | `Vector3Stamped`；x=ax/9.80665，y=ay/9.80665，无量纲 |
| `/state_estimation/planar_acceleration/status` | 输入年龄、回放、有效性、拒绝原因、回调窗口耗时 |
| `/state_estimation/planar_acceleration/debug` | 比力、重力、切向／向心项、未滤波重建、滤波输出 |

SI 与 G 使用相同 IMU 源时间和 `base_link`。G 的 z 是未观测占位；SI 的其他轴也未观测，对角方差为 1e6，不能解释成物理零值。无 TF 或控制命令发布。

沿用原节点门控：错误 frame、非有限输入、重复、未来和超出历史保留范围的输入拒绝；保留历史内晚到的 IMU/LIO 可以插入并重放，同一时钟 epoch 内数值发布保持源时间递增。IMU 最大源年龄 50 ms，LIO 500 ms，IMU 源间隔大于 30 ms 重置。断流不发布零值，status 继续报告失效；时钟回退开启新 epoch。源年龄不是物理响应延迟，最多 512 次回调的 P99 也不是整段或整车延迟。

另一个已知限制来自 14 日 B 包：一段突发密集源时间使角加速度回归支持跨度仅约 41–62 μs，杆臂项放大到 106.3 m/s²，滤波 ay 峰值达到 92.66 m/s²（9.45g），当时仍标记计算有效。当前没有最小回归时间跨度的可信度门控，不能将这些样本解释为车辆 G 或轮胎能力；需保留原始时间与补偿分项定位源时间问题。

## 启动、停止与录制

原有 `body_acceleration` 默认仍只输出 ax，`lateral_enabled=false`。新入口显式开启 XY，使用独立节点名、参数文件与话题。它可与旧节点并行用于对照。

NX 已建立独立软件安装，主安装未替换。在已有 IMU 零偏和 LIO 链运行后：

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-planar-observer-20261015/install/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system planar_acceleration.launch.py
```

使用 Ctrl-C 停止该观测节点。独立 launch 不启动车辆执行链。可传 `config:=/absolute/planar_acceleration.yaml`、`geometry_config:=/absolute/rear_axle_geometry.yaml`；bag 回放传 `use_sim_time:=true` 并提供 `/clock`。

```bash
ros2 topic echo /state_estimation/planar_acceleration/status --once
ros2 topic echo /state_estimation/planar_acceleration/g --once
```

vehicle/race 支持 `planar_acceleration:=true`，默认 false；按原驾驶流程传其他配置。`record:=true` 包含 SI/G/status/debug，并快照参数。Vicon 话题名未知，需现场另加实际话题，详见[Vicon 采集要求](vicon-observer-validation.zh-CN.md)。

下一步优先用已同步、已测外参的 Vicon 检查快速换向中的补偿相位与静态偏置，再决定是否增加角加速度预测／对齐。不能仅加大平滑来宣称该问题消失。侧向速度仍未实现；平面近似 `ay=d(vy)/dt+r*vx` 中，`vx*r` 不能替代独立真值。
