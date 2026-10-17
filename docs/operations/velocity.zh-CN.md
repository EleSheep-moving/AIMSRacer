# 独立后轴 Vx 观测

> 兼容单分量接口说明：当前推荐使用[后轴／质心统一五量节点](state-observer.zh-CN.md)。对应源码保留在 `src/aims_racer_system/`，按当前构建流程生成。


[English](velocity.md) · [质量报告](../reports/2026-10-15-vx-quality.zh-CN.md)

该节点只用于并行观察后轴车体系纵向速度。内部保留三维位置、速度及慢变加速度残差，使用 corrected IMU 推进，在 LIO 的历史源时刻更新位置和姿态，再重放至最新 IMU。轮速、EKF、LIO twist、指令及电流均不进入估计器。尚未通过 Vicon 独立精度验收。

## 输入和输出

| 方向 | 话题 | 定义 |
|---|---|---|
| 输入 | `/livox/imu_bias_corrected` | `sensor_msgs/Imu`；`livox_frame`；加速度仍为 **g**，角速度 rad/s，源时间戳不改写 |
| 输入 | `/fastlio2/lio_odom` | `nav_msgs/Odometry`；`odom` → `livox_frame`；仅使用物理 IMU 原点的位姿，忽略 twist 和零协方差 |
| 输出 | `/state_estimation/vx` | `TwistWithCovarianceStamped`；`base_link` 后轴；**只有 linear.x 有效** |
| 诊断 | `/state_estimation/vx/status` | 输入源/接收年龄、历史重放、拒绝计数、参考点及有效性 |
| 调试 | `/state_estimation/vx/debug` | 世界三维位置/速度、IMU 系有效残差、位置创新；不能当已验证的 vy |

输出时间是最新实际使用的 IMU 源时间。其余 twist 分量的零值只是占位，对应协方差对角项为 1e6。Vx 协方差属于未校准的条件有效模型；LIO 与 IMU 共享信息，不可用它证明独立精度。输入使用 reliable QoS；生产 corrected IMU/LIO 发布者均提供 reliable，测试发布器也须匹配。

## 启动和停止

先在 ROS Humble 工作区编译 `aims_racer_system` 并 source 对应 install。只在已有 corrected IMU 和 raw LIO 上增加此独立节点：

```bash
ros2 launch aims_racer_system velocity.launch.py
```

NX 上已经验收的独立安装可直接使用；先保留现有传感器主图，只在单独终端启动观测器：

```bash
source /opt/ros/humble/setup.bash
source /home/aims/aimsracer-vx-observer-20261015/install-final/share/aims_racer_system/local_setup.bash
ros2 launch aims_racer_system velocity.launch.py
```

Ctrl-C 停止该独立进程。既有 vehicle/race launch 的 `vx_observer` 默认 `false`；显式设置 `vx_observer:=true` 才加入节点。`record:=true` 时录制三条输出/诊断话题及配置快照。节点不发布 TF 或控制命令，也没有新增控制消费者。不要在已经运行车辆主图时再启动一个 vehicle/race 主图。

几何来自共享 `rear_axle_geometry.yaml`：目前后轴→IMU 近似杆臂 `[0.30, 0, 0.03] m`，安装旋转为单位阵。FAST-LIO 内部 `t_il/r_il` 不能再次叠加。它不是新的精密外参标定；质心高度未知，首版没有质心输出。

初始化需要 ≥3 个 LIO 位置、覆盖 ≥0.2 s。通过扣除 IMU 预积分位移后的直线回归估计初始速度，可在行驶中启动，不把初始 Vx 或内部 vy 设为已知零。

## 输入失效

- IMU 源间隔 >30 ms：清除当前速度初始化；等待新的三点位置支持，不跨断流积分。
- IMU 源年龄 >50 ms 或时间在未来：拒绝输入；定时诊断也会将过期输出标为无效。
- 接受的 LIO 锚点距当前源时间 >0.3 s：降级；>0.5 s 停止数值发布。诊断按接收时钟检查锚点是否已过期。
- 历史保留 2 s，并有 4096 条事件上限。过旧、重复、非法 frame、非有限数据、无效四元数及数值运算失败均拒绝。创新门限拒绝的位置/姿态不能刷新锚点。
- 锚点已过期时，新 LIO 触发重新初始化，仍需新的至少三点和 0.2 s 支持；期间停发，`lio_reinitializations` 记录次数。单个新位置不能直接恢复速度输出。
- ROS 时钟回退：清空状态和发布 epoch；同一 epoch 内输出时间严格递增。LIO 历史纠偏不会再次发布过去的输出。

`last_rejection` 是最近一次拒绝原因；LIO 创新拒绝总数见 `rejected_lio_innovations`，输入拒绝计数单独保留。`imu_gap`/`initializing`/`degraded`/`lio_stale` 不能通过删掉相应样本来评估成连续有效。

## 复核

```bash
ros2 topic echo /state_estimation/vx/status
ros2 topic echo /state_estimation/vx
```

默认参数在 `params/velocity.yaml`。位置标准差冻结为 0.03 m，慢变残差 PSD 为 1e−4 m²/s⁵。位置 NIS 门限 16.266、姿态创新角门限 0.35 rad 是拒绝规则，不是已校准故障检测概率。

构建、合成真值、故障测试、跨架构及完整 ROS 回放说明见 verification/velocity（记录中的工作区路径：`verification/velocity/README.md`）。实包观察与独立真值验收分开，当前不能接入控制或侧滑监督。
