# 独立五量估计器：计算链、延迟对照与警告优先策略

日期：2026-10-17。针对用户提出的独立性、低延迟及尽量持续发布要求，核对当前源码，并修订双参考点节点的年龄处理策略。[操作说明](../operations/state-observer.zh-CN.md)。

## 1. 用户要求与实际接线

目标分工为 Robot Localization 向 MPCC 提供 X、Y、Yaw，独立估计器提供后轴／质心 VX、VY、AX、AY、yaw rate。优先降低动态响应延迟；Robot Localization 是对照对象，不是假定正确的真值。

`state_observer` 实际只订阅 `/livox/imu_bias_corrected` 和 `/fastlio2/lio_odom`，不订阅 `/odometry/filtered`，也不读取轮速、控制命令或 LIO twist。两点五量已经独立发布。

但 **MPCC 的当前接线还没有完成上述目标分工**：[aims_mpcc_rt/src/node.cpp](../../src/aims_mpcc_rt/src/node.cpp) 的 `odometry()` 仍从导航 odometry 取 `twist.twist.linear.x` 作为控制状态 VX。发布独立观测器不等于控制器已使用它。本次未修改 MPCC 输入或 Robot Localization 融合配置，也未替换 NX 安装。

## 2. VY 的实际计算方法

车体平面运动、同一参考点和同一时刻下，关系为：

\[
\dot v_x=a_x+r v_y,\qquad \dot v_y=a_y-r v_x.
\]

AY 是惯性加速度在车体 y 轴上的分量；`r*vx` 是旋转坐标系导数中的耦合项。AY 本身不需要减去 `r*vx` 才成为真实侧向加速度。

当前 velocity.cpp（记录中的工作区路径：`src/aims_racer_system/src/velocity.cpp`） 不显式在平面累加上述第二式，而采用对应的三维惯导形式，在 IMU 原点推进：

\[
\dot p^W=v^W,\quad
\dot v^W=R_{WI}(g_0 f^I-b_a^I)+g^W,\quad
\dot R_{WI}=R_{WI}[\omega^I]_\times.
\]

其中 corrected Livox 比力 `f` 仍以 g 为单位，`g0=9.80665` 转成 SI；`b_a` 是速度核心估计的加速度偏置。收到 LIO 后，用**位置创新** `p_LIO−p_pred` 及 `[p,v,b_a]` 的九维交叉协方差同时纠正位置、速度、偏置，并锚定 LIO 姿态。晚到 LIO 在历史源时刻更新，再重放后续 IMU 至最新源时刻。

最后将世界速度转到车体系，并做参考点换算：

\[
v_P^B=R_{BI}R_{WI}^{T}v_{IMU}^{W}+\omega^B\times d_{IMU\to P}^{B}.
\]

因此“惯性推进、LIO 抑制漂移”是正确的概括；“把已发布 AY 积分，再融合 LIO 消息的 VY”**不是当前代码实现**。LIO 的位置已能通过状态交叉协方差约束速度，不需要把共享 IMU 的 LIO VY 再当作独立信息叠加。

## 3. AY、VX 和 r 是否来自自己的估计器

| 量 | 实际来源 | 是否读取 Robot Localization |
| --- | --- | --- |
| VX／VY | corrected IMU 推进、自身加速度偏置状态、LIO 位置／姿态纠偏、参考点转换 | 否 |
| AX／AY | corrected IMU 比力、gyro 推进与 LIO 锚定姿态、重力恢复、角加速度及向心杆臂补偿、末级滤波 | 否 |
| yaw rate | corrected gyro，经安装旋转与 `[r,alpha_z]` 滤波 | 否 |

需要区别内部推进与最终发布：速度核心用 corrected gyro 推进姿态，参考点速度换算也用同刻未平滑 gyro；发布的 yaw rate 经过另一滤波器。AX/AY 分支固定加速度偏置当前为零，未使用速度分支估计的 `b_a`，还有角加速度回归与末级滤波。因此不能把三条最终发布流逐点代入公式，当作严格离散恒等式检验。

这里的独立是**独立于 Robot Localization 的计算链**；仍使用 IMU/LIO，LIO 与观测器也共享 IMU。九维位置／速度／加速度偏置协方差加外部姿态锚定，是当前简化实现，不是完整的姿态／gyro bias／重力联合误差状态滤波器。

## 4. 成熟实现如何做

- FAST-LIO 的原始实现中，`get_f()` 用旋转后的去偏 IMU 加速度加重力推进世界速度；地图点到平面的残差用于激光更新。它支持“IMU 高频推进、外部几何观测纠偏”这一方向，不能据此说本项目的简化九维估计器与 FAST-LIO 完全相同。[原始传播代码](https://github.com/hku-mars/FAST_LIO/blob/main/include/use-ikfom.hpp)、[激光更新](https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp)。
- PX4 EKF2 在延迟融合时域融合异步观测，再利用缓冲 IMU 将输出推进到当前时刻；输出还补偿 IMU 到机体参考点的偏移。低频或晚到外部观测，不意味着控制输出必须停留在那个旧时刻。[官方说明 Overview](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf#overview)。
- Robot Localization 的 `sensor_timeout` 表示超时后做预测而不做观测修正，并不是一超过该时间就停发。其源码在已初始化而无新观测时继续预测。**拒绝一条异常观测、继续预测、警告输出质量，是不同的动作。** [官方参数文档](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst)、[实际处理代码](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/ros_filter.cpp)。

这些是成熟方法的可核对实例，不是对商业系统市场占比、所有赛车队采用同一公式或本车精度的证明。

## 5. 本轮已修改的门控

新节点默认 `warn_only_quality_gates=true`。原来的年龄条件与“能否计算”分开处理，原始时间戳保持不变。

| 条件 | 本轮后的行为 |
| --- | --- |
| 已初始化后 LIO 锚点超过 0.5 s | 继续 IMU 推进 VX/VY 和姿态、继续计算 AX/AY；reason=`lio_stale`、诊断 WARN |
| LIO 恢复 | 不再仅因超时强制重新等待三帧位姿；正常创新校验后纠偏 |
| LIO 迟到超过 0.5 s、但仍在 2 s 历史内 | 允许历史更新和重放；超出历史支持仍拒绝该条观测 |
| IMU 到达年龄超过 50 ms、源序列仍连续 | 按源时间继续计算并发布，reason=`imu_stale`，新鲜度 valid=false、computable=true |
| 没有新 IMU | 不重发旧值、不把旧值重新盖成当前时间；诊断继续更新 |
| LIO 位置／姿态创新异常 | 拒绝该次纠偏，继续 IMU 推进，不联动停止其他分支 |
| NaN、算术溢出、明确标记测量不可用、错误 frame／非法时间、未初始化 | 拒绝相关输入或停止受影响分支，防止将无定义结果伪装成估计 |
| IMU 源间隔大于 30 ms、gyro 回归窗口缺少足够时间支持 | 保留原来的重建／重新准备；角加速度不可计算时只停相应加速度分支 |

速度续推的协方差按原传播模型增长。AX/AY 在 LIO 超时后额外增加 `g0²*gyro_variance*coast_time²` 的保守模型方差，其中 `coast_time=max(anchor_age−0.5,0)`；它尚未用实测漂移标定，不能表示已知精度。长期失去外部约束仍可能漂移或无法接受恢复后的创新，持续发布不表示可无限期可信使用。

`*_computable` 解释是否可形成数值；`*_valid` 还要求 IMU 时间新鲜；`quality_degraded`、各 reason 与源年龄解释质量。LIO 超时但 IMU 新鲜时，可计算／valid 为 true，诊断仍 WARN。配置设为 false 可恢复新节点原来的严格年龄策略。旧 scalar VX、body/planar acceleration 节点默认策略不变。

## 6. 延迟边界与后续验证重点

新节点不等待下一帧 LIO 或未来样本，晚到 LIO 修正历史后输出最新 IMU 源时刻的状态。初始化所需约 0.2 s 位姿跨度属于启动准备，不是稳态每次输出固定延迟。

但**没有等待未来样本，不等于零动态延迟**。AX/AY 的尾随 25 ms gyro 回归存在时间分布／补偿相位问题；yaw 滤波和加速度末级滤波也各有动态响应。单线程回调中 LIO 重放可能暂时阻塞 IMU 处理，且 reliable DDS 队列可能积压。本轮没有改滤波参数或声称完成这些延迟优化。

继续优化应分别测量输入源年龄、接收至发布耗时、MPCC 实际使用年龄、合成真值下幅值／相位；同时检查 VX 偏置和 AY 快速转动误差。外部真值可用于精度验收，Robot Localization／LIO／轮速仅提供有各自误差与相关性的对照。质心与后轴只在同一参考点转换后比较。


## 7. 新版本与 Robot Localization 的同包对照

原包：`2026-10-15/022604_194643_race/bag/bag_0.db3`。导航值来自原包，新节点值来自本轮实际 ELF 的 1× 原接收顺序回放。使用同一后轴参考点、原源时间 44≤t<219 s，共 **35,000** 个 200 Hz 网格点；扫描 ±800 ms、步长 5 ms，两个估计器和所有候选偏移共用支持掩码，不拟合增益或偏置。

定义为 `estimate(t)` 对照 `reference(t−lag)`，正 lag 表示估计波形滞后于参考。轮速在滑移时可失真，LIO 已与导航融合且与本估计器共享 IMU，corrected gyro 是 yaw 分支自身输入。**这些结果不是独立真值误差，也不是物理延迟测量。**

| 量／参考 | 导航最佳波形偏移 ms | 自身最佳波形偏移 ms | 导航零偏移 RMSE | 自身零偏移 RMSE |
| --- | ---: | ---: | ---: | ---: |
| VX／轮速（m/s） | 30 | -10 | 0.070667 | 0.105999 |
| VY／LIO VY（m/s） | 265 | 25 | 0.099300 | 0.070013 |
| yaw rate／corrected gyro（rad/s） | 50 | 5 | 0.147812 | 0.042907 |

自身 VY／yaw rate 相对这些参考的波形滞后小于导航；VX 的 −10 ms 不能解释为物理负延迟。VX 与轮速的零偏移偏差反而更大，约 0.106 对 0.071 m/s，不能据此称自身五量全面更准。若人为平移到最佳位置，导航 VY 对 LIO 的 RMSE 为 0.04677 m/s，自身为 0.06839 m/s；它也说明“更及时”与“更接近某条共享观测曲线”不是同一个指标。

AX/AY 没有本次可用的独立同量真值；原导航配置也不发布可直接比较的加速度话题。不能把其内部加速度、轮速差分或 `vx*r` 直接当作本节点 AX/AY 的验收真值。

完整结果与哈希：[waveform-comparison.json](assets/2026-10-17-state-observer-latency-policy/waveform-comparison.json)。工具为 compare_reference.py（记录中的工作区路径：`verification/dual_reference/compare_reference.py`）。本轮 nominal 运动窗口的数值与修订前完全相同，说明发布策略修改没有改变该窗口的正常滤波响应；它解决的是超时／迟到时的持续计算与报告，不能当作本轮将动态响应加快的证据。

## 8. 软件与完整回放验证

完整 CTest **30/30 组通过**；26 个 XML 文件、**192 testcase**，失败／错误／跳过均为零。新增发布策略用例先在旧程序中复现 LIO 超时和连续迟到 IMU 停发，再运行修改后的代码。初次绿色检查还修复了测试中 ROS byte 类型与 Python int 的比较，之后完整套件通过。覆盖 LIO 超时持续输出及正常恢复不重初始化、迟到连续 IMU 原时间发布、0.55 s 迟到 LIO 历史更新，以及既有 NaN／溢出／不可用／源断流／时间支持与 epoch 恢复。

本轮完整原包 **221.929990 s、1×** 回放，发送 **42,315 IMU＋2,229 LIO**；每参考点收到 **42,255 速度、42,293 加速度、42,313 yaw rate**。两参考点源序列一致，速度刚体关系最大残差 **8.64e-16 m/s**、yaw rate 差 **0**。结束后的约 1 s 停流将所有新鲜度 valid 置为 false；computable 保留 true，数值不重复发布。

实际启动子进程 ELF SHA256：`9d1aa3b796b385bcc3d7a5a45558eef1dea8612dd642cfb5ccf0ffd5fea86c1c`；回放保存真实 `/proc` 命令、所选参数文件、有效参数和输入／输出哈希。x86 隔离 ROS Humble 容器内完成，部分时间与另一 ROS domain 的 CTest 并行，不是独占 CPU 的延迟基准；未部署 NX、未改 MPCC／导航接线、无独立 Vicon 精度验收。

证据：[CTest](assets/2026-10-17-state-observer-latency-policy/ctest-x86.txt)、[回放摘要](assets/2026-10-17-state-observer-latency-policy/replay-summary.json)、[源码与验证清单](assets/2026-10-17-state-observer-latency-policy/validation-summary.json)。原始完整日志及输出保存在 `aimsracer-data/experiments/dual-reference-state-20261017/warning-policy/`。
