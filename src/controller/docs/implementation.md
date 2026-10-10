# Native MPCC 实现与阅读路线

当前生产链为离线 Python 生成模型/参考/bundle，在线 C++ 调用 generated acados SQP-RTI。部署与启用命令见[使用](usage.md)；坐标和命令归属见[架构](../../../docs/architecture.md)。本文描述当前 v35 配置及 native 执行链。

## 从输入到执行

| 位置 | 责任 |
| --- | --- |
| [recorder.py](../aims_mpcc/recorder.py)、[prepare.py](../aims_mpcc/prepare.py) | 原始后轴 CSV 记录和闭环参考准备 |
| [path.py](../aims_mpcc/path.py) | 周期 quintic 参考、投影、坐标与几何元数据 |
| [speed_planner.py](../aims_mpcc/speed_planner.py) | 曲率限速、物理距离积分、闭环前后向加减速传播 |
| [backend_models.py](../aims_mpcc/backend_models.py)、[acados_backend.py](../aims_mpcc/acados_backend.py) | 离线离散模型、代价和约束 |
| [export_bundle.py](../../aims_mpcc_rt/scripts/export_bundle.py)、[build_bundle.py](../../aims_mpcc_rt/scripts/build_bundle.py) | 固定输入、生成 C、目标平台编译和 manifest |
| [core.cpp](../../aims_mpcc_rt/src/core.cpp) | 加载 bundle、RTI、候选重积分与验证 |
| [node.cpp](../../aims_mpcc_rt/src/node.cpp) | ROS 状态、worker、接管、输出、启动/停车及异步日志 |
| [history.hpp](../../aims_mpcc_rt/include/aims_mpcc_rt/history.hpp)、[execution.cpp](../../aims_mpcc_rt/src/execution.cpp) | 实际命令历史、接管对齐和输出采样 |
| [health.hpp](../../aims_mpcc_rt/include/aims_mpcc_rt/health.hpp) | protocol-v1 地图锚点/健康资格 |

## 状态、模型与输入

六个物理状态为 `[x,y,yaw,v,theta,delta]`：后轴位置、朝向、速度、参考进度参数和模型转角。三个控制为 `[acceleration,steering_endpoint,virtual_progress_speed]`。内部另存上一加速度、转向命令端点和命令速率，合计九个内部状态。

后轴模型的主要关系为：

\[
\dot x=v\cos\psi,\quad \dot y=v\sin\psi,\quad
\dot\psi=\frac{v\tan\delta}{L(1+k_u v^2)},\quad
\dot v=a,\quad \dot\theta=v_\theta,\quad
\dot\delta=\frac{\delta_{cmd}-\delta}{\tau_\delta}.
\]

`theta` 参数来自录制点的累计弦长；平滑后不与真实弧长严格相等，几何线性化与速度传播使用样条切线 norm。当前 `L=.36 m`、`tau_delta=.08 s`、`k_u=0`；零欠转向系数是模型假设，不是测得无侧滑。

100 ms 求解 stage 内有五个 20 ms RK4 子步，转向端点按输出持有的命令 ramp 采样。求解模型优化加速度，底盘实际接收速度设定；理想加速度传播并不证明未知电机响应。详细纵向辨识继续按后续加速度接口计划进行。

## 参考、误差与目标

参考是固定周期 quintic 曲线，物理状态在 `odom` 传播；map 参考使用同一资格载荷中的 `map ← odom` 对齐。每次 RTI 的局部几何由参考进度处冻结，第二次可更新几何。该模型使用 Cartesian 状态与 contour/lag 误差，不是 Frenet 状态动力学。

contour 为法向距离，lag 为切向误差，heading 与参考切线比较。代价包括归一化 contour、lag、heading、速度跟踪、虚拟进度速度跟踪、转向 feedforward 偏差、加速度以及转向命令速率/速率变化；终端放大状态代价。当前 `contour_weight=16`、`heading_weight=4`、`speed_weight=133.333...`。权重对应不同尺度和量纲，数值不能直接比较强弱。

当前 v 与虚拟进度速度追随曲率速度参考，没有线性推进奖励。曲率速度上限为 `min(cruise,max_speed,sqrt(ay/abs(kappa)))`，随后按周期物理距离传播加速与制动限制。规划参考被冻结进 bundle。实跑将 cruise 从 2.6 提到 3.5 后，规划理想滚动时间收益仅约 0.096 s；详见[外场复盘](../../../docs/reports/2026-10-10-mpcc-field-review.md)。

## 当前约束范围

`rate_bounded_v2` 保留速度、加减速、转角和转向速率边界；转向速率变化仍有代价。当前独立加速度设置 `combined_accel_constraint_enabled=false`，生成 OCP、重积分候选、执行认证和制动预算使用一致配置；曲率速度规划 ay=1 继续生效。它不保证轮胎横向力预算。

`enforce_corridor=false` 关闭参考 footprint 约束与实测/启动 footprint 停车检查。左右各 0.5 m 是保存的课程模型，不是实测赛道边界。恢复边界约束或改变物理参数须生成新 bundle，并以实际空间/车辆数据评价。后续研发假设不写入本次实跑证据。

## Measurement time, computation and takeover

一个请求固定原始 EKF source epoch、由实际 `/ackermann_cmd` 历史预测的执行起点和地图对齐。一次只拥有一个 pending plan，优化请求上限 20 Hz，命令输出 50 Hz。`solver_timeout=.05 s` 包括完整 worker 计算和结果 delivery 等待；超过预算的结果不进入执行。

候选先经非线性重积分和当前配置约束检查；接管时按实际状态和已应用前缀重新对齐并重新验证。执行不能跳过尚未实际发出的新控制。无法通过验证的候选丢弃，旧计划保持原 source TTL；N15/dt=.1 默认 `plan_ttl=1.2 s`。`handover_delay=.02 s` 是预计接管提前量，与求解预算和 source TTL 分别计时。

命令历史必须区分 `/drive` 提议和 `/ackermann_cmd` 转发；`minimum_drive_speed=.2 m/s` 为现有底盘小正设定映射，disabled/faulted 保持零，停车阶段低于阈值归零。物理速度仍能在起步/停车时经过该区间。

worker 调用、结果交付、接管与完整输出 callback 分别计时。状态/时钟/定位/RC 所有权变化会使计划失效。C++ 并不保证上游状态始终新鲜，TF 时间戳也不代替地图资格。

## 启动、单圈与故障

native 节点出生时 disabled，`/mpcc/enable` 执行统一条件检查。`auto_start=true` 在默认 60 s 内等待同一组条件，成功后只启动一次；显式 stop 取消等待。成功启动之后的 COMPLETE 或故障不会自动再次 enable，fatal 不自动重启。

`repeat_laps=false` 从实际启用位置计一圈，终点速度 cap 和停车逻辑负责停止；true 连续圈由操作员 stop。Plan expiry/失去资格会进入停车或恢复逻辑，重新运行需要显式授权。现场 RC/急停仍决定底盘执行权限。

## 日志与证据

`runtime.csv` 保存 worker disposition、delivery、接管接受/原因、reanchor 和命令发布（含零输出）。异步有界日志队列的丢项由诊断报告。`/mpcc/status` 保存实际状态、新鲜度、自动启动和求解/接管计数。`record=true` 将 bag、runtime 与系统快照放在一个会话目录，见[录制](../../../docs/operations/recording.md)。

2026-10-10 NX 已有真实 FAST-LIO2/EKF/NDT 下的闭环行驶，规划峰值 2.99、实测约 2.89 m/s。报告中的运动窗口含起步/制动，不能称 flying lap；完整 worker 时长与 native solver、输出 callback 时长分别解释。软件回归位于 `verification/`，新增入口与单 workspace 整合的检查不替代原始实跑。

模型和参考来源保留 [NOTICE.md](../NOTICE.md)、[LICENSE](../LICENSE) 和 native [NOTICE.md](../../aims_mpcc_rt/NOTICE.md)。
