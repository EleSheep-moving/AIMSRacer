# 求解诊断与轨迹热启动

当前车辆默认参数为 B 组转向加速度权重 `0.6`、巡航目标 `1.0 m/s`、
IPOPT 迭代上限 `35`。启动时读取实际 `vehicle_config`；历史实验目录中的
配置是当时的快照，仍可能为 50 次迭代，复试时应使用新的配置副本。

## 热启动处理

有效求解后保存控制序列。后续每次请求从最新预测交接状态重新积分，
并相对真实已施加的输入限制加速度、转角变化。
失败不会清空最近的有效序列，也不会用失败迭代点覆盖它。
移位使用距离最近成功请求的累计时间：例如连续间隔 0.2 秒请求时，
失败后的移位年龄依次为 0.2、0.4、0.6 秒。

当年龄达到预测时域（当前 1.0 秒），改用参考速度和曲率生成初值。
控制器重新启用、进入新 generation 时仍清空旧缓存。
仅复用原始变量初值，未加入对偶变量热启动。
该处理不延长执行计划的 0.8 秒有效期，也不改变失败结果的拒绝规则。

## 在线状态

`/mpcc/status` 和 `controller-*.jsonl` 新增 `solve_sequence` 与
`solver_diagnostics`。后者在 ROS Diagnostic KeyValue 中是 JSON 字符串：

| 字段 | 含义 |
|---|---|
| `primal_inf` / `dual_inf` | IPOPT 最后一次迭代的原始/对偶不可行度，沿用 IPOPT 自身缩放 |
| `objective` / `barrier_parameter` | 最后目标值与障碍参数 |
| `step_norm` / `primal_step_size` / `dual_step_size` | 搜索步范数与线搜索步长 |
| `max_constraint_violation` | 在最终点独立计算的最大约束违约量 |
| `constraint_violations` | 初始状态、动力学、速度、加速度、jerk、转向变化、纵横加速度椭圆、启用时的走廊等分组违约量 |
| `worst_constraints` | 违约最大的五个约束块：组别、预测区间、子步、行号、数值与上下界 |
| `warm_start_source` | `last_success` 或 `feedforward` |
| `warm_start_age_s` / `warm_start_shift_steps` | 本次使用缓存前的累计年龄和移位步数 |
| `warm_start_cache_expired` / `warm_start_retained` | 原缓存是否超时、返回后是否仍持有有效求解缓存 |
| `consecutive_failures` | 连续失败次数，成功或 generation 重置后归零 |
| `preparation_time_s` | 输入处理、参数设置、轨迹初值构造时间 |
| `optimizer_time_s` | `Opti.solve()` 调用时间；首次调用可能包含原生初始化 |
| `diagnostics_time_s` | 返回后的残差提取、失败快照等处理时间 |

不同约束的原始单位不同；分组最大值用于定位违约来源，不是统一物理量。
IPOPT 残差与独立计算的约束违约量定义不同，不能要求数值相同。
无法读取的残差和非有限数在 JSON 中记为 `null`，不伪装成零。
`solve_time` 保留求解函数总耗时；`request_timing` 继续记录队列、返回和交接时序。

## 完整失败记录

启动 MPCC 时指定 `log_directory:=/absolute/new-run/logs/mpcc`，同一目录会生成：

- `controller-*.jsonl`：主控制状态及最近一次求解摘要。
- `solver-*.jsonl`：求解进程单独记录，每次求解一条，不重复 50 Hz 状态。

求解日志首行为 `metadata`，包含实际车辆配置、轨迹元数据和 CSV SHA256、
horizon/dt、求解相关源文件 SHA256；随后有 `warmup` 和 `solve` 记录。
`solve_sequence` 与 `generation` 可关联主控制日志。
每次求解均保存完整 `request` 和 `solve_input`（参数与构造的初值轨迹）；
这些完整数组仅在求解进程中写盘，不发送到主控制进程。

失败记录额外包含 `request`、`failure_snapshot`：完整请求、展开后的初始状态、
速度参考、map→odom 快照、构造的初值轨迹、失败时的迭代轨迹、每次 IPOPT
迭代统计，以及约束值/上下界/分组索引。候选轨迹仅用于诊断。
无穷下界和上界分别编码为对应 bounds 数组中的 `null`。

求解进程先发送紧凑回复，再写磁盘。完整失败快照不经过主控制回调。
`solve_time` 不含回复后的磁盘写入；下一请求的 `worker_queue` 可以反映其影响。
未设置 `log_directory` 时仍发布在线摘要，但不落盘完整失败记录。

`prepare_solver` 也打印收敛摘要，便于确认实际迭代上限及诊断字段可用。
缓存准备中的首次求解时间包含初始化，不能直接当作行驶时延。
