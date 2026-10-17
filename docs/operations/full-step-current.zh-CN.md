# 全速度范围独立恒电流采集

> 2026-10-18 整理范围：本页记录已部署工作区的用法；这次文档提交不包含对应的新运行时代码，命令须在已部署环境执行。当前后轴／质心五量以[统一状态指南](state-observer.zh-CN.md)为准。


持续标定、PP 跟踪与 `inspect/start/stop` 操作工具已使用 C++17；启动时的一次性
Python 参数/冻结计划准备和离线分析继续保留。当前 NX 原标定环境也已更新，
入口与命令保持兼容，安装核验见[2026-10-16 迁移记录](../reports/2026-10-16-native-calibration.zh-CN.md)。

`full_range_step` 按实测轮速结束恒流：B 到 5 m/s、C 到 1 m/s 时立即请求
退出；在中途确认自然稳态也可结束。不预测提前撤流，不依赖经验响应表，不要求
另设运行速度上限。`max_speed: 5.0` 约束准备/回收目标速度，不表示退出尾段的
实测轮速峰值必须低于 5 m/s。元数据明确记录
`endpoint_exit_policy: measured_wheel_speed` 和
`operating_wheel_speed_limit_mps: null`。

A 的采集、`baseline` 导出和旧 `interval` 扫描保留。每个 B 电流重新从 1 m/s
稳速开始，施加独立固定请求电流；每个 C 电流重新从 5 m/s 开始。轮速是速度参考，
VESC `avg_iq` 是实际电流反馈。`current_motor`、`current_input` 和 IMU/KF
加速度保留各自含义，不替代轮速或实际 Iq 判据。PP 转向仍依赖定位和参考路径。

C 在有效直线内首次新鲜轮速达到或超过起始目标时，同一次控制回调立即请求
目标电流，不等待起始稳速或低加速度，也不等待轻微轮速过冲回落到 5 m/s。
仍要求最终速度指令匹配、控制权和输入有效，并保留直线退出余量。
`start_trigger_policy: first_wheel_at_or_above_start_speed` 标识该策略；
每次记录的 `start_trigger_observation` 保存触发轮速、源时间和切流请求时间。
它与命令确认后首个观测 `starting_wheel_observation` 分开，均不代表实际电流已建立。
`stable_seconds` 仍用于退出后的稳速交接，不用于 C 的起始切流。

B 从冻结 A 的起点稳速 Iq 加 5 A 开始，逐档增加 5 A，最后包含 100 A。
C 从起点稳速 Iq 减 5 A 开始，逐档降低正电流，包含 0 A 滑行，再以
−5 A 步长到 −100 A。每项一次，时间或直线不足时同档最多尝试三次；
各次片段独立保留，不拼成一条连续响应。转弯和回收目标为 2 m/s。

## 离线预览

在已经编译并加载本仓库 ROS overlay 的终端设置实际文件：

```bash
export AIMS_BASELINE=/absolute/path/to/baseline_current.json
export AIMS_MAP=/absolute/path/to/map.pcd
export AIMS_REFERENCE=/absolute/path/to/reference_directory
```

使用已通过 A 结果验收的冻结基准；已有多项式基准仅用于选择稳速参考电流。

```bash
ros2 run aims_calibration plan --stage accel_interval \
  --sweep-protocol full_range_step --start-speed 1 --end-speed 5 \
  --base-current-file "$AIMS_BASELINE" --current-step 5 --max-current 100 --repeats 1

ros2 run aims_calibration plan --stage decel_current \
  --sweep-protocol full_range_step --start-speed 5 --end-speed 1 \
  --base-current-file "$AIMS_BASELINE" --current-step 5 --max-current 100 --repeats 1
```

预览列出每个计划项及 `acquisition_policy`：实测轮速端点、实际电流交接启用、
未配置额外运行轮速上限、响应表未使用。旧参数 `--response-limits-file` 在完整协议
中仅输出 ignored 说明，不读取文件，也不根据表缺失或 140 ms 余量拒绝任何电流。
旧 interval 协议仍采用原有设置。

## 启动与启用

先在实车停稳后结束已有车辆会话，再开启新会话。B：

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=accel_full_step_1to5 base_current_file:="$AIMS_BASELINE" \
  steering_source:=pp map_file:="$AIMS_MAP" reference_directory:="$AIMS_REFERENCE" \
  record:=true longitudinal_acceleration:=false auto_start:=false rviz:=true
```

C 使用另一会话：

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=decel_full_step_5to1 base_current_file:="$AIMS_BASELINE" \
  steering_source:=pp map_file:="$AIMS_MAP" reference_directory:="$AIMS_REFERENCE" \
  record:=true longitudinal_acceleration:=false auto_start:=false rviz:=true
```

两个 profile 都设置 `predictive_exit: false`、`measured_current_handoff: true`、
`require_acceleration: false`，`response_limits_file` 为空。系统参数
`longitudinal_acceleration:=false` 关闭纵向观测器依赖，定位仍供 PP 使用。
车辆和 PP 使用已有 5 m/s 配置；VESC 起步电流替换关闭。配置未设置额外运行轮速
上限不会阻止采集，也不会自动套用 5.2 m/s 实测峰值停止条件。

另一终端加载相同环境后执行：

```bash
ros2 run aims_calibration inspect
ros2 run aims_calibration start
```

`auto_start` 为 false。`inspect` 核对静止、遥控权限、定位、输入新鲜度、唯一最终
命令来源、冻结文件和日志。RViz 核对方向、参考轨迹与直线区域后启用。
B/C 默认监控上限 7200 s；到期请求停用，不产生完成结论。

## 实测端点、稳态与尾段

B 在实测轮速达到或超过 5 m/s、C 在实测轮速达到或低于 1 m/s 时立即请求退出。
预测即将达到端点不构成正常退出。自然稳态要求 1 s 轮速窗口、至少 40 点、
回归斜率绝对值不超过 0.1 m/s²、速度跨度不超过 0.15 m/s，并继续确认 1 s。
实际 Iq 需在 0.25 s settle 后满足 0.5 s 窗口、至少 10 点，以及
至少 90% 样本误差满足 `max(1 A, 0.1×|请求Iq|)` 容差。稳态响应可以在到达目标速度前完成，因此查看
`stop_reason` 和 `endpoint_reached`，不要由 `success` 推断到达端点。

退出请求不会使实际电流瞬间改变。程序保留实际 Iq 确认：记录撤流请求、最终命令
回显、实际 Iq 接近过渡目标的确认、回收命令及回收稳速确认，然后才进入下一档。
原始日志持续覆盖准备、恒流、卸载与回收，记录轮速和尾段峰值/谷值。
`start_stamp/start_speed` 是命令确认后的首个成对有效观测，不代表实际 Iq 平台已经建立。
例如 80 A 真正建立时轮速已到 1.05 m/s，平台应从实际反馈对应的位置起算，不能
把请求电流的起始时刻直接当成 80 A 平台。退出后的轮速可继续变化；后续分析依据源时间和实际 Iq 平台
确定恒流有效范围，不把电流下降段直接当成原固定电流数据。本轮不拟合模型。
自然稳态若落在 A 基准覆盖区间外，使用 0 A 撤流并回收到 2 m/s，记录基准不可用原因，
不会外推 basic current 或让退出异常中断恒流请求。

保留 RC、控制权、通信新鲜度、命令一致性、时钟、日志、VESC 故障、区域退出、
8 s 试验超时。`protection_reverse_guard_enabled` 控制独立的 0.25 m/s
`near_reverse` 检查，默认开启；当前 C profile 显式设为 false，允许制动及交接
过程中出现低轮速或短暂负轮速，轮速原始值及交接尾段仍完整保存。
该开关及阈值写入 `protection_configuration` 和配置哈希。
时间/直线截断最多重试三次，耗尽后的 partial 留下覆盖缺口。

```bash
ros2 run aims_calibration stop
```

确认实车停稳后关闭整车 launch。0 A 请求本身不证明车辆已经停稳。

## 离线验收

```bash
ros2 run aims_calibration results /absolute/path/to/result_directory
```

schema 3 保存协议、实测端点策略、未配置额外运行轮速上限、冻结配置和计划哈希、
A 基准、跟踪/稳态/交接配置及每次响应。`response_limits` 为 null，角色为
`not_used`。旧软件原型的预测退出记录不能改标为新实测端点协议的完整响应。

`complete_responses`、`partial_responses`、`uncollected_responses` 和未发出恒流请求
的计划索引 `unexecuted` 分别统计。`execution_complete` 要求计划项均已完成或
明确耗尽尝试，并且所有实际脉冲的 Iq/回收交接已提交；`coverage_complete` 要求
每项都有可接受的完整响应。`valid` 同时要求执行、覆盖与证据完整。partial 和尾段
不会自动填充为完整恒流速度覆盖，故障及操作员停用原因始终可见。

续跑使用 `sweep_start_index`，保存原冻结计划长度与哈希，并重新从零编号后缀。
新会话只验收后缀，总体覆盖结合先前真实记录检查。`observations.csv` 保存完整原始
流程，`trials.csv` 中嵌套对象以 JSON 字符串保存。

## 电流与横向加速度标定导航

[电流标定 A/B/C 与当前结果](calibration.zh-CN.md#current-calibration) · [横向加速度 AY 校对及附着标定状态](lateral-acceleration-calibration.zh-CN.md)。操作按各指南注明的已部署环境执行；横向附着旧自动流程不作为当前启动入口。
