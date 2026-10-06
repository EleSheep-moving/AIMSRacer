# 现有 LIO 测试 bag 清单

核对日期：2026-10-04。范围为当前 Orin 的 `/home/aims`。用户删除原 C 后，
现有 **18 个 SQLite rosbag2 目录**，其中 **7 个包含原始 Livox 雷达和 IMU**；
排除一个严重缺帧的早期录制后，有 **6 个可用候选**。初检时为 19 个目录、
8 个原始输入 bag；后续已核实 C 的目录不存在。其余录制文件位于下列归档目录。
这些数据只在本机，GitHub 和新 checkout 不包含 bag。

本次整理统一使用 `~/aimsracer-data/sessions/<日期>/<实验>/`；同一实验的
bag、日志和分析结果放在一起。全量路径索引在 `~/aimsracer-data/bags.csv`
和 `bags.json`，旧路径对应关系在 `migration-2026-10-04.json`。后续录制遵循
[数据目录约定](recording.md#local-data-layout)。

检查依据是 `metadata.yaml`、数据库 topic 定义和只读消息检查。对初检中的原始输入
bag，核对了实际雷达/IMU消息数，读取了全部原始 IMU 和已录制的轮速，并将
字段解析与 ROS 反序列化交叉核对。除弃用录制外，原始 IMU 的源时间戳均未出现
倒退/重复或超过 30 ms 的相邻间隔。这不是逐点点云完整性或定位精度认证。

## 优先使用哪几个

地面运动测试优先使用 **A**，静止基线优先使用 **D**。B 仅按下文限制用于
高角速度/三维姿态压力测试；E/F/G 用于较长静止或近静止基线。原 C 删除后，
当前清单不再提供此前推荐的长时间地面运动候选，后续需要补录。

| 编号 | 推荐用途 | 时长 | 数据库大小 | 雷达 / 原始 IMU 消息数 | 特点 |
| --- | --- | ---: | ---: | ---: | --- |
| A | 短运动、修正前后对照 | 47.71 s | 325.8 MiB | 478 / 9539 | 轮速非零主要在约 12–32 s；原始 yaw rate 最大绝对值 0.88 rad/s。此前已用于独立核心计算对照。 |
| B | 高角速度、三维姿态和匹配收敛压力 | 71.45 s | 306.2 MiB | 714 / 14285 | 前约 15 s 为近水平运动；约 25–50 s 包含轮速为零的大幅转动/倾斜，原始 IMU z 角速度最大绝对值 3.94 rad/s。不能将整个 bag 当成纯地面剧烈转弯。此前已用于迭代数/耗时对照。 |
| D | 快速静止基线、MP 开关/线程对照 | 30.00 s | 117.0 MiB | 300 / 6001 | 专用近静止 benchmark 录制，只有两个原始输入 topic；此前六轮回放均处理 300 扫描、输出 297 帧里程计。 |
| E | 较长静止基线 | 162.47 s | 1111.8 MiB | 1624 / 32477 | 录制轮速全程为零；原始 yaw rate 最大绝对值 0.036 rad/s。 |
| F | 静止长尾、全栈录制参考 | 316.31 s | 2166.7 MiB | 3162 / 63251 | 录制轮速全程为零；此前原 LIO 接收年龄最大约 397 ms，可观察长尾，但不是约 1 s 事故复现。 |
| G | 长时间近静止运行、辅助基线 | 705.28 s | 3061.0 MiB | 7053 / 141057 | 录制轮速全程为零，前段有少量转动瞬态，原始 yaw rate 最大绝对值 1.03 rad/s；不适合作为持续行驶数据。 |

大小为 `.db3` 文件大小，以 MiB（1024² 字节）计，不包含旁边的日志。角速度
为原始 IMU 的 z 分量，未减静止偏置。运动区间按 bag 接收时间相对首条消息
计算，轮速阈值为 `abs(vx) > 0.05`；表中的区间是首次至末次超过阈值，
**不表示整个区间连续运动**。历史轮速比例、正负方向和滑移未重新标定，尤其
B 中轮速有较大正负值，不能将它解释成独立真实车速。

### 目录位置

回放传入包含 `metadata.yaml` 的目录，而不是只传 `.db3` 文件。当前每个目录
均只有一个数据库文件。

| 编号 | bag 目录 |
| --- | --- |
| A | `/home/aims/aimsracer-data/sessions/2026-10-03/venue-161459/bag` |
| B | `/home/aims/aimsracer-data/sessions/2026-10-03/raw-input-check-013910/bag` |
| D | `/home/aims/aimsracer-data/sessions/2026-10-03/lio_mp_benchmark/raw_input_complete` |
| E | `/home/aims/aimsracer-data/sessions/2026-10-03/venue-161459/bag-test` |
| F | `/home/aims/aimsracer-data/sessions/2026-10-03/venue-161459/bag-test2` |
| G | `/home/aims/aimsracer-data/sessions/2026-10-03/raw-input-selfcheck-004259/bag` |

A/B/E/F/G 同时有当时的 `/fastlio2/lio_odom`、后轴 LIO、轮速、EKF及控制
等输出。A/E/F 还包含 body/world cloud 和定位诊断，数据库较大；只测 LIO 时
筛选两个原始输入 topic 即可。当时录制的 LIO 输出只能作为旧算法的对照，
不等于外部定位真值。D 没有原始运行的 odometry，需要重新运行 LIO 得到输出。

### B 的定位质量与非平面运动说明

补充核对原始 LIO 和 EKF，按消息源时间对齐，使用末段估计的三轴静止陀螺偏置及完整
三维角速度积分：B 在约 25–50 s 的轮速为零，但 LIO 最大 roll/pitch 达
**60.7°/50.1°**；陀螺积分给出相近的倾斜，说明不能把大姿态变化直接判作
LIO 姿态发散。这段可能包含抬起、倾斜或搬动车辆的操作，缺少视频，未确认具体动作。

| B 的补充指标 | 观测值及含义 |
| --- | --- |
| 原始 LIO 相对三维陀螺积分的 yaw 增量差 | 全程最大约 3.34°，末尾约 2.26°；不是独立真值精度。 |
| EKF 与同源时间原始 LIO 的航向差 | 最大约 23.19°，发生于约 40.15 s；当时原始 LIO roll 约 57.7°、pitch 约 -17.6°。融合输出的明显异常与原始 LIO 需分开判断。 |
| 原始 LIO 接收年龄 | P95 约 104 ms，最大约 127 ms，没有本次事故的约 1 s 累积延迟。 |
| 60 s 后近静止段 | LIO 三轴位置各自变化范围不超过约 4.4 mm，净位移约 2.2 mm，yaw 范围约 0.10°。 |

这些证据不能确认或排除运动段的位置漂移，因为没有独立位置真值。B 可用于
比较相同原始输入下的内部计算量/收敛，以及非平面运动鲁棒性；验证正常地面
转弯宜先使用 A，或单独评估 B 的前约 15 s。此前同输入数学公式的耗时对照
仍可使用，不能由此声称地面车辆定位精度已验证。补充检查数据在本地忽略目录
`/home/aims/aimsracer-data/sessions/2026-10-04/lio-b-drift-check/`。

## 已删除的原 C

原 C：`/home/aims/mpcc-test-20261003-002036`，788.77 s，原数据库约 3366.8 MiB。
用户明确不再采用并已删除该 bag；核对时目录已不存在。该编号仅保留作历史说明，
不计入现有目录或可用候选，也不再用于回放推荐。原 B 仍在，未删除。

## 一个不要用于正式对照的原始输入 bag

`/home/aims/aimsracer-data/sessions/2026-10-03/lio_mp_benchmark/raw_input`

它虽有 `/livox/lidar` 和 `/livox/imu`，但 30.18 s 中只有 **156 帧雷达和
173 帧 IMU**，IMU 源时间间隔中位数约 **190 ms**，严重低于预期 200 Hz。
此前 benchmark 已弃用这份录制，实际使用上面的 **D：`raw_input_complete`**。
不要混淆两个目录，也不要将它的结果用于评价正常输入下的去畸变或处理性能。

## 缺少原始雷达的其余 11 个 bag

以下全部缺少 `/livox/lidar`，**无法重新运行 LIO 的扫描匹配**。其中的已计算
里程计、IMU和控制输出仍可用于 EKF、控制、模型或时序分析。

### 专门的转向响应/转角速度标定录制

以下两份仍在本机，已经重新核对数据库中的转向指令和运行模式。它们不是
无效录制：之前将其列在本节，是因为缺少原始雷达，不能重跑 LIO；转向标定
价值需要与 LIO 重算能力分开看。

| bag 目录 | 时长 / 数据库大小 | 已核对的内容与用途 |
| --- | --- | --- |
| `/home/aims/aimsracer-data/sessions/2026-09-27/calibration-234227/bag` | 233.87 s / 103.2 MiB | 46732 条最终控制指令，转向范围约 ±0.4751 rad；混合速度、电流和占空比模式。有重复左右转向阶跃，历史分析选出 21 个低速事件，等效转角 10–90% 平均变化速率中位数约 4.6 rad/s。录制时 ERPM 比例为旧的 4650，复查时需保留历史速度换算。 |
| `/home/aims/aimsracer-data/sessions/2026-09-28/calibration-005010/bag` | 324.37 s / 140.7 MiB | 64040 条最终控制指令全部为速度模式，转向范围约 ±0.4751 rad；主要约 1 m/s，历史分析有 10 个独立中位到左右满转向事件。它是当前 0.08 s 单一转向响应常数的重要拟合数据，更适合当前低速速度模式。 |

两份都有 `/ackermann_cmd`、`/commands/servo/position`、
`/sensors/servo_position_command`、原始 `/livox/imu`、轮速、原始 LIO 和 EKF。
舵机话题是命令/回显，没有物理转角传感器；因此已有的“转角速度”是通过
IMU yaw rate 和前向速度推算的等效转向响应，不能当作舵机机械速度上限。

既有结果见 [9月27日录制的转向响应分析](../reports/2026-09-28-vehicle-response.md)
及 [9月28日速度模式标定](../reports/2026-09-28-speed-mode-calibration.md)。
本次检索还尝试了 `/data`，但目录为 root 私有、当前用户无读取权限，且没有
免密码 sudo；那里的更早录制尚未完成检索，不能据此断言转向标定包只有这两份。

### 其余录制索引

| bag 目录 | 时长 | 主要用途/限制 |
| --- | ---: | --- |
| `/home/aims/aimsracer-data/sessions/2026-09-27/calibration-234227/bag` | 233.87 s | 早期车辆响应/速度比例分析；有原始 Livox IMU，无原始雷达。 |
| `/home/aims/aimsracer-data/sessions/2026-09-28/calibration-005010/bag` | 324.37 s | 低速速度模式、0.08 s 转向响应拟合；有原始 Livox IMU，无原始雷达。 |
| `/home/aims/aimsracer-data/sessions/2026-10-03/venue-161459/bag-light` | 158.13 s | 轻量控制/定位时序记录，不能重算 LIO。 |
| `/home/aims/aimsracer-data/sessions/2026-10-03/field-minimum02/start-bag` | 131.07 s | 起步/最小有效速度；记录了 `/sensors/imu/raw`，没有 `/livox/imu`。 |
| `/home/aims/aimsracer-data/sessions/2026-10-03/field-anywhere03/lap-bag` | 160.26 s | 从参考任意位置开始的控制记录；记录了 `/sensors/imu/raw`。 |
| `/home/aims/aimsracer-data/sessions/2026-10-03/field-tf-diagnosis/lap-bag` | 389.61 s | TF/控制时序诊断；记录了 `/sensors/imu/raw`。 |
| `/home/aims/aimsracer-data/sessions/2026-10-03/field-weights1ms/trial-bag` | 109.62 s | 控制权重试验，有原始 Livox IMU，无原始雷达。 |
| `/home/aims/aimsracer-data/sessions/2026-10-04/field-150ms/trial-bag` | 210.41 s | 历史 150 ms 交接调度试验，有原始 Livox IMU。 |
| `/home/aims/aimsracer-data/sessions/2026-10-04/field-aux-gates-off/trial-bag` | 71.81 s | 历史门控调整试验，有原始 Livox IMU。 |
| `/home/aims/aimsracer-data/sessions/2026-10-04/field-held-tf/trial-bag` | 116.82 s | 本次约 1 s LIO 积压/提前计划过期事故；可回放迟到 LIO 输入复现 EKF异常，不能让 LIO重新计算积压。 |
| `/home/aims/aimsracer-data/sessions/2026-10-04/field-5hz-250ms-no-corridor/trial-bag` | 97.74 s | 历史 5 Hz / 250 ms 控制试验，有原始 Livox IMU。 |

## LIO 单独回放示例

本例只启动 LIO，并只播放原始雷达/IMU；bag 中原先的 TF、LIO、EKF和控制
输出不会混入本轮结果。A/B 从头播放，保留初始化和地图建立阶段；不要直接
跳到运动中段，再将冷启动地图的结果当成原运行复现。每轮重启 LIO，保持配置、
回放倍率和输出开关一致。完整累计延迟还受地图、输入排队和背景负载影响，
更换 bag 不保证复现事故，见 [LIO/EKF调查结论](../reports/2026-10-04-lio-delay.md)。

清单初检时，车辆的 `install/fastlio2` 仍是旧生产二进制。2026-10-04 后续已
同步主仓库固定的 `30bc305` 并重新编译主安装，包含新的串行 worker 实现；
Orin NX 上 3 项 CTest 和 8 项隔离 ROS 生命周期检查均已通过。
以下回放默认使用这个主安装。先前的
`/home/aims/aimsracer-data/sessions/2026-10-04/push-review/install/fastlio2`
保留作旧版本对照，不要在测试新 worker 时叠加它的环境。
新 checkout 仍需同步主仓库指定的 submodule 版本并重新编译；源码已 pull
不代表安装的二进制自动更新。

所有回放终端先执行以下环境设置；选一个未使用的 DDS 域，本例为 193：

```bash
cd /home/aims/AIMSRacer
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
```

终端一：生成本轮局部配置，开启核心处理计时，并启动 LIO：

```bash
export session_dir="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-lio-replay"
mkdir -p "$session_dir/logs"
python3 - <<'PY'
from pathlib import Path
import os
import yaml
p = Path('src/aims_racer_system/params/fastlio_rear.yaml')
cfg = yaml.safe_load(p.read_text())
cfg.update(print_time_cost=True, publish_body_cloud=False,
           publish_world_cloud=False, publish_path=False)
Path(os.environ['session_dir'], 'lio.yaml').write_text(yaml.safe_dump(cfg))
PY
ros2 run fastlio2 lio_node --ros-args \
  -r __ns:=/lio_test -r /tf:=/lio_test/tf \
  -p use_sim_time:=true \
  -p config_path:="$session_dir/lio.yaml"
```

终端二：先订阅输出，节点只在有订阅者时发布 odometry：

```bash
ros2 topic echo /lio_test/lio_odom
```

终端三：先用 A 做短运动测试，改目录即可换 B/D 等现有候选：

```bash
ros2 bag play /home/aims/aimsracer-data/sessions/2026-10-03/venue-161459/bag \
  --topics /livox/lidar /livox/imu \
  --clock 200 --rate 1.0 --delay 3.0
```

`print_time_cost` 只测 `MapBuilder::process()`；不含全部输入转换、等待、ROS
回调排队或输出序列化。若降低回放倍率，会给计算额外墙钟余量，不能拿慢速
回放无积压来证明 1 倍输入下无积压。比较源时间延迟要使用同一模拟时钟；
计算耗时仍用墙钟。当前 `ieskf_max_iter: 5` 尚未接入 setter，实际默认上限
为 10，本清单没有修改算法或参数接线。

本地逐 bag 核对数据保存在
`/home/aims/aimsracer-data/sessions/2026-10-04/lio-bag-catalog/inventory.json`。清单和用法
文档可进入源码仓库，bag、检查脚本和生成数据继续留在本地忽略目录。
