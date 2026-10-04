# 现有 LIO 测试 bag 清单

核对日期：2026-10-04。范围为当前 Orin 的 `/home/aims`，共找到 **19 个
SQLite rosbag2 目录**。其中 **8 个包含原始 Livox 雷达和 IMU**；排除一个
严重缺帧的早期录制后，有 **7 个可用候选**。录制文件仍放在原目录，没有搬移
或复制大文件。这些数据只在本机，GitHub 和新 checkout 不包含 bag。

检查依据是 `metadata.yaml`、数据库 topic 定义和只读消息检查。对八个原始输入
bag，核对了实际雷达/IMU消息数，读取了全部原始 IMU 和已录制的轮速，并将
字段解析与 ROS 反序列化交叉核对。除弃用录制外，原始 IMU 的源时间戳均未出现
倒退/重复或超过 30 ms 的相邻间隔。这不是逐点点云完整性或定位精度认证。

## 优先使用哪几个

建议按 **静止基线 → 短运动 → 剧烈转弯 → 长时间运动** 的顺序测试。

| 编号 | 推荐用途 | 时长 | 数据库大小 | 雷达 / 原始 IMU 消息数 | 特点 |
| --- | --- | ---: | ---: | ---: | --- |
| A | 短运动、修正前后对照 | 47.71 s | 325.8 MiB | 478 / 9539 | 轮速非零主要在约 12–32 s；原始 yaw rate 最大绝对值 0.88 rad/s。此前已用于独立核心计算对照。 |
| B | 剧烈转弯、匹配收敛压力 | 71.45 s | 306.2 MiB | 714 / 14285 | 轮速非零约 1.5–59 s，较大转动主要在前 50 s；原始 yaw rate 最大绝对值 3.94 rad/s。此前已用于迭代数/耗时对照。 |
| C | 长时间运行、地图维护与积压观察 | 788.77 s | 3366.8 MiB | 7876 / 157496 | 约 13.1 分钟；轮速非零主要在 123–392 s，之后仍有少量转动；yaw rate 最大绝对值 2.17 rad/s。尚未做同等内部阶段对照。 |
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
| A | `/home/aims/mpcc-logs/venue-20261003-161459/bag` |
| B | `/home/aims/mpcc-test-20261003-013910` |
| C | `/home/aims/mpcc-test-20261003-002036` |
| D | `/home/aims/AIMSRacer/log/lio_mp_benchmark_20261003/raw_input_complete` |
| E | `/home/aims/mpcc-logs/venue-20261003-161459/bag-test` |
| F | `/home/aims/mpcc-logs/venue-20261003-161459/bag-test2` |
| G | `/home/aims/mpcc-shadow-selfcheck-20261003-004259` |

A/B/C/E/F/G 同时有当时的 `/fastlio2/lio_odom`、后轴 LIO、轮速、EKF及控制
等输出。A/E/F 还包含 body/world cloud 和定位诊断，数据库较大；只测 LIO 时
筛选两个原始输入 topic 即可。当时录制的 LIO 输出只能作为旧算法的对照，
不等于外部定位真值。D 没有原始运行的 odometry，需要重新运行 LIO 得到输出。

## 一个不要用于正式对照的原始输入 bag

`/home/aims/AIMSRacer/log/lio_mp_benchmark_20261003/raw_input`

它虽有 `/livox/lidar` 和 `/livox/imu`，但 30.18 s 中只有 **156 帧雷达和
173 帧 IMU**，IMU 源时间间隔中位数约 **190 ms**，严重低于预期 200 Hz。
此前 benchmark 已弃用这份录制，实际使用上面的 **D：`raw_input_complete`**。
不要混淆两个目录，也不要将它的结果用于评价正常输入下的去畸变或处理性能。

## 缺少原始雷达的其余 11 个 bag

以下全部缺少 `/livox/lidar`，**无法重新运行 LIO 的扫描匹配**。其中的已计算
里程计、IMU和控制输出仍可用于 EKF、控制、模型或时序分析。

| bag 目录 | 时长 | 主要用途/限制 |
| --- | ---: | --- |
| `/home/aims/mpcc-calib-20260927-234227` | 233.87 s | 早期车辆响应/速度比例分析；有原始 Livox IMU，无原始雷达。 |
| `/home/aims/mpcc-calib-20260928-005010` | 324.37 s | 低速速度模式、0.08 s 转向响应拟合；有原始 Livox IMU，无原始雷达。 |
| `/home/aims/mpcc-logs/venue-20261003-161459/bag-light` | 158.13 s | 轻量控制/定位时序记录，不能重算 LIO。 |
| `/home/aims/mpcc-logs/field-minimum02/start-bag` | 131.07 s | 起步/最小有效速度；记录了 `/sensors/imu/raw`，没有 `/livox/imu`。 |
| `/home/aims/mpcc-logs/field-anywhere03/lap-bag` | 160.26 s | 从参考任意位置开始的控制记录；记录了 `/sensors/imu/raw`。 |
| `/home/aims/mpcc-logs/field-tf-diagnosis/lap-bag` | 389.61 s | TF/控制时序诊断；记录了 `/sensors/imu/raw`。 |
| `/home/aims/mpcc-logs/field-weights1ms/trial-bag` | 109.62 s | 控制权重试验，有原始 Livox IMU，无原始雷达。 |
| `/home/aims/mpcc-logs/field-150ms-20261004/trial-bag` | 210.41 s | 历史 150 ms 交接调度试验，有原始 Livox IMU。 |
| `/home/aims/mpcc-logs/field-aux-gates-off-20261004/trial-bag` | 71.81 s | 历史门控调整试验，有原始 Livox IMU。 |
| `/home/aims/mpcc-logs/field-held-tf-20261004/trial-bag` | 116.82 s | 本次约 1 s LIO 积压/提前计划过期事故；可回放迟到 LIO 输入复现 EKF异常，不能让 LIO重新计算积压。 |
| `/home/aims/mpcc-logs/field-5hz-250ms-no-corridor-20261004/trial-bag` | 97.74 s | 历史 5 Hz / 250 ms 控制试验，有原始 Livox IMU。 |

## LIO 单独回放示例

本例只启动 LIO，并只播放原始雷达/IMU；bag 中原先的 TF、LIO、EKF和控制
输出不会混入本轮结果。A/B/C 从头播放，保留初始化和地图建立阶段；不要直接
跳到运动中段，再将冷启动地图的结果当成原运行复现。每轮重启 LIO，保持配置、
回放倍率和输出开关一致。完整累计延迟还受地图、输入排队和背景负载影响，
更换 bag 不保证复现事故，见 [LIO/EKF调查结论](../reports/2026-10-04-lio-delay.md)。

截至本次清单核对，车辆的 `install/fastlio2` 仍指向旧生产二进制。已经通过构建
及隔离验证的修正版本在
`/home/aims/AIMSRacer/log/push-review-20261004/install/fastlio2`，示例使用这个
独立安装目录。它是本机临时产物；若不存在，应先在独立目录构建当前 fork。
源码已 push 并不代表车辆安装的二进制自动更新。

所有回放终端先执行以下环境设置；选一个未使用的 DDS 域，本例为 193：

```bash
cd /home/aims/AIMSRacer
source /opt/ros/humble/setup.bash
source install/setup.bash
source log/push-review-20261004/install/local_setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
```

终端一：生成本轮局部配置，开启核心处理计时，并启动 LIO：

```bash
mkdir -p log/lio-bag-tests
python3 - <<'PY'
from pathlib import Path
import yaml
p = Path('src/aims_racer_system/params/fastlio_rear.yaml')
cfg = yaml.safe_load(p.read_text())
cfg.update(print_time_cost=True, publish_body_cloud=False,
           publish_world_cloud=False, publish_path=False)
Path('log/lio-bag-tests/lio.yaml').write_text(yaml.safe_dump(cfg))
PY
ros2 run fastlio2 lio_node --ros-args \
  -r __ns:=/lio_test -r /tf:=/lio_test/tf \
  -p use_sim_time:=true \
  -p config_path:=/home/aims/AIMSRacer/log/lio-bag-tests/lio.yaml
```

终端二：先订阅输出，节点只在有订阅者时发布 odometry：

```bash
ros2 topic echo /lio_test/lio_odom
```

终端三：先用 A 做短运动测试，改目录即可换 B/C/D 等候选：

```bash
ros2 bag play /home/aims/mpcc-logs/venue-20261003-161459/bag \
  --topics /livox/lidar /livox/imu \
  --clock 200 --rate 1.0 --delay 3.0
```

`print_time_cost` 只测 `MapBuilder::process()`；不含全部输入转换、等待、ROS
回调排队或输出序列化。若降低回放倍率，会给计算额外墙钟余量，不能拿慢速
回放无积压来证明 1 倍输入下无积压。比较源时间延迟要使用同一模拟时钟；
计算耗时仍用墙钟。当前 `ieskf_max_iter: 5` 尚未接入 setter，实际默认上限
为 10，本清单没有修改算法或参数接线。

本地逐 bag 核对数据保存在
`/home/aims/AIMSRacer/log/lio-bag-catalog-20261004/inventory.json`。清单和用法
文档可进入源码仓库，bag、检查脚本和生成数据继续留在本地忽略目录。
