# Orin NX 静止定位实测

## 结论

NX 已完成原生构建与现场静止运行。约三分钟的采样中，去掉初始化后得到
172.57 s 稳态数据：EKF 输出 200.00 Hz，NDT 的 1,726 次扫描配准全部 `ok`，
1,726 次可信锚点均被独立监测核验，去畸变没有排队丢帧。

**局部航向仍存在严重漂移：小车静止时，EKF 在 172.57 s 内转了 184.66°，
平均 1.07°/s。下一步需要处理 IMU 零偏，之后再做移动验证。** NDT 本次持续
修正地图对齐，全局匹配位姿保持稳定；这些静止结果不能证明多圈跟踪可用。

## 条件与来源

- 硬件：Jetson Orin NX，16 GB RAM；Ubuntu 22.04.5、ROS 2 Humble、PCL 1.12。
- 功耗配置：设备原有 `MAXN_SUPER`，CPU 运行约 1,984 MHz，配准使用 CPU。
- 小车架起、静止，处于新场地。只启动 Livox、VESC 遥测、轮速与定位节点。
  电机/转向指令订阅被转入专用空话题，命令 watchdog 关闭。
- 原有 `/home/aims/AIMSRacer` 位于 `feat/aims-mpcc` 且有未提交修改。
  实验代码和构建隔离在 `/home/aims/ndt_nx_test_20261005_5ebf163/`。
- 基线源提交：`5ebf1632af567e70a5c357c29b0a3b7675ec421a`。
  对照后新增的唯一运行改动为：给 `localization_monitor.py` 设置
  `OPENBLAS_NUM_THREADS=1`。范围仅限该节点进程。
- 定位依赖仍使用 [固定版本和补丁](../../src/aims_racer_system/replay/dependencies.yaml)。
  远端 `git diff HEAD` 的 SHA256 与两份固定补丁逐一相等。
- 运行时确认三个定位包的 prefix 均来自独立 overlay；可执行文件为 ARM aarch64。
  首次原生构建三个包成功，耗时 18 min 17 s。

原始输入实测为 LiDAR 10.00 Hz、IMU 199.93 Hz、轮速 50.00 Hz，轮速为零。
新链路使用原始 Livox，内部 NDT IMU/deskew 关闭，外部 EKF 与去畸变工作线程开启。
现场 NDT 仍为 2 条 OMP 线程。

## 当前场景地图

使用车上已安装的 FAST-LIO 建图程序，采集 337 对同时间戳 body cloud / odom。
丢弃前 50 帧后，把各帧按 LIO 位姿转换到建图坐标系，累积并按 0.10 m 去重，
保存 11,072 点的二进制 PCD。随后停止 FAST-LIO，开始 NDT 的新扫描匹配。

地图 SHA256：`1e81e80471c9d93aec430aa70171c70d872d9870350fb5d3e91b11add6553054`。
初值由最后一次 LIO 位姿和现有约定的安装外参转换为 map/base_link。
该地图覆盖单一静止视角，只支持本次静止检查；地图与初值文件已保留。

## 现场时序与负载

第一段稳态采集约 60 s，使用默认 BLAS 线程；第二段原始采集 180 s，
按 initialpose 发布时间加 5 s 截取稳态，得到 172.57 s。
NDT 配准时间为原生 `alignment_time_sec`，不包含完整输入链路延迟。
可信观测接收年龄以扫描末端源时间到首次核验公告的接收时间计算。

| 指标 | 默认 BLAS | 监测节点 BLAS 单线程 |
| --- | ---: | ---: |
| EKF 接收频率 | 199.57 Hz | 200.00 Hz |
| EKF 接收间隔 P99 / max | 8.95 / 17.12 ms | 6.12 / 11.00 ms |
| EKF 状态接收年龄 P99 | 4.47 ms | 1.80 ms |
| NDT 配准次数 / `ok` | 599 / 599 | 1,726 / 1,726 |
| NDT 配准 P50 / P95 / max | 31.01 / 49.13 / 65.95 ms | 28.93 / 52.24 / 61.93 ms |
| 系统所有进程 CPU 占用 P50 | 464% | 288% |
| CPU 温度 max | 63.53°C | 62.38°C |
| 设备输入功率 P50 | 12.20 W | 10.73 W |

CPU 总占用来自 tegrastats，100% 代表一个核；包括采集脚本及其他进程。
两个时间段长度、扫描内容和温度不同，不能把配准小幅变化归因于线程限制。
监测进程的 `ps` 生命周期平均 CPU 占用从约 265% 降到约 68%；原进程加载
OpenBLAS pthread 库，4 条活跃计算线程各约 65%。限制后验证进程环境为 1。

单线程监测阶段的其他指标：

- 可信扫描接收年龄 P95 / max：71.66 / 82.46 ms。
- 健康消息中的观测年龄 P95 / max：156.29 / 157.91 ms；稳态全部 tracking。
- 去畸变阶段 P95 / max：4.24 / 4.79 ms；队列等待 P95：0.025 ms。
- 队列丢帧 0；启动时缺历史覆盖丢弃 1 帧，累计输出 1,780 / 接收 1,781 帧。
- 全局扫描匹配位姿相对样本中位数的 XY 距离 P95 / max：6.63 / 9.09 mm；
  yaw 偏差 P95 / max：0.138 / 0.291°。这些是静止散布，不是绝对精度。

## 独立匹配率的口径差异

原监测的全点云 inlier fraction 中位数约 83.22%。另取一对同源时间戳的
去畸变点云与 NDT 位姿，保留完整点数据和距离，得到：

- 总点数 19,968；其中 3,382 点距离雷达小于 0.5 m，占约 16.94%。
- 按原监测的全部点口径计算：82.92%。
- 按 NDT 的 0.5–30 m 距离范围单独计算：99.81%，参与点数 16,586。

建图与 NDT 过滤近距点，原独立监测包含这些点，因此两种比例不能直接对比。
本次保留原监测定义；99.81% 是一帧的解释性复算，不是整个测试的新验收分数。

## 原场地冻结输入的 NX 配准基准

为补充小地图测试，使用此前调查保存的同一组 source / target / seed：
source 6,384 点、target 983,110 点。参数为 resolution 1 m、step 0.1、
epsilon 0.01、最大迭代 35、DIRECT7。每种线程数独立启动一次，配准 31 次。
第一次计入 cold，后续 30 次作为 warm。测量时没有并行编译，现场定位尚未启动。

| NDT 线程 | Cold align | Warm P50 | Warm P95 | Warm max |
| --- | ---: | ---: | ---: | ---: |
| 2 | 571.11 ms | 65.42 ms | 66.37 ms | 66.47 ms |
| 4 | 541.02 ms | 50.44 ms | 50.72 ms | 50.74 ms |

两种配置均为 2 次迭代，fitness 约 0.002351753。cold align 包含首次 align
内部初始化；加载 PCD 和 setInputTarget 的开销在计时窗口外。
这个基准不包含 EKF、deskew、地图裁剪、fitness 查询和 ROS 调度，也没有覆盖
移动中困难扫描或迭代预算耗尽的情况。4 线程现场效果仍需单独验证。

## IMU 零偏与后续工作

静止原始 IMU 的两分钟均值 z 为 0.02031 rad/s；现场 EKF 稳态角速度
中位数为 0.01867 rad/s，局部航向累计漂移 184.66°。
Livox 的 [MID360 官方协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
将 gyro 单位定义为 rad/s，现有硬件驱动直接复制该字段。
当前 gyro 适配只处理坐标系与有效性，EKF 的状态也不包含 gyro bias。

应先补有明确静止条件的启动零偏估计/校正，再重复静止测试和短时 NDT
中断检查。移动与多圈测试需要地图覆盖和实际运动条件。C 长包仍按用户要求取消。

还发现 NX 系统时间比桌面慢约 8 小时。本次 ROS 节点和年龄计算全部在 NX
本地；跨设备使用源时间戳之前需要校时。设备系统时间本次保持原有设置。

## 证据与停止状态

原始证据在 NX 的上述实验目录 `evidence/`，桌面副本位于
`log/nx-localization/evidence/`；均为本地生成，fresh checkout 不包含：

- `build.log`、`monitor-thread-build.log`、`source-provenance.json`。
- `stationary-smoke-v2.jsonl` / summary；`stationary-blas1.jsonl` / steady / summary。
- `frozen-{2,4}threads.jsonl`；`derived-results.json`；`tegrastats.log`。
- `stationary-map/{map.pcd,initial_pose.json,map_capture.json}`。
- `quality-snapshot.json`、`quality-snapshot.npz`。

第一份采集脚本因 Humble DiagnosticStatus.level 为 bytes 而中止，错误片段
和 traceback 保留。修正采集脚本后重新完整采样；定位节点未因采集错误退出。
用于正式结果的是后续完整文件。

测试结束后，核对所有任务进程 manifest，均已退出；未启动任何控制器，
未发送电机或转向指令。代码、构建、地图与日志保留供后续复测。
