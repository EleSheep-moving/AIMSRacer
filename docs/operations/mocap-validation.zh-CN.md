# 动捕校对工具

> 2026-10-18 整理范围：本页记录已部署工作区的用法；这次文档提交不包含对应的新运行时代码，命令须在已部署环境执行。当前后轴／质心五量以[统一状态指南](state-observer.zh-CN.md)为准。


同一目录用于主机与 NX。只订阅、录包和分析，不启动估计器、FastLIO、MPCC、驱动或动作程序；唯一业务发布话题是采集健康 `/mocap_validation/capture_status`。现场动作沿用遥控、现有标定程序或 MPCC。

## 代码与构建

- `aims_mocap_validation/src/capture_node.cpp`：持续运行的 C++ 时间采集器。
- `aims_mocap_validation/aims_mocap_validation/`：Python 离线参考生成、三项分析、绘图及命令编排。
- `vrpn/`、`vrpn_mocap/`：官方固定版本源码与原许可证；版本/提交见 `sources.lock.json`。不修改其协议、时间戳或坐标逻辑。
- `verification/vicon_state/run.py` 保留旧命令与状态报告格式，调用同一份规范实现；导入旧 `capture.record()` 也转入 C++ 采集流程。

在已含 `vesc_msgs`、`ackermann_msgs`、ROS Humble 和 NumPy/SciPy/Matplotlib/PyYAML 的工作空间构建：

```bash
source /opt/ros/humble/setup.bash
cd ~/AIMSRacer
src/mocap_validation/build.sh
source install/setup.bash
colcon test --packages-select aims_mocap_validation --event-handlers console_direct+
colcon test-result --verbose
```

NX 的 CMake 4 需要构建脚本中的最低策略版本参数，以及编译器实际确认 Threads_FOUND 后设置原版 FindVRPN 所需的旧变量；不为此修改上游源码。系统需 `netbase` 提供 `/etc/protocols`。在主机 Docker 使用相同命令；无需安装到主机非 ROS 的 Python 环境。

## 现场配置

将 `aims_mocap_validation/aims_mocap_validation/config.template.json` 复制到数据目录。`null` 是现场未知值，不得随意改成零以获得可计算结果。至少设置：

1. `vrpn.server/port/tracker`。刚体可被自动发现，填写 tracker 可推导标准话题；用 `inspect` 检查实际名称。也可直接填写 `vicon.topic/message_type`，支持 PoseStamped、TransformStamped、Odometry。
2. `use_vrpn_timestamps` 必须为 `true`。使用已有独立桥时设 `start_bridge=false`，并独立确认已有桥参数。
3. `clock`：`t_ros = ros_anchor_ns + scale*(t_vrpn-source_anchor_ns)`，以及对时不确定度和 `verified`。即使同一 epoch 也需核对；VRPN 时间不自动等于相机曝光时间。不要用本次运动的互相关来伪造时钟标定。
4. `body_orientation_in_marker_xyzw`：把车体系向量旋转到标记系的四元数。`references.*.origin_in_marker_m`：后轴/质心原点在标记系的位置，单位米。测量后设置各 `measured` 标志。
5. FastLIO 的 `rear_to_imu_body_m` 和安装旋转需核对。模板名义值沿用现有车辆记录，`geometry_measured=false`。现有质心高度只是约 3 cm 的估计，不能视为外参测量。
6. FastLIO `world_alignment` 给固定 LIO 世界到 Vicon 世界变换；或指定从首个已映射 Vicon 时间起的 `static_window_s: [0,2]`。程序要求完整静止窗口覆盖、无大缺口、姿态和位置稳定，只估计一次并冻结。不提供两者时该分析明确不可计算。
7. `evaluation_windows` 可指定 `name/start_s/end_s`（如静止、直线、转弯、S 弯）；若留空，按运动特征自动分段，报告保留其分类依据。
8. 执行器阈值用于识别有效激励和响应，不是合格线。可在 `actuator.event_windows` 指定 `channel/start_s/end_s`（speed/current/brake/duty/steering），或使用绝对源时间 `source_ns/end_ns`。

未完成对时或外参测量可以先录制。报告会列出尚不能判定的指标；“已算出”不等于独立参考已合格。

## 操作

在 NX 使用 `run.sh`；主机已准备 `aims-mocap-main-20261018` 容器，可把下列 `run.sh` 换成 `host.sh`，从主机终端调用。主机数据目录同时挂载到原绝对路径，连接现场服务器需容器 host 网络。目录均为新目录，防止覆盖已有证据：

```bash
cd ~/AIMSRacer
src/mocap_validation/run.sh inspect --config /绝对路径/config.json
src/mocap_validation/run.sh record --config /绝对路径/config.json \
  --output ~/aimsracer-data/sessions/2026-10-19/mocap/run01
# Ctrl-C 结束并落盘；也支持 --seconds 60
src/mocap_validation/run.sh extract --config /绝对路径/config.json \
  --bag ~/aimsracer-data/sessions/2026-10-19/mocap/run01/bag \
  --output ~/aimsracer-data/sessions/2026-10-19/mocap/run01-extracted
src/mocap_validation/run.sh analyze --config /绝对路径/config.json \
  --input ~/aimsracer-data/sessions/2026-10-19/mocap/run01-extracted \
  --task all --output ~/aimsracer-data/sessions/2026-10-19/mocap/run01-report
```

`analyze --task state|fastlio|actuator` 可单独分析。`extract` 自动读取 bag 同级 `timing.csv`、健康文件和采集清单；另行保存时用 `--timing` 指定。支持 SQLite 分片包及外部 Vicon CSV（列见旧 Vicon 文档）。

```bash
src/mocap_validation/run.sh demo --output /绝对路径/全新合成示例目录
```

合成示例只验证软件，所有报告显式标注合成。`inspect` 的缺话题/类型不匹配返回 2；分析任务配置不足返回 2 并生成原因报告。录制允许尚未出现的话题，已有类型冲突则拒绝，以免误解消息。

## 参考和计算

Vicon 测得的是标记原点世界位置 `p_WM` 与姿态 `R_WM`。对参考点 `j`：

- `p_Wj = p_WM + R_WM * p_Mj`；车体姿态 `R_WB = R_WM * R_MB`。
- `v_Bj = R_WBᵀ * d(p_Wj)/dt`；`a_Bj = R_WBᵀ * d²(p_Wj)/dt²`。AX/AY 是惯性加速度在车体系投影，含转弯向心项，不是车体系速度分量的直接导数。
- 三维旋转局部多项式给车体角速度，yaw rate 取其 z 分量；不把欧拉 yaw 导数冒充三维车体 wz。
- 在每个有效连续区间使用 0.08/0.12/0.18 s 中心求导窗，主窗 0.12 s。遮挡、重复/倒退时间、边缘样本和大缺口保留掩码，禁止跨缺口插值。

状态主指标：未平移、未去偏的 bias/RMSE/MAE/P95绝对误差/最大绝对误差、覆盖率、缺口和分段结果。另给各求导窗口敏感性、源时间年龄及波形响应偏移。低激励和周期多解不强行给唯一偏移。Robot Localization 是对照流，不作为真值；独立估计器与 LIO 的一致性也不等于独立精度。

FastLIO raw 保留 IMU 原点位姿、世界速度；与后轴流分开分析。按安装旋转和杆臂转换，速度需要同步陀螺做 `v_rear = v_imu - ω×r_rear_to_imu`；缺少有效角速度时不伪造 raw 后轴速度。固定世界对齐后报告位置/姿态/速度、累计漂移、源时间年龄和波形偏移，不使用全轨迹拟合压低漂移。

执行器以最终 `/vesc/command` 为事件起点。分别检测命令到 ERPM、Vicon VX/AX、yaw rate/AY；同时动作、没有平台、激励不足、遮挡和断流会使事件不可辨识。实际使用原始反馈与向后有限差分的完整时间支持，给首次响应区间、持续响应确认时间，以及 0.5/1/2 倍阈值敏感性。精度参考的中心求导不会用于物理起点。无实际舵角反馈，转向测到的是整车响应，不能称为纯舵机延迟。

## 时间与证据

C++ 记录每条消息源时间、可用的 DDS 发布时间/接收时间、回调 ROS/系统/单调时间、原始 CDR SHA256。中间件未提供的字段留空，不用回调时间补造。ROS 与 DDS 系统时钟不一致时不混算；Vicon 时钟未映射时不计算其源时间年龄。DDS 发布时间→采集回调含传输和调度，不是生产节点内部计算耗时。

有界异步写入队列记录溢出计数，健康文件还记录源时间重复/倒退/缺口、接收断流和时钟跳变。写入或最终 flush/健康文件失败返回非零；队列丢失在清单标记 degraded。Ctrl-C 依次停止 rosbag、采集器和本工具启动的桥，并等待落盘。

每次录制保存选定配置、发现图、实际 ROS 参数、可读取本机进程的 ELF 哈希/启动参数、固定上游版本、采集器哈希、结束状态。远端进程不能由本机 `/proc` 证明。提取保存原包/CSV哈希；分析先核对输入哈希并保留配置与方法。输出中文总入口 `report.zh-CN.md`、三项子报告、PNG、指标 JSON 和 CSV 明细。数值结果不预设实车合格线。
