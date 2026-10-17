# Vicon 后轴／质心五量对比工具

为 2026-10-19 计划测试准备。在线估计器仍为 C++；这里的 Python/SciPy 只负责观察、录包和离线分析，不改变 MPCC 或 Robot Localization，不向车辆发送控制命令。

## 比什么

| 量 | 主参考 | 其他对照 |
| --- | --- | --- |
| VX、VY | Vicon 目标点轨迹一阶导，投影到车体 | 后轴 Robot Localization、LIO、轮速 |
| AX、AY | Vicon 目标点轨迹二阶导，投影到车体 | 可配置额外 AccelWithCovarianceStamped 流；不把 `vx*r` 当 AY 真值 |
| yaw rate | Vicon 三维姿态增量的车体 omega_z | Robot Localization、LIO／wheel 所报 r、corrected gyro |

每个比较必须处于同一参考点。质心流直接与质心参考比较；不将后轴 LIO／导航速度直接与质心速度作差。wheel 的非 VX 分量可能是模型占位或转向推算，LIO／导航共享传感器；它们只作为基线，主参考独立由 Vicon 位姿构造。

程序对主要误差保留原源时间和原始偏置，不做曲线平移。输出 bias、RMSE、MAE、P95／最大绝对误差、源周期／缺口、参考网格覆盖率、接收减源时间年龄、误差趋势以及诊断消息计数。支持按静止／直线／圆弧／S 弯及保留验证组分别统计。

除了各流原发布时刻的指标，报告另给同一参考网格上的比较：同量所有可用估计器共用相同时刻和缺口支持。默认自身／导航／轮速最大支持间隔 40 ms，低频 LIO 单独为 250 ms，可按真实采样设置各 stream 的 `max_gap_s`；插值不等于新增独立观测。某个基线缺失时会明确少一行，不用零值补齐。

响应偏移是单独的诊断：在共同动态支持上最大化去均值后的归一化相关。去均值只用于找形状偏移，原始误差及偏移处的 RMSE 不去偏。固定偏置可能使原始 RMSE 最小值偏离真实波形偏移，JSON 同时保留这个差异。静止／激励不足、相关过低、搜索边界、周期多解不接受为可辨识偏移；5 ms 扫描步长不表示 5 ms 测量精度。时钟不确定度、Vicon 采样／桥接过程和参考求导带宽必须同时考虑。

这里的相关偏移与此前“原始 RMSE 最小值对齐”的定义不同，JSON 同时保存 `raw_rmse_minimum_offset_ms`；不能把两种定义的数值直接混比。

## 环境与自检

Python 3、NumPy、SciPy、Matplotlib；inspect／record／extract 另需 ROS 2 Humble、rclpy、标准消息、ros2 bag CLI 和 sqlite3 存储插件。analyze 和 demo 不依赖 ROS。仓库验证环境为 Python 3.10、NumPy 1.26.4、SciPy 1.15.3、Matplotlib 3.8.4。

在仓库根目录运行完整合成自检示例：

```bash
python3 verification/vicon_state/run.py demo \
  --output /absolute/new-vicon-demo
```

它生成 `data/`、已填写的合成 `config.json` 和 `report/report.zh-CN.md`、后轴／质心五量 PNG。例子包含时钟偏移／漂移、三维姿态、标记点杆臂、遮挡、发布缺口、固定偏置和已知滞后。所有结果明确标记 synthetic，不能当作实车效果。现成示例见数据目录 `experiments/vicon-state-preparation-20261017/` 中的验证证据。

所有输出目录必须是新目录，防止覆盖原试验或把两次结果混在一起。

## 现场配置

复制 [config.template.json](config.template.json) 到本次 session 外部目录，编辑副本；不修改模板中的 null 为猜测的零外参。接入信息未知时仍可先准备脚本，**录包只要求接入配置完整，分析还要求时钟和外参已填写**。

```bash
mkdir -p /absolute/session-20261019
cp verification/vicon_state/config.template.json /absolute/session-20261019/config.capture.json
```

现场填写以下项：

- `vicon.topic` 和 `message_type`：支持 `geometry_msgs/msg/TransformStamped`、`PoseStamped`、`PoseWithCovarianceStamped`、`nav_msgs/msg/Odometry`。不猜未知桥的专用消息字段，专用消息原样放 `extra_topics` 录制，或另导出规范 CSV。
- `position_unit`：明确 `m` 或 `mm`。标准 ROS 位姿通常为 m；SDK 原始导出需核对单位。所有杆臂配置始终为 m。
- `tracking_semantics`：已确认桥只发布有效跟踪帧时填 `bridge_publishes_valid_only`；外部 CSV 使用真实 valid 字段时填 `csv_valid_flag`。标准位姿没有遮挡字段；有限且四元数有效只能证明数值可用。若桥在遮挡时保持上一帧，必须提供外部有效性 CSV，不可填“有效帧专用”。
- `bridge_timestamp_semantics`：写明 header 是相机帧／服务端还是 ROS 接收时间；另记录 `camera_rate_hz` 和 `prediction_or_retiming_enabled`。帧号、timecode、遮挡、SDK latency 等原始话题放入 `extra_topics`；原始 Vicon 会话也保留。
- `body_orientation_in_marker_xyzw=q_MB`：把车体向量旋转到标记刚体系的旋转；不是 Vicon 世界姿态。车体轴为 x 前、y 左、z 上。确认实测后将 `body_orientation_measured` 置 true。
- 两点的 `origin_in_marker_m`：`position(target)−position(marker)`，以标记刚体系表达；测量后各自设置 `measured=true`。默认 frame 为后轴 `base_link`、质心 `center_of_mass`。

当前在线名义后轴→质心为 `[0.21,-0.03,0] m`，由近似后轴→IMU 与用户提供的 IMU→质心推得。若临时使用它，配置中应保持 `measured=false`；其在 marker 系的换算为：

```text
origin_CG_in_marker = origin_rear_in_marker + R_MB * [0.21,-0.03,0]
```

质心高度仍来自用户估计 IMU 下方约 3 cm。名义质心点的对照不能伪装为已实测实际质心位置的精度验收。

时钟映射以两个整数纳秒锚点表达，避免对约 10^18 的 epoch 直接做浮点运算：

```text
t_ROS_ns = ros_anchor_ns + scale * (t_Vicon_ns - source_anchor_ns)
```

填写同一物理时刻的锚点、测得的 `scale`、`uncertainty_ns`；经过独立同步核对后才设置 `verified=true`。若 header 确实使用同一时钟，可以使用相等锚点、scale=1，但必须先核对。若桥只填 ROS 接收时刻，它不自动等于相机采样时刻。不要通过移动估计曲线来反推并冒充独立时钟标定。未知映射不默认为 identity；未实测的参数在报告中保留提示。

`evaluation_windows` 为空时统计完整共同窗口。需要分动作统计时，例如：

```json
[
  {"name":"static_start","start_s":1,"end_s":55,"role":"evaluation"},
  {"name":"straight","start_s":60,"end_s":90,"role":"evaluation"},
  {"name":"held_out_S","start_s":120,"end_s":150,"role":"held_out"}
]
```

时间均相对**第一帧映射后的 Vicon 源时刻**，与图横轴一致；按实际时段填写。分析先使用完整历史求导，再裁指标窗口，不裁掉动作边界所需的参考支持。

## inspect、record、停止

先 source 试验机上已准备的 ROS／工作空间。沿用原车辆启动方式增加 `state_observer:=true`；观测器应是已编译的新版本。MPCC 继续现有输入。若采用这里的专用录包，原启动命令用 `record:=false`，避免重复录包。无需为本工具另启一套车辆图。

```bash
source /opt/ros/humble/setup.bash
source /absolute/validated-workspace/install/setup.bash
python3 verification/vicon_state/run.py inspect \
  --config /absolute/session-20261019/config.capture.json
python3 verification/vicon_state/run.py record \
  --config /absolute/session-20261019/config.capture.json \
  --output /absolute/session-20261019/run01
```

inspect 检查配置、实际话题类型和短时收包，报告源时间／frame；话题缺失、类型冲突或未知接入返回状态 2。标记几何／时钟未填写列为分析准备项，不阻止记录。record 对已存在的类型冲突报错；暂未出现的发布者保留订阅等待，便于记录启动阶段。开始动作前检查 preflight 和 recorder.log 是否持续收到了所需数据。

**在 record 终端 Ctrl-C 正常停止并落盘。** 可用 `--seconds 60` 自动结束录包，参数只控制录制时间，不触发车辆动作。

录包输出：

- `bag/`：sqlite3 ROS2 bag，含两点五量及状态、Vicon、raw/corrected IMU、raw/rear LIO、导航、轮速、雷达、TF、控制和定位上下文，以及配置额外话题。
- `config.selected.json`、`preflight.json`、`capture.json`、`recorder.log`。
- 可发现的 observer／导航参数 dump；本机能读取时，保存实际 `state_observer` ELF 哈希、命令和实际参数文件。若录包机不是 NX，NX 进程证据须另在 NX 留档，不把本地候选文件当远端运行证明。

配置检查和录包不会启动车辆、估计器或控制器。具体 Vicon ROS 桥由现场现有系统提供。

## 提取与分析

保留采集配置；另外复制为 `config.analysis.json` 填写最终时钟／外参／动作时段，分析报告同时保存采集／提取时的配置和所用分析配置哈希。

```bash
python3 verification/vicon_state/run.py extract \
  --config /absolute/session-20261019/config.capture.json \
  --bag /absolute/session-20261019/run01/bag \
  --output /absolute/session-20261019/run01/extracted
python3 verification/vicon_state/run.py analyze \
  --config /absolute/session-20261019/config.analysis.json \
  --input /absolute/session-20261019/run01/extracted \
  --output /absolute/session-20261019/run01/analysis
```

提取支持多个 `.db3` 分片，按接收时间、文件序和行号稳定合并，只读原包；验证实际 topic 类型，保存原包和提取 CSV 哈希。当前不支持 MCAP，专用 record 已固定 sqlite3。分析会校验提取文件哈希，拒绝后来被改动的数据。

若 Vicon 位于另一记录系统，可增加 `extract --vicon-csv /absolute/vicon.csv`，无需在车辆 bag 中存在 Vicon 话题。规范 CSV：

```text
source_ns,receive_ns,px,py,pz,qx,qy,qz,qw,valid
```

时间是整数 ns，四元数 xyzw，`valid` 为 0／1 或 false／true。source 为真实来源时间，receive 必须为 ROS 侧记录／可映射的接收时钟，不把 Vicon 专有接收计数当 ROS epoch。保留帧号、timecode、原始状态等额外列，程序不会从有限位姿推断遮挡。

## 参考形成及限制

先将 marker 位姿换成目标点轨迹，再做中心 Savitzky–Golay 一／二阶导，并用同期车体姿态旋转到车体系。角速度通过局部相对中心姿态的 SO(3) 对数多项式导数得到，取 omega_z；有 roll/pitch 时不以欧拉 yaw 差分替代。重力不加到位置二阶导上，静止参考加速度为零。

默认 0.08／0.12／0.18 s 三个参考窗口，各指标共用三个窗口都有支持的样本；主窗口 0.12 s。参考使用未来数据，首尾半窗口无效；遮挡和源间隔超过 max_gap_s 分段。JSON 保存实际窗口样本数、时间跨度和采样周期。不能用插值后的网格频率冒充动捕独立帧率。源时钟回退／重复需按实际时钟 epoch 拆分记录，不能静默排序或跨 epoch 插值。

`analysis/report.zh-CN.md` 为入口；`metrics.json` 含完整误差、窗口敏感性、时间年龄／缺口、偏移诊断、状态计数及哈希；`aligned.csv` 保留主窗口逐点误差；`reference.npz` 保留全部窗口的两点参考。PNG 不跨缺口连线，仅显示抽稀，统计仍用全部匹配点。

原值年龄、波形偏移、动捕／网络时间和估计器计算耗时各有边界。这里的 bag receive−source 不是完整物理延迟，也不是节点回调耗时。真实接线／Vicon／几何／时钟信息未补齐前，不判定实车精度或低延迟验收通过。

测试：`source /opt/ros/humble/setup.bash` 后运行 `python3 -m pytest verification/vicon_state -q`。测试含解析三维运动、固定偏置／延迟、遮挡／缺口、四种真实 ROS CDR、分片 sqlite3、CLI 完整报告及实际 DDS→rosbag→提取链路；这些是程序验证，实车五量验收留待 Vicon 试验。
