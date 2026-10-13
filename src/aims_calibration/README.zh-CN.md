# 纵向标定

[English](README.md)

`aims_calibration` 通过现有整车指令选择器进行纵向标定。节点以 50 Hz 发布 native `vesc_msgs/VescCommand` 到 `/calib/vesc_command`，时间戳为当前 ROS 时间，坐标系为 `base_link`。选择器独占 `/vesc/command`；标定节点不发布 `/drive`。控制与状态定时器采用稳定时钟，因此 `/clock` 停止时仍能检查数据是否过期。

节点可执行文件为 `longitudinal_calibration`，节点 launch 为 `longitudinal_calibration.launch.py`，完整整车入口为 `aims_racer_system/calibration.launch.py`。节点 launch 参数包括 `config`、`reference_directory`、`vehicle_config`、`pp_config`、`log_directory`、`stage`、`steering_source`、`base_current_file`、`auto_start`。完整入口同时配置整车、定位、录包与加速度估计。默认不启用，必须调用 `/calib/enable`（`std_srvs/SetBool`）；`auto_start: true` 会被拒绝。`/calib/status` 提供状态、原因、实际与请求电流及输出目录。完整启动命令见[操作指南](../../docs/operations/pp-calibration.zh-CN.md)。

## 正式标定命令

六个操作入口都由仓库内 `aims_calibration/cli.py` 实现；共享检查位于
`operator.py`，采样与控制仍由 `node.py` / `engine.py` 负责。
不依赖 `field-tests` 下的临时 Python 脚本、固定 PID 或聊天授权文本。

车辆链已经启动时，只执行：

```bash
ros2 run aims_calibration inspect
# RViz 确认后轴位置与前进方向，遥控解锁并选择 CH8 标定；检查通过后执行一次：
ros2 run aims_calibration start
# 人工中止时先遥控锁车，再请求停止；此命令不关闭车辆链或录包：
ros2 run aims_calibration stop
# 把 inspect/start 显示的 output_directory 填到这里，离线验收：
ros2 run aims_calibration results /absolute/path/to/result-directory
```

`start` 与直接 `/calib/enable` 共用节点中的检查：输入与权限新鲜、车辆
静止（轮速绝对值 ≤0.1 m/s）、最终请求为零速度、驱动起步电流替换已关闭、
指令发布者唯一、日志可用；PP 还要求匹配的可信定位、横向距离 ≤0.25 m、
航向偏差 ≤0.35 rad。开启加速度观测时，也检查加速度可用。
启动时重新验证车辆、PP、标定配置、参考和直线区间文件哈希。
未通过检查可以调整后重试；成功开始后拒绝再次启用，避免重置采集。
完成、故障或中止后，下一次采集需要新的 launch/会话。

`start` 只提交一次启用请求；响应不确定、监控失联或 Ctrl+C 时请求禁用。
显示结束状态后，还需连续 3 秒新鲜轮速确认停稳。零电流只是滑行请求；
无法确认停稳时明确提示人工停车，不自动关闭车辆链。
`results` 检查完整目标覆盖、配置哈希及每条记录的 `success` /
`evidence_valid`；空记录、重复档位和部分成功均返回非零退出码。

在 NX 本地图形终端可给 launch 追加 `rviz:=true`，它加载仓库内 `calibration.rviz`，显示先验地图与绿色参考线。默认使用软件渲染以适配此次 NX 黑屏问题；可用 `rviz_software_rendering:=false` 切回 GL 渲染。

### 仓库内九档配置

以下是显式可选配置，普通启动默认保持不变。地图、参考目录由场地数据指定，
脚本、配置、launch 与文档全部随仓库安装。九档配置同时选择仓库内的
`speed_hold_1to5.yaml`、`speed_hold_vehicle.yaml`、`speed_hold_pp.yaml`，
不再需要临时 `calib-vehicle-v5.yaml` 或 `pp-v1p8-baseline.yaml`。

**仅在当前没有车辆链运行时**启动一次；已有 launch 时使用上面的命令即可：

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=speed_hold_1to5 steering_source:=pp \
  map_file:=/absolute/path/to/map.pcd \
  reference_directory:=/absolute/path/to/pp-routev2-straight-trials \
  record:=true longitudinal_acceleration:=true auto_start:=false
```

九档 1.0、1.5、…、5.0 m/s，每档重新稳速 1 秒、累计 5 秒有效直线数据；
弯道以 1 m/s 恢复行驶，弯道与不稳定期间不计入采样时间。
`/calib/reference` 是 latched `nav_msgs/Path`，RViz 添加 Path 显示并设置
Fixed Frame 为 `map`；位置修正沿用 NDT `/initialpose`。
程序会显示结果目录和录包会话路径，不把 PID 写死。
实车停稳后用运行 launch 的终端 Ctrl+C 关闭，并检查该会话的 `bag`。

## 阶段与自动扫描

| 阶段 | 仓库 profile | 测量 |
| --- | --- | --- |
| A `speed_hold` | `speed_hold_1to5` | 九档巡航实际电流，每档累计五秒有效直线样本 |
| B `accel_interval` | `accel_sweep_1to5` | 相邻 +0.5 m/s 区间，电流 `I0(起速)+5k`，最高 100 A |
| C `decel_current` | `decel_sweep_1to5` | 低于基准的正电流减速到起速−0.5，再滑行、负电流制动到 1 m/s |

详见 [B/C 操作指南](../../docs/operations/current-sweep.zh-CN.md)。离线
`baseline`、`plan` 与另外四个操作入口共享 `cli.py`；基准必须明确指定完整
有效的九档 A 结果，不自动挑选会话。

每项恢复自己的起速，在 ±0.2 m/s 内连续稳速一秒后切换 native 电流模式。
离开直线同项最多尝试三次，PP 弯道保持 1 m/s 恢复。依据实际电机反馈判断电流
能力，只跳过对应分组；速度超时本身不触发跳档，新鲜端点成功优先。
短响应可记录为电流跟踪尚未验证。电流上限 ±100 A，速度至多
`min(config.max_speed, vehicle.max_speed, 5)` m/s。B/C 冻结结构化计划并保存 v2；A 保留 v1。

手动模式仍接受显式正电流 B、负电流 C 和单个起止区间，默认空列表拒绝电流阶段。
节点 launch 显式传匹配的 `stage`、`steering_source`，整车 profile 默认用
`stage:=auto` 自动选择。标定驱动必须关闭起步电流替换；最终请求不是实测电流。

## 九档巡航电流方案

使用 `config/speed_hold_1to5.yaml`：速度列表为 `[1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]`，每档累计 5 秒，`repeats: 1`，每次恢复采样前连续稳速 1 秒，弯道 `recovery_speed: 1.0`。例如 5 m/s 单条直线不足 5 秒时，可以在后续直线继续累计，不能把这称为连续 5 秒。

该配置明确选择 `max_speed: 5.0`；外场补丁不再以实测速度过冲停止，有效样本仍要求稳速容差。使用 `profile:=speed_hold_1to5` 会选择仓库内车辆与 PP 配置，或者同时传入专用 `vehicle_config`，其 `max_speed` 为 5.0；不修改普通 race 的车辆配置。若仍选择 4 m/s 车辆配置，启动会拒绝九档列表，不静默跳过 4.5 或 5 m/s。诊断包含当前目标速度、累计秒数、段数、计划档数和采样模式。权限、定位或日志故障仍作废当前整档；新 launch/会话从第一档重新开始。

## 转向与有效采样区间

`steering_source: pp` 复用 `aims_pp` 的增强 PP 核心、`vehicle_config` 几何和数值参考加载器，沿用独立 PP 的定位健康与地图身份检查。默认前视增益 1、最小 0.3 m、最大 4.5 m，航向增益 0.1、曲率前馈增益 0.1；可用 `pp_config` 调整。车辆几何、前进朝向、位姿连续性、源时间、定位 epoch 和地图 SHA 必须一致。仅当车辆配置启用 `enforce_corridor` 时检查车体与走廊。直线区间之外使用 `recovery_speed` 跟踪，准备阶段使用速度模式。

PP 需要 `reference_directory/reference.json` 与编辑器导出的 `calibration_regions.json`。区间格式为 `{schema_version: 1, reference_sha256: "参考文件原始字节SHA", regions: [{id: "straight1", name: "straight 1", kind: "straight", start_s: 2.0, end_s: 10.0}]}`。进度采用参考的节点/弦长坐标，不是精确曲线弧长。区间为左闭右开；`start_s > end_s` 表示跨越闭环接缝，相同端点与重叠区间被拒绝。手动直线标注不证明距离、曲率、抓地或电流安全。累计稳速方式在退出或切换区间时关闭当前采样段，之后重新稳速再开新段；每段单独保存源时间与电流统计，过弯、准备时间不计入有效时长。连续稳速方式和正负电流响应试验仍在退出或切换区间时使未完成试验无效。

`steering_source: rc` 不依赖定位或参考；速度来自轮速里程计。默认 profile 为 `steering_ch1_throttle_ch3_aux_ch5_to_ch10`：转向 CH1、油门 CH3、锁定 CH5、ESC CH6、控制源 CH7、标定 CH8、限幅 CH10。通道从 1 开始计数；中心 992，端点 172/1810，转向反向，选择器转向比例 0.4751 rad。采样要求新鲜有效遥控、CH5 ≥992 解锁、CH8 >992 选择标定，转向处于中心 ±50 计数内。这个死区只控制采样资格，不改变实际转向指令。转弯暂停累计稳速采样；连续稳速/电流响应试验则作废未完成试验，并返回速度模式，恢复时仍可遥控转向。`sequential_ch1_ch2` 只适用于选择器明确采用该 profile 的情况（转向 CH2、锁定 CH3、标定 CH7）。

## 权限、时序与退出

刚启动且未启用时，持续新鲜的零速度请求供选择器建立标定选择。启用后等待真实的 `/control/calibration_enabled == true`，并检查遥控、新轮速、新电流数据和与近期请求对应的最终 `/vesc/command`。选择器会重写时间戳，因此按短历史中的模式与内容匹配，不要求恰好对应最新请求。只填充当前模式的目标字段；正负电流都使用 `MODE_CURRENT`，不发布 `MODE_BRAKE_CURRENT` 或 `MODE_ACCELERATION`。

权限撤销、遥控锁定或模式变化、源/接收年龄超过 0.2 秒、无效遥测、外来最终指令、地图/定位失效、ROS 或源时间回退、ROS 时间冻结、定时器堵塞、额外标定发布者都会锁存 `FAULT`，使当前试验无效并持续请求零电流，直到新会话显式启用。人工禁用也使未完成试验无效并请求零电流。新会话启用重置时钟基线、缓存和请求历史；遥控、权限、轮速、电流观测必须在本次启用之后到达，并确认本次新零速度请求已被选择，才进入准备。不恢复中断试验。正常完成请求零速度；零电流是滑行请求，不保证物理停车，外部选择器和操作者仍掌握控制权。

## 输出与指标定义

每次运行在 `log_directory` 下创建带 UTC 时间和 UUID 的目录，包含 `observations.csv`、`runtime.csv`、原子替换的 `trials.json`，以及出现试验后生成的 `trials.csv`。保留有效配置、车辆文件 SHA、PP 参考与区间 SHA、遥控 profile、有效 PP 参数、调参文件 SHA、通过 launch 使用的标定 YAML SHA 与有效配置 SHA。

有界后台队列负责写盘，溢出或写失败使采集无效并请求零电流。刚完成的记录无法入队时，撤销成功状态、设置 `evidence_valid: false`、清空 `mean_acceleration`，用 `response_completed` 保留是否实际越过边界；队列腾出空间后重试结果检查点。**JSON 是结果验收依据**：先完成 CSV 检查点，再提交成功 JSON；后台写失败立即尝试通过独立 JSON 路径发布无效结果，即使没有后续排队任务。CSV 可能保留前一个检查点，应结合 JSON 与诊断判断。持续磁盘故障可能无法保存新结果，但仍在诊断中可见。

`runtime.csv` 保存 ROS 时间、回调耗时（截至采集入队）、定时器间隔、队列计数和状态，用于联合负载分析。原始观测保存轮速/遥测源时间、轮速、`current_motor`、`voltage_input`、ERPM（`state.speed`）、请求与最终目标、权限、转向/区间资格，以及可选加速度估计、时间与质量。缺失或过期的加速度记为不可用，不替代基于轮速的区间响应。

实际电流只使用 `/sensors/core.state.current_motor`，不用 `current_input`、`avg_iq` 或目标电流替代。每行要求两个物理源时间都前进，重复定时器不增加样本。稳速和响应窗口按轮速源时间计算；累计稳速只积分同一采样段中两个唯一有效观测之间的轮速时间，两项物理源时间的间隔都须不超过 freshness，重复遥测、段间间隔和稳速等待不增加累计时长。电流响应样本要求两项物理源时间均不早于首次最终电流确认 `first_current_ack_stamp`；这个边界只锁存一次，后续重发不会排除合法的延迟遥测。成功响应的平均加速度为 `(实测结束速度−实测开始速度)/(结束轮速源时间−开始轮速源时间)`。使用首个有效越界观测，不插值。首次确认样本就已越界时无法测量时长，记为失败。稳速电流提供有效样本均值与总体标准差，保留轮速和电流的时间差；累计模式的 `duration_s` 为有效采样时长，`wall_duration_s` 为从首个到最后观测的时间跨度，`segments` 提供各采样段的区间、首尾两项源时间、时长、样本数和电流均值/总体标准差；不声称样本独立或电流已经台架校准。

软件与虚拟串口测试不等于实车闭环标定，也不证明停车距离、电流准确性或实车实时性能。
