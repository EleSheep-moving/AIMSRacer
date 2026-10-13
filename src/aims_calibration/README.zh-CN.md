# 纵向标定

[English](README.md)

`aims_calibration` 通过现有整车指令选择器进行纵向标定。节点以 50 Hz 发布 native `vesc_msgs/VescCommand` 到 `/calib/vesc_command`，时间戳为当前 ROS 时间，坐标系为 `base_link`。选择器独占 `/vesc/command`；标定节点不发布 `/drive`。控制与状态定时器采用稳定时钟，因此 `/clock` 停止时仍能检查数据是否过期。

节点可执行文件为 `longitudinal_calibration`，节点 launch 为 `longitudinal_calibration.launch.py`，完整整车入口为 `aims_racer_system/calibration.launch.py`。节点 launch 参数包括 `config`、`reference_directory`、`vehicle_config`、`pp_config`、`log_directory`、`stage`、`steering_source`、`auto_start`。完整入口同时配置整车、定位、录包与加速度估计。默认不启用，必须调用 `/calib/enable`（`std_srvs/SetBool`）；`auto_start: true` 会被拒绝。`/calib/status` 提供状态、原因、实际与请求电流及输出目录。完整启动命令见[操作指南](../../docs/operations/pp-calibration.zh-CN.md)。

## 三个阶段

- `speed_hold`：默认以 `hold_sampling: cumulative` 累计有效直线稳速样本。每次进入直线，连续 1 秒处于目标 ±0.2 m/s 内后才采样；入弯、切换直线区间、速度不稳或采样源有未覆盖的间隔时暂停，保留当前速度档已采集的数据，重新稳速后继续。累计达到采样时长后立即切换下一档，全部速度完成后请求零速度。基础配置仍为 `[1, 2, 3]`、每档 10 秒；九档方案使用下述专用配置。`hold_sampling: continuous` 保留原来的连续窗口方式，离开区间或速度不稳即作废并重采。
- `accel_interval`：先用速度模式达到 `start_speed`，再依次请求显式配置的正 `currents`，使用 `MODE_CURRENT`。只有最终被选择的指令回显该请求后才开始计算响应。新轮速达到或超过较大的 `end_speed` 时，立即请求零速度；即使电流遥测未更新或尚无电流确认也会退出，并将不完整响应记为失败。
- `decel_current`：先用速度模式达到正的 `start_speed`，再请求显式配置的负 `currents`，同样使用 `MODE_CURRENT`。新轮速达到或低于正的 `end_speed`（默认 0.5 m/s）时立即请求零速度。边界到达但缺少新的电流遥测时，响应记为失败。不会主动请求倒车。

电流阶段的默认列表为空，必须复制 `config/calibration.yaml` 并明确设置带符号电流与速度区间。launch 的 `stage`、`steering_source` 会覆盖 YAML 对应项，选择电流阶段或遥控转向时要同时传入这些参数。格式示例：加速 `currents: [5.0, 10.0]`、`start_speed: 1.0`、`end_speed: 2.0`；减速 `currents: [-3.0, -5.0]`、`start_speed: 2.0`、`end_speed: 0.5`。这仅是配置示例，不是已经实车验证的电流推荐。

若准备阶段的稳定轮速已经越过结束速度，例如区间过窄与稳速容差重叠，则直接请求零速度，不发送任何非零电流，记录为未开始。默认响应超时为 8 秒；失败试验不计算平均加速度。电流上限为 100 A；`speed_hold` 请求速度上限为 `min(config.max_speed, vehicle.max_speed, 5)` m/s，正负电流阶段仍至多 4 m/s。专用稳速配置可设置至多 0.2 m/s 的 `speed_overshoot_tolerance`，仅用于实测速度的超速判定，不提高请求速度。不自动推测基础电流或自动扫描；若依据稳速实际电流制定后续列表，应预先计算并确认带符号值。标定入口关闭 VESC 驱动的 40 A 起步替代功能，避免请求电流被该功能替换。

## 九档巡航电流方案

使用 `config/speed_hold_1to5.yaml`：速度列表为 `[1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]`，每档累计 5 秒，`repeats: 1`，每次恢复采样前连续稳速 1 秒，弯道 `recovery_speed: 1.0`。例如 5 m/s 单条直线不足 5 秒时，可以在后续直线继续累计，不能把这称为连续 5 秒。

该配置明确选择 `max_speed: 5.0`、`speed_overshoot_tolerance: 0.2`，需同时传入专用 `vehicle_config`，其 `max_speed` 为 5.0；不修改普通 race 的车辆配置。若仍选择 4 m/s 车辆配置，启动会拒绝九档列表，不静默跳过 4.5 或 5 m/s。诊断包含当前目标速度、累计秒数、段数、计划档数和采样模式。权限、定位或日志故障仍作废当前整档；显式重新启用从第一档重新开始。

## 转向与有效采样区间

`steering_source: pp` 复用 `aims_pp` 的增强 PP 核心、`vehicle_config` 几何和数值参考加载器，沿用独立 PP 的定位健康与地图身份检查。默认前视增益 1、最小 0.3 m、最大 4.5 m，航向增益 0.1、曲率前馈增益 0.1；可用 `pp_config` 调整。车辆几何、前进朝向、位姿连续性、源时间、定位 epoch 和地图 SHA 必须一致。仅当车辆配置启用 `enforce_corridor` 时检查车体与走廊。直线区间之外使用 `recovery_speed` 跟踪，准备阶段使用速度模式。

PP 需要 `reference_directory/reference.json` 与编辑器导出的 `calibration_regions.json`。区间格式为 `{schema_version: 1, reference_sha256: "参考文件原始字节SHA", regions: [{id: "straight1", name: "straight 1", kind: "straight", start_s: 2.0, end_s: 10.0}]}`。进度采用参考的节点/弦长坐标，不是精确曲线弧长。区间为左闭右开；`start_s > end_s` 表示跨越闭环接缝，相同端点与重叠区间被拒绝。手动直线标注不证明距离、曲率、抓地或电流安全。累计稳速方式在退出或切换区间时关闭当前采样段，之后重新稳速再开新段；每段单独保存源时间与电流统计，过弯、准备时间不计入有效时长。连续稳速方式和正负电流响应试验仍在退出或切换区间时使未完成试验无效。

`steering_source: rc` 不依赖定位或参考；速度来自轮速里程计。默认 profile 为 `steering_ch1_throttle_ch3_aux_ch5_to_ch10`：转向 CH1、油门 CH3、锁定 CH5、ESC CH6、控制源 CH7、标定 CH8、限幅 CH10。通道从 1 开始计数；中心 992，端点 172/1810，转向反向，选择器转向比例 0.4751 rad。采样要求新鲜有效遥控、CH5 ≥992 解锁、CH8 >992 选择标定，转向处于中心 ±50 计数内。这个死区只控制采样资格，不改变实际转向指令。转弯暂停累计稳速采样；连续稳速/电流响应试验则作废未完成试验，并返回速度模式，恢复时仍可遥控转向。`sequential_ch1_ch2` 只适用于选择器明确采用该 profile 的情况（转向 CH2、锁定 CH3、标定 CH7）。

## 权限、时序与退出

刚启动且未启用时，持续新鲜的零速度请求供选择器建立标定选择。启用后等待真实的 `/control/calibration_enabled == true`，并检查遥控、新轮速、新电流数据和与近期请求对应的最终 `/vesc/command`。选择器会重写时间戳，因此按短历史中的模式与内容匹配，不要求恰好对应最新请求。只填充当前模式的目标字段；正负电流都使用 `MODE_CURRENT`，不发布 `MODE_BRAKE_CURRENT` 或 `MODE_ACCELERATION`。

权限撤销、遥控锁定或模式变化、源/接收年龄超过 0.2 秒、无效遥测、外来最终指令、地图/定位失效、ROS 或源时间回退、ROS 时间冻结、定时器堵塞、额外标定发布者都会锁存 `FAULT`，使当前试验无效并持续请求零电流，直到显式重新启用。人工禁用也使未完成试验无效并请求零电流。重新启用重置时钟基线、缓存和请求历史；遥控、权限、轮速、电流观测必须在本次启用之后到达，并确认本次新零速度请求已被选择，才进入准备。不恢复中断试验。正常完成请求零速度；零电流是滑行请求，不保证物理停车，外部选择器和操作者仍掌握控制权。

## 输出与指标定义

每次运行在 `log_directory` 下创建带 UTC 时间和 UUID 的目录，包含 `observations.csv`、`runtime.csv`、原子替换的 `trials.json`，以及出现试验后生成的 `trials.csv`。保留有效配置、车辆文件 SHA、PP 参考与区间 SHA、遥控 profile、有效 PP 参数、调参文件 SHA、通过 launch 使用的标定 YAML SHA 与有效配置 SHA。

有界后台队列负责写盘，溢出或写失败使采集无效并请求零电流。刚完成的记录无法入队时，撤销成功状态、设置 `evidence_valid: false`、清空 `mean_acceleration`，用 `response_completed` 保留是否实际越过边界；队列腾出空间后重试结果检查点。**JSON 是结果验收依据**：先完成 CSV 检查点，再提交成功 JSON；后台写失败立即尝试通过独立 JSON 路径发布无效结果，即使没有后续排队任务。CSV 可能保留前一个检查点，应结合 JSON 与诊断判断。持续磁盘故障可能无法保存新结果，但仍在诊断中可见。

`runtime.csv` 保存 ROS 时间、回调耗时（截至采集入队）、定时器间隔、队列计数和状态，用于联合负载分析。原始观测保存轮速/遥测源时间、轮速、`current_motor`、`voltage_input`、ERPM（`state.speed`）、请求与最终目标、权限、转向/区间资格，以及可选加速度估计、时间与质量。缺失或过期的加速度记为不可用，不替代基于轮速的区间响应。

实际电流只使用 `/sensors/core.state.current_motor`，不用 `current_input`、`avg_iq` 或目标电流替代。每行要求两个物理源时间都前进，重复定时器不增加样本。稳速和响应窗口按轮速源时间计算；累计稳速只积分同一采样段中两个唯一有效观测之间的轮速时间，两项物理源时间的间隔都须不超过 freshness，重复遥测、段间间隔和稳速等待不增加累计时长。电流响应样本要求两项物理源时间均不早于首次最终电流确认 `first_current_ack_stamp`；这个边界只锁存一次，后续重发不会排除合法的延迟遥测。成功响应的平均加速度为 `(实测结束速度−实测开始速度)/(结束轮速源时间−开始轮速源时间)`。使用首个有效越界观测，不插值。首次确认样本就已越界时无法测量时长，记为失败。稳速电流提供有效样本均值与总体标准差，保留轮速和电流的时间差；累计模式的 `duration_s` 为有效采样时长，`wall_duration_s` 为从首个到最后观测的时间跨度，`segments` 提供各采样段的区间、首尾两项源时间、时长、样本数和电流均值/总体标准差；不声称样本独立或电流已经台架校准。

软件与虚拟串口测试不等于实车闭环标定，也不证明停车距离、电流准确性或实车实时性能。
