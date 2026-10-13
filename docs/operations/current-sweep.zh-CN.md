# 基准电流、加速与减速标定

[English](current-sweep.md) · **简体中文**

本流程以 NX 外场实测的 A 阶段为基础，明确选择一份通过验收的 A 结果来生成
B/C。A 的采集设置与 v1 结果格式保留。控制、配置和操作命令均在 AIMSRacer
仓库内；地图、参考轨迹、派生基准表和录包属于外部数据。

## 1. 加载 B/C 环境

分支为 `feat/calibration-current-sweep`，NX 每个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source /home/aims/aimsracer-data/code/pp-calibration-reference-editor/install/setup.bash
source /home/aims/aimsracer-data/code/calibration-current-sweep/install/setup.bash
export AIMS_MAP=/home/aims/maps/20261013-field-expanded/map.pcd
export AIMS_REFERENCE=/home/aims/aimsracer-data/field-tests/20261013-field/new-references/pp-routev2-straight-trials
export AIMS_BASELINE=/home/aims/aimsracer-data/calibration/baselines/20261013-repeat-battery.json
```

旧功能环境提供已有的整车编译依赖，新环境提供 PP、标定和系统 launch。
A 原来的工作目录、编译环境与录制数据仍保留。

## 2. 离线导出通过验收的 A 基准

明确选择这次九档均通过的采集：

```bash
export AIMS_A_RESULT=/home/aims/aimsracer-data/sessions/2026-10-13/222944_118387098-pp-speed-hold-1to5-repeat-battery/20261013T142946Z-1849a491/trials.json
ros2 run aims_calibration results "$AIMS_A_RESULT"
ros2 run aims_calibration baseline "$AIMS_A_RESULT" --output "$AIMS_BASELINE"
ros2 run aims_calibration plan --stage accel_interval --base-current-file "$AIMS_BASELINE"
ros2 run aims_calibration plan --stage decel_current --base-current-file "$AIMS_BASELINE"
```

上述命令不启用车辆。导出要求 A 的九档完整、成功且证据有效，验证配置与源
文件哈希，保留实际电机电流均值、标准差、样本数、有效时长和源元数据。
有分段边界和 `observations.csv` 时，仅使用 A 实际纳入统计的样本计算电压，
并保存 CSV 哈希；老结果缺少这部分证据时，电压保持未知，不猜测。

源 A 文件不修改。新 A 采集、车辆配置变化或电池状态明显变化后，应明确选择
新的基准表。电压用于说明测量条件，不自动补偿或增加运行期电压门控。
PP 扫描检查源记录中已有的地图、参考、直线区间和车辆身份；遥控转向扫描
检查车辆身份。基准文件在节点启动后被修改，会拒绝启用。

## 3. B/C 测什么

令 `I0(v)` 为 A 在速度 `v` 下测得的电机电流均值。

| 阶段 | 速度 | 请求电流 | 结束条件 |
| --- | --- | --- | --- |
| A `speed_hold_1to5` | 1、1.5、…、5 m/s | 速度模式，测实际电流 | 每档累计五秒有效直线数据 |
| B `accel_sweep_1to5` | 1→1.5 到 4.5→5，共八个相邻区间 | `I0(v0)+5`、`I0(v0)+10`、…，最高 100 A | 轮速 ≥ `v0+0.5` |
| C 正电流减速 | 起速 1.5、2、…、5 | `I0(v0)-5`、`I0(v0)-10`、…，严格大于零 | 轮速 ≤ `v0-0.5` |
| C 滑行 | 同上 | 0 A | 轮速 ≤ 1 m/s |
| C 负电流制动 | 同上 | −5、−10、…、−100 A | 轮速 ≤ 1 m/s |

B 最后一档不足 100 A 时补一个 100 A；若第一档 `I0+5` 已超过上限，直接拒绝
该计划。低于或等于基准的正电流不会进入 B，放在 C 中。
C 保留小数：`I0(1.5)=10.1045 A` 时，正电流档约为 5.1045 和 0.1045 A，
不把小正电流舍入成零。所选 A 结果生成 **B 145 项、C 184 项**，这是跳过前数量。

每项先恢复速度模式到该项起速，在 ±0.2 m/s 内连续稳速一秒，再切到指定电流。
响应超时八秒。离开标记直线使本次尝试无效，同一项最多尝试三次；
PP 弯道恢复速度仍为 1 m/s，重新进入直线后再次准备起速。
普通项间切换立即恢复下一项起速，最后一项结束才请求零速度。
正电流、零电流和负电流均走 native `MODE_CURRENT`，不走 brake mode。

### 判断电流能力与跳档

最终被选择的电流请求确认后等待 0.25 秒，使用 0.5 秒滚动窗口，要求至少十个
唯一且新鲜的实际电流观测。容差为 `max(1 A, |目标电流|×10%)`，至少 90%
有效样本一致不满足要求时，才形成能力不足判断：

- B 正电流实际幅值不足：跳过同一速度区间后续更强的电流，进入下一速度区间。
- C 正电流持续同方向超出绝对跟踪容差：跳过该起速剩余正电流，仍测试零电流
  和负电流，不能把整个速度档跳掉。
- C 负电流制动幅值不足：跳过该起速后续更强的负电流，再进入下一起速。
- 实际电流正常但速度响应超时，不证明电气能力不足，仍测试下一档电流。
  新鲜速度端点成功与电流失败同时出现时，优先保留端点成功。

响应太短、没有完整跟踪窗口时可成功，但记作 `tracking_status: unverified`，
不能据此跳过更强电流。生成计划按有序分组跳档；手动列表只跳过紧接着的同组
更强项，不会跨过乱序项任意扫描整个后续列表。

## 4. 启动一个阶段

先完成已有实验，实车停稳后关闭已有车辆 launch。切换阶段使用新的会话。
**B** 启动一次：

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=accel_sweep_1to5 base_current_file:="$AIMS_BASELINE" \
  steering_source:=pp map_file:="$AIMS_MAP" reference_directory:="$AIMS_REFERENCE" \
  record:=true longitudinal_acceleration:=true auto_start:=false rviz:=true
```

**C** 在另一会话中执行同样的命令，将 profile 改为 `decel_sweep_1to5`。
配置自动选择仓库内的 5 m/s 车辆和 PP 参数，在录包或整车启动前验证基准并冻结
完整计划。默认 `stage:=auto` 根据 profile 选阶段，冲突的显式 stage 会被拒绝。
A 仍使用 `profile:=speed_hold_1to5`，不需要基准文件。

标定入口关闭 VESC 的 40 A 起步电流替换，只有一个 native 标定发布者，
请求电流上限 ±100 A。实车能否达到请求值取决于电机、VESC 限制、电压和速度。
该入口不同时启动 MPCC 或另一个 PP 指令发布者。

## 5. 检查、启用一次与退出

另一终端加载同样环境后：

```bash
ros2 run aims_calibration inspect
# RViz 确认定位、前进方向、标记直线；遥控解锁并选标定。
ros2 run aims_calibration start
```

启用检查静止、权限、新鲜输入、唯一指令来源、定位与参考身份、起步替换关闭、
日志正常。程序不会自动启用。沿用 NX 补丁，PP 投影跳变和实测速度过冲不会
单独触发旧停止门控，A 有效采样仍要求稳速容差；电流阶段拒绝 VESC 非零故障码。

`start` 的 B/C 默认监控上限 7200 秒，A 仍为 900 秒；可用
`start --max-seconds 3600` 覆盖。终端显示区间、类型、目标电流、尝试和跳档。
中止时先遥控锁车，再执行：

```bash
ros2 run aims_calibration stop
```

禁用/故障请求零电流，仅表示滑行，不证明物理停车。CLI 连续检查三秒新鲜轮速
停稳证据。确认实车停稳后，在 launch 终端 Ctrl+C；程序不会自动关闭整车链。

## 6. 区分执行完成、证据有效与覆盖完整

```bash
ros2 run aims_calibration results /absolute/path/from/output_directory
ros2 bag info /absolute/path/to/session/bag
```

B/C v2 保存冻结计划与哈希、基准、实际跟踪配置、每次真实尝试和跳过项。
`attempt_id` 只标记真实尝试，跳过项关联触发能力判断的尝试与反馈证据。A 保持 v1。

- `execution_complete`：每项已有执行结果或跳过记录。
- `evidence_valid`：日志、来源与内容检查通过。
- `coverage_complete`：每项有成功测量或可验证的能力不足/跳过判断。
  超时与耗尽重试仍留下缺口。
- `accepted`：只统计成功测量；能力不足和跳过单独统计。

能力不足不能伪装为成功加速度样本。队列溢出或后台写失败会使证据无效，完成
边界同样重新检查。持续磁盘故障无法写新检查点时，状态仍报告故障，旧成功 JSON
不足以完成在线验收。平均加速度使用实测速度差除以配对轮速源时间间隔；
`response_duration_s` 另行记录从 producer 请求到端点的总响应时间。

软件、隔离 ROS 和模拟 VESC 串口测试只验证程序。
**B/C 的实车小电流与外场验收尚待进行。**

[PP 与 A 流程](pp-calibration.zh-CN.md) · [包接口](../../src/aims_calibration/README.zh-CN.md)

[验证报告](../reports/2026-10-14-calibration-current-sweep.zh-CN.md)
