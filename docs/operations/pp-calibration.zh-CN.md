# 地图轨迹编辑、PP 与纵向标定

[English](pp-calibration.md) · **简体中文**

B/C 自动电流扫描、基准导出与验收见 [B/C 操作指南](current-sweep.zh-CN.md)。

三个流程分别负责离线编辑参考轨迹、PP 跟踪和分阶段速度／电流试验。
整车默认仍是场地验证过的 MPCC；浏览器编辑器不发布车辆控制命令。

本次场地顺序为旧图先调 MPCC，再扩展建图、新轨迹测 PP，直接使用[场地命令链](field-mpcc-then-pp.zh-CN.md)。

## 功能分支环境

当前功能位于 `feat/pp-calibration-reference-editor`；NX 已编译的目录为：

```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
```

主机同名工作区位于 `/home/elesheep/aimsracer-data/code/pp-calibration-reference-editor`。
原有 main 目录与场地 bundle 保持不变。每个运行整车命令的终端都需加载功能 overlay。

## 1. 绘制参考轨迹

按[编辑器说明](../../tools/reference/editor/README.zh-CN.md)安装依赖，在仓库环境启动：

```bash
export PYTHONPATH="$PWD/src/controller:$PWD:${PYTHONPATH:-}"
/home/aims/aimsracer-data/dependencies/reference-editor-venv/bin/python \
  -m tools.reference.editor.server \
  --map "$HOME/maps/20260928_010503/map.pcd" \
  --config src/controller/config/vehicle.yaml \
  --output "$HOME/aimsracer-data/references/editor"
```

打开 `http://127.0.0.1:8765`。服务运行在 NX 时，主机使用
`ssh -L 8765:127.0.0.1:8765 aims@192.168.123.107` 转发，再打开同一主机网址。
GUI 的 Python 依赖安装独立于 ROS 工作空间。

加载已知 PCD，选择合适高度切片，绘制至少六个路点，闭合轨迹。
坐标表示 **后轴 base_link 在 map 中的位置**，不能将地图原点改成首个路点。
设置起点与向前行驶方向，检查平滑曲线、曲率、所需转向及建议速度。
稀疏路点之间的曲线可能向外偏移；点云显示不能证明那里可通行，也不能自动提供赛道边界。

使用 PP 标定时，标记直线试验区间。预览检查通过后导出，目录包括现有
`path.csv`、`metadata.json`、统一曲线系数的 `reference.json`、可继续编辑的工程及绑定哈希的标定区间。
保留整个目录；轨迹改变后重新导出，不能复用旧区间文件。
显示点云做了降采样，定位仍使用原始 PCD 和原始 SHA。

MPCC 使用新参考时沿用[原有 export/build 流程](../../src/controller/docs/usage.zh-CN.md)；
PP 直接读取参考 JSON，不需要生成求解器动态库。

## 2. 使用 PP 跟踪

加载整车工作空间环境，一次只启动一个控制器：

```bash
ros2 launch aims_racer_system race.launch.py controller:=pp \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  reference_directory:="$HOME/aimsracer-data/references/editor/my-route" \
  speed_limit:=1.0 record:=true
```

按照[初始化说明](known-map-mpcc.zh-CN.md)设置 **base_link 在 map 中的初值**，
遥控器选择解锁、导航、速度模式，然后显式启用 PP：

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
```

`controller` 默认 `mpcc`，原来的 MPCC `artifact_directory` 命令继续兼容。
PP 也能从已有 `artifact_directory` 读取统一参考及车辆配置。
PP 默认额外速度上限为 1 m/s，默认跑一圈。

PP 保留旧增强几何转向算法，包括航向误差和曲率补偿，见[PP 参数说明](../../src/aims_pp/README.zh-CN.md)。
没有求解器或计划过期门控；定位缺失、状态过期或控制权限撤回后停止，需重新显式启用。
用 `{data: false}` 或遥控锁车停止，车辆停稳后再关闭 launch。MPCC 与 PP 不能同时运行。

## 正式标定命令

四个在线操作入口都由仓库内 `aims_calibration/cli.py` 实现；共享检查位于
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

## 3. 纵向标定

整车标定 launch 启动车辆链路及**一个**标定发布者，不启动 MPCC 或独立 PP 输出节点：

```bash
ros2 launch aims_racer_system calibration.launch.py \
  stage:=speed_hold steering_source:=pp \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  reference_directory:="$HOME/aimsracer-data/references/editor/my-route" \
  record:=true
```

遥控转向选择 `steering_source:=rc`，不必提供地图与参考。
当前车辆配置使用 CH1 转向、CH5 锁车和 CH8 标定。选择解锁及标定模式后，再启用试验：

```bash
ros2 service call /calib/enable std_srvs/srv/SetBool '{data: true}'
```

待命时新鲜零速度请求用于建立选择器链路；服务启用不能绕过遥控权限。
`/control/calibration_enabled` 表示选择器实际选中有效标定命令；
`/calib/vesc_command` 是原始请求，`/vesc/command` 是交给驱动的最终请求。

| 阶段 | 内容 | 配置 |
| --- | --- | --- |
| `speed_hold` | 速度稳定后，在直线采集唯一实际电机电流样本 | 默认 1／2／3 m/s，稳定 1 s，采样 10 s |
| `accel_interval` | 相邻速度区间的正电流加速 | `accel_sweep_1to5` 加有效基准；也支持手动列表 |
| `decel_current` | 低于基准的正电流减速、滑行、负电流制动 | `decel_sweep_1to5` 加有效基准；也支持手动列表 |

将[标定 YAML](../../src/aims_calibration/config/calibration.yaml)复制到源码外，配置试验后通过
`calibration_config:=/absolute/path.yaml` 传入。默认空电流列表会拒绝电流阶段启动。
请求受车辆边界约束，驱动限幅仍为 ±100 A；B/C profile 可从明确指定的基准生成电流扫描，详见 [B/C 流程](current-sweep.zh-CN.md)。
正负电流试验均使用 `MODE_CURRENT`，不使用默认禁用的 brake 模式或预留 acceleration 模式。


九档巡航电流方案使用 [speed_hold_1to5.yaml](../../src/aims_calibration/config/speed_hold_1to5.yaml)，并传入专用 `vehicle_config`（`max_speed: 5.0`）：1–5 m/s、步长 0.5 m/s，每档累计 5 秒有效直线数据；过弯暂停，每次恢复采样前重新稳速 1 秒，完成九档后请求零速度。配置项 `hold_sampling: cumulative`；选择 `continuous` 可保留原连续窗口方式。每个独立采样段都会保留源时间和电流统计，段间时间不计入 5 秒。现场使用上面的 `profile:=speed_hold_1to5`，或者显式传入仓库内配置。5 m/s 请求只适用于显式选择该配置的稳速标定，普通 race 参数不变。

整车标定 launch 单独关闭本次驱动的 40 A 起步电流替换；普通 vehicle／race 默认值保持不变。
标定默认开启独立 acc 观察节点，不把它接入电机闭环。
只启动包内标定节点时，需要事先对驱动应用这一覆盖参数。

累计稳速模式在离开 PP 试验区间或速度不稳时暂停采样，保留当前档的有效数据；重新进入直线且稳速后继续。连续稳速及电流响应模式仍作废中断窗口。人工干预导致权限丢失、输入过期或故障时，当前整档作废并保留原因。
PP 最近点投影的参考进度发生跳变时继续跟踪，不将投影跳变判作定位故障；采样区间按当前投影判断，实际定位失效仍停止。
实测轮速超过目标或配置上限时不再触发 `overspeed` 停车；稳速采样仍只计入目标速度容差内、重新稳速后的数据。
重新启用从准备阶段开始，不继续半途电流试验。正常完成输出零速度；
手动停用或故障输出零电流，标定模式下起步替换已关闭。

## 4. 查看证据

`record:=true` 创建新的日期会话，保存 bag、实际配置与参考快照、阶段状态和试验结果。
录包不会赋予控制权限；关闭 bag 录制时仍保存标定结果。

- 对照原始 `/calib/vesc_command`、最终 `/vesc/command` 与 `/control/calibration_enabled`，不能只看程序请求。
- 实际电机电流取 `/sensors/core.state.current_motor`；`current_input` 是电池侧电流，最终命令也不是电流测量。
- 保留轮速、ERPM、电压、acc 及质量状态的源时间戳；重复消息不能当成独立样本。中断或超时不是成功加减速结果。
- 地图、导出参考、试验 YAML、代码版本和会话一起保留；软件、虚拟串口、回放与实车验收分别报告。

旧采集教程仍在历史归档中，其中 `/calib/ackermann_cmd` 和直接绕过驱动入口的 remap 不属于当前启动流程。
