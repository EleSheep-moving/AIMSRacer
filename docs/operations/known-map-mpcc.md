# 已知地图上的 native MPCC

已知地图运行由 `race.launch.py` 一次启动车辆、NDT、定位 monitor 和 native MPCC。FAST-LIO2 提供连续本地运动，EKF 提供后轴状态，NDT 将这段本地运动锚定到保存地图。坐标与时间戳约定见[架构](../architecture.md)。

## 地图和 bundle

当前示例地图：`~/maps/20260928_010503/map.pcd`。当前实跑 bundle 的部署目标：`~/aimsracer-data/bundles/field-v35-20261010`。它应完整包含 `input_reference/`、`input_config.yaml`、source manifest、native manifest、生成 C 和目标平台共享库。文件由部署过程复制；路径示例不意味着新机器已经具有这些数据。

参考的 `metadata.json` 绑定原始 PCD SHA-256；bundle 绑定参考、配置和生成源码，native manifest 绑定目标平台和依赖库。地图内容、参考或配置变化后，按 [MPCC 使用](../../src/controller/docs/usage.md)离线准备新 bundle。保存地图与参考的历史关联证据见[地图参考报告](../reports/2026-09-28-map-reference.md)。 新参考使用 [tools/reference](../../tools/reference/README.md)的两个恢复脚本，再执行 prepare/export/build。

## 启动与初始化

先按[启动指南](bringup.md) source 当前 workspace 和 acados 库，然后执行：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

默认等待手动初始化与启用。初始化命令的位姿始终是 **base_link 在 map 中的 x y z roll pitch yaw**，角度单位为弧度；这里 base_link 是后轴中点。将下列数值替换为车辆实际摆放：

```bash
ros2 run aims_racer_system relocalize_known_map.py \
  "$HOME/maps/20260928_010503/map.pcd" --pose-frame base_link \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

也可在 RViz 中把 fixed frame 设置为 `map`，用 2D Pose Estimate 发布 `/initialpose`，其箭头指向 base_link 的位置与方向。不要把 Livox 点云坐标或车头位置直接作为后轴初值。初始化工具等待新可信 epoch，并检查实际 map identity 与新定位状态；初始化完成后再检查地图中的车辆摆放。

启动时已知道初值，可直接传六个有限数值：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  initial_pose:="0 0 0 0 0 0" record:=true
```

省略 `initial_pose` 就等待后续工具或 RViz 输入；示例零初值仅适用于实际车辆在该地图位置。

## 启用、一次自动启动与停止

定位 `ready=true`、状态和实际命令历史新鲜、车辆静止、RC 已选择自主 speed 模式、唯一 `/drive` 发布者，且启动方向在参考切线 30° 内时，手动启用：

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

加 `auto_start:=true` 后，native 节点等待上述同一组 enable 条件，成功后只启动一次。native `auto_start_timeout` 默认 60 s；等待期间的显式 stop 会取消该请求。超时/拒绝原因在 `/mpcc/status` 中可见。完成或故障后不自动重新启用，进程 fatal 后不自动重启。需要新一圈时，由操作员明确启用或重新启动。

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

默认 `repeat_laps=false`，走完从实际启用位置计起的一圈后停车。`repeat_laps:=true` 连续绕圈，操作员明确 stop 后结束；它不改变自动启动次数。现场停止依靠 RC/急停，停稳后再关闭 launch。

## 定位的资格与限制

NDT 消费 `/fastlio2/body_cloud` 并使用 scan stamp 的 EKF TF 预测；它拥有 `map → odom`。50 Hz TF 可持有上次修正，不能据 TF 时间戳推断新的配准。monitor 通过 protocol-v1 anchor、epoch、sequence、map SHA 与源/接收新鲜度形成健康状态；native 控制器同时检查可信对齐载荷。单独 `map_valid` 或 fresh TF 不足以授权运动。

当前 `enforce_corridor=false`，参考文件左右各 0.5 m 是课程模型，不是从地图测量的真实边界；footprint OCP/启动边界检查关闭。曲率规划 ay=1 仍参与速度规划，组合加速度椭圆关闭不等于轮胎横向能力无限。已有实跑与未验证条件见[外场复盘](../reports/2026-10-10-mpcc-field-review.md)。
