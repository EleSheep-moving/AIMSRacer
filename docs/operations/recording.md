# 录制传感器与控制器数据

现场录制由 launch 管理，无需另写录包 wrapper。在[同一 workspace 环境](bringup.md)中使用：

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true
```

建图使用 `ros2 launch aims_racer_system mapping.launch.py record:=true`。录制本身不授予自主控制权限。

## 目录与数据

`session_directory` 留空时，在 `~/aimsracer-data/sessions/YYYY-MM-DD/` 下生成带时间和模式的目录；指定时必须是新目录。

```bash
ros2 launch aims_racer_system race.launch.py \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  artifact_directory:="$HOME/aimsracer-data/bundles/field-v35-20261010" \
  record:=true session_directory:="$HOME/aimsracer-data/sessions/my-new-run"
```

会话包含 `bag/`、`session.json` 和系统 `params/` 快照；race 同时把 native `runtime.csv` 写入会话目录。`record=true` 会选择该目录作为 `log_directory`；单独使用 `log_directory` 可只记录 runtime。

| 数据 | 自动录制内容 |
| --- | --- |
| 原始与修正 IMU | `/livox/imu`、`/livox/imu_bias_corrected`、`/imu/gyro_bias/status` |
| 重放 LIO 的原始输入 | `/livox/lidar`、`/livox/imu` |
| LIO 与后轴数据 | `/fastlio2/lio_odom`、`/fastlio2/body_cloud`、`/rear_axle/lio_odom`、`/rear_axle/imu` |
| wheel / EKF | `/rear_axle/wheel_odom`、`/odometry/filtered` |
| RC 与执行命令 | `/rc/channels`、`/ackermann_cmd`、`/control/autonomy_speed_enabled`、motor speed/current、servo position、`/sensors/core` |
| TF | `/tf`、`/tf_static`、隔离的 `/fastlio2/tf` |
| race 定位与控制 | `/localization/status`、anchor/ndt status、map identity/valid、NDT/bridge pose、`/drive`、`/mpcc/status`、reference/prediction |
| mapping | `/pgo/loop_markers` |

完整列表由 [launch_support.py](../../src/aims_racer_system/aims_racer_system/launch_support.py)维护。录包使用 [recording_qos.yaml](../../src/aims_racer_system/params/recording_qos.yaml)：**原始 `/livox/imu` 为 reliable、keep_last、depth=4096、volatile**，保留高频输入；其他 topic 使用 rosbag 的发布者 QoS 适配。保持 `/fastlio2/body_cloud` 与 raw LIO scan-end 时间对应，以便重放 NDT；world cloud 是可选显示输出。

## 结束与核验

车辆停稳、撤销自主权限，再用 Ctrl-C 正常关闭 launch，让 rosbag 写完索引：

```bash
ros2 bag info "$HOME/aimsracer-data/sessions/my-new-run/bag"
```

核对 topic 数量、消息计数、时长和 `metadata.yaml`；保留完整 bundle、地图文件及其哈希、源码版本、现场起终点和干预说明。原始 bag、bundle 和日志保存在源码目录外。

## 时间和圈时口径

source stamp 是测量时刻，bag 接收时间包含传输与排队，两者分别统计。EKF 新鲜 header 不意味着 LIO 测量无延迟；`/ackermann_cmd` 是转发命令，servo command echo 不是转向角测量。

当前[外场复盘](../reports/2026-10-10-mpcc-field-review.md)的运动窗口为首次 RUNNING 后 `abs(vx)>0.05 m/s` 的首末时间差，包含静止起步和终点制动；它不是 flying lap。RUNNING→COMPLETE 另含终点等待。比较圈速时固定计时横截面、同配置重复，分开记录运动、完成、停车等待、误差和人工干预。

## MPCC 参考录制

`record_path` 保存 `odom/base_link` 原始 CSV，默认输入 `/odometry/filtered`：

```bash
ros2 run aims_mpcc record_path --ros-args -p output:="$HOME/aimsracer-data/lap.csv"
```

手动前进绕一圈并少量重叠后停止 recorder。建图时可用 `-p odom_topic:=/rear_axle/lio_odom`。CSV 的 odom 原点属于该次定位会话；保存地图后需离线关联地图位姿，形成真正的 `map/base_link` CSV，再准备持久参考。`prepare_path` 不会仅靠 `--map-file` 转换坐标。参考与 bundle 准备见 [MPCC 使用](../../src/controller/docs/usage.md#准备参考)。
