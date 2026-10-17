# NX 独立五量估计器：现场命令

本目录是统一源码内的现场入口，NX 主目录快捷方式指向这里；不再依赖实验目录中的脚本。

主工作区：`/home/aims/AIMSRacer`；安装：`/home/aims/AIMSRacer/install/aims_racer_system`。

同时发布后轴 `base_link` 和质心 `center_of_mass` 的 VX、VY、AX、AY、车体 omega_z。质心 IMU 偏移为 `[-0.09,-0.03,-0.03] m`，高度仍是估计值。警告优先策略开启。MPCC 当前状态输入和导航 EKF 保持原接线。

## 启动与查看

沿用原整车入口，加一个参数即可启动观测器并由原录包入口记录两套输出：

```bash
~/run_mpcc_imu_fit.sh state_observer:=true
```

上面的命令仍使用原地图、bundle、速度设置与显式 MPCC enable。部署过程没有执行整车入口。

如果已有 corrected IMU 和 raw LIO 传感器链在运行，则单独启动：

```bash
~/run_state_observer.sh
```

只启动一份 `state_observer`：整车入口已带 `state_observer:=true` 时，不再单独启动。前台 Ctrl-C 停止独立观测器。

另一终端查看：

```bash
source ~/state_observer_env.bash
ros2 topic echo /state_estimation/rear_axle/velocity --once
ros2 topic echo /state_estimation/center_of_mass/velocity --once
ros2 topic echo /state_estimation/center_of_mass/acceleration --once
ros2 topic echo /state_estimation/center_of_mass/yaw_rate --once
ros2 topic echo /state_estimation/center_of_mass/status --once
```

velocity.linear.x/y 对应 VX/VY；acceleration.linear.x/y 对应 AX/AY；yaw_rate.angular_velocity.z 对应 r。没有输入时等待有效数据，不发布假零。上述回显消息分别是 TwistWithCovarianceStamped、AccelWithCovarianceStamped 和 Imu，向量在其相应嵌套字段内。

## Vicon 采集准备

现场配置由 `~/.config/aimsracer/state-observer-site.bash` 中的 `AIMS_VICON_CONFIG` 指定；迁移保留原配置路径。未填写现场配置时回退到仓库只读模板，不将未知外参填成零。

当前保持未填写模板；现场确定 Vicon topic、标准 ROS 消息类型、单位和有效帧语义后填写。分析还需独立核对时钟及 marker 到车体/两目标点的外参。

```bash
source ~/state_observer_env.bash
~/vicon_state.sh inspect --config "$AIMS_VICON_CONFIG"
~/vicon_state.sh record --config "$AIMS_VICON_CONFIG" \
  --output /home/aims/aimsracer-data/sessions/2026-10-19/vicon-state-validation/run01
```

run01 必须不存在，录制终端 Ctrl-C 正常落盘。使用 Vicon 专用录包时，原整车入口增加 `record:=false`，避免重复录包。配置检查和录制不启动车辆或控制器。

工具说明：`/home/aims/AIMSRacer/verification/vicon_state/README.zh-CN.md`。

部署证据：`/home/aims/aimsracer-data/experiments/state-observer-deployment-20261018/`。
