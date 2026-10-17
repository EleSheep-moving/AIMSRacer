# 双参考点 ROS 输出验证

新入口操作说明：[后轴与质心五量](../../docs/operations/state-observer.zh-CN.md)。

原生测试在 `src/aims_racer_system/tests/test_velocity.cpp` 和 `test_body_acceleration.cpp`；真实 ROS/DDS 合同在 `test_state_observer_runtime.py`，通过 CTest `state_observer_runtime` 运行。

测试覆盖非零 VY、安装 yaw 旋转、两参考点刚体速度/加速度解析解、XY 协方差、yaw 参考点不变性、输入失效、独立 gyro 分支、源时间/epoch、最小角加速度回归时间支持和录包参数选择。旧 scalar VX 与旧加速度默认输出另用冻结实包 trace 对照，不把新版接口测试当成独立运动真值。

警告优先修订增加：LIO 超时继续 IMU 推进及不强制重新初始化、迟到连续 IMU 仍按源时刻发布、历史内迟到 LIO 继续纠偏；保留源断流/时间支持/数值失败的分支恢复测试。新模式默认开启，旧节点默认参数保持不变。

## 真实包 1× 回放

`replay.py` 使用原速度验证抽取的 `input.npz` 和 `metadata.json`，只发送 corrected IMU 与 raw LIO 位姿，保留原接收顺序/间隔，用共同 epoch 平移保持输入源年龄。计时前预加载 NPZ；使用独立 ROS domain、安装后的 standalone launch，不启动传感器、导航或车辆执行链。

```bash
source /opt/ros/humble/setup.bash
source /path/to/validated/install/share/aims_racer_system/local_setup.bash
export ROS_DOMAIN_ID=201 ROS_LOCALHOST_ONLY=1
python3 verification/dual_reference/replay.py \
  --input /absolute/vx-observer-20261015/inputs/mpcc \
  --output /absolute/new-full-replay
```

输出目录必须不存在。默认回放所有原始观测事件；可显式指定 `--start-source-ns`、`--duration` 限定范围。输入 schema 与 `verification/velocity/extract.py` 一致，运行需要 NumPy、rclpy 与已安装 ROS 消息。

输出 CSV 保留平移后和原始 int64 源时间、两点三种数值输出及模型方差。检查每个消息源时间属于发送的 IMU、同一输出流严格递增、frame/有效字段/未知协方差正确；两参考点同类序列相同，速度差满足配置刚体关系，yaw 完全相同，数值有限且节点无 wheel/EKF/control/TF 接口。

回放记录通过 `/proc` 确认实际运行的子进程 ELF，保存安装目录、二进制、launch、模型/几何文件哈希，归档节点实际参数文件和 `ros2 param dump`。播放与排空阶段持续检查节点进程；`test_replay.py` 会在已有有效输出后终止子节点，要求回放立即判失败且不生成通过摘要。该负例使用独立 domain 203：

```bash
DUAL_REPLAY_INPUT=/absolute/vx-observer-20261015/inputs/mpcc \
  python3 -m pytest verification/dual_reference/test_replay.py -q
```

本工具按默认安装几何的两点差 `[0.21,-0.03,0] m` 验证恒等式；这是所用近似后轴安装与质心估计配置的代数差，不是新测得的后轴到质心外参。不要用自定义安装几何运行本默认验证工具并把它当成相应配置的验证结果。

这些检查证明接口、时序及参考点计算的一致性，没有独立真值；不构成 VY、加速度、侧滑或整车负载实时验收。源数组/二进制/日志在外部数据目录，Git 仅保留代码和必要摘要。

## 同包导航与独立输出比较

`compare_reference.py` 读取 `verification/state_quality` 使用的 `raw.npz/raw-meta.json` 和本工具产出的完整回放 CSV。验证输入／输出哈希及原 bag 身份，在共同后轴参考点、源时间 44≤t<219 s、200 Hz 网格上对比 VX/轮速、VY/LIO、yaw rate/corrected gyro；扫描 ±800 ms，步长 5 ms，同一量的两个估计器及所有候选偏移共用支持掩码，不拟合增益或偏置。

```bash
python3 verification/dual_reference/compare_reference.py \
  --raw /absolute/critical-review \
  --replay /absolute/new-full-replay \
  --output /absolute/new-waveform-comparison.json
```

正偏移表示估计波形相对参考滞后；负偏移不是物理负延迟。参考信号都有自身误差／相关性，结果只能说明波形差异，不能代替独立动态精度。[最新复核报告](../../docs/reports/2026-10-17-state-observer-latency-policy.zh-CN.md)。
