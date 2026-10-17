# 后轴转换、陀螺修正与加速度观测器 C++ 迁移

[English](2026-10-15-native-state-nodes.md) · **简体中文** · [实验报告](README.zh-CN.md)

2026-10-15，将三个持续运行的 Python 传感器/观测器节点迁移为 C++17，并在 NX 主工作区完成编译安装。本机 Humble 容器与 NX 均通过 **20 组 CTest**；NX 的 `colcon test-result` 汇总为 **119 项、0 错误、0 失败、0 跳过**。这验证软件与合成 ROS 行为；迁移后尚未实车行驶，也未做同工况性能基准比较。

## 运行入口变化

| 安装的可执行文件 | 职责 | 启动配置 |
| --- | --- | --- |
| `livox_gyro_bias` | 在 Livox 坐标系标定 gyro 零偏，完成后持续发布修正 IMU | `rear_axle_frames.launch.py` |
| `lio_to_rear_axle` | 将保留源时间的 LIO 位姿、速度与协方差转换到后轴；可选建图 TF | `rear_axle_frames.launch.py` |
| `longitudinal_acceleration` | 独立轮速/比力 `[v,a,d]` 卡尔曼观测器 | `longitudinal_acceleration.launch.py` |

[后轴启动配置](../../src/aims_racer_system/launch/rear_axle_frames.launch.py)和[加速度启动配置](../../src/aims_racer_system/launch/longitudinal_acceleration.launch.py)已使用原生程序。`imu_to_rear_axle` 原本就是 C++。话题、坐标系、YAML 参数、默认阈值及 launch 参数保持原样；加速度节点仍须显式开启，不接入导航或控制反馈。[CMake](../../src/aims_racer_system/CMakeLists.txt)不再安装三个旧 Python 包装脚本，保留的 Python 模块用于数值参考和离线分析。

部署对象是主工作区。其他标定工作区及已有不可变 MPCC bundle 是独立产物；本次迁移不重新生成或改变它们的车辆模型。

## 保留并检查的行为

**陀螺修正。** 默认仍需 10 秒静止窗口、至少 1,000 个样本，并检查新鲜轮速/LIO、运动及噪声。静止判断中的 Livox 原始加速度单位仍是 g。标定完成前不发布修正数据；完成后固定零偏，持续转发真实旋转。IMU 源时间不再递增时，清除标定和外部观测历史。修正保留消息头、加速度和全零未知 gyro 协方差；已有协方差保留原来的对角增量。

**后轴里程计。** 严格保留 LIO 整数扫描结束时间。速度杠杆补偿使用源时刻精确陀螺结点，或最大间隔 30 ms 的前后实测结点插值；缺少后一结点则等待，不外推，也不跨大缺口插值。历史上限仍为 2,000 条 gyro、20 条等待中的 LIO。几何测试用非零安装旋转和互协方差，逐元素对照 Python/SciPy 的完整位姿/速度协方差。原生 DDS 测试覆盖迟到 LIO、等待上结点、TF 内容与源时间、gyro 倒退清空队列和已发布时间锚点，以及大间隔拒绝。

**加速度。** 测量使用整数纳秒，源时间相同时保留接收顺序。迟到测量重放保留后缀；按时间/条数裁剪时保留前缀后验。Joseph 协方差更新及数值检查通过整个后缀后才提交。轮速回调更新历史但不发布；IMU 回调触发当前接收时刻预测，初始化后保留比力不可用心跳。ROS 时间倒退会重置初始化和历史；坐标系、非有限值、未来源时间、年龄及耗时诊断接口保留。

原生测试包含 10,000 次小方差更新，以及重放后缀溢出时不得改变状态/历史的事务检查。Python/C++ 对照包含固定种子的有序、同时间戳、迟到序列、时间/条数锚点、无效输入及 UNIX 大时间戳下的长间隔预测。合成 DDS 输出在受控 ROS 时钟下与 Python 数值参考比较。见[测试目录](../../src/aims_racer_system/tests)和[便携验证摘要](assets/2026-10-15-native-state-nodes/validation-summary.json)。

## 复跑与证据

在依赖已齐备的普通 Humble workspace：

```bash
colcon build --packages-select aims_racer_system --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ctest --test-dir build/aims_racer_system --output-on-failure
colcon test-result --test-result-base build/aims_racer_system/test_results --verbose
```

20 组 CTest 和 119 项 XML 测试属于不同汇总层级，不能相加。NX 程序已核对为 ARM aarch64 ELF，20 个源码/构建/启动文件与本机 SHA256 一致；应用迁移前保留了 NX 源码和安装备份。哈希标识实际测试的工作区，不能把基础 commit 单独当成已包含本次迁移。

仓库证据：[本机 CTest](assets/2026-10-15-native-state-nodes/desktop-ctest.txt)、[NX CTest](assets/2026-10-15-native-state-nodes/nx-ctest.txt)、[汇总及源码哈希](assets/2026-10-15-native-state-nodes/validation-summary.json)、[资产来源](assets/2026-10-15-native-state-nodes/artifact-manifest.json)。完整外部归档为 `$AIMSRACER_DATA/experiments/native-state-port-20261015/`；NX 备份位于 `$HOME/aimsracer-native-state-port-20261015/backup-before-deploy.tgz`，两者不随 clone 提供。
