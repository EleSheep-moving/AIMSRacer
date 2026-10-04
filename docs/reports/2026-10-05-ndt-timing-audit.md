# NDT 配准耗时调查与修正

## 结论范围

针对 A 第三次 `NDT P95=134.98 ms`，本次通过原速复现、配准阶段插桩和
冻结输入对照，定位到固定依赖 `ndt_omp_ros2` 的步长搜索入口条件错误。
修正入口后还暴露了旧搜索代码的端点数值边界问题；候选版发生 NaN 退出，
随后补齐边界处理并加入两项回归测试。候选失败日志保留，不作为最终验收。

这是桌面已知地图定位软件调查；没有定位真值、Orin 实测或车辆验收。

## 1. 135 ms 实际花在哪里

原历史三次 A 回放 P95 分别为 17.28、16.69、134.98 ms。此次先在未修改
算法的环境重跑一次，得到 16.60 ms，说明它并非每次运行都会发生。
随后对 `runAlignmentAttempt()` 加入仅用于调查的 steady-clock 分段计时、
进程 CPU 时间和迭代计数，三次原速回放得到：

| 调查回放 | 配准扫描数 | 达到迭代上限的扫描数 | 慢扫描核心耗时中位数 | 最大配准锁等待 | 最大状态锁等待 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 411 | 130 | 128.05 ms | 0.008 ms | 0.171 ms |
| 2 | 391 | 187 | 124.43 ms | 0.009 ms | 0.129 ms |
| 3 | 462 | 5 | 141.77 ms | 0.009 ms | 0.167 ms |

慢扫描实际执行 37 次迭代；配置为 35，但依赖在递增前用 `>` 判断退出，
因而多执行两次。前两组慢扫描的进程 CPU 时间中位数约 263、256 ms，
符合两条 OMP 线程持续计算。此次复现的 135 ms 主要消耗在 `align()` 内部。

上游公开的 alignment time 包括配准锁等待、`align()` 和状态锁重新获取，
不包括此前地图 crop 和其后的 fitness 计算。因此它不等于整个定位链路耗时。
插桩和点云快照会额外影响运行，最终验收使用重新构建的无插桩程序。

另外，上游在达到迭代上限时也设置 `converged_=true`。原日志中的 `ok`
不能证明优化在上限前收敛。此语义在本次补丁中未改变。

## 2. 冻结同一输入后的因果对照

保存一帧配准实际使用的过滤后 source、实际 target 和 4×4 initial guess，
两版使用同样的 2 线程、1 m resolution、0.1 m step、0.01 epsilon、35 次
配置上限、DIRECT7。每版独立启动，重复 21 次；第 0 次是冷启动，以下
暖态统计使用剩余 20 次，不通过降低迭代上限或放松接受门限减少耗时。

| 冻结输入版本 | 暖态中位数 | 暖态 P95 | 暖态最大值 | 迭代数 | fitness | 冷启动 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 原实现 | 128.95 ms | 130.04 ms | 130.25 ms | 37 | 0.0040866 | 303.83 ms |
| 最终搜索与边界修正 | 29.43 ms | 33.56 ms | 37.94 ms | 2 | 0.0023518 | 209.25 ms |

仅修正入口条件的中间版本在该帧上也得到 2 次迭代、约 29 ms；它在完整
程序中出现数值退出，不能仅凭单帧基准判定可以运行。最终修正仍有冷启动
约 209 ms，不保证每次配准都在 100 ms 内结束。

两版最终位姿相差约 5.85 cm / 0.639°。fitness 改善与独立地图一致性只能
说明本数据上的配准行为变化；没有外部真值，不能据此宣称绝对精度提升。
点云、初值、逐次位姿及计时保存于本地证据目录，SHA256 见 forensic-summary。

## 3. 源码修正及数值回归

固定依赖原代码使用：

```cpp
bool interval_converged = (step_max - step_min) > 0;
```

当前参数的区间宽度为 `0.1 - 0.005 > 0`，于是循环入口
`!interval_converged` 为 false，More–Thuente 搜索被跳过。
修正为 `< 0`，与[官方 PCL 实现](https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/impl/ndt.hpp)
一致。旧 koide3 上游也保留了此条件，本调查不将错误起源归因于某一维护者。

仅修正这一条件的候选在线回放发生 `radiusSearch` NaN assertion，已中止。
旧 `trialValueSelectionMT()` 在试探点与端点重合时会出现除零；两个直接调用
回归测试均得到 NaN。补齐 PCL 的边界处理：

- 三个端点重合时返回当前试探点。
- 试探点等于下端点时选择 case 4，避开对零宽区间进行插值。

两项测试修正前失败、修正后通过；原 public API 测试也通过，共 3 项 gtest。
`colcon test-result` 同时统计 CTest 包装项，汇总显示 4 tests / 0 failures。
最终 Humble Release 定位依赖构建成功。补丁包括实现修正及上述测试：
[ndt-line-search.patch](../../src/aims_racer_system/replay/ndt-line-search.patch)。

固定 commit 仍为 `63bf15b965b71d3a53db1757abe8e31b6114372a`；完整补丁 SHA256：
`58461a7c5f7508158bc4c02d823f5da83da9acae5e00bd7b7b52da95608838a6`。
环境安装脚本检查该 HEAD、完整补丁及未跟踪文件，拒绝额外依赖改动。
FAST-LIO、EKF、去畸变、配准参数和接受门限没有更改。

## 4. 监测节点自身的确认与公告延迟

完成优化器与边界修正后，先进行三次完整 A 回放，配准 P95 约 72 ms，
但三次可信观测年龄仍未通过 300 ms 门槛。继续调查发现监测节点只在
10 Hz tick 中核验并公告新锚点，最多增加 100 ms 等待，且一次 tick 之间
多个有效扫描只能公告最后一个。第一组中，NDT 诊断到达年龄 P95 为
163 ms，monitor 最终接受 streak 为 464，回放器却只收到 425 个不同锚点
公告；稳定阶段 rejection reason 全为 `ok`。这是一项额外的监测延迟。

修正为诊断、位姿、TF 或缺少的 EKF 到达后，立即执行原核验并公告结果。
10 Hz heartbeat 仍保留；不提前接受尚未匹配的扫描，不用 timer bridge
刷新测量源时间，不改变门槛、TF、优化器或局部状态。非 map→odom TF
不触发核验。回调与 tick 共享 epoch 检查，避免时钟回退时重复清空新锚点。

三项新增回归用例分别覆盖即时确认与 future timer 拒绝、odom-first 回退后
下一 tick 不得重复重置、新 epoch map TF 到达时必须先清空旧 pending。
新增用例修正前失败、修正后通过，全包测试 39 项 / 0 failures。
源码独立只读审查通过；最终耗时和完整链路结论仍以以下原速回放为准。

## 5. 最终完整程序原速回放

| 最终案例 | NDT P95 | NDT 最大值（冷启动） | 可信观测年龄 P95 | 新配准 / 可信记录 | 全部门槛 |
| --- | ---: | ---: | ---: | ---: | --- |
| A 第一次 | 77.62 ms | 278.61 ms | 252.12 ms | 465 / 465 | PASS |
| A 第二次 | 73.58 ms | 284.28 ms | 253.20 ms | 465 / 465 | PASS |
| A 第三次 | 74.69 ms | 254.37 ms | 253.49 ms | 465 / 465 | PASS |

三次 A 的独立 inlier 诊断中位数均约 99.35–99.36%；没有连续 lost 区间。
每轮只有初始化后的第一帧配准超过 100 ms，其余 464 次均低于 100 ms。
表中配准统计包含第一帧；观测年龄按原规则排除初始化后 2 s，并以每 50 ms
实际已收到的可信锚点计算，300 ms P95 门槛未改。

稳定运行的代价也需要保留：新的 P95 高于旧实现正常轮次的约 17 ms。
恢复搜索后会执行更多步长试探，不能宣称所有情况都更快；此次解决的是
旧实现有时持续跑迭代上限并落后的问题。桌面回放没有建立 Orin 算力余量。

B 回归仍未通过：NDT P95 为 108.07 ms，可信观测年龄 P95 为 3.420 s，
最长连续 lost 3.15 s；680 次配准诊断，521 次原生 `ok`，516 次可信记录。
其中 98 次 seed correction guard 拒绝、61 次 fitness 拒绝；完整旋转纠正
最大约 53.31°。独立 inlier 诊断中位数约 58.39%，全局定位不能据此验收。
用户已确认 B 有手动平移，该运动不能仅由轮速预测；本次没有证明它是 B
全部退化的唯一原因，也未放松门槛将 B 改为 PASS。

实际 ROS 节点回归测试另跑 2 项 / 0 failures，覆盖监测构造、诊断/位姿/TF
到达顺序、bridge 不刷新观测，以及去畸变完整覆盖/缺覆盖丢弃。

地图相同，A 每次重新启动完整定位图，原始输入 1× 播放、雷达约 10 Hz；
每次只用历史地图位姿初始化一次，后续不播放历史 LIO/TF。程序使用原生
三线程 executor、独立 timer callback group 和两条 OMP 线程。测试期间
本调查没有并行编译或启动 CPU 基准。主机 CPU 未隔离，不能据此宣称
最坏系统负载下的实时保证。

新结果只代表本 A 数据下持续慢配准问题的验证。地图观测新鲜度仍包含雷达
采样、去畸变、配准和诊断到达时间，不能用配准 P95 替代完整链路延迟。

## 6. 证据与复现

所有原始失败与中间候选结果保留在 `log/wheel-imu-ndt/timing-audit/`：

- `baseline-A-r1`：未修改算法的完整原速重跑。
- `diagnostic-A-r1..3`：阶段插桩与迭代计数调查。
- `capture/{source.pcd,target.pcd,seed.txt}`：冻结输入；`frozen-*.jsonl`：逐次计时与位姿。
- `fixed-A-r1`：仅修正入口的失败候选，已中止。
- `cold-diagnostic-A`：限时 20 s 的快照调查片段，不作为验收。
- `full-fix-A-r1..3`、`full-fix-B-r1`：修正优化器后、监测仍等待 timer 的中间回放。
- `final-A-r1..3`、`final-B-r1`：最终优化器和即时核验监测的完整回放，summary 与 provenance。
- [forensic-summary.json](../../log/wheel-imu-ndt/timing-audit/forensic-summary.json)、
  [final-audit.json](../../log/wheel-imu-ndt/timing-audit/final-audit.json)：冻结对照及来源汇总。
- [timing-comparison.png](../../log/wheel-imu-ndt/timing-audit/timing-comparison.png)：历史慢回放与最终三次 A 对照。

构建和数值测试日志位于 `log/wheel-imu-ndt/` 的
`timing-edge-red-results.log`、`timing-full-fixed-final-build-test.log`；
监测和全包验证见 `timing-monitor-red.log`、`timing-monitor-epoch-red.log`、
`timing-final-adapter-build-test.log`、`timing-final-runtime-test.log`。
原始历史报告数据保留，参见[之前的回放报告](2026-10-05-wheel-imu-ndt.md)。

最终回放命令与[操作说明](../operations/wheel-imu-ndt.md)相同，仅使用新的
输出目录。数值回归验证需以固定补丁后的 NDT 依赖运行：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
cd /ws
colcon build --base-paths /deps/ndt_omp_ros2 --packages-select ndt_omp_ros2 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths /deps/ndt_omp_ros2 --packages-select ndt_omp_ros2 \
  --ctest-args -R test_public_api --output-on-failure
colcon test-result --test-result-base build/ndt_omp_ros2 --verbose
```

C 测试仍按用户要求取消，当前不安排实车启动、静止观察或控制器测试。
