# 2026-10-13 PP、纵向标定与参考编辑器

[English](2026-10-13-pp-calibration-reference-editor.md)

## 交付内容与版本

功能分支为 `feat/pp-calibration-reference-editor`，基于场地 main
`098976b93ef14696aaf0bf229eaffc062b787b5c`。运行与性能采样版本为
`e0261439a5eb1753d9715a5fa041578ba3c19d14`；后续提交只更新文档。
主机与 NX 工作区分别为 `/home/elesheep`、`/home/aims` 下的
`aimsracer-data/code/pp-calibration-reference-editor`。main 未合并，本次没有真实电机动作。

- 离线 FastAPI/Three.js 编辑器：原始 PCD 身份、后轴 XY 闭合轨迹、高度过滤、
  3D 查看、撤销重做、工程保存、曲线/速度预览、统一参考导出与哈希绑定标定区间。
  与 native/PP 使用相同五次曲线序列化，不证明地图上的通行空间。
- 独立 `aims_pp`：恢复旧增强 PP 数值公式，使用实际几何、统一参考与可信定位，
  明确启用，不需要生成求解器。
- 独立 `aims_calibration`：稳速、正电流区间、负电流减速；PP 核心或遥控转向；
  检查真实选中回显、唯一物理样本、实际 `current_motor`，记录无效试验并后台写日志。
- 整车 launch 选择一个竞速控制器，或启动独立标定链。仅标定入口关闭驱动起步
  电流替代，保留最终 ±100 A 与现有 MPCC/FAST-LIO/EKF/NDT 参数。
  录包覆盖原始 `/calib/vesc_command`、最终指令和真实选中权限。
- 补齐中英文操作指南、包说明、架构与索引。

## 软件验证

NX 编译了六个功能包，复用未改动的 main underlay。七个 PP/标定 Python 模块和
五份生产参数的源码与安装文件一致。FAST-LIO 子模块保持
`afe5f4c5761d0fdf536e535df2a32017c50962a5`，VESC 保持
`e3fa74c912f0f322b6aa8dc5218f5b8a662eb74c`，使用 main 已验证的安装。

- NX **99 项通过**：PP 32、标定 58、新 launch 合约 7、回放统计 2。包含四项
  实际安装节点 → 选择器 → VESC 驱动 → 虚拟串口测试，检查带符号线缆电流及权限撤销。
- NX **25 项编辑器/参考/速度规划测试**、**4 项可移植 native 测试**通过。
- 桌面选定 ROS 包的完整回归通过；评审修复后 PP 32、标定 58 通过，launch 19 项通过。
  Chromium 操作测试通过，覆盖导出响应延迟时编辑轨迹的情况。
- 实际原图 **990,482 点**，SHA 为
  `1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc`。
  显示限制为 200,000 点；NX 编译期间一次地图/API 加载为 2.12 秒，这是冒烟采样，
  不是耗时分布。
- 定向回归覆盖首次电流前已越界、边界时缺少新遥测、时钟重新启用、首次电流确认后的
  采样、日志溢出、CSV 失败撤销成功 JSON，以及过期 GUI 下载结果。

## NX 联合负载采样

原始 Livox 回放同时运行 FAST-LIO、后轴适配、EKF、NDT。不启动真实驱动，使用虚拟
遥控权限与原录包电机遥测检查选择器/标定数据流。播放器启动后执行 60 秒墙钟回放，
包括 5 秒发现延迟。所用 2026-10-10 包长 76.35 秒；轮速超过 0.2 m/s 的运动区间
在包起点后 15.66–37.09 秒，实测峰值 2.88 m/s，当前回放覆盖了这个运动区间。
录包轮速约 50 Hz，不等于当前配置的 100 Hz。

| 节点 | 有效样本 | 回调 ms：P50 / P95 / 最大 | 输出间隔 ms：P50 / P95 / 最大 |
| --- | ---: | ---: | ---: |
| PP | 276 | 4.35 / 5.06 / 6.10 | 20.00 / 20.67 / 24.13 |
| Calibration / 标定 | 299 | 3.98 / 4.71 / 5.93 | 19.98 / 20.36 / 21.04 |

PP 只统计 RUNNING 的输出回调，不掺入传感器回调；标定排除 IDLE、STOPPING、FAULT、
COMPLETE。耗时采用单调时钟，资格检查使用源/ROS 时间戳及接收新鲜度。P95 为样本
百分位，不保证最坏情况。有效回调均小于 20 ms；输出间隔的抖动单独报告，略高于
20 ms 不等于求解超时。

**以上是多个有效片段汇总，不是连续跟踪验收。** 两组采样各用了 40 次故障后的显式
虚拟重新启用，原来的时间门控仍生效。TF 审计中 11,088 个 odom 变换有 163 个出现
负的源/接收 ROS 时间差，最低 −5.23 ms；monitor 同时记录 `invalid_ekf_stamp`，
PP 记录过期/未来 odometry 停机。这一现象与回放时各节点异步接收模拟时钟的顺序相符，不能
直接证明物理观测来自未来或实车出现同类延迟。连续回放资格仍受该行为限制。
修复后不重新启用的首次 PP 回放仅 59 个有效回调，标定仅 182 个，随后故障退出。
生产节点没有新增自动恢复。

首次 PP 回放有效样本为零：rosbag 发现阶段会回拨时钟，PP 清掉了只发布一次的静态
地图身份。已修复为仅保留此前确认匹配的静态 SHA，动态定位、地图有效性、位姿、权限
与指令仍清空，仍需要明确启用和新鲜合格输入。

原录包车辆响应来自旧指令，不能验证新电流指令或当作物理标定。全圈采样区间是调度
测试构造的，不是认可的实车直线。合并 main 前仍须低速 PP、小电流标定和停止/遥控
接管的实车验收。

## 复现与证据


```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
python3 verification/calibration/replay_runtime.py \
  --bag /home/aims/aimsracer-data/sessions/2026-10-10/mpcc-acados/192955_v35_n15_accel1_cap4_lap01/bag \
  --map /home/aims/maps/20260928_010503/map.pcd \
  --seed /home/aims/aimsracer-data/sessions/2026-10-13/pp-calibration-editor/seed.json \
  --bundle /home/aims/aimsracer-data/bundles/field-v35-20261010 \
  --output /absolute/new-output --mode pp --seconds 60 --domain 223 --timing-rearms 40
```

标定改用 `--mode calibration --domain 224`，输出目录必须是新的。
`--timing-rearms` 只属于隔离测试脚本，默认零。

NX 证据位于 `/home/aims/aimsracer-data/sessions/2026-10-13/pp-calibration-editor`；
主机证据位于 `/home/elesheep/aimsracer-data/sessions/2026-10-13/pp-calibration-editor`，
其中 `nx-evidence/` 为 NX 副本。查看 `software-tests.xml`、
`source-install-provenance.json`、各组 `summary.json`、控制 CSV、定位事件与 TF 审计。
这些生成文件留在本地，新 Git clone 不包含它们。完整编辑器与实车启动流程见
[操作指南](../operations/pp-calibration.zh-CN.md)。
