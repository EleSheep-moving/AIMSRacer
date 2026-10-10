# 开发验证与重放

本目录保留当前控制器与定位的开发验证工具和参考配置。它不作为 ROS 生产包安装，车辆运行使用[当前启动指南](../docs/operations/bringup.md)。历史实验代码可通过 Git `5ad071c` 及日期化[报告](../docs/reports/README.md)查阅，不向车辆恢复已移除的实验依赖。

| 内容 | 用途 / 边界 |
| --- | --- |
| [mpcc/replay_requests.cpp](mpcc/replay_requests.cpp) | 已记录请求的独立冷启动重放，要求 bundle/config/N/dt 匹配；不重建原 warm-start 历史 |
| [mpcc/replay_runtime.cpp](mpcc/replay_runtime.cpp) | 有顺序的请求重放，比较 frozen/refresh 几何策略；需要匹配 bundle 与捕获 JSON |
| [localization/fastlio_ndt_replay.py](localization/fastlio_ndt_replay.py) | raw Livox bag 上的 FAST-LIO2/EKF/NDT 隔离重放；不是实车控制反馈 |
| [localization/evaluate_replay.py](localization/evaluate_replay.py) | 评估已收集 replay 证据，将持续 tracking 与安全失去资格分别判断 |
| [localization/tf_authority_audit.cpp](localization/tf_authority_audit.cpp) | 记录 TF 发布者身份与源/接收时间，检查唯一归属 |
| [config](config/) | 开发比较所需的配置快照；生产默认配置在 `src/controller/config/vehicle.yaml` |

包内仍保留当前测试：

- [controller/tests](../src/controller/tests/)：参考、模型、速度规划、bundle 和约束检查。
- [aims_mpcc_rt/tests](../src/aims_mpcc_rt/tests/)：原生数值、执行、时间、生命周期和定位资格检查。
- [aims_racer_system/tests](../src/aims_racer_system/tests/)：后轴转换、gyro、定位协议和 launch 合约。
- [tools/tests](../tools/tests/)：固定依赖准备行为。

原生包当前 Python 检查保留 `test_export.py`、`test_fractional_interval.py`、`test_node_startup.py`，以及现役 C++ 数值/执行测试；依赖已移除实验工具的旧 Python 检查不恢复到生产。

测试按源码/CMake 的测试注册执行，数值项需要显式准备的合适 bundle 和 acados 动态库环境。重放使用原始 lidar/IMU 与录制 QoS，在隔离 ROS domain 中进行，输出到新证据目录。先查看参数：

```bash
python3 verification/localization/fastlio_ndt_replay.py --help
python3 verification/localization/evaluate_replay.py --help
```

重放脚本使用仓库内 [replay launch](localization/launch/fastlio_ndt_replay.launch.py)的绝对路径启动 ROS，而非已安装生产 launch。TF 审计程序显式来自开发 build：

```bash
python3 verification/localization/fastlio_ndt_replay.py \
  /absolute/path/to/raw-bag /absolute/path/to/new-evidence \
  --audit-executable "$PWD/build/aims_racer_system/tf_authority_audit" \
  --map /absolute/path/to/map.pcd --seed /absolute/path/to/seed.json
```

`seed.json` 包含 map/base_link 的 `position: [x,y,z]` 与 `orientation: [qx,qy,qz,qw]`，按实际摆放填写；指定输入必须实际存在。

检查 source/config、构建、软件运行、联合负载和现场表现时分别报告证据。软件通过不证明未试过的路面抓地或更高速度；实跑记录的运动窗口也不是 flying lap。完整边界见[2026-10-10 外场复盘](../docs/reports/2026-10-10-mpcc-field-review.md)。
