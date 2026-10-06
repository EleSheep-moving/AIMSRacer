# NX 剩余工作目录修订与同步（2026-10-06）

## 范围

按用户要求，将 NX 原 AIMSRacer 的剩余有效修订提交并与桌面、GitHub 的同名分支同步。
操作基线：`feat/aims-mpcc` 的 `0c86e4f` 和 `feat/fastlio-ndt-mpcc` 的 `ed9b861`。
保留两个分支各自的定位方案；本次不合并到 main。

## 保存与修正

- `.gitignore` 增加 `/log`，兼容 NX 日志目录指向外部数据目录的符号链接。
- 修正硬件独立的 `aims_mpcc/integration.py` 测试工具：使用现有 C++ `joystick_control_v2` 与显式通道 profile，移除已废弃的 output_mode；manual/rc_loss 验收按“选择器撤销驱动权限，控制器仍计算”判断。
- 保存原来未跟踪的 C++ selector 与 rear IMU 回归、合成 golden CSV 和 IMU benchmark 源文件；为两个包注册 gtest，并为 selector 注册隔离 ROS pytest，补齐测试依赖。
- 保存地图坐标转换/身份、动力学对齐和计划过期回归。
- 原 MPCC 的后轴合同测试不再导入已经删除的 Python IMU 实现，C++ 测试保留力补偿和重力检查；集成分支保留既有共享 gyro 接线检查。
- 两个分支使用相同的后轴合成测试源码，ERPM 比例读取实际参数。集成分支先通过默认生产参数的 10 秒静止标定，再进入合成转弯；加入 0.02 rad/s 合成 raw gyro 偏置，核对输出保留真实 0.4 rad/s 转弯。

除上述测试工具外，没有改变实车估计或控制算法。本轮保持集成分支的 EKF Q(vx)=0.4、50 次迭代设置和 gyro 门槛；原 MPCC 分支保留其已有配置。两个分支的 MPCC 权重、外参和 FAST-LIO 源码均未变化。IMU benchmark 本轮只编译，不宣称新的性能结果。

## 验证

所有 ROS 测试使用 localhost 的独立非零域，不启动硬件驱动，不向实车 ROS 图发布控制指令。
两个工作区分别重建 aims_racer_system、ackermann_mux、aims_mpcc，均成功。

| 验证 | 原 MPCC 分支 | FAST-LIO + NDT 集成分支 |
|---|---|---|
| 后轴 C++ 回归 | 6 个 gtest case 通过 | 同样 6 个通过，既有力补偿测试也通过 |
| Selector C++ 回归 | 12 个 gtest case 通过 | 同样 12 个通过 |
| Selector 隔离 ROS 回归 | 2 个 pytest case 通过，含冻结仿真时钟 | 同样 2 个通过 |
| 后轴隔离 ROS 全链路 | 2 个通过，约 16.57 s | 2 个通过，约 36.62 s，包含生产零偏初始化 |
| 地图/计划过期/native cache 回归 | 13 个通过，约 4.29 s | 全部 controller/tests：21 个通过，约 4.27 s |
| 集成现有 gyro / localization policy / monitor 等回归 | 不属于该分支方案 | 与新增后轴 C++ 测试合计，colcon 报 52 项、0 错误、0 失败；selector 独立报 16 项、0 错误、0 失败 |

原集成分支的合成测试修改前两个 case 都因 raw IMU 订阅数量变化而 discovery 超时；日志保留为 red-pipeline.log。适配新的接线和初始化后通过，不通过降低生产标定门槛绕过。

colcon 测试选择受影响的功能项目；本轮没有运行所有包的历史 lint 项目或实车闭环。
构建有既有 underlay/ament/CMake 提示，退出成功。
原 MPCC build 目录含历史测试 XML，不能将其全目录 test-result 数字当成本轮唯一 case 总数；本表按本次项目日志与测试输出记录。

## FAST-LIO 本机文件与备份

原子模块仅有三个未跟踪文件：两个本机测试源码和一个生成的 pyc，没有 tracked 源码变化。
已先备份全部未提交文件与 diff，再逐文件核对 SHA256，移出子模块至外部证据目录。
子模块仍固定为 `30bc305b4240369879c346398ac6e0a1ea5ed420`，其 working tree 干净。
本轮未运行这些归档的 FAST-LIO 测试，不把它们计入验收。

证据目录：

`/home/aims/aimsracer-data/sessions/2026-10-06/nx-workspace-sync`

包括 before-sync.tar.gz、before-sync.patch、before-sync-sha256.json、implementation-plan.md、
archived-fastlio-local-tests/、构建/回归日志、pytest 临时输出与 solver-cache/。
原始 bag、其他实验和旧目录均保留；数据及编译产物不加入 Git。

## 发布与使用

本轮分别提交、推送 MPCC 与集成分支，并核对桌面、NX、GitHub 的同名分支 HEAD。
公共修订文件逐一比对内容；分支特有的初始化、定位门控和配置保留。
代码直接来自当前 NX 工作目录；没有以桌面旧文件覆盖 NX 修订。
具体发布提交以 Git 历史为准。正式 merge main 后续执行。

实际车载测试仍以 `/home/aims/AIMSRacer-fastlio-ndt` 集成 overlay 为入口。
本轮验证说明源码和测试工具已对齐，不代表落地行驶、摆动消除、多圈定位或温漂验收。
