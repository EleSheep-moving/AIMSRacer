<a id="source-and-dependency-attribution"></a>
# 来源与依赖归属

**简体中文** · [English](NOTICE.md)

本文件是供参考的信息性中文翻译，不作法律解释。权威原文 [NOTICE.md](NOTICE.md) 保持原样保留；来源归属和许可应以原文及相应许可材料为准。

C++ 运行时是 AIMSRacer 对后轴运动学控制器和 ROS 接口的实现。其生成求解器导出现有 `aims_mpcc` CasADi 模型、目标与约束；生成过程不替代原始来源归属或许可。原始建模 NOTICE 以及 BSD-3-Clause、Apache-2.0 和 LGPL-3.0 许可材料保留在 `licenses/` 中。精确 quintic 参考系数和已准备的速度曲线从同一个 AIMSRacer NumPy/SciPy 表示导出。

acados 运行时固定为 v0.5.5，commit `59d93e17d2985fdd73fc58b8a83ed8f83a024171`。acados 和 HPIPM 使用 BSD-2-Clause；BLASFEO 使用 BSD-2-Clause。其上游来源与依赖归属仍为权威依据。CasADi 仍是离线建模/代码生成依赖。YAML-CPP 与 OpenSSL 是动态链接的系统依赖。

求解器/发布器分离、生成 C 部署和有界 RTI 策略的调研来源为 `npu-ius-lab/Roboracer_China_2026`，分支 `real-car-original`，commit `6c5012b9cd310c8fca5281c408298ffb5d4b3885`。NPU ROS 1 节点、动态自行车模型、车辆参数和残差模型未复制到本包。原始源码冒烟结果与本包自身车辆模型及 ROS 2 验证分别记录。

每个准备好的 bundle 包含原始源文件、source/native 哈希、依赖哈希、配置/参考身份和本来源归属说明。
