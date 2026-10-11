<a id="source-attribution"></a>
# 来源归属

**简体中文** · [English](NOTICE.md)

本文件是供参考的信息性中文翻译，不作法律解释。权威原文 [NOTICE.md](NOTICE.md) 保持原样保留；来源归属和许可应以原文及相应许可材料为准。

当前 acados 模型移植了现有本地 f1tenth-mpcc CasADi/IPOPT 适配器中的数学表达；该运行时已退役。车体角点约束、六状态转向滞后模型、20 ms RK4 子步、目标权重和平滑转向命令保留该适配器的数学表达。AIMSRacer 的修改提供普通本地导入、经过显式验证的车辆限制、非对称用户走廊、后轴状态输入、外部速度参考和重置支持。未导入 Isaac 或占据地图依赖。

早期版本从 local_mpcc 复制了 global_kinematic_model.py、contouring_lag.py 和 normalized_cost.py。这些文件已退役，其适配后的模型和归一化常数仍保留在 backend_models.py 与 cost_constants.py 中。track.py 保留其 wrap_s/PeriodicCubic 实现和原始来源归属，AIMSRacer 使用 PeriodicQuintic 扩展共享的 NumPy/CasADi 系数求值器。运行时参考采用 C4 quintic，在切向量非零的条件下，使依赖曲率的代价在参考结点和圈接缝处具有连续二阶导数。其 Alexander Liniger Apache-2.0 归属与上游 commit 保留在文件头中。local_mpcc 仓库许可为 LGPL-3.0；保留的适配内容仍受该许可约束。参见 aims_mpcc/vendor/alexliniger_MPCC_LICENSE、aims_mpcc/vendor/local_repository_LICENSE（LGPL-3.0）和 aims_mpcc/vendor/GPL-3.0_LICENSE（纳入的 GPL-3.0 条款）。这些许可文件随 vendor 包安装。来源适配器的 BSD-3-Clause 许可保留为 LICENSE；它不替代第三方许可。

新的路径准备使用该适配器的周期参考原则，并加入录制验证、后轴转换和有界重采样；未复制占据地图或骨架化实现。依赖 CasADi、acados、NumPy 和 SciPy 仍各自采用其许可。

来源快照：f1tenth-mpcc commit 283f124a83ebe95f37878cc2e721be428ae99cf4（以下复制文件 SHA-256 记录工作树内容）。

- `f1tenth_mpcc/solver.py`: `ff1a75fc7654d9f839ddd34bea691293906d326d2ebb373a9f03e9536e388024`
- `f1tenth_mpcc/track_tools.py`: `f006e0c7ab9ac33f688d9e03fe406929b48e30dc5ac9d5e84662733a70147580`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/global_kinematic_model.py`: `0244612f5e35b0c0796e80da7ea08c4e1c8ddbd2cbe76deb7d3885688928ddbb`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/contouring_lag.py`: `a016482d30875cdb049f0d65e9ab779ad70af9a3e840b8c8651ee42313323d00`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/normalized_cost.py`: `ac96edcb8269c16a688cccbebd8c1fd29937c06a84e8be107b8b5a220b944c42`
- `third_party/local_mpcc/scripts/barc_acados_mpcc/track.py`: `395789d90a18759f865775af6043a11b2cb7e1072daab67dbd5fe6eddb0c6201`

<a id="native-periodic-projector"></a>
## Native 周期投影器

`aims_mpcc/kernels/projector.c` 将 SciPy 1.15.3 的标量有界最小化器和 PPoly 多项式求值转换为 C，并采用 NumPy 1.26.4 的余数运算。公共运算已与 SciPy 1.8.0 / NumPy 1.21.5 核对。外部标量内核原型在 x86_64 和 aarch64 上针对各自版本组合进行了精确查询与执行门控比较；集成的实时几何包装器需要自身资格验证。这保留现有首次粗采样 argmin、xatol=1e-12 和最多 500 次调用的有界最小化器策略。不支持的库版本与自定义几何继续使用 Python 算法。

SciPy 版权所有：Enthought, Inc.（2001-2002）和 SciPy Developers（2003-2024）。NumPy 版权所有：NumPy Developers。两者的 BSD-3-Clause 再分发许可保留为 `aims_mpcc/vendor/SCIPY_LICENSE` 和 `aims_mpcc/vendor/NUMPY_LICENSE`，并作为包数据安装。Travis E. Oliphant 的原始 optimize.py 模块声明（允许复制/使用但不暗示任何保证，且须保留该声明）也保留在 C 文件头中。

主要来源：

- https://github.com/scipy/scipy/blob/v1.15.3/scipy/optimize/_optimize.py
- https://github.com/scipy/scipy/blob/v1.15.3/scipy/interpolate/_ppoly.pyx
- https://github.com/scipy/scipy/blob/v1.15.3/scipy/interpolate/_poly_common.pxi
- https://github.com/numpy/numpy/blob/v1.26.4/numpy/core/src/npymath/npy_math_internal.h.src

该库仅通过显式离线准备编译。运行时分派读取当前 PPoly 系数/结点表和现有粗采样几何，不改变其可写性，也不复制几何快照。编译器收缩、重新结合和 fast math 均禁用。此数值优化不改变控制器限制、物理验证或接受策略。
