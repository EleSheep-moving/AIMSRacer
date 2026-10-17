# 恒速 q 轴电流、电压与速度

[English](2026-10-13-steady-current-voltage-model.md) · **简体中文**

2026-10-14 修正：撤回旧版以 `current_motor` 近似转矩和建立保持电流基准的系数。原始 bag、遥测和 A 采集记录保留；本文和后续 B/C 基准统一使用实际 **`avg_iq`**。电池输入功率仍由 `voltage_input × current_input` 计算。

## 1. 电流口径

| `/sensors/core.state` 字段 | 用途 |
| --- | --- |
| `avg_iq` | 有符号 q 轴电流，近似转矩，A/B/C 实际电流统计与跟踪 |
| `avg_id` | d 轴电流，用于检查运行区间与弱磁 |
| `current_motor` | 独立电机电流遥测，仅保留诊断，不代替 Iq |
| `current_input` | 电池侧 VESC 输入电流，用于功率和能量 |
| `voltage_input` | VESC 输入端电压，用于测量条件与电压能力判断 |

Iq 是旋转坐标系中的电流分量，不等于任意定义的相电流 RMS。所查 VESC FOC 源码中，电流命令设置 q 轴目标；电机平均电流与 Iq 平均值使用独立遥测路径。实车固件设置仍决定实际限幅。依据：[FOC 实现](https://github.com/vedderb/bldc/blob/master/motor/mcpwm_foc.c)、[遥测统计](https://github.com/vedderb/bldc/blob/master/motor/mc_interface.c)、[原生命令契约](../vesc-command.zh-CN.md)。

## 2. 为什么 Iq、速度、电压不能混成一个关系

PMSM 转矩为：

\[
T_e=\frac32p\left[\psi_m I_q+(L_d-L_q)I_dI_q\right].
\]

磁阻转矩项较小时，转矩近似正比于 Iq。这是有条件的近似，尚未测出本车的转矩常数。恒速时，转矩用于抵消滚阻、风阻、传动损耗、坡度和转弯负载；所以保持电流首先由速度和负载决定。依据：[转矩公式](https://www.mathworks.com/help/mcb/ref/pmsmtorqueestimator.html)。

电压约束是另一个问题：

\[
v_d=R_sI_d+L_d\dot I_d-\omega_eL_qI_q,
\qquad v_q=R_sI_q+L_q\dot I_q+\omega_e(L_dI_d+\psi_m).
\]

速度越高，反电动势越大。电压余量充足时，母线电压下降，电调可以提高调制比例，继续维持同样 Iq；接近电压饱和时，目标 Iq 可能跟不上。弱磁采用负 Id，又会改变可用电流预算。因此应分开描述：

- 需求：`Iq0(速度, 负载)`，维持车速需要多少转矩。
- 能力：`Iq_max(速度, 电压, 温度, 限幅)`，当前状态下最多能实现多少转矩。

**不能把恒速 Iq 默认按 `1/V` 放大。** 依据：[电压限制与弱磁](https://www.mathworks.com/help/mcb/gs/field-weakening-control-mtpa-pmsm.html)。

纵向运动可在局部近似为 `m_eff × a = K_w × Iq − F_res(v)`，其中 `K_w` 包含传动比、轮胎半径和效率。因此正 Iq 低于当前保持基准，也可以使车减速；零 Iq 是滑行，负 Iq 对前进产生反向转矩。负转矩不保证电池能无限回充，仍受损耗及电池/母线电压限制。请求 Iq 与实际达到的 Iq 必须分开。

电池侧电流则与功率有关：相同速度、相同负载，机械功率和损耗接近；电压降低时，电池输入电流可能增加，而 Iq 基本不变。本文逐条计算 `P_k=V_k×I_input,k`，再求平均；不能默认均值乘积等于乘积均值。这是电调输入功率，不是 NX、雷达等支路在内的整车总功率。

## 3. 原始数据重算

五个有效 bag 组成三组完整的 1–5 m/s 扫描：轻电池三个分段包、重电池两次完整包。共 **27 个成功试次、57 个有效直线段、6,733 个样本、135.258 秒有效时间**；另一个失败试次不参与拟合。准备包和异常会话保留但不参与拟合。

严格沿用原始轮速与遥测源时间边界，在原始遥测中提取 Iq/Id，无需把电机电流转换成 Iq。核验 168 个证据文件和五份原始提取文件的哈希，重建样本数及原始电机电流统计完全一致。这证明纳入的样本一致，不证明两种电流相等。

![q轴电流、电压、输入电流与功率](assets/2026-10-13-steady-current/current_voltage_comparison.png)

5 m/s 对照：

| 扫描 | 平均电压 V | 平均 Iq A | 平均输入电流 A | 逐条计算的平均输入功率 W |
| --- | ---: | ---: | ---: | ---: |
| 轻电池补测 | 14.798 | 14.215 | 3.148 | 46.577 |
| 重电池 21:53 | 16.526 | 14.080 | 2.782 | 45.969 |
| 重电池 22:29 | 15.900 | 14.123 | 2.899 | 46.099 |

同一重电池两次扫描中，Iq 变化 **−0.48%～+1.29%**，输入电流增加 **2.94%～5.10%**，输入功率变化 **−0.73%～+1.33%**。这是描述性对照；温度、路段、速度误差、质量以及按升速顺序采集的时间变化，妨碍把差异全部归因于电压。

重电池重复扫描各档平均 Id 约为 −0.0085～+0.0203 A，回传 duty 约为 0.062～0.243。这些有效恒速段未显示明显平均弱磁；并不证明能实现 100 A，也不能直接确定通用饱和阈值。

![同一重电池对照](assets/2026-10-13-steady-current/heavy_battery_voltage_effect.png)

## 4. 拟合与完整扫描留组验证

对 27 个试次均值等权拟合，使用实际轮速；每次用两组训练，预测第三组，轮流留出三组。相关遥测消息不能算作几千次独立试验。用于选模型的这三组验证，也不是新的独立最终验收。

| Iq 模型 | 留组 RMSE A |
| --- | ---: |
| 速度一次式 | 0.323 |
| 速度二次式 | 0.172 |
| 速度三次式 | **0.156** |
| 二次式＋电压 | 0.200 |
| 二次式＋电压＋交互项 | 0.234 |
| 二次分子除以电压 | 0.700 |

研究 `Iq0(v)` 时，当前速度一元候选中完整扫描留组误差最低的是三次式：

\[
I_{q,0}(v)\approx5.775448+3.513670v-0.594009v^2+0.0451659v^3\quad[\mathrm A].
\]

仅适用于标称 1–5 m/s 的已测行驶条件。训练 RMSE 为 0.114 A，留组 RMSE 为 0.156 A，留组最大绝对误差为 0.348 A；曲线在此范围保持单调递增。这是经验拟合，不是已验收的控制器前馈，也不是独立地面真值。5.775 A 截距属于外推，不能解释为静止所需电流。

![Iq0(v)拟合曲线与实测试次均值](assets/2026-10-13-steady-current/curve-fit-iq-v2/iq0_speed_curve.png)

[系数与验证](assets/2026-10-13-steady-current/curve-fit-iq-v2/iq0_speed_fit.json) · [速度档位预测](assets/2026-10-13-steady-current/curve-fit-iq-v2/iq0_speed_predictions.csv) · 复现源码（原始分析附件留在外部实验目录：`docs/reports/assets/2026-10-13-steady-current/curve-fit-iq-v2/fit_iq0_curve.py`） · [SVG](assets/2026-10-13-steady-current/curve-fit-iq-v2/iq0_speed_curve.svg)

一个便于查看的暂定二次描述式为：

\[
I_{q,0}(v)\approx6.60464+2.41870v-0.185854v^2\quad[\mathrm A],\quad1\le v\le5\ \mathrm{m/s}.
\]

三次式留组误差更小；二次式仅作为紧凑参考，不能称为验证最优。**运行期 B/C 使用实测基准点，不使用这个多项式。** 电压项在当前数据中没有改善留组预测，不等于证明电压永远不影响 Iq。

重新核验后，电池输入电流的经验式保持不变：

\[
I_{\rm input}(v,V)\approx\frac{-0.516192+6.918272v+0.476891v^2}{V}\quad[\mathrm A].
\]

留组 RMSE 为 0.02016 A。此式依据试次均值，限于已测行驶条件及约 14.8–16.8 V；不是静止、起步、制动或通用电池模型。负截距的外推结果没有怠速功率含义。

![模型对照与q轴预测误差](assets/2026-10-13-steady-current/curve-fit-iq-v2/model_comparison.png)

## 5. 对 B/C 的影响及复现

选用的重电池重复扫描得到 `Iq0(1.5)=9.98185 A`。B 从 `Iq0+5` 开始，每 5 A 递增至 100 A；C 测严格为正的 `Iq0−5`、`Iq0−10` 等，再测零电流与负电流。修正后为 **B 146 项、C 183 项**，尚未扣除跳档。缺失 Iq 视为证据不可用，不能用电机电流、电池电流或目标值替代。操作见 [B/C 手册](../../src/aims_racer_system/docs/calibration.zh-CN.md)。

旧电机电流基准被拒绝。修正 A 文件另存并携带原始结果与原始提取文件哈希，不修改原 A 记录。旁边保留原 observations CSV，用于电压及源时间核验，历史电机电流列没有改名成 Iq。新采集分别保存两种电流。

查看 [有效试次](assets/2026-10-13-steady-current/accepted_trials.csv)、[重建核验](assets/2026-10-13-steady-current/validation.json)、[模型指标](assets/2026-10-13-steady-current/curve-fit-iq-v2/model_metrics.csv)、已执行 notebook（原始分析附件留在外部实验目录：`docs/reports/assets/2026-10-13-steady-current/curve-fit-iq-v2/steady_iq_curve_fit.ipynb`）、拟合源码（原始分析附件留在外部实验目录：`docs/reports/assets/2026-10-13-steady-current/curve-fit-iq-v2/fit_models.py`） 和 重建源码（原始分析附件留在外部实验目录：`docs/reports/assets/2026-10-13-steady-current/analyze_iq.py`）。

主机原始证据：`/home/elesheep/aimsracer-data/sessions/2026-10-13/constant-speed-current-review`；新派生证据：`/home/elesheep/aimsracer-data/sessions/2026-10-14/iq-current-correction/recomputed`。原始 bag/NPZ 属于外部数据，新克隆仓库不包含它们。复现命令与英文页一致。多项式电流前馈和 B/C 实车验收仍未完成。

[返回电流标定入口](../operations/calibration.zh-CN.md) · [横向加速度标定与校对](../operations/lateral-acceleration-calibration.zh-CN.md)。本报告的日期、数值与证据边界保持原口径。
