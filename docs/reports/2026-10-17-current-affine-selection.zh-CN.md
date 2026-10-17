# 2026-10-17：当前采用未补偿、带截距的区间电流模型

当前选型为 **`a_bar=b+k·mean(avg_iq)`，加速与减速分别拟合**。本次同步记录用户确认的模型形式和离线描述系数；车辆运行代码、NX安装及控制配置没有因此改变。

## 当前参数和数据范围

`I_bar` 是原始 `/sensors/core.state.avg_iq` 在同一源时间区间内的时间加权均值，单位A；`a_bar=[Vx(end)−Vx(start)]/T`，单位m/s²。Iq和加速度保留正负号。

| 分支 | 当前区间平均描述式 | 样本 | 实际平均Iq支持域 | 留一电流档RMSE |
|---|---|---|---|---|
| B，加速 | `a_bar=0.105147·I_bar−1.370669` | 请求23.74–68.74 A，共10档 | 23.660–68.169 A | 0.04186 m/s² |
| C，减速／滑行 | `a_bar=0.129365·I_bar−1.250321` | 请求−35、−30、…、0、+4.14 A，共9档 | −34.712–+4.173 A | 0.04959 m/s² |

这些参数采用冻结Vx在1–5 m/s支持域内的同区间拟合口径。B中13.74、18.74 A和C中+9.14 A没有完整响应，不计入；B≥73.74 A全部排除。C先按制动电流幅值≤45 A选出11档，再按此前固定的持续分离规则排除−40、−45 A。筛查阴性不等于已经证明零滑移，尤其C仍有短时轮速差异。完整四模型结果、定义及每档输入见重算报告（外部实验路径：`aimsracer-data/sessions/2026-10-16/stage-bc-analysis/low-current-refit-20261017/report.zh-CN.md`）和[区间特征](assets/2026-10-17-current-affine-selection/interval-features.csv)。

**B/C是采集阶段，不能仅按Iq正负号选择分支。** C的+4.14 A仍在减速。两条描述式的斜率差异是本批观测结果，尚未分离传动、电压、路面、瞬态及速度观测的作用，不解释为已经测得两套电机转矩常数。

## 为什么带截距有物理上的合理性

在PMSM的磁链和电感近似固定、d轴磁阻转矩项较小的条件下，q轴电流与电磁转矩近似成正比。这是局部近似的依据，本车的转矩常数未由本次拟合直接测得。[R1：MathWorks，Equations首式](https://www.mathworks.com/help/mcb/ref/pmsmtorqueestimator.html)

纵向牵引还需要抵消行驶阻力。Liniger等人的RC赛车模型在驱动力中显式保留常数阻力与速度平方阻力项；**原文输入是PWM占空比d**，可支持“驱动减去阻力”的结构解释，不能直接替代本车的Iq模型或证明其系数。[R2：作者公开稿，第5页，§II-A，式(2c)](https://arxiv.org/pdf/1711.07300#page=5)

据此，在直线、滑移较小且有效参数变化有限的条件下，可作如下本项目近似推导：

```text
m_eff·dv/dt ≈ K_F·Iq − F_res(v)
a_bar ≈ (K_F/m_eff)·I_bar − mean(F_res)/m_eff
      ≈ k·I_bar + b
```

`m_eff`可吸收转动惯量的等效作用；`b`是有限工况下的经验偏置，可包含平均阻力和未分离的观测／模型残差。它不是独立测出的摩擦系数，也不证明阻力始终为常数。上面的区间近似是本项目解释，文献未直接给出本车的两条拟合式。

## 为什么本批不再显式补basic曲线

四模型比较中的basic确实使用冻结Stage A曲线，逐时刻计算并积分：`U_bar=mean[Iq−Iq0(Vx)]`。若同类响应的`mean(Iq0)`近似为常数c，则`k·U_bar≈k·I_bar−k·c`，基本等价于一个截距；这是代数关系。

本批B十档平均basic只在11.98–12.19 A间变化，跨度约0.21 A，对应斜率0.105时约0.02 m/s²。因此，曲线虽随速度变化，整段平均后的区分度很小。**是否保留补偿由本车对比决定**：B带截距留出RMSE为0.04186，补偿加截距为0.04329；C筛查后分别为0.04959、0.05300 m/s²。不能据此断言其他速度段、场地或工况都不需要basic。证据见[同区间四模型指标](assets/2026-10-17-current-affine-selection/model-metrics.csv)。

当前公式的截距已经吸收了本批平均偏置，使用这组系数时不再先扣basic；否则输入定义和拟合时不一致。Stage A曲线仍保留为独立的匀速／阻力研究结果。

## 使用边界

- 目前采用的是有限域、区间平均描述，尚未验证瞬时加速度、电流反解、分支切换或闭环控制行为；不外推到高电流滑移区或静止／倒车。
- 加减速共用一条未补偿带截距直线，在筛查后19档上的留出RMSE为0.20393 m/s²；当前分别保存B/C系数。分别拟合共有4个参数，共用直线只有2个，不能宣称这是同等复杂度的独立比较。
- 这是同批逐电流档留出验证，每档主要只有一次完整响应。Vx与LIO共享传感器，尚无Vicon独立真值；小RMSE不等于物理精度或实车验收。
- 初轮总报告中`0.105497·I−1.386397`采用整段和样本算术均值，是历史口径；当前选型统一采用上表支持域裁切、时间加权的系数。

## 引用原文证据位置

| 引用 | 可直接定位的原文证据 | 支持范围 |
|---|---|---|
| R1，MathWorks，*PMSM Torque Estimator* | [Equations](https://www.mathworks.com/help/mcb/ref/pmsmtorqueestimator.html)：首式`Te=(3/2)p{ψm·Iq+(Ld−Lq)·Id·Iq}`。 | 在所述近似条件下解释转矩与Iq的局部线性；不是本车参数或模型精度证据。 |
| R2，Liniger、Domahidi、Morari，2015 | [作者公开稿PDF第5页，§II-A，式(2c)](https://arxiv.org/pdf/1711.07300#page=5)：`Fr,x=(Cm1−Cm2·vx)d−Cr−Cd·vx²`；紧随公式的文字含“a friction model for the rolling resistance and the drag”。[期刊DOI](https://doi.org/10.1002/oca.2123)。 | 支持驱动／阻力分项的建模思路；论文使用PWM输入，未证明本车采用截距更准。 |

R2期刊卷期为 *Optimal Control Applications and Methods*, 36(5), 628–647（2015）；在线首发为2014，作者公开稿上传于2017。上述页码按已核对的公开稿PDF计数。来源核查日期：2026-10-17。全部本车数值证据来自[重算指标](assets/2026-10-17-current-affine-selection/metrics.json)与[独立复算](assets/2026-10-17-current-affine-selection/numerical-verification.json)，与文献结构性解释分开。

本仓库保留[选型参数及来源](assets/2026-10-17-current-affine-selection/selected-model.json)、[引用定位记录](assets/2026-10-17-current-affine-selection/citation-evidence.json)和[BibTeX](assets/2026-10-17-current-affine-selection/references.bib)。原始包、完整重放与外部重算报告保留在相邻`aimsracer-data`目录；外部报告链接在仅克隆代码仓库时可能不存在。
