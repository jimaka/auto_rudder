## 舵减摇与航向-横摇联合自适应控制

舵减摇（Rudder Roll Stabilization, RRS）利用同一副舵在执行航向保持的同时产生横摇稳定力矩，实现"一舵两用"：在低频段舵用于艏摇/航向控制，在横摇固有频率附近舵用于横摇阻尼。该技术自 1970 年代提出以来，已在荷兰皇家海军 M 级护卫舰、美国海岸警卫队巡逻舰、丹麦海军 SF300 级等多型实船上验证，对不宜或不便安装减摇鳍的高速舰艇与游艇具有特别的吸引力——25 kn 级游艇舵效随速度平方增长，恰好是 RRS 最有利的工况。本章先梳理舵-横摇耦合机理与频带分离条件，再按年代梳理代表性文献，进而讨论参数摄动下的自适应 RRS 设计、与航向保持的冲突权衡、高海况限制，最后给出对本项目（PD + 增益调度 + ESC 在线调参 + 极点配置基线）的具体结合建议。

### 舵减摇原理

#### 舵-横摇耦合机理

舵在水下产生的侧向升力 $Y_\delta$ 作用点位于重心（CG）下方，垂向距离为 $l_r$（舵压心到重心的力臂），因此舵力同时产生艏摇力矩 $N_\delta$ 与横摇力矩 $K_\delta = -l_r Y_\delta$。回转中的横倾（turn heel）即这一耦合的直接表现。线性化后，舵角到横摇角与艏摇角的通道可写成：

$$\frac{\varphi(s)}{\delta(s)}=\frac{K_{\varphi\delta}(1+T_z s)}{(1+T_4 s)(s^2+2\zeta_\varphi\omega_\varphi s+\omega_\varphi^2)},\qquad \frac{\psi(s)}{\delta(s)}=\frac{K(1+T_3 s)}{s(1+T_1 s)(1+T_2 s)}$$

其中 $\omega_\varphi$ 为横摇固有频率、$\zeta_\varphi$ 为横摇阻尼比（裸船体典型仅 0.05–0.15，这是横摇在遭遇频率共振时幅值剧增的原因）。RRS 的本质是：在 $\omega_\varphi$ 附近用舵提供**人工横摇阻尼**（反馈 $\varphi$、$\dot\varphi$），等效提高 $\zeta_\varphi$；而在远低于 $\omega_\varphi$ 的频段，舵仍按常规方式控制艏向。

#### 非最小相位特性

舵-横摇传递函数通常含**右半平面零点**（上式中 $T_z<0$，或由横荡-艏摇-横摇三自由度耦合产生）：打舵瞬间，舵力的横摇分量先使船向"错误方向"横倾，随后回转诱导的横倾才反向建立。van der Klugt 的博士论文系统分析了这一非最小相位特性，它是 RRS 可达性能的根本限制——按 Freudenberg–Looze 型灵敏度积分约束，右半平面零点限制了横摇反馈通道可用的带宽与灵敏度峰值，减摇效果存在与舵机速率饱和共同决定的**理论上限**，这一认识在 Perez & Blanke 2012 的综述中被概括为"实船装船后普遍观察到减摇效果存在上限"的解释 [1][6]。

#### 频率条件与舵机带宽要求

RRS 可行的前提是**频带分离**（图 10-1）：艏摇/航向控制的闭环带宽必须显著低于横摇固有频率，使两条通道近似解耦：

$$\omega_{yaw}\ \ll\ \omega_\varphi\ \lesssim\ \omega_e\ <\ \omega_{servo}$$

其中 $\omega_e$ 为波浪遭遇频率（横摇激励的主要能量所在），$\omega_{servo}$ 为舵机伺服带宽。van der Klugt 给出了可工程化检验的量化准则（1990，*Naval Engineers Journal*，"The Dutch Solution"）[5]：舵机二阶模型自然频率 $\omega_0 \ge 10\,\omega_\varphi$，大信号回路极点 $\ge 5\,\omega_\varphi$，伺服带宽 $B = 0.64\,\omega_0 \ge 6.4\,\omega_\varphi$，调节时间 $T_s = 0.4\,T_\varphi$，-3 dB 处相滞不超过 65°；舵速方面，其所针对的护卫舰要求**最大舵速不低于 15°/s**，且舵速低于该值时减摇能力显著下降、高于该值后边际收益很小 [5]。这组数据同时说明：RRS 对舵机是硬指标要求，舵机选型必须在设计早期就按减摇工况校核，而不能沿用常规航向自动舵的舵机规范。

![RRS 频带分离原理与舵→艏向/舵→横摇幅频特性](../figs/fig_10_frequency_separation.png)

*图 10-1 RRS 频带分离原理示意：舵→艏向通道呈低频特性，航向控制带宽必须远低于横摇固有频率 $\omega_\varphi$；舵→横摇通道在 $\omega_\varphi$ 处共振，减摇反馈作用于该频段（与波频遭遇频率区间重叠）；舵机伺服带宽须覆盖 $6.4\,\omega_\varphi$ 以上（van der Klugt 1990 准则）。参数为高速游艇量级示意。*

#### 为何高速船更适合 RRS

三点原因：其一，舵升力 $\propto U^2$，单位舵角产生的横摇力矩随航速平方增长，高速时舵的"减摇效能"远大于低速——25 kn 游艇的舵效横摇力矩是同尺度排水量船低速时的数倍；其二，高速船的横摇固有频率相对较高（船体小、初稳性高大），与艏摇控制带宽的频带分离更容易满足；其三，高速时遭遇频率 $\omega_e = \omega - \frac{\omega^2 U}{g}\cos\chi$ 整体上移，横摇共振区与低频操纵区的重叠更小。反过来，低速船舵效弱、横摇周期长且与操纵时间尺度接近，RRS 往往力不从心——这也是 RRS 实船成功案例集中于军舰、巡逻舰与高速船的原因 [1][4]。

### 代表性工作与文献脉络

RRS 的概念验证始于 1970 年代（Cowley & Lambert 1972、Lloyd 1975，均在 SCSS 会议发表），1980 年代在荷兰（TU Delft/特文特大学）、美国（DTNSRDC）、瑞典（SSPA）三条线上完成实船验证，1990 年代进入鲁棒性与性能极限分析阶段，2000 年后由 Fossen 教材与 Perez & Blanke 综述完成理论收束。

| 作者/机构 | 年代 | 系统/文献 | 一句话贡献 |
|---|---|---|---|
| Cowley & Lambert（英国） | 1972 | 3rd SCSS, Bath | 首次提出"以舵兼作横摇稳定器"概念 [6] |
| Lloyd | 1975 | 4th SCSS, Den Helder | 舵减摇的早期系统性分析 [6] |
| Baitis, Woolaver & Beck（美 DTNSRDC） | 1980/1983 | DTNSRDC 报告；*Naval Engineers Journal* 95(3):267–282 | 美国海岸警卫队巡逻舰与护卫舰 RRS，被称为"首次成功的实船试验" [7][1] |
| van Amerongen, van der Klugt, Pieffers | 1984 | 7th SCSS, Bath | 8 m 自航船模 + 实船试验的荷兰 RRS 系统验证 [1] |
| van Amerongen & van Nauta Lemke | 1986 | ACC, pp. 237–242 | 自适应性能指标（adaptive criterion）LQG 型 RRS 控制器 [8] |
| van der Klugt（TU Delft） | 1987 | 博士论文 *Rudder Roll Stabilization*（192 页） | 非最小相位分析、舵机设计要求推导、自适应高通滤波器、ASSA 舵减摇自动舵设计 [3][5] |
| Källström, Wessel, Sjölander（SSPA） | 1988 | SNAME STAR Symposium, pp. 67–76 | 瑞典实船试验：RRS 对多型船特别是军舰可实现有效减摇 [9] |
| van Amerongen, van der Klugt, van Nauta Lemke | 1990 | *Automatica* 26(4):679–690 | RRS 奠基性文献：LQG + 自适应加权 + AGC 防舵速饱和，仿真-船模-护卫舰海试全链条验证，减摇可达减摇鳍水平，促成荷兰 M 级护卫舰采用 RRS [1][2] |
| Blanke, Haals & Andreasen（丹麦） | 1989 | IFAC CAMS'89, Lyngby | 丹麦舵减摇实船经验 [10] |
| Roberts & Braham（英国） | 1990 | 9th SCSS (SCSS'90), Bethesda | 军舰舵-鳍联合横摇稳定（integrated rudder and fins） [11] |
| Blanke & Christensen | 1993 | 10th SCSS (SCSS'93), Ottawa, pp. A.93–A.119 | 指出横荡-艏摇-横摇耦合是 RRS 自动舵鲁棒性恶化的主因，须以解耦/滤波设计处理 [10] |
| Oda, Ohtsu & Hotta（日本） | 1996 | *Control Engineering Practice* 4(3):351–358 | 基于多元自回归模型的统计分析与自校正型 RRS 设计 [12] |
| Roberts, Sharif, Sutton & Agarwal | 1997 | *IEE Proc. Control Theory Appl.* 144(2) | 鲁棒控制方法设计军舰操舵-稳定联合系统 [13] |
| Fossen（NTNU） | 2011/2021 | *Handbook of Marine Craft Hydrodynamics and Motion Control*（Wiley） | 教材收束：第 1 版 §13、第 2 版 §16.1.7 "Optimal Rudder-Roll Damping Systems" 案例研究 [14] |
| Perez & Blanke | 2012 | *Annual Reviews in Control* 36(1):129–147 | 船舶横摇阻尼控制权威综述：舵速与非最小相位决定 RRS 性能上限，含丹麦海军 SF300 实船数据 [6] |

横向规律：RRS 的实船证据链在 1990 年代即已闭合（荷、美、瑞典、丹麦、英国均有装船记录），其工程定位是**减摇鳍的低成本替代/补充**而非全新执行器；学术重心随后转向性能极限（非最小相位 + 舵速饱和）与鲁棒性（耦合、参数摄动），这正是"自适应 RRS"问题的来源。

### 自适应 RRS

#### 参数摄动下的增益自适应

舵-横摇通道的增益 $\propto U^2$、横摇固有频率与阻尼随载况（GM、吃水）变化，波浪遭遇频率随海况/浪向/航速漂移——固定参数 RRS 控制器离开设计点后性能迅速恶化。文献中的自适应手段主要有四类：

1. **自适应性能指标 LQG**（van Amerongen & van Nauta Lemke 1986 [8]、van Amerongen et al. 1990 [1]）：性能指标 $J$ 中横摇项与艏摇项的加权因子按实测横摇/艏摇响应在线调整——横摇能量大时自动把控制重心移向减摇，海况平静时自动退出减摇模式以省电省舵机；其 MRAS 状态观测器继承自 van Amerongen 的模型参考自适应自动舵（1982/1984）[15]。这是"自校正"思想在 RRS 上最早也最完整的实船化实现。
2. **自校正/统计辨识型**（Oda, Ohtsu & Hotta 1996 [12]）：用多元自回归（AR）模型在线辨识浪-船响应，据此统计地设计/更新 RRS 控制器，属辨识-控制分离的自校正路线。
3. **L1 自适应**：Breu & Fossen（2011）将 L1 自适应与极值搜索（ESC）用于横摇参数共振的稳定化 [16]——与本项目 ESC 在线调参的技术路线直接同源，说明"L1/ESC 类自适应 + 横摇通道"的组合在文献中已有先例。
4. **鲁棒/神经网络自适应**：如双 EKF 训练 NN 的舵-横摇联合稳定、NN 鲁棒自适应 RRS（Zhu et al. 2023 [17]）等，近年仿真研究活跃，但实船验证记录少。

#### 频带分离的滤波器实现

工程上频带分离靠滤波器组落实（van Amerongen et al. 1990 [1]、Fossen 教材 [14]）：

- **横摇反馈通道**：横摇角信号经**高通/带通滤波器**（截止频率低于且靠近 $\omega_\varphi$）后进入减摇反馈，剔除回转诱导的低频横倾分量——否则减摇回路会对抗正常回转横倾，与航向通道打架；van der Klugt 采用**自适应高通滤波器**，截止频率随航速/海况在线调整 [3][5]。
- **航向通道**：艏向信号经低通/陷波（notch）滤波，剔除 $\omega_\varphi$ 与遭遇频率处的波频分量——与第 8 章波浪滤波的要求完全一致，RRS 自动舵的航向通道就是一台常规波浪滤波自动舵。
- **增益调度结合**：舵-横摇通道增益按 $U^2$ 标度律随航速调度，加权因子按海况调度，滤波器截止/中心频率随 $\omega_\varphi$ 与遭遇频率估计在线调整——"调度管已知规律（航速）、自适应管未知漂移（海况、载况）"的分工与第 3 章结论一致。

### 与航向保持的冲突及权衡

RRS 与航向保持复用同一执行器，冲突体现在三个方面：

- **舵机磨损与活动量**：减摇通道以横摇固有频率往复打舵，舵机活动量显著高于纯航向保持工况；舵机必须按减摇工况重新校核（15°/s 级舵速、高带宽伺服）[5]，否则速率饱和引入的相位滞后会同时恶化减摇与航向性能。van Amerongen et al. 为此设计了专利的 Automatic Gain Controller（AGC）：检测到舵速接近饱和时自动压低减摇回路增益，避免饱和相位滞后破坏闭环 [1]。
- **LQ 权重权衡**：联合性能指标可写为 $J = \int\left(\psi_e^2 + q_\varphi\varphi^2 + \lambda\delta^2\right)dt$，减摇权重 $q_\varphi$ 与舵动惩罚 $\lambda$ 此消彼长：$q_\varphi$ 大则减摇好但舵动剧增、航向误差略增；van Amerongen et al. 的自适应加权方案宣称"高海况下最大减摇、低海况自动退出，且航向保持性能几乎不恶化" [1]。
- **量化效果**：一手文献中的权威结论是定性的——van Amerongen et al. 1990 实测"RRS 减摇能力可达到常规减摇鳍的水平" [1]；Perez & Blanke 2012 确认多艘实船装船后减摇存在由非最小相位与舵速饱和决定的上限 [6]。**"典型 30–60% 横摇幅值削减"这一区间在二手文献中广泛流传，但未能核实到一手出处**，本文将其作为量级参考而非引用数据；舵使用率上升与航向误差增大的具体百分比同样未能从一手来源核实。图 10-2 的示意仿真（二阶横摇模型、舵速限 10°/s）给出约 47% 的横摇 RMS 削减，仅作机理演示。

![有无 RRS 的横摇角时域对比示意仿真](../figs/fig_10_rrs_simulation.png)

*图 10-2 有无 RRS 的横摇角时域对比（示意仿真）：简化二阶横摇模型（$\omega_\varphi=0.6$ rad/s，$\zeta_\varphi=0.07$），波浪激励为遭遇频率附近窄带谱；RRS 通道为 $\varphi$/$\dot\varphi$ 反馈，含舵角限幅 ±12°、舵速限制 10°/s。横摇 RMS 削减约 47%，代价是舵机以横摇频率持续往复（下图）。*

### 高海况下的效果与限制

- **遭遇频率与横摇固有频率错开**：RRS 的减摇机制是在 $\omega_\varphi$ 附近提供阻尼，当遭遇谱峰显著偏离 $\omega_\varphi$（如随浪航行 $\omega_e$ 大幅下移、或涌浪主导的远场浪），横摇响应峰值移出减摇通道的有效频段，效果下降；这也是自适应高通滤波器/遭遇频率跟踪的必要性来源 [3][5]。
- **舵机速率饱和成为瓶颈**：海况升高 → 所需减摇舵角与舵速同步增大 → 舵速饱和 → 相位滞后 → 减摇性能非线性恶化甚至闭环失稳风险。van der Klugt 的舵速-减摇曲线显示低于要求舵速时减摇能力显著下降 [5]；AGC 类增益回退机制是必需的安全兜底 [1]。
- **非线性横摇**：参数激励横摇（parametric roll，随浪/尾斜浪中 GM 时变引发，$\omega_e \approx 2\omega_\varphi$）不服从线性阻尼模型，常规 RRS 对其抑制能力有限；Breu & Fossen 2011 用 L1 自适应 + ESC 专门处理该工况 [16]。大角度横摇下横摇-艏摇-纵摇耦合非线性增强，线性化 RRS 设计假设失效。
- **与第 8 章结论的衔接**：高海况下航向通道本身已要求"降增益、强滤波、防饱和"，叠加减摇通道后舵机活动量预算更紧张——RRS 的投入决策应与海况估计联动，超阈值时优先保航向、退出减摇。

### 对本项目的适用性分析

**结论：本项目是 RRS 的良好对象，建议作为二期功能纳入架构，但舵机带宽/舵速指标必须先校核。**

有利因素：25 kn 高速下舵效横摇力矩 $\propto U^2$，减摇效能充裕；游艇以横摇舒适性为核心指标，RRS 直接命中需求；小艇横摇固有频率高，与艏摇控制带宽的频带分离容易满足；无需加装减摇鳍/陀螺，成本与舱容代价为零（仅控制软件 + 舵机校核）。

约束与风险：小艇 $\omega_\varphi$ 高（0.5–0.9 rad/s 量级）意味着舵机带宽要求 $\ge 6.4\,\omega_\varphi \approx 3.2$–$5.8$ rad/s、舵速要求 10–20°/s 量级 [5]——须核实本项目舵机实测指标，这是 RRS 可行性的第一判定条件；横摇角/角速度量测需新增（IMU 垂荡/横摇轴向输出或倾角传感器）。

与现有架构（PD + 海况×船速增益调度 + ESC 在线调参 + 极点配置）的结合建议：

1. **增加独立横摇反馈通道**：在现有 PD 航向回路之外并联 $\delta_{rrs} = -(k_1\varphi_{HP} + k_2\dot\varphi_{HP})$ 通道，$\varphi_{HP}$ 为带通滤波（通带覆盖 $\omega_\varphi \pm 40\%$，如 0.35–0.85 rad/s）后的横摇角；总舵令 $\delta = \delta_{heading} + \delta_{rrs}$ 后统一过舵角/舵速限幅。此二通道结构与 van Amerongen et al. 1990 的滤波器组方案同构 [1]，对现有代码侵入最小。
2. **航向通道补陷波**：按第 8 章建议为 PD 反馈与 ESC 辨识输入增加 $\omega_\varphi$/遭遇频率处的陷波器，保证减摇通道打舵引起的高频艏摇响应不污染航向回路与 K/T 辨识。
3. **增益调度扩展**：减摇通道增益按 $U^2$ 标度律随船速调度（与现有 Nomoto $K=K'U/L$ 调度同一框架）；海况调度维度增加"减摇权重"——海况 0–1 退出减摇（省舵机）、2–3 投入、超上限回退增益保航向，即 van Amerongen 自适应加权思想的查表实现 [1][8]。
4. **AGC 式饱和保护**：监测舵速利用率，接近饱和时按比例压低 $\delta_{rrs}$ 增益（而非简单限幅舵令），避免速率饱和相位滞后恶化航向回路 [1]。
5. **ESC 的角色**：ESC 继续负责航向通道 Nomoto $K/T$ 辨识与极点配置基线；减摇通道的 $k_1, k_2$ 初期按 $\omega_\varphi$ 解析整定（提供目标附加阻尼比 $\zeta_{add}$），后续可用 Breu & Fossen 2011 已验证的"ESC 调横摇通道参数"路线做在线优化 [16]——与本项目 ESC 技术栈完全同源。
6. **验证路径**：先仿真（二阶横摇 + Nomoto 艏摇耦合模型，本章图 10-2 即最小可行模型），再水池/实艇按"仅航向 → 航向+减摇"对比试验，指标为横摇 RMS 削减率、舵机活动量增量与航向误差增量三项联合考核。

### 参考资料

1. [van Amerongen, van der Klugt & van Nauta Lemke, "Rudder roll stabilization for ships", Automatica 26(4):679–690, 1990（全文 PDF）](https://afscheid.vanamerongen.org/publicaties/amerongen1990.pdf)（DOI: [10.1016/0005-1098(90)90045-J](https://doi.org/10.1016/0005-1098(90)90045-J)）
2. [特文特大学机构库著录：Rudder roll stabilization for ships](https://research.utwente.nl/en/publications/rudder-roll-stabilization-for-ships/)
3. [van Amerongen 主页：指导博士论文列表（van der Klugt, Rudder Roll Stabilization, TU Delft, 1987）](https://vanamerongen.org/?main=publications)
4. [Roberts, "A note on the applicability of rudder roll stabilization for ships", ACC 1993（Perez《Ship Motion Control》参考文献页确认）](https://link.springer.com/content/pdf/10.1007/1-84628-157-1.pdf)
5. [van der Klugt, "Rudder Roll Stabilization: The Dutch Solution", Naval Engineers Journal, 1990（舵机带宽/舵速量化准则，全文 PDF）](https://www.pkmarine.nl/wp-content/uploads/2016/12/PK-Marine-900500-Naval-Engineers-Journal-RRS-The-Dutch-Solution-1.pdf)
6. [Perez & Blanke, "Ship roll damping control", Annual Reviews in Control 36(1):129–147, 2012](https://doi.org/10.1016/j.arcontrol.2012.03.010)
7. [Baitis, Woolaver & Beck, "Rudder Roll Stabilization for Coast Guard Cutters and Frigates", Naval Engineers Journal 95(3):267–282, 1983](https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1559-3584.1983.tb01644.x)
8. [van Amerongen & van Nauta Lemke, "Rudder Roll Stabilization: Controller Design Based on an Adaptive Criterion", ACC 1986（NASA ADS 著录）](https://ui.adsabs.harvard.edu/abs/1986acc..conf...48V/abstract)
9. [Källström, Wessel & Sjölander, "Roll Reduction by Rudder Control", SNAME STAR Symposium 1988（TRID 著录）](https://trid.trb.org/View/400657)
10. [Blanke & Christensen, "Rudder-Roll Damping Autopilot Robustness to Sway-Yaw-Roll Couplings", SCSS'93（奥尔堡大学机构库全文）](https://vbn.aau.dk/ws/files/72604842/P93_4026_v11c.pdf)
11. [Wiley《Handbook of Marine Craft Hydrodynamics and Motion Control》参考文献页（Roberts & Braham, SCSS'90 著录）](https://onlinelibrary.wiley.com/doi/10.1002/9781119994138.refs)
12. [Oda, Ohtsu & Hotta, "Statistical analysis and design of a rudder roll stabilization system", Control Engineering Practice 4(3):351–358, 1996](https://www.sciencedirect.com/science/article/pii/0967066196000123)
13. [NTNU（IFAC 论文集参考文献页：Roberts et al., IEE Proc. CTA 144(2), 1997 著录）](https://skoge.folk.ntnu.no/prost/proceedings/ifac11-proceedings/data/html/papers/3670.pdf)
14. [Fossen 教材第 2 版（Wiley-VCH 官方目录页，§16.1.7 Optimal Rudder-roll Damping 案例）](https://www.wiley-vch.de/de/fachgebiete/ingenieurwesen/handbook-of-marine-craft-hydrodynamics-and-motion-control-978-1-119-57505-4)
15. [van Amerongen 博士论文：Adaptive Steering of Ships — A Model Reference Approach, 1982（全文 PDF）](https://afscheid.vanamerongen.org/publicaties/1982/amerongen1982.pdf)
16. [Breu & Fossen, "L1 Adaptive and Extremum Seeking Control Applied to Roll Parametric Resonance", IEEE MSC 2011（全文 PDF）](https://www.fossen.biz/publications/2011%20Breu%20and%20Fossen%20ICCA.pdf)
17. [Zhu et al., "Robust adaptive rudder roll stabilization control based on neural network", Chinese Journal of Ship Research, 2023](https://www.ship-research.com/en/article/Y2023/I3/86)
