## 自校正调节器（STR）与最小方差 / LQG 自适应自动舵

### 1. 算法原理

#### 1.1 自校正调节器的基本框架

自校正调节器（Self-Tuning Regulator, STR）由 Åström 与 Wittenmark 于 1973 年提出（[On self-tuning regulators, Automatica](https://doi.org/10.1016/0005-1098(73)90073-3)），其核心思想是把**在线参数辨识**与**基于模型的控制器综合**在每个采样周期内交替执行：

1. 用递推最小二乘（RLS）等算法在线辨识一个输入输出模型（通常取受控自回归滑动平均模型，ARMAX）的参数；
2. 按**确定性等价（certainty equivalence）**原则，把当前参数估计当作真值，代入设计方程直接算出控制器参数。

带遗忘因子的 RLS 递推式为

$$
\hat{\theta}(t)=\hat{\theta}(t-1)+K(t)\,\varepsilon(t),\qquad
K(t)=\frac{P(t-1)\varphi(t)}{\lambda_f+\varphi^{T}(t)P(t-1)\varphi(t)},
$$

$$
P(t)=\frac{1}{\lambda_f}\left[P(t-1)-K(t)\varphi^{T}(t)P(t-1)\right],
$$

其中 $\lambda_f\in(0,1]$ 为遗忘因子，用于跟踪船舶动力学的慢时变（航速、载况、海况引起的 $K$、$T$ 变化）。

#### 1.2 最小方差（MV）与广义最小方差（GMV）控制

对 ARMAX 模型

$$
A(z^{-1})\,y(t)=z^{-k}B(z^{-1})\,u(t)+C(z^{-1})\,e(t),
$$

最小方差控制律通过 Diophantine 方程 $C=AF+z^{-k}G$ 求得，使 $k$ 步预报误差方差最小：

$$
u(t)=-\frac{G(z^{-1})}{B(z^{-1})F(z^{-1})}\,y(t).
$$

船舶航向保持的真正目标并非航向误差方差最小，而是**操舵引起的附加阻力最小**。Koyama（1967）与 Norrbin（1972）给出的近似关系（被 Källström 等直接用作自校正器的损失函数，见 [TFRT-3145](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf) 式 (2.2)）为

$$
\frac{\Delta R}{R}\approx k\left(\overline{\psi_e^{2}}+\lambda\,\overline{\delta^{2}}\right),\qquad
k\approx 0.014\ \mathrm{deg}^{-2},\quad \lambda=1/12,
$$

其中 $k$、$\lambda$ 的具体数值为 Norrbin（1972）针对典型油轮所给（推导见 1.4 节），TFRT-3145 在其全部仿真中恒定取 $\lambda=1/12$ 计算损失函数。

即航向误差均方值与舵角均方值的加权和正比于阻力增量。这自然导出广义最小方差型准则

$$
J=E\left[\left(\psi(t+k)-\psi_{ref}\right)^{2}+\lambda\,\delta^{2}(t)\right],
$$

其中权重 $\lambda$ 有明确的物理含义（操纵性附加阻力），而非抽象调参旋钮。采样化后该准则近似等价于最小化航向的多步预报值，预报时域越长等价于对控制量的加权越重（[Källström et al., 1977, LTH Report TFRT-3145](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf)）。

#### 1.3 LQG 自适应结构

LQG 路线把问题写成状态空间形式：用（自适应）Kalman 滤波器在线估计航向、艏摇角速度、横漂速度、舵角偏置及传感器偏置等状态，再用线性二次型状态反馈

$$
u(t)=-L\,\hat{x}(t),\qquad
J=E\left[\int \left(x^{T}Q_1x+\rho\,u^{2}\right)dt\right]
$$

构成完整控制器。依据**分离原理**，滤波与反馈可独立设计；辨识模块（RLS 或扩展 Kalman 滤波）在线刷新对象模型后，重新求解 Riccati 方程（或多项式方程）更新 $L$。Katebi 等采用 Kučera/Grimble 的**多项式 LQG** 方法，避免显式求解 Riccati 方程，并把操舵附加阻力项直接嵌入二次型准则（[Katebi et al., LQG Adaptive Autopilot Design, IFAC 1985](https://www.sciencedirect.com/science/article/pii/S1474667017607420)；[Katebi & Byrne, LQG adaptive ship autopilot, Trans. Inst. MC, 1988](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf) 参考文献[18]）。

![STR/LQG 自适应自动舵结构框图](../figs/fig_02_str_lqg_structure.png)

上图概括了 1970–80 年代自适应自动舵的典型结构：RLS 在线辨识 + 最小方差/LQG 控制器 + Kalman 滤波（含波浪滤波），航速变化由**增益调度**快速补偿、其余慢变因素（载况、海况）交由自适应环路处理。

#### 1.4 GMV 损失函数中 λ 与操舵附加阻力关系的推导

TFRT-3145 式（2.2）的经验关系 $\Delta R/R\approx k(\overline{\psi_e^2}+\lambda\overline{\delta^2})$ 并非纯曲线拟合，其结构可由小角度假设下的船体/舵水动力二阶展开推出。按 [TFRT-3145](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf) 对航向保持问题的归因，附加阻力来自三部分：艏摇角速度–横漂速度耦合的减速力、漂角增大引起的船体阻力增量、航向偏离造成的航程损失（最后一项较小，通常忽略）。前两项分别对应损失函数中的两项：

- **船体项（航向误差）**：定常航速 $V$ 下船体阻力对漂角 $\beta$ 是对称（偶）函数，小角度泰勒展开的一阶项为零，得 $R(\beta)\approx R_0(1+k_\beta\beta^2)$。航向误差振荡时船体平均漂角与航向误差同量级，$\overline{\beta^2}\propto\overline{\psi_e^2}$，故 $\Delta R_{hull}/R_0\propto\overline{\psi_e^2}$；
- **舵项（操舵）**：舵角 $\delta$ 产生舵升力 $L=\tfrac12\rho V^2A_R C_{L\alpha}\delta$，其诱导阻力 $D_i=L^2/(\tfrac12\rho V^2A_R\cdot\pi e\,\Lambda_R)\propto\delta^2$（$\Lambda_R$ 为舵展弦比），叠加舵型阻随迎角的二次增长，得 $\Delta R_{rudder}/R_0\propto\overline{\delta^2}$。

两式合并即得 $\Delta R/R_0 = k\,\overline{\psi_e^2}+k_\delta\,\overline{\delta^2}=k\left(\overline{\psi_e^2}+\lambda\,\overline{\delta^2}\right)$，其中

$$
\lambda=\frac{k_\delta}{k}=\frac{\text{单位均方舵角的附加阻力}}{\text{单位均方航向误差的附加阻力}} .
$$

Norrbin（1972）对典型油轮估算得 $k\approx0.014\ \mathrm{deg}^{-2}$、$\lambda=1/12\approx0.083$（经 Källström 等引用为工程取值）。一个直观算例：$\psi_{e,rms}=1^\circ$、$\delta_{rms}=3^\circ$ 时，两项对损失函数的贡献分别为 $1$ 与 $\lambda\cdot 9=0.75$，同量级——说明"为压航向误差而多打舵"本身就在消耗推进能量，这正是不能只最小化航向误差方差的原因。

由此，GMV 准则 $J=E[\psi_e^2+\lambda\delta^2]$ 中的权重 $\lambda$ 有了明确的物理标定：$\lambda$ 偏大则惩罚操舵、舵机磨损小但航向保持差；$\lambda$ 偏小则航向紧、舵动剧烈，附加阻力反而上升；$\lambda=1/12$ 给出（该型油轮）阻力最优的折中点。需要注意该关系是**定常速、小舵角、航向保持工况**下的二阶近似，$k$ 与 $\lambda$ 均随船型与工况（载况、航速、水深）变化；大转向等非线性工况不适用。

#### 1.5 辨识残差白性检验的工程做法

RLS 类辨识的一致性依赖于一个可检验的前提：**预报误差（创新序列）$\varepsilon(t)$ 为白噪声**。若噪声模型 $C(z^{-1})$ 的阶次不足——例如把波浪引起的有色扰动当作白噪声——残差必然有色，参数估计随之有偏。TFRT-3145 明确记载，"RLS + LQ（解 Riccati 方程）"版本性能反不如结构更简单的最小方差自校正器，原因正是有色波浪扰动导致参数估计有偏。因此残差白性检验不是理论装饰，而是辨识质量的在线仪表。工程上的标准做法：

1. **滑动窗口样本自相关 + Bartlett 置信带**：在最近 $N$ 个样本上计算归一化自相关 $\hat\rho(\tau)=\hat r(\tau)/\hat r(0)$。白噪声假设下 $\hat\rho(\tau)\sim\mathcal N(0,1/N)$（Bartlett 渐近结果），故 95% 置信带为 $\pm1.96/\sqrt{N}$；若出带滞后点显著超过 5%，判定残差非白。
2. **Ljung–Box 混成检验**：$Q=N(N+2)\sum_{\tau=1}^{m}\hat\rho^2(\tau)/(N-\tau)$ 在白性假设下近似服从 $\chi^2(m)$，$m$ 取 10–20，$Q$ 超临界值即拒绝白性假设——比逐点看置信带更不易被偶然出带误导。
3. **残差–输入互相关检验**：$\hat r_{\varepsilon u}(\tau)$ 用于检验对象动态模型是否充分（白性检验暴露噪声模型缺陷，互相关检验暴露对象模型缺陷），两者应同时监控。
4. **在线实现要点**：用与遗忘因子一致的指数加权递推估计 $\hat r(\tau)$，避免存储全窗；窗口长度应覆盖若干波浪遭遇周期（遭遇频率典型 0.05–0.3 Hz，1 Hz 采样时 $N$ 取数百至数千）；检验统计量本身应低通后再触发报警，避免在海况瞬变时抖动。

   ![RLS 辨识残差白性检验示意](../figs/fig_02_residual_whiteness.png)

   上图为数值演示：噪声模型匹配时残差自相关几乎全部落在 Bartlett 带内（出带 2%，左）；未建波浪有色噪声模型时，二阶波浪成型滤波器（谐振峰约 0.1 Hz）产生的有色残差大量出带（90%，右），其缓慢起伏形态正是波浪谱在滞后域的指纹。

5. **判定非白后的处置**：(a) 增广噪声模型——提高 ARMAX 中 $C(z^{-1})$ 阶次，或显式并入二阶波浪成型滤波器 $H(z)=\sigma^2 z^{-1}/(1-2\zeta\omega_0 z^{-1}+\omega_0^2 z^{-2})$；(b) 对输入输出数据做高通/陷波预处理，把波浪频段隔离在辨识带宽之外；(c) 在白性恢复之前冻结参数更新或收紧参数限幅，防止有偏估计进入控制律。
6. **对本项目的映射**：对 Nomoto $K/T$ 在线辨识的残差做同样的白性监控，可作为"海况升高导致 $K/T$ 估计系统性偏移"的**早期预警量**——残差在波浪频段出现能量集中时，先于控制性能恶化给出信号，触发波浪滤波或噪声模型增广。

### 2. 代表性工作与工程应用

- **船舶操纵动力学辨识奠基**：Åström & Källström, *Identification of ship steering dynamics*, Automatica, 1976——用极大似然法从油轮实船试航数据辨识 Nomoto 型操纵模型及波浪扰动模型，为后续自适应设计提供了模型与噪声统计基础（引自 [船舶航向保持控制研究综述, 2022](http://www.aquaticjournal.com/article/doi/10.3969/j.issn.1673-9159.2022.06.006) 文献列表）。
- **KADPIL 1 / KADPIL 2（瑞典 LTH + Kockums Automation，1974–1979）**：Åström、Källström 与 Kockums 工程师合作的经典工程。简单版 KADPIL 1 = 速度调度 + RLS/最小方差自校正器（仅用罗经航向）；高级版 KADPIL 2 增加 Kalman 滤波，可融合艏摇角速度、横漂速度、舵角等多传感器信息，并能估计传感器偏置、检测传感器故障（graceful degradation）。转向采用**三段式固定增益转向调节器**（启动比例速率反馈 → 恒定旋回速率控制 → PD 式制动），航速影响按 $G_y(V)=(V_0/V)^2$（时间尺度不变）或 $V_0/V$（航迹不变）做增益调度并限幅。成果发表于 Källström, Åström, Thorell, Eriksson, Sten, *Adaptive autopilots for tankers*, Automatica 15(3):241–254, 1979，工程细节见 [LTH Report TFRT-3145 (1977)](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf)。
- **实船验证规模**：在 3 艘不同油轮（25.5 万吨级、35.5 万吨级等）上完成 **130 余次实船试验**；为 "Sea Swift" 设计的自适应自动舵自 **1974 年 10 月起投入日常运营**，是最早长期商用的自适应自动舵之一（同上 LTH 报告）。
- **理论总结**：Åström, *Why use adaptive techniques for steering large tankers*, Int. J. Control 32(4):689–708, 1980（引自 [Automation for the Maritime Industries](https://digital.csic.es/bitstream/10261/2906/1/Maritime_Industries.pdf) 文献[15]）——指出固定增益 PD 自动舵在不同海况/载况下的损失函数可相差 **5～8 倍**，而最优整定的收益随扰动增强而增大，这正是需要自适应的定量理由。
- **MRAC 平行路线**：van Amerongen（代尔夫特理工）的模型参考自适应自动舵（PhD 1982；[Automatica 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)），相对常规 PID 演示了最高约 **5%** 的燃油节省，并走向商品化。
- **LQG 自适应路线**：Katebi 与 Byrne（Strathclyde 大学），*LQG adaptive ship autopilot*, Trans. Inst. MC 10(4):187–197, 1988；Grimble & Katebi（1986）把航向保持损失函数与船体纵荡方向运动方程导出的能量损失联系起来，用 LQG 同时优化航向精度与操舵能耗（引自 [Liu et al., Ocean Engineering, 2016](https://www.sciencedirect.com/science/article/pii/S002980181500623X) 及 [EOLSS 综述](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf) 文献[18]）。此外还有 Sheffield 大学 Mort（1983 博士论文）的自校正自动舵研究（同 CSIC 文献[6]）。
- **商业化历程**：KADPIL 系列由 Kockums Automation AB 推向产品化，1970 年代中期即有商用系统在船服役；1980 年代起自适应（含 MRAC 与自校正）自动舵进入多家厂商产品线。但值得注意的是，由于整定与认证成本、可靠性顾虑，**至今市场上绝大多数在役自动舵仍是 PID + 增益调度结构**（[Jurnal Teknologi 综述](https://journals.utm.my/jurnalteknologi/article/download/4635/3223/12968)；[EOLSS 综述](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf)）。

![不同方案相对精调 PID 自动舵的燃油节省](../figs/fig_02_fuel_saving.png)

关键量化结论（[Källström et al., LTH Report TFRT-3145](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf) 第 3 节仿真总结，实船试验趋势一致）：满载工况下 KADPIL 2 相对精调 PID 节省燃油约 **0.3–0.9%**（低值对应 PID 使用 Kalman 滤波估计、高值对应 PID 直接用未滤波测量）；海况越恶劣收益越大——255,000 tdw 满载油轮仿真中，17–20 m/s 大风（fresh gale，蒲福 8 级）下 KADPIL 2 相对 KADPIL 1 的增益达 **1.4%**，远高于 6–8 m/s 和风（moderate breeze）时的 0.5–0.6%（报告引言按 0.5% 估计，仿真节给出 0.6%）；低速航行时收益同样显著增大。报告同时指出：仿真略微高估了精调 PID 的性能，**实船试验中自适应自动舵的收益比仿真更大**。叠加 Schilling 关于"PID 从通常整定到最优整定可省 1–2%"的估计，自适应自动舵相对粗放整定的常规 PID 总节省可达 **2–3%**。

### 3. 工程优缺点

**优点**

- **免人工整定、面向经济指标**：损失函数直接对应操舵附加阻力，控制器参数随海况/载况/航速自动收敛到近最优点，避免"水手凭经验调灵敏度旋钮"的随意性；
- **恶劣海况与低速下收益最大**——恰恰覆盖固定增益 PID 表现最差、被迫切换人工操舵的场景；
- **Kalman 滤波带来的附加价值**：平滑的航向/艏摇估计、波浪滤波、传感器偏置估计与故障检测，系统可优雅降级（仅航向一路信号仍可运行）；
- **速度调度 + 自适应分工合理**：可解析、变化快的航速因素用增益调度即时补偿，不可解析、慢变的载况/海况因素用自适应慢速跟踪，两者互补。

**缺点与风险**

- **激励依赖与参数漂移**：直线巡航时激励弱，RLS 参数可能缓慢漂移（协方差"上爆"），工程上需配合遗忘因子下限、协方差监控、死区与参数限幅等"安全夹克"；
- **有色噪声下辨识有偏**：Källström 报告明确记载，基于"RLS + LQ（解 Riccati 方程）"的版本因波浪扰动为有色噪声导致参数估计有偏，性能反而不如结构更简单的最小方差自校正器——说明**辨识结构必须与噪声模型匹配**；
- **大转向时非线性显著**：满载油轮的"舵角–旋回速率"稳态曲线强非线性（不稳定载况下存在多值区），单一线性自适应律无法覆盖，KADPIL 不得不为转向单独设计三段式调节器；
- **稳定性与收敛性分析困难、认证门槛高**：这也是自适应方案未能大规模取代 PID 自动舵的现实原因之一；
- **计算与调试成本**（1970 年代小型机 2k 字代码量曾是工程考量；今天已非瓶颈，但验证确认成本仍在）。

### 4. 与本项目（PD + 增益调度 + ESC）方案的关联与借鉴

1. **结构同源**：本项目"Nomoto $K/T$ 在线辨识 + 极点配置"属于**显式（间接）自适应**——先辨识对象参数、再算控制器参数；Källström 的最小方差 STR 属于**隐式（直接）自适应**——直接辨识控制器参数。前者物理可解释性强、便于监控与限幅（$K$、$T$ 有明确物理意义），后者实现更简、无需显式设计步骤。本项目的选择与 Källström 报告中"RLS+LQ"尝试同属一族，应吸取其教训：**关注辨识残差是否白化**，必要时为波浪扰动增补有色噪声模型或对信号做高通/陷波预处理，避免海况升高时 $K/T$ 估计出现系统性偏移。
2. **损失函数可直接借用**：GMV 准则 $J=E[\psi_e^2+\lambda\delta^2]$（$\lambda\approx 1/12$ 源自附加阻力物理推导）可作为本项目 ESC 在线调参的代价函数或至少作为海试评价指标，使 ESC 的寻优目标从"航向误差最小"升级为"阻力增量最小"，与 KADPIL 的 0.3–0.9% 节油结论直接可比。
3. **增益调度分工验证**：KADPIL 证明"航速用增益调度（$G_y\propto(V_0/V)^2$，限幅）+ 其余慢变因素用自适应"是稳健的工程分工；本项目的"海况 × 船速段"调度表 + ESC 微调与之同构，可借鉴其**调度响应快、自适应跟踪慢**的带宽分离原则，避免 ESC 去追逐本应由调度表处理的航速瞬变。
4. **恶劣海况是自适应的主战场**：历史数据一致表明海况越高、航速越低，自适应相对固定增益的收益越大（大风下 1.4% vs 和风下 0.5%）。对本项目"最大海况适应性"的结论性启示：海况 3 级段应保留并强化在线调整能力，而不是退回保守的固定参数。
5. **转向与大机动需独立通道**：三段式转向调节器（启动—恒速率—制动）与航向保持自校正器分离的做法，可直接映射到本项目航向保持/改向双模式设计；大改向时的舵角饱和与速率限制必须显式处理，不能依赖辨识-控制环路线性外推。
6. **状态估计的增值空间**：KADPIL 2 的经验表明，仅增加一个设计良好的 Kalman 滤波器（航向 + 艏摇 + 偏置估计），即使控制器不变也能节油 0.3–0.6%，并获得传感器故障检测能力——这是本项目在现有 ESC 框架下成本最低的性能升级选项之一。

### 5. 参考资料

1. [Källström C.G., Åström K.J., Thorell N.E., Eriksson J., Sten L., "Adaptive Autopilots for Steering of Large Tankers", LTH Report TFRT-3145（报告编号 LUTFD2/(TFRT-3145)/1-66/(1977)），1977 年 7 月，66 页（全文 PDF，隆德大学）](https://lucris.lub.lu.se/ws/files/4680687/8777572.pdf)
2. Källström C.G., Åström K.J., Thorell N.E., Eriksson J., Sten L., "Adaptive autopilots for tankers", *Automatica*, 15(3):241–254, 1979, [DOI: 10.1016/0005-1098(79)90042-6](https://doi.org/10.1016/0005-1098(79)90042-6)（题录见 [隆德大学研究门户](https://portal.research.lu.se/sv/publications/adaptive-autopilots-for-tankers/) 与 [智能船舶远程驾驶控制技术研究现状与趋势](https://www.opticsjournal.net/Articles/OJcce32d184bc6b11b/FullText) 文献[86]）
3. Åström K.J., Källström C.G., "Identification of ship steering dynamics", *Automatica*, 12(1):9–22, 1976（题录见 [船舶航向保持控制研究综述](http://www.aquaticjournal.com/article/doi/10.3969/j.issn.1673-9159.2022.06.006)）
4. Åström K.J., Wittenmark B., "On self-tuning regulators", *Automatica*, 9:185–199, 1973, [DOI](https://doi.org/10.1016/0005-1098(73)90073-3)
5. Åström K.J., "Why use adaptive techniques for steering large tankers", *Int. J. Control*, 32(4):689–708, 1980（题录见 [Automation for the Maritime Industries, DIGITAL.CSIC](https://digital.csic.es/bitstream/10261/2906/1/Maritime_Industries.pdf) 文献[15]）
6. [Katebi M.R., et al., "LQG Adaptive Autopilot Design", IFAC Workshop, 1985 (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1474667017607420)
7. Katebi M.R., Byrne J.C., "LQG adaptive ship autopilot", *Trans. Inst. MC*, 10(4):187–197, 1988（题录见 [EOLSS: Ship Control 综述](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf) 文献[18] 及 [ACM 文献列表](https://dl.acm.org/doi/abs/10.5555/3151573.3151575) 文献[11]）
8. [van Amerongen J., "Adaptive steering of ships – a model reference approach", *Automatica*, 1984（作者主页全文）](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)
9. [Liu Z. et al., "Ship forward speed loss minimization using nonlinear course keeping and roll motion controllers", *Ocean Engineering*, 2016（Grimble & Katebi 1986、Katebi & Byrne 1986 操舵附加阻力 LQG 工作的转述）](https://www.sciencedirect.com/science/article/pii/S002980181500623X)
10. [Velasco F.J. et al., "Automation for the Maritime Industries"（自适应自动舵文献谱系与 Mort 1983 等）](https://digital.csic.es/bitstream/10261/2906/1/Maritime_Industries.pdf)
11. [EOLSS, "Ship Control" 章节（PID 自动舵历史与自适应动因综述）](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf)
