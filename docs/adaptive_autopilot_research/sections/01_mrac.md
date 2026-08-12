## 模型参考自适应控制（MRAC）在船舶自动舵中的应用

### 1. MRAC 基本原理

模型参考自适应控制（Model Reference Adaptive Control, MRAC/MRAS）的核心思想是：用一个**参考模型**显式地给出闭环系统的期望动态，当对象实际输出与参考模型输出出现偏差时，由**自适应机构**在线调整控制器参数（直接自适应），使对象-控制器组成的闭环逐渐逼近参考模型的行为。其优化目标可写为最小化广义误差的平方积分

$$
J = \int_0^{T} e^2(t)\,\mathrm{d}t, \qquad e(t) = y(t) - y_m(t)
$$

其中 $y$ 为对象输出，$y_m$ 为参考模型输出（[van Amerongen 的 MRAS 讲义](https://dynamicalsystems.nl/intelligentcontrol/Intelligent_Control_MRAS.pdf)）。MRAC 属于**直接自适应控制**：不显式辨识对象参数，而是直接调整控制器增益，这是它与"辨识 + 控制器再设计"的间接自适应（自校正）路线的本质区别。

![MRAC 自动舵结构框图](../figs/fig_01_mrac_structure.png)

#### 1.1 MIT 规则（梯度法）

最早的 MRAC 设计方法由 MIT 仪器实验室的 Whitaker 等人于 1958 年提出（[Whitaker et al., MIT Report R-164, 1958；转引自 Springer 教材章节](https://link.springer.com/content/pdf/10.1007/978-1-4471-4396-3_9.pdf?pdf=inline%20link)），称为 **MIT 规则**。其出发点是沿准则 $J(\theta)=\frac{1}{2}e^2$ 的负梯度方向调整控制器参数 $\theta$：

$$
\dot{\theta} = -\gamma\,\frac{\partial J}{\partial \theta} = -\gamma\, e\,\frac{\partial e}{\partial \theta}
$$

其中 $\gamma>0$ 为自适应增益，$\partial e/\partial\theta$ 称为**灵敏度导数**（sensitivity derivative）。由于灵敏度导数依赖未知的对象参数，实际实现时需用参考模型参数近似（例如用模型输出对参数的灵敏度滤波器代替）。

MIT 规则结构简单、易于实现，但它本质上是局部梯度下降，**不保证闭环稳定**：当自适应增益 $\gamma$ 过大或参考输入幅值过大时，自适应回路可能失稳。这一缺陷促使 Parks 于 1966 年提出基于 Lyapunov 稳定性理论的重设计方法（[Parks, IEEE Trans. Automatic Control, 11(3): 362–367, 1966；转引自 Springer 教材章节](https://link.springer.com/content/pdf/10.1007/978-1-4471-4396-3_9.pdf?pdf=inline%20link)）。

#### 1.2 Lyapunov 设计

Lyapunov 设计把自适应律的选取反过来做：先构造包含跟踪误差与参数误差的 Lyapunov 函数，再选择使 $\dot{V}\le 0$ 的自适应律，从而**按构造保证稳定性**。以一阶对象为例，设

$$
\dot{y} = -a\,y + b\,u, \qquad u = \theta_1 r - \theta_2 y
$$

参考模型为 $\dot{y}_m = -a_m y_m + b_m r$（$a_m>0$）。理想匹配参数为 $\theta_1^* = b_m/b$，$\theta_2^* = (a_m-a)/b$。定义参数误差 $\tilde{\theta}_i = \theta_i - \theta_i^*$，则误差方程为

$$
\dot{e} = -a_m e + b\left(\tilde{\theta}_1 r - \tilde{\theta}_2 y\right)
$$

取 Lyapunov 函数（设 $b>0$ 已知符号）

$$
V = \frac{1}{2}e^2 + \frac{b}{2\gamma}\left(\tilde{\theta}_1^2 + \tilde{\theta}_2^2\right)
$$

沿轨迹求导并选择自适应律

$$
\dot{\theta}_1 = -\gamma\, e\, r, \qquad \dot{\theta}_2 = \gamma\, e\, y
$$

即可使 $\dot{V} = -a_m e^2 \le 0$，闭环所有信号有界且 $e(t)\to 0$。这一"一阶对象 + 前馈/反馈两参数"结构恰好对应一阶 Nomoto 船舶模型下的艏向控制，因此成为船舶 MRAC 自动舵的标准设计模板（推导框架见 [van Amerongen 的 MRAS 讲义](https://dynamicalsystems.nl/intelligentcontrol/Intelligent_Control_MRAS.pdf) 与 [Python Control 库的 MRAC 示例](https://python-control.readthedocs.io/en/0.10.2/examples/mrac_siso_lyapunov.html)）。

### 2. Norrbin / Nomoto 一阶参考模型

船舶操舵响应最经典的简化模型是 Nomoto 一阶模型（1957），描述舵角 $\delta$ 到转艏角速度 $r=\dot{\psi}$ 的传递关系：

$$
T\dot{r} + r = K\delta \quad\Longleftrightarrow\quad T\ddot{\psi} + \dot{\psi} = K\delta
$$

其中增益与时间常数随船速 $U$、船长 $L$ 按如下规律换算（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）：

$$
K = K^*\frac{U}{L}, \qquad T = T^*\frac{L}{U}
$$

$K^*$、$T^*$ 为关于船速近似不变的无因次常数，量级在 0.5–2.0。Norrbin（1963）在一阶模型中引入非线性项以描述大舵角下的非线性操舵特性：

$$
T\dot{r} + H_N(r) = K\delta, \qquad H_N(r) = n_3 r^3 + n_2 r^2 + n_1 r + n_0
$$

Nomoto 一阶模型可视为 Norrbin 模型的线性特例（$H_N(r)=r$）。在 MRAC 自动舵中，参考模型通常就取这种**一阶（Norrbin/Nomoto 型）环节**，并附加转艏速率限幅，用于把阶跃式改向指令整形为"起转—恒速率转艏—无超调回正"的理想艏向轨迹：

$$
T_m\ddot{\psi}_m + \dot{\psi}_m = K_m\,\psi_r
$$

Van Amerongen 建议参考模型时间常数 $T_m$ 取为船舶主导时间常数的 $1/2 \sim 1/3$，因为合理设计的航向控制器（带速率反馈）会把闭环等效时间常数压缩到开环的 $1/2 \sim 1/3$，这样能保证实际艏向跟得上模型轨迹（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）。

### 3. Van Amerongen 等人的经典 MRAC 自动舵

MRAC 在船舶自动舵中的系统性应用由荷兰代尔夫特理工大学的 **J. van Amerongen** 及其合作者开创，代表性工作包括：

- **Van Amerongen & Udink ten Cate (1975)**，《Model reference adaptive autopilots for ships》，Automatica 11(1), 441–449——首个基于梯度法（MIT 规则）的船舶 MRAC 自动舵（[文献线索见 EOLSS 船舶自动化章节](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf)）。
- **Van Amerongen & Van Nauta Lemke (1978–1980)**：数字 MRAC 自动舵的海上试验与最优操舵准则研究。
- **Van Amerongen (1982/1984)**：博士论文及发表于 Automatica 20(1) 的总结性论文《[Adaptive Steering of Ships — A Model Reference Approach](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)》（被引 300 余次），给出了完整的 MRAC 自动舵体系：
  - **改向（course changing）模式**：采用串联参考模型对设定值整形（生成限速率转艏轨迹），并用并联参考模型 + MIT 规则**直接自适应**调整控制器增益，补偿船速、载况、水深变化引起的 $K$、$T$ 摄动；对舵机饱和、舵速限制（典型值 $|\delta|\le35°$、$|\dot{\delta}|\le 2\sim7°/s$）这类非线性给出了专门处理；
  - **保向（course keeping）模式**：采用**间接自适应**——在线辨识 + 基于性能指标
    $$
    J = \int \left(\varepsilon^2 + \lambda\,\delta^2\right)\mathrm{d}t
    $$
    的 LQ 最优增益（$K_p = 1/\sqrt{\lambda}$ 等），其中 $\varepsilon$ 为艏向误差、$\delta$ 为舵角；$\lambda$ 建议取值范围 0.1（精确操舵）到 4（小船高海况经济性操舵）；并配合**自适应状态估计器**估计被波浪污染的低频艏向/转艏速率；
  - 海上实船试验表明，相对常规 PID 自动舵**节油最高约 5%**（原文："fuel savings up to 5% … mainly possible because of the adaptive state estimator"），主要来自自适应状态估计（含波浪滤波）（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）。
- **Van Amerongen, van der Klugt & van Nauta Lemke (1990)**：《Rudder roll stabilization for ships》，Automatica 26(4)，把 MRAC 自动舵扩展到舵减摇（[EOLSS 文献目录](https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf)）。
- **Holzhüter & Strauch (1987)**：《A commercial adaptive autopilot for ships: design and operational experiences》，IFAC——MRAC 类自适应自动舵的商用化与运行经验（[NTNU 学位论文引目](https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2773646/Andreas%20Bell%20Martinsen_PhD.pdf)）。

后续工作中，Fossen 等人把 Lyapunov 设计、输出反馈（仅需罗经艏向测量）与陷波（notch）波浪滤波结合起来：如《[A globally stable adaptive ship autopilot with wave filter using only yaw angle measurements](https://www.mic-journal.no/PDF/1996/MIC-1996-2-4.pdf)》（1996，全局稳定 MRAC + 级联陷波滤波器，并用参考模型整形技术引入积分作用）和 Lauvdal & Fossen 的《[Robust adaptive ship autopilot with wave filter...](https://www.fossen.biz/publications/1998%20Lauvdal%20and%20Fossen%20IJACSP.pdf)》（IJACSP, 1998），解决了 Van Amerongen 方案依赖全状态反馈（需转艏角速度）的问题。

### 4. 参考模型如何抑制波浪频率响应

波浪引起的艏摇力矩峰值频率典型范围为 **0.05–0.2 Hz**（约 0.3–1.3 rad/s）（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）。这一频段远高于船舶操舵响应的有效带宽，用舵去补偿波浪频率运动不仅无效，反而因舵机大舵角快速动作的相位滞后**放大**船体运动并增加阻力。MRAC 从三个层面抑制波浪频率响应：

1. **参考模型本身是低通滤波器**：一阶参考模型 $G_m(s)=K_m/(T_m s+1)$ 的截止频率 $1/T_m$ 远低于波浪频率，设定值经模型整形后只含低频分量，闭环只需跟踪低频轨迹，结构上就不会去"追浪"（见下图 a）。
2. **慢自适应避免波浪注入参数回路**：自适应律由广义误差 $e=\psi-\psi_m$ 驱动。只要自适应增益 $\gamma$ 取得小（自适应回路带宽远低于波浪频率），波浪频率分量在参数更新积分中被平均掉，不会引起参数漂移；反之波浪噪声大时若 $\gamma$ 过大，会产生参数漂移甚至失稳——这正是 1996 年 Fossen 等人引入陷波滤波器的原因（[MIC 1996 论文](https://www.mic-journal.no/PDF/1996/MIC-1996-2-4.pdf)）。
3. **滤波量随噪声水平自适应**：Van Amerongen 指出常用的死区并不是正确解法，必须精心设计低通滤波器去除舵令中的高频噪声，且滤波强度应随波浪噪声水平自适应调整；"精度—经济性"旋钮同时改变 $\lambda$ 与滤波量——高海况下取大 $\lambda$（少动舵）并允许最大滤波（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）。

![参考模型的低通特性与波浪谱的频率分离及对改向指令的整形作用](../figs/fig_01_refmodel_filtering.png)

上图中：(a) 一阶参考模型（$T_m=20$ s）在波浪主要频段（0.3–1.3 rad/s）的衰减约为 16 dB（低频端 0.3 rad/s）至 28 dB（高频端 1.3 rad/s）——按 $|G_m(j\omega)|=1/\sqrt{1+(\omega T_m)^2}$ 计算，低频端约 15.7 dB，严格说在频段下沿**不足 20 dB**，需配合舵令低通/陷波滤波才能保证充分隔离；实现期望动态与波浪扰动的频率分离；(b) 阶跃改向指令经带转艏速率限幅的参考模型整形为平滑轨迹，实测艏向中的波浪高频分量仅围绕期望轨迹波动，不进入舵令。

### 5. 工程优缺点

**优点**

- 期望性能由参考模型**显式表达**，物理意义直观；操作员只需调整"转艏速率""精度/经济性"等少量高层旋钮，无需理解 PID 参数整定（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）；
- 直接自适应无需显式参数辨识，算法简单、计算量小，适合嵌入式实现；参数自适应回路具有"记忆"，对象参数缓慢变化（船速、载况）时性能优良；
- 参考模型天然完成设定值整形与波浪频率隔离，配合自适应滤波可显著降低操舵阻力，实船试验节油最高约 5%。

**缺点**

- MIT 规则本身**不保证稳定**，需 Lyapunov 重设计；而 Lyapunov 设计对对象相对阶、未建模动态（舵机饱和与舵速限制）敏感，需要专门的抗饱和/抗漂移处理（见第 6 节）；
- 经典方案假设**全状态反馈**（需转艏角速度），罗经只提供艏向时必须引入状态估计器，增加复杂度（后续输出反馈方案缓解了这一点）；
- 波浪噪声环境下存在**参数漂移**风险，必须配合陷波/低通滤波、死区或归一化自适应律等鲁棒化措施；
- 高海况下对象非线性增强（Norrbin 立方项、舵机饱和），一阶线性参考模型与实际动态失配加大，自适应收敛变慢甚至失效，需与增益调度等手段结合。

### 6. 舵机饱和下的鲁棒修正自适应律（σ-/e-modification）

理想 Lyapunov 自适应律 $\dot{\theta}=-\gamma e\,\zeta$（$\zeta$ 为回归信号，如 $r$、$y$）的稳定性证明建立在"对象与参考模型严格匹配、无扰动、执行机构无约束"的理想前提上，而自动舵的实船环境两条都不满足：

- **舵角/舵速饱和**：饱和期间艏向无法跟上参考模型，广义误差 $e$ 持续偏大，自适应机构却把这部分"不可行误差"照样积分进参数，形成与积分饱和（integrator windup）同构的**参数回卷**；
- **波浪噪声与未建模动态**：Rohrs 等人 1985 年的著名反例表明，即使是有界小扰动加未建模动态，也足以使理想 MRAC 的参数缓慢漂移直至突发失稳（bursting）（[Rohrs et al., IEEE Trans. Automatic Control, 30(9): 881–889, 1985；转引自 MIT 学位论文引目](https://dspace.mit.edu/bitstream/handle/1721.1/111744/1004393438-MIT.pdf?sequence=1&isAllowed=y)）。

工程上的标准对策是在自适应律中加入**鲁棒修正项**，代表性形式有：

**σ-modification**（Ioannou & Kokotovic, 1984）：加入把参数持续"拉回"原点的固定泄漏项

$$
\dot{\theta} = -\gamma\, e\,\zeta - \gamma\sigma\,\theta, \qquad \sigma>0
$$

泄漏项 $-\gamma\sigma\theta$ 使参数更新方程成为以 $-\gamma e\zeta$ 为输入的稳定一阶滤波器，从而保证所有信号一致有界；代价是引入 $O(\sigma)$ 量级的残余误差——扰动消失后 $e$ 不再渐近归零，参数稳态值相对 $\theta^*$ 存在偏差（[Ioannou & Kokotovic, Automatica, 20(5): 583–594, 1984；文献信息见 arXiv 综述引目](https://arxiv.org/pdf/2202.09834v1.pdf)）。

**e-modification**（Narendra & Annaswamy, 1987）：把固定泄漏改为与跟踪误差幅值成正比的可变泄漏

$$
\dot{\theta} = -\gamma\, e\,\zeta - \gamma\sigma\,|e|\,\theta
$$

饱和或波浪扰动使 $|e|$ 增大时泄漏自动加强、抑制漂移；$e\to 0$ 时泄漏消失，因此保留了理想律的收敛性质（无扰动时仍可使 $e\to 0$），且不需要持续激励（PE）条件即保证有界（[Narendra & Annaswamy, IEEE Trans. Automatic Control, 32(2): 134–145, 1987；文献信息见 IntechOpen 章节引目](https://www.intechopen.com/chapters/1175769)）。

此外常用的还有**参数投影**（把 $\theta$ 约束在已知凸集内）、**死区**（$|e|$ 低于噪声水平时冻结更新）与**归一化自适应律**。值得指出的是，Van Amerongen 1984 年的方案在结构层面已包含抗饱和思想：舵角与舵速限幅被直接做进**串联参考模型**——参考轨迹本身受限、永远可行，饱和期间广义误差不会因"追不上的指令"而发散，本质上是一种"参考模型级抗饱和"（[Van Amerongen, Automatica, 1984](https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf)）。

![饱和与波浪扰动下理想 Lyapunov 律与 σ-/e-modification 的参数漂移对比](../figs/fig_01_sigma_modification.png)

上图为一阶 Nomoto 型对象 + 舵角饱和 + 波浪扰动（0.08/0.17 Hz）下的数值仿真：(a) 理想 Lyapunov 律的反馈增益 $\theta_2$ 在 2000 s 内持续漂移、远离理想匹配值近两个数量级，而 σ- 与 e-modification 始终将其约束在 $\theta_2^*$ 附近；(b) 修正律的代价是略大的残余跟踪误差——这正是"鲁棒性—精度"折中的直观体现（仿真脚本见 `figs/make_fig_01_sigma_mod.py`）。

#### 6.1 与本项目 ESC 在线调参的对比

- **漂移机制不同，但风险同构**：MRAC 是连续积分型更新，误差信号被直接积分，饱和/噪声期会产生单向漂移；本项目 ESC 是"扰动—平均"型极值搜索，按目标函数梯度的慢变估计更新，其解调平均环节对高频波浪噪声天然不敏感。但饱和期间实测性能指标 $J$ 被"削顶"，梯度估计出现偏置，同样会把增益推向错误方向——因此 ESC 也需要"饱和段冻结更新/剔除饱和数据"的纪律，与 MRAC 的死区、泄漏思想同构；
- **泄漏思想可直接移植到 ESC**：在 ESC/极点配置更新回路中加入向先验值（增益调度表插值结果）的回拉项 $\dot{k}=-\gamma\,\hat{g}-\sigma\,(k-k_{\text{sched}})$，把海况×船速调度表当作参数"锚"，防止 ESC 在异常海况或传感器故障下漂出安全域——这相当于给间接自适应回路加一层 σ-modification；
- **参考模型级抗饱和已部分落地**：本项目若采用第 2 节所述带舵角/舵速限幅的参考模型做设定值整形，饱和期的误差就不会注入 ESC 的目标函数，是最廉价的第一道防线；
- **实现成本**：两类修正律只需在参数更新方程中各加一项，与现有 ESC、极点配置代码完全解耦，适合作为安全加固项引入，无需改动控制结构。

### 7. 与本项目（PD + 增益调度 + ESC 在线调参）方案的关联与借鉴

本项目当前方案为 PD 控制器 + 海况（0–3 级）× 船速段的增益调度 + 基于 Nomoto $K$/$T$ 在线辨识与极点配置的 ESC 自动调参。从自适应控制分类看（[van Amerongen MRAS 讲义](https://dynamicalsystems.nl/intelligentcontrol/Intelligent_Control_MRAS.pdf)）：

- 本项目的"船速段增益调度"对应 Van Amerongen 所说的 **gain scheduling**（$K=K^*U/L$、$T=T^*L/U$ 的显式换算）；"ESC 在线辨识 $K$/$T$ + 极点配置"则属于**间接自适应**（显式辨识 + 控制器再设计），与 Van Amerongen 保向模式的技术路线一致；
- **MRAC 提供了另一条互补路线**：不辨识对象参数，用参考模型 + MIT/Lyapunov 律**直接**调整 PD 增益。具体可借鉴点：
  1. **引入串联一阶参考模型（带转艏速率限幅）对艏向设定值整形**：避免阶跃改向指令直接冲击 PD 产生大舵角，同时把期望动态限制在低频段，是抑制波浪频率舵动的结构性手段，实现成本极低；
  2. **性能指标 $J=\int(\varepsilon^2+\lambda\delta^2)\mathrm{d}t$ 可作为 ESC 调参的目标函数**，并把 $\lambda$ 与在线估计的波浪噪声水平挂钩，将当前离散的"海况 0–3 级"档位**连续化**——高海况自动加大 $\lambda$ 与滤波量、少动舵；
  3. **自适应回路带宽纪律**：若未来在 ESC 之外叠加 MIT/Lyapunov 增益微调回路，其带宽必须远低于波浪频率（或加陷波滤波），否则波浪分量会注入参数更新导致漂移——这对本项目 ESC 的激励设计与收敛速度选取同样是硬约束；
  4. **抗饱和处理**：Van Amerongen 对舵角/舵速饱和下的 MRAC 给出了专门方案，对本项目 25 kn 游艇在大舵角改向时的积分/增益回卷设计有直接参考价值（具体修正律形式与 ESC 对比见第 6 节）。

### 8. 参考资料

1. J. van Amerongen, "Adaptive Steering of Ships — A Model Reference Approach", *Automatica*, 20(1), 3–14, 1984. <https://afscheid.vanamerongen.org/publicaties/amerongen84.pdf>
2. J. van Amerongen, "Intelligent Control part 1 — Model Reference Adaptive Control Systems"（University of Twente 讲义）, 2004. <https://dynamicalsystems.nl/intelligentcontrol/Intelligent_Control_MRAS.pdf>
3. J. van Amerongen, A.J. Udink ten Cate, "Model reference adaptive autopilots for ships", *Automatica*, 11(1), 441–449, 1975（文献信息见 <https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf>）
4. H.P. Whitaker, J. Yamron, A. Kezer, "Design of Model Reference Adaptive Control Systems for Aircraft", MIT Instrumentation Lab Report R-164, 1958；P.C. Parks, "Lyapunov redesign of model reference adaptive control systems", *IEEE Trans. Automatic Control*, 11(3), 362–367, 1966（均转引自 <https://link.springer.com/content/pdf/10.1007/978-1-4471-4396-3_9.pdf?pdf=inline%20link>）
5. T. Lauvdal, T.I. Fossen, "Robust adaptive ship autopilot with wave filter...", *Int. J. Adaptive Control and Signal Processing*, 1998. <https://www.fossen.biz/publications/1998%20Lauvdal%20and%20Fossen%20IJACSP.pdf>
6. T.I. Fossen, J.P. Strand（等）, "A globally stable adaptive ship autopilot with wave filter using only yaw angle measurements", *Modeling, Identification and Control*, 1996. <https://www.mic-journal.no/PDF/1996/MIC-1996-2-4.pdf>
7. T.I. Fossen, "High Performance Ship Autopilot With Wave Filter", SCSS, 1993. <https://www.fossen.biz/publications/1993%20Fossen%20SCSS.pdf>
8. T. Holzhüter, H. Strauch, "A commercial adaptive autopilot for ships: design and operational experiences", *IFAC Proceedings Volumes*, 20(5), 233–238, 1987（文献信息见 <https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/2773646/Andreas%20Bell%20Martinsen_PhD.pdf>）
9. J. van Amerongen, P.G.M. van der Klugt, H.R. van Nauta Lemke, "Rudder roll stabilization for ships", *Automatica*, 26(4), 679–690, 1990（文献信息见 <https://www.eolss.net/sample-chapters/c18/E6-43-36-03.pdf>）
10. Python Control Systems Library, "Model-Reference Adaptive Control (MRAC) SISO, direct Lyapunov rule"（一阶对象 Lyapunov MRAC 推导示例）. <https://python-control.readthedocs.io/en/0.10.2/examples/mrac_siso_lyapunov.html>
11. P.A. Ioannou, P.V. Kokotovic, "Instability analysis and improvement of robustness of adaptive control"（σ-modification 的原始出处）, *Automatica*, 20(5), 583–594, 1984（文献信息见 <https://arxiv.org/pdf/2202.09834v1.pdf>）
12. K.S. Narendra, A.M. Annaswamy, "A New Adaptive Law for Robust Adaptation Without Persistent Excitation"（e-modification 的原始出处）, *IEEE Trans. Automatic Control*, 32(2), 134–145, 1987（文献信息见 <https://www.intechopen.com/chapters/1175769>）
13. C.E. Rohrs, L. Valavani, M. Athans, G. Stein, "Robustness of continuous-time adaptive control algorithms in the presence of unmodeled dynamics", *IEEE Trans. Automatic Control*, 30(9), 881–889, 1985（文献信息见 <https://dspace.mit.edu/bitstream/handle/1721.1/111744/1004393438-MIT.pdf?sequence=1&isAllowed=y>）
