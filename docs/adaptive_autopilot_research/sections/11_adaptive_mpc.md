## 自适应与学习型 MPC 在船舶运动控制

### 1. 船舶航向/航迹 MPC 基础

模型预测控制（Model Predictive Control, MPC）在每个采样时刻 $t_k$，以当前状态估计 $\hat{x}(t_k)$ 为初值，在线求解一个有限时域开环最优控制问题：

$$
\min_{\boldsymbol{\delta}(\cdot)}\; \sum_{i=0}^{N-1}\Big(\|\psi(t_k+i)-\psi_d\|_Q^2 + \lambda\|\delta(t_k+i)\|_R^2 + \mu\|\Delta\delta(t_k+i)\|_S^2\Big)
$$

$$
\text{s.t.}\quad \dot{x}=f(x,\delta,w),\qquad |\delta|\le\delta_{\max},\qquad |\dot{\delta}|\le\dot{\delta}_{\max},
$$

只执行最优序列的第一个舵角指令，下一采样时刻用新量测重新求解（滚动时域）。预测模型 $f(\cdot)$ 通常取 Nomoto 一阶/二阶模型 $T\dot r+r=K\delta$、3 自由度操纵模型（surge–sway–yaw）或其非线性扩展；$w$ 为风浪流扰动。相对 PID/PD 类自动舵，MPC 的本质优势有两点：**约束被显式写进优化问题**（舵角/舵速饱和在指令生成阶段即被整形，而不是事后限幅截断），以及**代价函数可直接嵌入操舵附加阻力、舵机能耗等物理目标**（与第 2 章 GMV/LQG 的损失函数一脉相承）。

工程上普遍采用"LOS 导引 + MPC 执行"的分层结构：上层视线（LOS）导引把航迹跟踪转化为期望航向 $\psi_d$，下层 MPC 只需完成航向跟踪，避免了在预测时域内同时处理位置与航向的高维问题。代表性基础工作包括：

- **Oh & Sun（2010）**：[LOS-MPC 欠驱动水面船航迹跟踪](https://doi.org/10.1016/j.oceaneng.2009.10.008)（Ocean Engineering），用 LOS 导引律生成航向参考、MPC 在舵角/舵速约束下跟踪，是船舶 LOS-MPC 分层框架的奠基性论文；
- **Li et al.（2009）**：在 MPC 中同时考虑舵角与横摇约束的航迹跟踪（引自 [Xu et al., 2023](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032) 引言）；
- **Sandeepkumar et al.（2022）**：[统一操纵模型 + NMPC 的规则波航迹跟踪](https://www.sciencedirect.com/science/article/abs/pii/S0029801821014864)（Ocean Engineering），在规则波中对比 NMPC 与 PID，并做了水池模型试验验证——是"波浪中 NMPC"少见的实验证据；
- **He et al.（2023）**：[经水池模型试验验证的航迹跟踪 MPC](https://www.vliz.be/imisdocs/publications/98/393698.pdf)（Ocean Engineering，论文全文见 VLIZ 库），针对预定义航迹点与代价插值简化在线优化，进一步降低计算负担。

这些工作确立了船舶 MPC 的基本范式：低维航向回路用线性 MPC/QP 即可，航迹/避碰/动力定位等非线性多约束任务才需要 NMPC/SQP。

### 2. 自适应 MPC：在线辨识 + 重规划，与 STR 的关系

**自适应 MPC** 在第 1 节的框架上增加一条学习支路：用递推辨识（RLS/FFRLS、EKF 联合估计等）在线刷新预测模型的参数（如 Nomoto $K$、$T$ 或水动力导数），再按**确定性等价**原则把最新估计代入预测模型重新求解优化问题。其架构如图 1 所示。

![自适应/学习型 MPC 自动舵架构](../figs/fig_11_adaptive_mpc_structure.png)

**图 1　自适应与学习型 MPC 自动舵典型架构（在线辨识/学习更新预测模型 + 滚动时域优化 + 安全监督层）**

从控制理论谱系看，自适应 MPC 就是**自校正调节器（STR，见第 2 章）的直接后裔与"现代约束版"**：二者共享"在线辨识 + 确定性等价 + 每拍重设计控制器"的骨架，差别仅在于控制器综合环节——STR 解 Diophantine/Riccati 方程得到固定结构反馈律，自适应 MPC 则在每次采样时重新求解一个带约束的有限时域优化问题。因此 Åström–Källström 时代的自适应自动舵，若把最小方差设计方程换成在线 QP，在概念上就是今天的自适应 MPC。近年代表性工作：

- **Huang et al.（2026）**：[FFRLS-MPC：带遗忘因子 RLS + MPC 的鲁棒航向保持](https://pubmed.ncbi.nlm.nih.gov/42260025/)（Scientific Reports），FFRLS 实时辨识操纵模型关键参数，扰动效应被吸收为参数的自适应变化；在静水与风-浪-流扰动仿真中相比 PID 与鲁棒自适应 MPC 基线，响应更快、稳态误差更小——是"STR 骨架 + MPC 综合"思路的近期完整实现；
- **Dai et al.（2020）**：自适应 EKF-FMPC（Fast MPC）轨迹跟踪，用 EKF 在线补偿未建模不确定性（引自 [Xu et al., 2023](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032) 引言）；
- **Liu et al.（2019）**：在线 LS-SVM 辨识 + MPC 的自适应航迹跟踪（同上引言）。

自适应 MPC 的理论短板也与 STR 相同：确定性等价忽略了估计不确定性与控制之间的耦合（对偶控制问题），持续激励条件难以在闭环中严格保证，因此**闭环稳定性证明普遍缺失**，工程上要靠监督切换与约束收紧来弥补。

### 3. 学习型与数据驱动 MPC

当航速、载况、海况变化超出低维参数模型的表达能力时，研究界转向用机器学习模型做 MPC 的预测器。系统性综述见 [Hewing et al., 2020, Learning-Based MPC（Annual Review of Control, Robotics, and Autonomous Systems）](https://doi.org/10.1146/annurev-control-090419-075625)。三条主流路线：

**（a）GP-MPC（高斯过程）**。GP 给出预测均值与方差，方差可直接用于约束收紧（"越不确定越保守"），天然适配安全控制。船舶方向的标志性工作是第 7 章已详述的大阪大学 SPMPC（[Cui et al., 2021, J. Field Robotics](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21990)）：GP 在线学习动力学 + MPC 规划，**真尺船数十次试验学会自动舵**，是 learning-MPC 真船落地的最强证据。纯数据驱动方向，[Xu et al.（2023, Ocean Engineering）](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032) 用相似度稀疏 GP 建立无先验结构的非参数操纵模型，结合 LOS 把航迹跟踪降为航向跟踪，在 MSS 集装箱船模型（含量测噪声与环境扰动）上验证了航向控制与航点跟随；GP 学习类 MPC 的教程式入门见 [arXiv:2404.03689](https://arxiv.org/html/2404.03689v1)。

**（b）Koopman-MPC**。Koopman 算子理论把非线性动力学升维（lifting）为高维线性系统，[Korda & Mezić（2018, Automatica）](https://mgroup.me.ucsb.edu/sites/default/files/publications/koopmanmpc.pdf) 证明提升后的线性预测器可直接嵌入线性 MPC——即"非线性对象、线性 QP 求解"，计算量与线性 MPC 相当。船舶/水下方向已有应用探索，如 [Xia et al.（2025, Ocean Engineering）](https://www.sciencedirect.com/science/article/abs/pii/S0029801825024849) 的 ROV Koopman 鲁棒 MPC（KORMPC）轨迹跟踪。局限在于升维字典（观测函数）靠人工选取或深度学习，外推性与物理可解释性有限。

**（c）SINDy-MPC（稀疏辨识）**。SINDy（[Brunton et al., 2016, PNAS](https://www.pnas.org/doi/10.1073/pnas.1517384113)）从候选函数库中稀疏回归出显式微分方程，SINDYc 扩展到含控系统；[Kaiser et al.（2018, Proc. R. Soc. A）](https://arxiv.org/pdf/1711.05501) 证明 SINDy 模型可在**低数据极限**下支撑 MPC。对船舶场景，SINDy 的吸引力在于：以 Nomoto/Abkowitz 水动力项为候选库，辨识结果直接是物理可解释的操纵方程（如自动"发现" $r^3$、$v|r|$ 等非线性项），而非黑箱。目前船舶 SINDy-MPC 仍处早期，未见实船报道。

**（d）Tube-based 鲁棒 MPC 处理波浪扰动**。波浪对航向回路是有界加性扰动，tube MPC（[Mayne, Seron & Raković, 2005, Automatica](https://doi.org/10.1016/j.automatica.2005.01.011)）把闭环分解为"标称轨迹优化 + 误差反馈管（tube）"：对标称系统求解 MPC，同时用鲁棒不变集收紧约束，保证实际轨迹始终落在标称轨迹周围的管道内。船舶方向的代表是 [Nascimento et al.（KTH/ABB，2018）](https://search.abb.com/library/Download.aspx?DocumentID=9AKK107991A2057&LanguageCode=en&DocumentPartId=&Action=Launch) 的动力定位 tube-MPC：线性化船舶模型 + LMI 综合鲁棒不变集 + Pontryagin 差集收紧约束，在执行器饱和与有界环境扰动下保证约束满足。更早的 Li & Sun（2011）扰动补偿 MPC 则走"扰动观测器 + MPC"路线抑制风浪扰动（引自 [Xu et al., 2023](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032) 引言）。

四条路线的定位对比：

| 路线 | 预测模型 | 不确定性量化 | 计算代价 | 船舶应用成熟度 |
|---|---|---|---|---|
| 自适应 MPC（RLS） | 参数化（Nomoto 等） | 无（确定性等价） | 极低 | 中：仿真充分，实船少 |
| GP-MPC | 非参数（均值+方差） | 有（方差收紧约束） | 中（需稀疏化） | 高：真尺船验证（Cui 2021） |
| Koopman-MPC | 升维线性模型 | 弱 | 低（线性 QP） | 低：ROV 仿真/初步实验 |
| SINDy-MPC | 显式稀疏方程 | 弱 | 低 | 很低：尚无船舶实例 |
| Tube/鲁棒 MPC | 任意标称模型 | 有（不变集） | 低~中 | 中：DP 仿真验证 |

### 4. 实时性与嵌入式实现：25 kn 游艇控制周期

对本项目 25 kn（约 12.9 m/s）游艇做量级估算：船长数十米量级时 Nomoto 时间常数 $T$ 为秒级，航向回路带宽低，**航向控制采样周期取 50–100 ms（10–20 Hz）已非常充裕**；预测时域取 $3\sim5$ 倍 $T$（约 10–20 s）、离散步数 $N=20\sim40$ 即可覆盖机动动态。此时 QP 规模仅几十个决策变量，实时性完全不是瓶颈：

- **QP 求解器**：HPIPM（[Frison & Diehl, 2020](https://arxiv.org/pdf/2003.02547v2)）对这类小规模部分凝聚 QP 的求解时间**稳定在 1 ms 以下**；qpOASES（[Ferreau et al., 2014](https://doi.org/10.1007/s12532-014-0071-1)，参数化有效集、支持暖启动）与 OSQP（[Stellato et al., 2020](https://doi.org/10.1007/s12532-020-00179-2)，ADMM）在同类规模上为毫秒级；
- **NMPC 框架**：acados（[Verschueren et al., 2021](https://publications.syscop.de/Verschueren2021.pdf)）采用实时迭代（RTI）方案——每个采样周期只执行一次 SQP 迭代，把大部分计算挪到反馈到达前完成，无人机等非线性平台上单次迭代约 0.1–数 ms；
- **真正的计算大头是学习模型推理**而非优化：非稀疏 GP 推理随训练样本数 $O(n^2)\sim O(n^3)$ 增长，数百样本即达十毫秒级，必须稀疏化（诱导点/相似度剪枝，Xu et al. 2023 正是为此引入稀疏 GP）；Koopman 与 SINDy 模型的推理只是矩阵乘法，代价可忽略。

图 2 给出各环节的耗时量级与控制周期预算的对比：对 2–3 状态的航向 MPC，QP 求解仅占 20 Hz 控制周期的 **1–2%**，即使叠加稀疏 GP 推理也不足 10%。换言之，**25 kn 游艇的慢动态 + 现代嵌入式求解器，使"嵌入式算不动 MPC"这一传统顾虑在本应用量级上不成立**；真正的门槛在于辨识/学习模块的鲁棒性、监督逻辑与认证成本。

![MPC 计算预算量级示意](../figs/fig_11_mpc_runtime_budget.png)

**图 2　25 kn 游艇航向 MPC 各环节计算耗时量级与控制周期预算对比（依据 HPIPM/acados 公开基准量级，示意非实测）**

### 5. 代表性文献与实船证据

| 工作 | 年代 | 方法 | 平台与验证级别 |
|---|---|---|---|
| Oh & Sun | 2010 | LOS-MPC | 仿真（欠驱动水面船模型） |
| Li & Sun | 2011 | 扰动补偿 MPC | 仿真（航向控制） |
| Nascimento et al.（KTH/ABB） | 2018 | Tube-MPC | 仿真（DP 船） |
| Cui et al.（大阪大学） | 2019–2021 | GP model-based RL + MPC（SPMPC） | **真尺船实验**（数十次试验收敛） |
| Sandeepkumar et al. | 2022 | NMPC（规则波） | 仿真 + **水池模型试验** |
| Martinsen et al.（NTNU） | 2022 | RL 在线调参 MPC | **USV 实验**（引自 [Xu et al. 2023](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032) 引言） |
| He et al. | 2023 | 航迹跟踪 MPC | **水池模型试验验证** |
| Xu et al. | 2023 | 稀疏 GP-NMPC | 高保真仿真（MSS 集装箱船 + 噪声扰动） |
| Xia et al. | 2025 | Koopman 鲁棒 MPC | 仿真（ROV） |
| Huang et al. | 2026 | FFRLS-MPC 自适应 | 仿真（风-浪-流扰动） |

可以看出：**船舶 MPC 的实船证据链整体偏薄**——最高级别的证据是大阪大学的真尺船 SPMPC 与少量水池模型试验，全尺度商船的 MPC 自动舵在公开文献中未见报道；自适应 MPC 与 Koopman/SINDy-MPC 目前基本停留在仿真阶段。这与第 2 章 KADPIL 时代自适应自动舵"130 余次实船试验、商用化运营"的证据强度形成鲜明反差，也说明 MPC 类方法的主要瓶颈不在性能，而在**工程化与认证**。

### 6. 与本项目 ESC + 极点配置方案的对比定位

本项目现行方案（PD + 海况×船速增益调度 + ESC 在线调参，Nomoto $K/T$ 在线辨识 + 极点配置基线）与自适应/学习型 MPC 的关系，可精确表述为：**极点配置 + 辨识相当于自适应 MPC 的"单步、无约束、线性二次型"特例**——LQ 极点配置本身就是一个无限时域无约束 MPC 的闭式解；MPC 把它推广到多步预测与显式约束，ESC 则把"重设计"环节换成了不依赖模型的在线参数优化。

| 维度 | ESC + 极点配置（本项目） | 自适应/学习型 MPC |
|---|---|---|
| 预测能力 | 无显式预测（反馈 + 极点） | 多步预测，可预整形指令 |
| 约束处理 | 事后限幅截断 | 优化问题内显式处理舵角/舵速/横摇 |
| 在线计算 | 微秒级（查表 + 递推） | 毫秒级 QP/SQP + 学习推理 |
| 稳定性保证 | 有（极点配置 + ESC 平均化理论） | 自适应 MPC 普遍缺失；tube/鲁棒 MPC 有 |
| 模型依赖 | Nomoto $K/T$ 即可 | 线性 MPC 同级；学习路线需更多数据 |
| 认证与调试 | 结构简单、参数物理意义明确 | 优化器与学习模块调试成本高，船级社认证无先例 |
| 收益场景 | 航向保持已接近结构内最优 | 大舵角机动、强约束、多目标（能耗+精度）权衡 |

对本项目的借鉴点：

1. **辨识成果直接复用**：现有在线辨识的 Nomoto $K/T$ 就是线性 MPC 的预测模型，可在仿真中低成本搭建"辨识 + 线性 MPC"原型，量化"显式约束处理 + 多步预测"相对现行 PD 限幅方案的实际收益（预期主要在大舵角转向与海况切换瞬态，而非静水定常航向保持），再决定是否值得工程化；
2. **约束收紧/tube 思想下沉到现有方案**：tube MPC 的"按扰动界收紧约束"思想可低成本移植为 ESC 扰动幅度与舵角限幅值随海况估计自适应收紧/放宽的调度规则，无需引入完整 MPC；
3. **学习路线选型**：若未来引入学习组件，优先 GP-MPC/SPMPC 式"小数据 + 不确定性量化"路线（真船证据最强），Koopman/SINDy 适合作为离线模型发现工具补充辨识候选结构；model-free RL 式的高样本路线（见第 7 章）不适合在线直接训练；
4. **算力结论**：图 2 的量级分析表明嵌入式实时性不是否决项，决策变量应聚焦在收益是否覆盖调试与认证成本。

### 7. 参考资料

1. Oh & Sun, 2010, [Path following of underactuated marine surface vessels using line-of-sight based model predictive control](https://doi.org/10.1016/j.oceaneng.2009.10.008)
2. Sandeepkumar et al., 2022, [A unified ship manoeuvring model with a nonlinear model predictive controller for path following in regular waves](https://www.sciencedirect.com/science/article/abs/pii/S0029801821014864)
3. Huang et al., 2026, [FFRLS-MPC: An adaptive model predictive control method for robust ship course keeping](https://pubmed.ncbi.nlm.nih.gov/42260025/)（Scientific Reports）
4. Xu et al., 2023, [Data-driven model predictive control for ships with Gaussian process](https://www.sciencedirect.com/science/article/abs/pii/S0029801822027032)（Ocean Engineering）
5. Cui et al., 2021, [Autonomous boat driving system using sample-efficient MPC-based reinforcement learning](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21990)（J. Field Robotics）；预印本 [arXiv:1901.07905](https://arxiv.org/abs/1901.07905)
6. Hewing et al., 2020, [Learning-Based Model Predictive Control: Toward Safe Learning in Control](https://doi.org/10.1146/annurev-control-090419-075625)；GP-MPC 教程：[arXiv:2404.03689](https://arxiv.org/html/2404.03689v1)
7. Korda & Mezić, 2018, [Linear predictors for nonlinear dynamical systems: Koopman operator meets model predictive control](https://mgroup.me.ucsb.edu/sites/default/files/publications/koopmanmpc.pdf)（Automatica）
8. Xia et al., 2025, [Trajectory tracking control of work-class ROVs using KORMPC strategy](https://www.sciencedirect.com/science/article/abs/pii/S0029801825024849)
9. Brunton et al., 2016, [Discovering governing equations from data by sparse identification of nonlinear dynamical systems (SINDy)](https://www.pnas.org/doi/10.1073/pnas.1517384113)；Kaiser et al., 2018, [Sparse identification of nonlinear dynamics for model predictive control in the low-data limit](https://arxiv.org/pdf/1711.05501)
10. Mayne, Seron & Raković, 2005, [Robust model predictive control of constrained linear systems with bounded disturbances](https://doi.org/10.1016/j.automatica.2005.01.011)（Automatica，tube MPC 奠基论文）
11. Nascimento et al.（KTH/ABB）, 2018, [Tube-based model predictive control for dynamic positioning of marine vessels](https://search.abb.com/library/Download.aspx?DocumentID=9AKK107991A2057&LanguageCode=en&DocumentPartId=&Action=Launch)
12. Frison & Diehl, 2020, [HPIPM: a high-performance quadratic programming framework for model predictive control](https://arxiv.org/pdf/2003.02547v2)；Ferreau et al., 2014, [qpOASES](https://doi.org/10.1007/s12532-014-0071-1)；Stellato et al., 2020, [OSQP](https://doi.org/10.1007/s12532-020-00179-2)；Verschueren et al., 2021, [acados: a modular open-source framework for fast embedded optimal control](https://publications.syscop.de/Verschueren2021.pdf)
