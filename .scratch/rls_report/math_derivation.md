# RLS 方案数学推导与算法实现详解——Nomoto 模型在线辨识的数学核心

> 对象工程：`control/`（自动舵控制算法，C++17 header-only 库，namespace `ar`）
> 核心源码：`control/include/nomoto_identifier.hpp`（辨识器）、`control/include/auto_tuner.hpp`（消费方）、`control/include/autopilot_core.hpp`（调用方）
> 本文约定：所有公式使用 KaTeX 兼容 LaTeX 记法；代码引用格式为 `文件路径:行号`；所有数值默认值以源码为准。
> 符号约定：$\delta$ 为舵角（deg），$r=\dot\psi$ 为艏摇率/转向率（deg/s），$\dot r$ 为转向加速度（deg/s²），$K$、$T$ 为 Nomoto 一阶模型参数。

## 0. 全链路速览

```
船舶(δ→r) → NomotoIdentifier.update(δ,r,dt)   [50 Hz 调用，1 Hz RLS 拍]
          → IdSnapshot{excited,valid,K,T}     autopilot_core.hpp:176-178
          → AutoTuner.tick()                  auto_tuner.hpp:150-183
              ├─ 漂移门 ±25%                   auto_tuner.hpp:308-313
              ├─ 极点配置 polePlacement        auto_tuner.hpp:228-237
              └─ 稳定门 stabilityCheck         auto_tuner.hpp:241-254
          → PD 增益基线 → ESC 微调
```

本文依次推导：对象建模（§1）→ 回归化（§2）→ 加权最小二乘（§3）→ RLS 递推（§4）→ 遗忘因子（§5）→ 激励条件（§6）→ 自降频采样（§7）→ 收敛与误差（§8）→ 下游衔接（§9）。

---

## 1. 被控对象建模：一阶 Nomoto 模型的物理来历

### 1.1 从船舶操纵性方程出发

船舶水面运动在"定常直航 + 小扰动"假设下，只保留横荡（sway，速度 $v$）与艏摇（yaw，角速度 $r$）两个自由度的线性化方程（Abkowitz 线性模型）：

$$
\begin{aligned}
\text{横荡：}\quad & (m - Y_{\dot v})\,\dot v + (m x_G - Y_{\dot r})\,\dot r = Y_v\,v + (Y_r - mU)\,r + Y_\delta\,\delta \\
\text{艏摇：}\quad & (m x_G - N_{\dot v})\,\dot v + (I_z - N_{\dot r})\,\dot r = N_v\,v + N_r\,r + N_\delta\,\delta
\end{aligned}
$$

其中 $m$ 为船体质量，$I_z$ 为艏摇转动惯量，$x_G$ 为重心纵向位置，$U$ 为前速；$Y_v, Y_r, Y_\delta$ 与 $N_v, N_r, N_\delta$ 分别为横荡力、艏摇力矩对 $v, r, \delta$ 的流体动力导数；$Y_{\dot v}, Y_{\dot r}, N_{\dot v}, N_{\dot r}$ 为附加质量（附加惯量）导数。水动力导数随船型、航速、载况、水深而变——这正是 $K/T$ 需要**在线**辨识而非一次标定了事的物理根源。

### 1.2 消去横荡：一阶 Nomoto 模型

舵角变化引起的艏摇响应远慢于横荡力的建立（横荡压力平衡在秒的零头量级即完成），因此对横荡方程取**准定常近似** $\dot v \approx 0$、$\dot r$ 的横荡耦合项忽略不计：

$$
v \approx \frac{(mU - Y_r)\,r - Y_\delta\,\delta}{Y_v}
$$

（注意 $Y_v<0$，分母非零。）代入艏摇方程并同样略去 $\dot v$ 项：

$$
(I_z - N_{\dot r})\,\dot r
= N_v\,\frac{(mU - Y_r)\,r - Y_\delta\,\delta}{Y_v} + N_r\,r + N_\delta\,\delta
$$

把含 $r$ 的项归并到左边、含 $\delta$ 的项留在右边：

$$
\underbrace{(I_z - N_{\dot r})}_{\text{有效艏摇惯性 } \mathcal I}\,\dot r
\;+\;
\underbrace{\Bigl(\frac{N_v\,(Y_r - mU)}{Y_v} - N_r\Bigr)}_{\text{等效艏摇阻尼 } \mathcal N}\,r
\;=\;
\underbrace{\Bigl(N_\delta - \frac{N_v Y_\delta}{Y_v}\Bigr)}_{\text{舵效艏摇力矩系数 } \mathcal M}\,\delta
$$

两边同除以等效阻尼 $\mathcal N$，即得**一阶 Nomoto 模型**：

$$
\boxed{\;T\,\dot r + r = K\,\delta\;}
$$

三个集总参数与流体动力导数的关系（示意，符号随导数符号约定而异）：

$$
T = \frac{\mathcal I}{\mathcal N}, \qquad K = \frac{\mathcal M}{\mathcal N}
$$

对航向稳定的船舶，等效阻尼 $\mathcal N>0$（在常用符号约定下），故 $T>0$、$K>0$——这是 `valid()` 物理门限要求 $K>0$、$T\ge 1$ 的力学依据（`nomoto_identifier.hpp:119-124`）。

### 1.3 参数的物理意义与量纲

| 参数 | 名称 | 定义 | 物理意义 | 量纲（本工程单位制） | 工程取值范围 |
|---|---|---|---|---|---|
| $K$ | 旋回性指数 | $\mathcal M/\mathcal N$ | 单位舵角产生的**稳态**转向率：$\delta$ 恒定时每度舵角换来 $K$ 度/秒的转首角速度 | $\mathrm{s^{-1}}$（(deg/s)/deg） | $(0,2]$，典型 $0.05\sim0.5$ |
| $T$ | 追随性指数 | $\mathcal I/\mathcal N$ | 艏摇惯性相对阻尼的时间尺度：转首响应滞后舵角约 $T$ 秒 | $\mathrm{s}$ | $[1,60]$ |

对 $\delta$ 取拉氏变换（零初值），$\delta\to r$ 是一阶惯性环节：

$$
\frac{r(s)}{\delta(s)} = \frac{K}{Ts+1}
$$

阶跃响应 $r(t) = K\delta_0\bigl(1 - e^{-t/T}\bigr)$：稳态增益 $K$、上升时间由 $T$ 决定，两个参数各管一件事——这决定了 §6 的可辨识性结构：$K$ 由**稳态段**数据决定，$T$ 只能由**瞬态段**数据决定。

再串一级航向积分 $\dot\psi = r$，舵角到航向的通道为：

$$
\frac{\psi(s)}{\delta(s)} = \frac{K}{s\,(Ts+1)}
$$

即下游极点配置面对的被控对象（§9）。

---

## 2. 回归化推导：从微分方程到 $y_k = \varphi_k^{\mathsf T}\theta$

### 2.1 后向差分离散

辨识器拿不到 $\dot r$ 的直接量测（罗经/惯导给的是 $\psi$ 或 $r$），只能用差分近似。采用**后向欧拉**：

$$
\dot r\big|_{t_k} \;\approx\; \frac{r_k - r_{k-1}}{\Delta t}
$$

其中 $\Delta t$ 为两次用于递推的样本之间的真实间隔（实现中为两次 RLS 拍之间的间隔，见 §7）。代入 $T\dot r + r = K\delta$：

$$
T\,\frac{r_k - r_{k-1}}{\Delta t} + r_k = K\,\delta_k
$$

把未知参数 $K$、$T$ 整理到"已知量 × 参数"的位置上——将 $r_k$ 单独留在左边：

$$
r_k = K\,\delta_k - T\,\frac{r_k - r_{k-1}}{\Delta t}
$$

记差分转向加速度

$$
dr_k \;=\; \frac{r_k - r_{k-1}}{\Delta t}
$$

则方程变成标准的**线性参数回归**：

$$
\boxed{\;y_k = \varphi_k^{\mathsf T}\,\theta\;}
\qquad
y_k = r_k,\qquad
\varphi_k = \begin{bmatrix} \delta_k \\ -dr_k \end{bmatrix},\qquad
\theta = \begin{bmatrix} K \\ T \end{bmatrix}
$$

逐分量验证：$\varphi_k^{\mathsf T}\theta = K\delta_k + T\cdot(-dr_k) = K\delta_k - T\,dr_k = r_k = y_k$。✓

**关键点**：$K$、$T$ 以线性方式出现，整个辨识问题归结为（加权）线性最小二乘，无需任何非线性迭代；这是选择"对 $r$ 回归"而非"对 $\psi$ 回归"或"直接拟合传递函数"的原因。

实现对应：回归化写在模块头注释（`nomoto_identifier.hpp:3-4`），差分在 `nomoto_identifier.hpp:65-67`，$\varphi$ 构造在 `nomoto_identifier.hpp:76`，$y=r$ 在 `nomoto_identifier.hpp:88`。

### 2.2 差分 $dr$ 的噪声放大问题

差分是回归化的必要代价。设 $r$ 的量测含零均值白噪声 $n_k$，$\mathrm{Var}[n_k] = \sigma_r^2$，相邻样本独立。则 $dr$ 中的噪声分量为

$$
\nu_k = \frac{n_k - n_{k-1}}{\Delta t},
\qquad
\mathrm{Var}[\nu_k] = \frac{\mathrm{Var}[n_k] + \mathrm{Var}[n_{k-1}]}{\Delta t^2} = \frac{2\sigma_r^2}{\Delta t^2}
$$

噪声标准差 $\sigma_\nu = \sqrt{2}\,\sigma_r / \Delta t$，**与采样间隔成反比**：

- $\Delta t = 0.02\,\mathrm{s}$（50 Hz 直接差分）：$\sigma_\nu \approx 70.7\,\sigma_r$；
- $\Delta t = 1\,\mathrm{s}$（1 Hz RLS 拍差分）：$\sigma_\nu \approx 1.41\,\sigma_r$。

同一传感器，仅把差分间隔从 20 ms 放大到 1 s，$dr$ 噪声标准差压低 **50 倍**。而只要船舶动态远慢于 $1/\Delta t$（$T\ge1\,\mathrm{s}$ 保证），差分对**真实信号**的估计 $\dot r$ 基本不损失（见 §7.2）。信噪比因此近似正比于 $\Delta t$ 改善——这是 §7 自降频设计的核心收益。

实现中首拍无差分锚点，置 $dr=0$ 跳过（`first_` 标志，`nomoto_identifier.hpp:66`）；被拒绝的非法输入拍**不写** `prevR_`，防止毛刺成为下一拍的差分锚点（`nomoto_identifier.hpp:50-55`，详见 §6.4）。

### 2.3 归一化：dScale/drScale 消除量级病态

原始回归量两个分量量级悬殊：舵角 $\delta$ 典型变化幅度 $3^\circ\sim35^\circ$（激励门限 3°、舵限幅 35°），差分加速度 $dr$ 典型幅度 $0.2\sim1\ \mathrm{deg/s^2}$。设二者的 RMS 幅度为 $\delta_{\mathrm{rms}}\approx 10$、$dr_{\mathrm{rms}}\approx 0.5$，则信息矩阵

$$
R_N = \sum_{k=1}^{N}\lambda^{N-k}\,\varphi_k\varphi_k^{\mathsf T}
= \begin{bmatrix} \sum \lambda^{N-k}\delta_k^2 & -\sum \lambda^{N-k}\delta_k dr_k \\[1mm] -\sum \lambda^{N-k}\delta_k dr_k & \sum \lambda^{N-k}dr_k^2 \end{bmatrix}
$$

的对角元之比达 $(\delta_{\mathrm{rms}}/dr_{\mathrm{rms}})^2 \approx (10/0.5)^2 = 400$，工况变化时可达 $10^3\sim10^4$。$R_N$ 的病态直接传染给协方差 $P_N = R_N^{-1}$（条件数相同量级），使增益向量 $L_N$ 的数值计算在 $T$ 方向上分辨率粗劣、对舍入误差敏感。

实现做法：给两个回归分量各配一个归一化尺度，令其均为 $O(1)$（`nomoto_identifier.hpp:24-26`）：

$$
\varphi_k = \begin{bmatrix} \delta_k / \mathrm{dScale} \\ -dr_k / \mathrm{drScale} \end{bmatrix}
= \begin{bmatrix} \delta_k / 50 \\ -dr_k / 1 \end{bmatrix}
$$

这等价于对参数做对角预 conditioning：记 $S = \mathrm{diag}(1/\mathrm{dScale},\,1/\mathrm{drScale})$，则归一化回归 $\varphi = S\varphi_{\mathrm{raw}}$，归一化参数 $\theta_n = S^{-1}\theta$，即

$$
\theta_n = \begin{bmatrix} K\cdot\mathrm{dScale} \\ T\cdot\mathrm{drScale} \end{bmatrix} = \begin{bmatrix} 50K \\ T \end{bmatrix}
$$

估计完成后反归一化还原物理量（`nomoto_identifier.hpp:112-115`）：

$$
K = \frac{\theta_0}{\mathrm{dScale}} = \frac{\theta_0}{50},\qquad T = \frac{\theta_1}{\mathrm{drScale}} = \theta_1
$$

效果：回归分量 $\varphi_0\in[-0.7,\,0.7]$（35°/50）、$\varphi_1$ 为 $O(0.2\sim1)$，信息矩阵条件数从 $10^3\sim10^4$ 压到 $O(1\sim10^2)$；同时归一化参数 $\theta_0 = 50K \in (0,100]$、$\theta_1 = T \in [1,60]$ 同为 $O(1\sim10^2)$，使协方差初值可以取统一的 $p_0 = 1$（`nomoto_identifier.hpp:23` 注释"归一化后回归量 O(1)，p0 取 1 量级"）。注意归一化不改变最小二乘的数学解，只改变数值条件与 $P$ 阵的坐标标度——$P$ 的保底值 1e-3（§5.4）也因此在归一化坐标下才有统一意义。

---

## 3. 加权最小二乘目标与正规方程

### 3.1 为什么要遗忘因子：时变参数跟踪

普通（等权）最小二乘隐含"参数恒定"假设：第 $N$ 步的估计是全部 $N$ 个样本的等权平均，样本越多，旧数据占比越大，估计对新工况的响应越迟钝——$P_N \sim O(1/N)\to 0$，增益 $L_N\to 0$，估计器"睡死"。

但 §1 已说明 $\mathcal I,\mathcal N,\mathcal M$ 随航速、载况、水深、海况**慢变**，$K/T$ 是时变参数。需要的是"旧数据逐渐遗忘、新数据主导"的估计器。标准做法：给第 $k$ 个样本配指数衰减权重 $\lambda^{N-k}$，$0<\lambda\le1$ 为**遗忘因子**：

$$
\boxed{\;J_N(\theta) = \sum_{k=1}^{N}\lambda^{\,N-k}\,\bigl(y_k - \varphi_k^{\mathsf T}\theta\bigr)^2\;}
\qquad (\lambda = 0.98,\ \texttt{nomoto\_identifier.hpp:18})
$$

当前样本权重 1，$a$ 拍之前的样本权重 $\lambda^a$：$\lambda=1$ 退化为普通最小二乘；$\lambda$ 越小遗忘越快。权重序列 $\{\lambda^a\}_{a=0}^{\infty}$ 之和 $= 1/(1-\lambda)$ 是"有效记忆窗口"的样本数（§5.1 展开）。

### 3.2 对 $\theta$ 求导：正规方程的完整推导

$J_N$ 是 $\theta$ 的二次型，极小值在梯度为零处。逐项展开：

$$
J_N(\theta) = \sum_{k=1}^{N}\lambda^{N-k}\bigl(y_k^2 - 2y_k\varphi_k^{\mathsf T}\theta + \theta^{\mathsf T}\varphi_k\varphi_k^{\mathsf T}\theta\bigr)
$$

对 $\theta$ 求梯度。用恒等式 $\nabla_\theta(\varphi^{\mathsf T}\theta) = \varphi$、$\nabla_\theta(\theta^{\mathsf T}\varphi\varphi^{\mathsf T}\theta) = 2\varphi\varphi^{\mathsf T}\theta$：

$$
\nabla_\theta J_N
= \sum_{k=1}^{N}\lambda^{N-k}\bigl(-2y_k\varphi_k + 2\varphi_k\varphi_k^{\mathsf T}\theta\bigr)
= -2\sum_{k=1}^{N}\lambda^{N-k}\,\varphi_k\bigl(y_k - \varphi_k^{\mathsf T}\theta\bigr)
$$

置零，移项得**加权正规方程**：

$$
\boxed{\;\underbrace{\Bigl(\sum_{k=1}^{N}\lambda^{N-k}\varphi_k\varphi_k^{\mathsf T}\Bigr)}_{R_N\ \text{（加权信息矩阵）}}\theta
\;=\;
\underbrace{\sum_{k=1}^{N}\lambda^{N-k}\varphi_k\,y_k}_{r_N\ \text{（加权互相关向量）}}\;}
$$

Hessian 矩阵 $\nabla^2 J_N = 2R_N$：只要 $R_N$ 正定（§6 的持续激励条件保证），$J_N$ 是严格凸二次型，正规方程的解唯一且为全局最小：

$$
\theta_N = R_N^{-1}\,r_N
$$

批处理形式每次要解 $2\times2$ 线性方程组——本问题维数低，直接求逆并非不可行，但递推形式 $O(n^2)$ 每拍更新、无需存储历史数据、且天然携带协方差信息（§4.5），是在线辨识的标准选择。下一节由 $R_N, r_N$ 的递推结构出发，逐步推出 RLS 三式。

---

## 4. RLS 递推三式的完整推导

### 4.1 信息矩阵与互相关向量的递推结构

由定义出发，把第 $N$ 步求和式的最后一项拆出来：

$$
R_N = \sum_{k=1}^{N}\lambda^{N-k}\varphi_k\varphi_k^{\mathsf T}
= \lambda\underbrace{\sum_{k=1}^{N-1}\lambda^{N-1-k}\varphi_k\varphi_k^{\mathsf T}}_{R_{N-1}} + \varphi_N\varphi_N^{\mathsf T}
$$

$$
\boxed{\;R_N = \lambda\,R_{N-1} + \varphi_N\varphi_N^{\mathsf T}\;}
\qquad\text{同理}\qquad
\boxed{\;r_N = \lambda\,r_{N-1} + \varphi_N\,y_N\;}
$$

遗忘因子的力学读法：**旧信息整体乘 $\lambda$ 衰减一拍，再叠加新信息**。这就是 §3.1 指数加权在递推域的精确体现。

### 4.2 矩阵求逆引理（Woodbury / Sherman–Morrison）

**引理**：设 $A$ 可逆，$u$ 为列向量，且 $1 + u^{\mathsf T}A^{-1}u \neq 0$，则秩一更新 $A + uu^{\mathsf T}$ 可逆，且

$$
\bigl(A + uu^{\mathsf T}\bigr)^{-1} = A^{-1} - \frac{A^{-1}uu^{\mathsf T}A^{-1}}{1 + u^{\mathsf T}A^{-1}u}
$$

**验证**（右乘 $(A + uu^{\mathsf T})$ 应得单位阵）：

$$
\begin{aligned}
\Bigl(A^{-1} - \frac{A^{-1}uu^{\mathsf T}A^{-1}}{1 + u^{\mathsf T}A^{-1}u}\Bigr)\bigl(A + uu^{\mathsf T}\bigr)
&= I + A^{-1}uu^{\mathsf T} - \frac{A^{-1}uu^{\mathsf T} + A^{-1}u\,(u^{\mathsf T}A^{-1}u)\,u^{\mathsf T}}{1 + u^{\mathsf T}A^{-1}u} \\
&= I + A^{-1}uu^{\mathsf T} - \frac{\bigl(1 + u^{\mathsf T}A^{-1}u\bigr)\,A^{-1}uu^{\mathsf T}}{1 + u^{\mathsf T}A^{-1}u} = I \quad\blacksquare
\end{aligned}
$$

意义：$R_N$ 每拍都要"求逆"，但秩一更新的逆可以用旧逆**显式写出**，无需重新求逆。

### 4.3 协方差更新式与增益向量

定义协方差矩阵（信息矩阵之逆）

$$
P_N = R_N^{-1}
$$

对 $R_N = \lambda R_{N-1} + \varphi_N\varphi_N^{\mathsf T}$ 用引理，取 $A = \lambda R_{N-1}$（故 $A^{-1} = \dfrac{1}{\lambda}P_{N-1}$）、$u = \varphi_N$：

$$
P_N = \frac{1}{\lambda}P_{N-1}
- \frac{\bigl(\frac{1}{\lambda}P_{N-1}\bigr)\varphi_N\varphi_N^{\mathsf T}\bigl(\frac{1}{\lambda}P_{N-1}\bigr)}
{1 + \varphi_N^{\mathsf T}\bigl(\frac{1}{\lambda}P_{N-1}\bigr)\varphi_N}
$$

第二项分子分母同乘 $\lambda^2$：

$$
P_N = \frac{1}{\lambda}P_{N-1}
- \frac{1}{\lambda}\cdot\frac{P_{N-1}\,\varphi_N\varphi_N^{\mathsf T}P_{N-1}}{\lambda + \varphi_N^{\mathsf T}P_{N-1}\varphi_N}
$$

定义**增益向量**（注意其分母是标量）：

$$
\boxed{\;L_N = \frac{P_{N-1}\,\varphi_N}{\lambda + \varphi_N^{\mathsf T}P_{N-1}\,\varphi_N}\;}
$$

则协方差更新压缩为：

$$
\boxed{\;P_N = \frac{1}{\lambda}\Bigl(P_{N-1} - L_N\,\varphi_N^{\mathsf T}P_{N-1}\Bigr)\;}
$$

**一个有用的恒等式**（下一步推导要用）：由 $L_N$ 定义有 $P_{N-1}\varphi_N = L_N\bigl(\lambda + \varphi_N^{\mathsf T}P_{N-1}\varphi_N\bigr)$，于是

$$
P_N\varphi_N = \frac{1}{\lambda}\Bigl(P_{N-1}\varphi_N - L_N\,\varphi_N^{\mathsf T}P_{N-1}\varphi_N\Bigr)
= \frac{1}{\lambda}\Bigl(L_N\lambda + L_N\varphi_N^{\mathsf T}P_{N-1}\varphi_N - L_N\varphi_N^{\mathsf T}P_{N-1}\varphi_N\Bigr) = L_N
$$

即 $\;P_N\varphi_N = L_N$：增益向量就是"新回归量经最新协方差加权后的方向"。

### 4.4 参数更新式

由 $\theta_N = P_N r_N$，代入 $r_N = \lambda r_{N-1} + \varphi_N y_N$ 与 $P_N$ 的更新式：

$$
\theta_N = \frac{1}{\lambda}\Bigl(P_{N-1} - L_N\varphi_N^{\mathsf T}P_{N-1}\Bigr)\Bigl(\lambda\,r_{N-1} + \varphi_N y_N\Bigr)
$$

展开四项：

$$
\theta_N = \underbrace{P_{N-1}r_{N-1}}_{\theta_{N-1}}
\;-\; L_N\varphi_N^{\mathsf T}\underbrace{P_{N-1}r_{N-1}}_{\theta_{N-1}}
\;+\; \frac{1}{\lambda}P_{N-1}\varphi_N\,y_N
\;-\; \frac{1}{\lambda}L_N\,\bigl(\varphi_N^{\mathsf T}P_{N-1}\varphi_N\bigr)\,y_N
$$

后两项合并（提出 $\frac{1}{\lambda}y_N$，并代入 $P_{N-1}\varphi_N = L_N(\lambda + \varphi_N^{\mathsf T}P_{N-1}\varphi_N)$）：

$$
\frac{1}{\lambda}\Bigl[L_N\bigl(\lambda + \varphi_N^{\mathsf T}P_{N-1}\varphi_N\bigr) - L_N\bigl(\varphi_N^{\mathsf T}P_{N-1}\varphi_N\bigr)\Bigr]y_N
= \frac{1}{\lambda}\,L_N\lambda\,y_N = L_N\,y_N
$$

于是

$$
\theta_N = \theta_{N-1} - L_N\,\varphi_N^{\mathsf T}\theta_{N-1} + L_N\,y_N
= \theta_{N-1} + L_N\Bigl(y_N - \varphi_N^{\mathsf T}\theta_{N-1}\Bigr)
$$

定义**先验残差（新息）** $e_N = y_N - \varphi_N^{\mathsf T}\theta_{N-1}$——用旧参数对本拍输出的预测误差——得参数更新式：

$$
\boxed{\;\theta_N = \theta_{N-1} + L_N\bigl(y_N - \varphi_N^{\mathsf T}\theta_{N-1}\bigr)\;}
$$

### 4.5 RLS 三式汇总与逐行代码对照

$$
\boxed{\;
\begin{aligned}
L_N &= \frac{P_{N-1}\,\varphi_N}{\lambda + \varphi_N^{\mathsf T}P_{N-1}\varphi_N}
&& \text{增益} & \texttt{nomoto\_identifier.hpp:79-85} \\
e_N &= y_N - \varphi_N^{\mathsf T}\theta_{N-1}
&& \text{新息} & \texttt{nomoto\_identifier.hpp:88-90} \\
\theta_N &= \theta_{N-1} + L_N\,e_N
&& \text{参数更新} & \texttt{nomoto\_identifier.hpp:93-94} \\
P_N &= \frac{1}{\lambda}\Bigl(P_{N-1} - L_N\varphi_N^{\mathsf T}P_{N-1}\Bigr)
&& \text{协方差更新} & \texttt{nomoto\_identifier.hpp:97-103}
\end{aligned}\;}
$$

代码逐行对应（2 维显式展开，无矩阵库依赖）：

- `Pphi[2]` $= P_{N-1}\varphi_N$（`nomoto_identifier.hpp:79-82`）；
- `denom` $= \lambda + \varphi^{\mathsf T}P\varphi$，并带 $|\mathrm{denom}|<10^{-12}$ 除零保护（`:83-84`）；
- `gainK[2]` $= L_N$（`:85`）；
- `err` $= e_N$（`:88-90`）；
- `theta_[i] += gainK[i]*err`（`:93-94`）；
- `P_[i][j] = (1/λ)(P_[i][j] − gainK[i]*Pphi[j])`，注意 `gainK[i]*Pphi[j]` $= L_N\varphi_N^{\mathsf T}P_{N-1}$ 的第 $(i,j)$ 元（`:99-103`）。

**每拍计算量**：2 维情形下为常数次乘加（约 20 次 FLOP 量级），1 Hz 节拍对 ECU 完全无负担。

### 4.6 $P$ 阵与参数不确定度的关系

设量测噪声 $y_k = \varphi_k^{\mathsf T}\theta^* + \varepsilon_k$，$\varepsilon_k$ 零均值、方差 $\sigma^2$、与 $\varphi$ 不相关，则加权最小二乘估计的误差协方差为

$$
\mathrm{Cov}[\theta_N - \theta^*] = \sigma^2\,R_N^{-1}\,\Bigl(\sum_k \lambda^{2(N-k)}\varphi_k\varphi_k^{\mathsf T}\Bigr)\,R_N^{-1}\;\propto\; P_N
$$

（$\lambda\to1$ 时严格有 $\mathrm{Cov} = \sigma^2 P_N$——中间的双重加权因子退化为 $R_N$ 自身；$\lambda<1$ 时只差一个 $O(1)$ 的加权系数，§8.2 给出平稳情形的精确值 $\frac{1-\lambda}{1+\lambda}\sigma^2\bar R^{-1}$。）因此 **$P$ 的对角元正比于对应参数的估计方差**：$P_{00}$ 大 $\Rightarrow$ $\theta_0$（即 $50K$）不确定；$P_{11}$ 大 $\Rightarrow$ $T$ 不确定。初值 $P_0 = p_0 I$（$p_0=1$）表达"重置后完全不确定"（`nomoto_identifier.hpp:38-39`），随有效样本进入 $P$ 单调收缩（在持续激励下），增益随之减小——估计器从"轻信新数据"过渡到"渐进固化"。两个极端的防护（发散与塌陷）见 §5.3–5.4。

---

## 5. 遗忘因子分析（$\lambda = 0.98$）

### 5.1 有效记忆窗口 $1/(1-\lambda) = 50$ 样本

权重序列 $w_a = \lambda^a$（$a$ 为距今拍数）的总质量

$$
N_{\mathrm{eff}} = \sum_{a=0}^{\infty}\lambda^a = \frac{1}{1-\lambda} = \frac{1}{0.02} = 50\ \text{个样本}
$$

RLS 拍间隔 1 s（§7），即有效记忆约 **50 s**——恰好覆盖一个 Z 形机动的 2~3 个换舵周期。另外两个常用口径：

- **半衰期**：$\lambda^a = 1/2 \Rightarrow a = \dfrac{\ln 2}{\ln(1/\lambda)} = \dfrac{0.693}{0.0202} \approx 34.3$ 拍（约 34 s 前的数据权重减半）；
- **加权样本的平均年龄** $\bar a = \dfrac{\sum_a a\lambda^a}{\sum_a \lambda^a} = \dfrac{\lambda}{1-\lambda} = 49$ 拍——估计值实质上是"约 49 s 前至今数据的质心"，这是 §8.3 跟踪滞后偏差的来源。

### 5.2 偏差–方差权衡

- **$\lambda$ 小（遗忘快）**：窗口短，跟踪快（滞后偏差小），但有效样本少、估计方差大；
- **$\lambda$ 大（遗忘慢）**：方差小，但对参数真实漂移的响应迟钝，极端 $\lambda\to1$ 完全丧失时变跟踪能力。

半定量（详见 §8.2–8.3）：稳态下估计方差 $\propto \dfrac{1-\lambda}{1+\lambda}$；跟踪线性漂移参数的稳态滞后 $\approx \dfrac{\lambda}{1-\lambda}\times$（每拍漂移量）。$\lambda=0.98$ 意味着：方差压缩系数 $\approx 0.01$（强滤波），代价是约 49 拍的滞后——对"分钟级才显著变化"的船况漂移完全够用，对"秒级突变"（如紧急减速）则由 §9 的重基线机制兜底。

### 5.3 协方差发散风险

观察协方差更新式：当回归量近似为零（$\varphi_N \approx 0$，稳态直航、舵角在门限以内）时，若仍执行更新，

$$
P_N = \frac{1}{\lambda}\Bigl(P_{N-1} - \underbrace{L_N\varphi_N^{\mathsf T}P_{N-1}}_{\approx\,0}\Bigr) \approx \frac{P_{N-1}}{\lambda}
$$

**每拍放大 $1/\lambda \approx 1.0204$（+2.04%）**，指数发散：50 拍后 $P$ 膨胀至 $e^{50\times0.0202}\approx 2.75$ 倍，500 拍（约 8 分钟直航）后约 $2.2\times10^{4}$ 倍。$P$ 发散 $\Rightarrow$ 增益 $L_N$ 发散 $\Rightarrow$ 一旦重新出现激励，$\theta$ 会被微小新息剧烈拉动，产生危险的增益跳变。这就是激励门控的第一重作用：**$\varphi$ 没有足够能量时整拍跳过，$P$ 冻结**（门控实现见 `nomoto_identifier.hpp:68-73`，数学对应见 §6）。

### 5.4 $P$ 对角保底 1e-3 的作用与代价

与发散相对的另一个极端是**塌陷**：在持续激励下 $P \to 0$，增益趋于零，估计器"睡死"，丧失对后续参数漂移的跟踪能力（§3.1 的等权 LS 困境在数值上同样会发生——浮点下 $P$ 的最小特征值可能压到与舍入误差相当，更新式失去数值意义）。实现对 $P$ 的对角元加下界（`nomoto_identifier.hpp:98,105-106`）：

$$
P_{ii} \leftarrow \max\bigl(P_{ii},\ 10^{-3}\bigr),\qquad i = 0,1
$$

作用：保底后增益存在非零下界 $L_N \approx \dfrac{P_{ii}\varphi_i}{\lambda + \cdots} \gtrsim \dfrac{10^{-3}\,\varphi_i}{\lambda + \varphi^{\mathsf T}P\varphi}$，估计器**永远保持最低限度的自适应能力**（"永不睡死"）。

代价（必须如实指出）：保底后 $P_N$ **不再等于**加权信息矩阵的严格逆 $R_N^{-1}$，递推三式与 §3.1 目标函数的严格最优解脱钩——算法从"有限记忆最优估计"退化为"有界增益的次优估计器"。工程上这是有意为之的交换：牺牲渐近最优性（稳态时参数本就在 $P$  floor 对应的精度带内抖动），换取任意时长运行后的跟踪活性与数值鲁棒性。同时注意保底只钳对角元，不重构特征分解，非对角元不受干预——这是一种廉价的近似，在 2 维问题中足够。

---

## 6. 激励条件分析：持续激励（PE）与门控的数学对应

### 6.1 持续激励条件的数学表述

§3.2 要求 $R_N$ 正定，§8 的收敛性则要求它**一致**正定。持续激励（Persistent Excitation, PE）条件的标准表述：存在整数 $m$ 与常数 $\alpha,\beta>0$，使得对任意起始时刻 $j$ 都有

$$
\alpha\,I \;\preceq\; \sum_{k=j+1}^{j+m}\varphi_k\varphi_k^{\mathsf T} \;\preceq\; \beta\,I
$$

即任意长度 $m$ 的滑动窗内，回归量在各个方向上都有有界非零的能量。本问题 $\varphi \in \mathbb{R}^2$，PE 退化为一个非常直观的几何条件：**序列 $\{\varphi_k\}$ 不能被约束在过原点的某一条直线附近**——否则 $R_N$ 沿该直线法向的特征值恒为零，$T$ 方向（或某个参数组合）欠定。

### 6.2 为什么稳态直航时 $T$ 欠定

稳态直航（或定常旋回）时 $\dot r \approx 0$，故 $dr_k \approx 0$，回归向量

$$
\varphi_k = \begin{bmatrix}\delta_k/50 \\ -dr_k\end{bmatrix} \approx \begin{bmatrix}\delta_k/50 \\ 0\end{bmatrix}
$$

全部分量躺在 $\varphi_1 = 0$ 这条坐标轴上，任意 $N$ 都有

$$
R_N \approx \begin{bmatrix} \sum_k\lambda^{N-k}\delta_k^2/2500 & 0 \\ 0 & 0 \end{bmatrix},
\qquad \lambda_{\min}(R_N) = 0
$$

正规方程退化为单方程 $r_k = K\delta_k$——只能定出 $K$（稳态增益），$T$ 方向完全无信息。物理上这与 §1.3 的阶跃响应分析一致：**稳态数据里根本不含时间常数信息**；$T$ 只影响响应的上升沿，只藏在 $\dot r \neq 0$ 的瞬态里。

反过来，$\varphi$ 两个分量"共线"的更一般情形是 $dr_k \approx c\,\delta_k$（某固定比例 $c$）——此时 $\varphi_k \propto [1/50,\ -c]^{\mathsf T}$ 仍在一条直线上，同样欠定。Z 形机动之所以有效：换舵瞬间 $\delta$ 阶跃而 $r$ 因惯性滞后，$dr$ 出现与 $\delta$ **不成比例**的尖峰，$\varphi$ 的方向在换舵前后扫过二维平面，$R_N$ 两个特征方向都被喂饱。特别地，$r$ 过零（转向反向）附近 $|r|$ 最小而 $|dr|$ 最大——**$T$ 信息密度最高的样本恰好是 $|r|$ 最小的样本**。

### 6.3 门控公式与 PE 的逐条对应

实现中的激励门控（`nomoto_identifier.hpp:68-69`，参数在 `:20-22`）：

$$
\text{本拍更新} \iff |\delta| > \underbrace{3.0^\circ}_{\texttt{deltaMin}} \;\wedge\; \Bigl(|r| > \underbrace{0.5^\circ/\mathrm{s}}_{\texttt{rMin}} \;\vee\; |dr| > \underbrace{0.2^\circ/\mathrm{s}^2}_{\texttt{drMin}}\Bigr)
$$

逐条对应：

| 门控条件 | 数学作用 |
|---|---|
| $\|\delta\|>3°$ | 保证 $\varphi_0 = \delta/50$ 分量能量远离量测噪声底（$\vert \varphi_0\vert >0.06$），是 $R_N$ 第一方向 $\alpha$ 下界的来源；同时保证"输入确实在激励系统" |
| $\|r\|>0.5°/s$ | 保证输出信号本身超出噪声，即新息 $e_N$ 中信噪比可接受（$K$ 方向的信息质量） |
| $\|dr\|>0.2°/s^2$（**或**条件） | **专门把 $r$ 过零附近的高 $dr$ 瞬态保留进来**——按 §6.2，这正是 $\varphi_1$ 方向（$T$ 方向）能量最集中的样本群，是 PE 第二方向的来源。若没有这一条（只门 $\vert r\vert $），过零瞬态会被误杀，$T$ 依旧欠定 |

三条合起来是 PE 条件的**在线启发式实现**：只有二维回归量的两个方向都被注入足够能量时才做递推。门控未通过的拍整体跳过（`return false`，`:70-73`），既不更新 $\theta/P$（同时避免 §5.3 的 $P$ 发散），也不让低质量样本稀释 $R_N$ 的有效条件数。

### 6.4 拒绝拍不写差分锚点（P11 整改）

差分 $dr_k = (r_k - r_{k-1})/\Delta t$ 依赖上一拍锚点 $r_{k-1}$（`prevR_`）。实现区分两类拒绝：

- **非法输入拍**（NaN/Inf、$|\delta|>35°$、$|r|>30°/s$、$dt\le0$）：直接 `return false`，**不写** `prevR_`（`nomoto_identifier.hpp:53-55`）。若写了，毛刺 $r_{\text{bad}}$ 会成为下一拍差分锚点，$dr_{k+1} = (r_{k+1} - r_{\text{bad}})/\Delta t$ 被污染成一个虚假大值——它恰好能通过 §6.3 的 $|dr|$ 门控进入 $\varphi$，把毒化样本当作"高价值 $T$ 信息"永久写入 $R_N$（$R_N$ 只加不减，遗忘也要 50 拍才衰减）。
- **未激励拍**：跳过递推但**仍写** `prevR_ = r`（`:70-72`）——保证差分锚点始终是"上一拍的样本"，维持点采样语义（见 §7.3），否则激励恢复时 $dr$ 会错误地跨多拍计算而与自测 $\Delta t$ 失配。

数学含义一句话：**差分把单点毛刺放大成两点冲激，锚点管理决定这个冲激能否进入回归**。

---

## 7. 自降频的采样理论分析

### 7.1 50 Hz → 1 Hz：辨识带宽够不够？

控制主循环 50 Hz（$dt = 0.02$ s），RLS 拍固定 $1$ Hz（`beatSec = 1.0`，`nomoto_identifier.hpp:19`）：模块内部累计真实间隔，满 1 s 才递推一次（`:58-61`）。降频后采样率 $f_s = 1$ Hz，Nyquist 频率 $0.5$ Hz。

被辨识动态的带宽：一阶惯性环节 $K/(Ts+1)$ 的截止频率

$$
f_c = \frac{1}{2\pi T} = \begin{cases} 0.159\ \mathrm{Hz}, & T = 1\ \mathrm{s} \\ 2.7\times10^{-3}\ \mathrm{Hz}, & T = 60\ \mathrm{s} \end{cases}
$$

即便对最快的船（$T=1$ s），$f_c = 0.159$ Hz 也只有 Nyquist 的 $1/3$；典型船 $T = 5\sim20$ s 时裕度超过 30 倍。**结论：1 Hz 节拍对 Nomoto 一阶动态的辨识带宽充分**，Z 形机动（换舵周期 $\ge 2T \ge 2$ s，基波 $\le 0.5$ Hz 且富含低频能量）的信息几乎完整保留。注意这里辨识的是"拍时刻的点采样序列"而不是重构连续信号，差的只是模型阶次以上的高频细节——那些本来就在一阶模型假设之外（§8.4）。

代价侧要写清：1 Hz 采样会把 $>0.5$ Hz 的量测噪声（风浪高频分量、传感器噪声）**混叠**进辨识带宽。本方案没有前置抗混叠滤波（局限见 §8.4 与原文档 §8.3），靠两样东西部分对冲：差分信噪比的改善（下节）与 $\lambda=0.98$ 的 50 样本等效平均。

### 7.2 低采样率下差分 $dr$ 的信噪比改善

把 §2.2 的结论放进采样框架定量比较。设真实信号 $r(t)$ 在拍间隔内近似线性（$T\ge1$ s 保证，$r$ 的曲率 $\sim \ddot r\,\Delta t^2$ 可忽略），则差分给出的信号估计无系统失真：

$$
dr_k = \frac{r(t_k) - r(t_{k-1})}{\Delta t} = \dot r(t_k) + O\bigl(\ddot r\,\Delta t\bigr)
$$

而噪声分量方差 $2\sigma_r^2/\Delta t^2$（§2.2）。于是 $dr$ 的信噪比

$$
\mathrm{SNR}_{dr} = \frac{|\dot r|^2\,\Delta t^2}{2\sigma_r^2} \;\propto\; \Delta t^2
$$

50 Hz（$\Delta t=0.02$）与 1 Hz（$\Delta t=1$）相比，**SNR 提升 $50^2 = 2500$ 倍（标准差口径 50 倍）**。举例：设 $r$ 量测噪声 $\sigma_r = 0.1°/s$，则 50 Hz 差分噪声 $\sigma_\nu \approx 7.1°/s^2$——比门控阈值 $0.2°/s^2$ 大 35 倍，真实瞬态完全淹没；1 Hz 差分噪声 $\sigma_\nu \approx 0.14°/s^2$——低于门控阈值，与真实 $dr$ 信号（换舵瞬态 $0.2\sim1°/s^2$）同量级且偏小。**降频不是为了省算力（§4.5 已说明算力可忽略），而是为了让差分这一回归分量在物理上可用**；`beatSec` 注释"降频以减轻 ECU 负担、保证差分信噪比"（`nomoto_identifier.hpp:19`）中的后一半才是主因。

### 7.3 $\Delta t$ 自测的必要性

差分公式里的 $\Delta t$ 必须是**两次 RLS 拍之间的真实间隔**，而非任何名义值。实现（`nomoto_identifier.hpp:58-61`）：

$$
\texttt{elapsed} \mathrel{+}= \texttt{dtNow};\quad
\text{若}\ \texttt{elapsed} \ge \texttt{beatSec}:\ \Delta t = \texttt{elapsed},\ \texttt{elapsed} \leftarrow 0
$$

即 $\Delta t$ 由模块对输入的真实时间戳自累计测得（$\approx 1$ s 但不假设恰为 1 s）。若改用名义常数（旧设计的"50 Hz × 1 s 计数器 + 改写 `Params.dt`"约定，见模块头注释 `:5-8`）：调用频率一旦偏离 50 Hz（任务调度抖动、与其他模块复用、仿真变速），每拍真实间隔与名义 $\Delta t$ 失配 $\Rightarrow$ $dr$ 整体错一个比例因子 $\Rightarrow$ 回归第二分量错一个比例因子 $\Rightarrow$ **$T$ 的估计静默地错同一个比例因子，且无任何报错**。自测 $\Delta t$ 把这个"调用频率语义陷阱"从设计上删除——这正是 `Params` 中不再有 `dt` 字段的原因。

---

## 8. 收敛性与误差分析

### 8.1 PE 下的指数收敛（陈述 + 速率刻画）

**标准结论**：若回归序列满足 §6.1 的 PE 条件、量测噪声零均值且与 $\varphi$ 不相关、真实参数恒定，则带遗忘因子的 RLS 参数误差 $\tilde\theta_N = \theta_N - \theta^*$ 指数收敛到零的邻域。

**推导骨架**：由三式可得误差的齐次递推

$$
\tilde\theta_N = \bigl(I - L_N\varphi_N^{\mathsf T}\bigr)\,\tilde\theta_{N-1} + L_N\varepsilon_N
= \lambda\,P_N R_{N-1}\,\tilde\theta_{N-1} + L_N\varepsilon_N
$$

反复迭代，齐次部分 $\tilde\theta_N^{\text{hom}} = \lambda^N P_N R_0\,\tilde\theta_0$。PE 保证 $P_N$ 一致有界（$R_N \succeq \alpha' I$），故

$$
\bigl\|\tilde\theta_N^{\text{hom}}\bigr\| \le C\,\lambda^N\,\bigl\|\tilde\theta_0\bigr\|
$$

**收敛速率由 $\lambda$ 直接决定**：$\lambda^N = e^{-N(1-\lambda)+O((1-\lambda)^2)}$，$\lambda=0.98$ 时误差包络每拍收缩约 2%，时间常数 $\tau_{\text{conv}} \approx 1/(1-\lambda) = 50$ 拍 $\approx$ 50 s。这与仿真实测一致：Z 形 ±40°/15 s 参考下 $t \approx 13$ s 建立基线、$K/T$ 偏差约 +18%（见《辨识流程原理推导与工作流程》§6.4）。

### 8.2 噪声导致的稳态误差点（遗忘因子的代价）

参数恒定 + 平稳噪声的稳态，$R_N$ 以概率收敛到

$$
R_\infty = \sum_{a=0}^{\infty}\lambda^a\,\bar R = \frac{\bar R}{1-\lambda},
\qquad \bar R = \mathbb E[\varphi\varphi^{\mathsf T}]
$$

估计误差协方差（白噪声 $\sigma^2$，计入双重加权的精确系数）：

$$
\mathrm{Cov}[\tilde\theta_\infty]
= R_\infty^{-1}\Bigl(\sum_{a=0}^{\infty}\lambda^{2a}\,\sigma^2\bar R\Bigr)R_\infty^{-1}
= \frac{1-\lambda}{1+\lambda}\,\sigma^2\,\bar R^{-1}
$$

$\lambda=0.98$ 时系数 $\dfrac{0.02}{1.98} \approx 0.0101$：**遗忘因子把稳态误差钉在一个非零的"误差点"上**——即使参数恒定、数据无限，估计也不会收缩到真值，而是在真值附近做幅度 $\propto\sqrt{1-\lambda}$ 的平稳抖动。对照：等权 LS（$\lambda\to1$）系数 $(1-\lambda)/(1+\lambda)\to0$，渐近一致。这就是 §5.2 权衡的定量形式：用 1% 量级的稳态方差换时变跟踪能力。注意 §5.4 的 $P$ 保底（1e-3）在实际代码里给这个误差点再加了一个下界——$P$ 不会低于保底值，增益不会低于对应下界，稳态抖动也不会低于该水平。

### 8.3 三类误差来源的定性–半定量清单

**(a) 量测噪声（方差型误差）**。路径有两条：经 $y_k = r_k$ 直接进入新息（上节，良性）；经差分进入**回归量本身**（§2.2、§7.2）——这构成 errors-in-variables（EIV）问题：$\varphi$ 含噪时最小二乘估计产生**向零收缩的衰减偏差**，半定量地

$$
\hat\theta \;\approx\; \theta^*\cdot\frac{s_\varphi^2}{s_\varphi^2 + \sigma_\varphi^2}
$$

$s_\varphi^2$ 为回归分量信号功率，$\sigma_\varphi^2 = 2\sigma_r^2/\Delta t^2$ 为差分噪声功率。以 $T$ 方向为例：$\Delta t = 1$ s、$\sigma_r = 0.1°/s$ 时 $\sigma_\varphi \approx 0.14°/s^2$，Z 形机动 $dr$ 信号 RMS 约 $0.3\sim0.5°/s^2$，衰减系数 $\approx 0.9$——即 $T$ 估计存在 10% 量级的系统性偏低倾向（仿真 +18% 偏差为噪声、门控选择性采样与初值瞬态的综合结果）。这解释了为什么激励门控（提高 $s_\varphi$ 的选择性）与降频（压低 $\sigma_\varphi$）对精度缺一不可。

**(b) 遗忘导致的跟踪滞后（偏差型误差）**。参数以每拍 $d$ 的速率线性漂移时，加权窗口的质心滞后真值 $\bar a = \lambda/(1-\lambda) \approx 49$ 拍（§5.1），稳态滞后偏差

$$
\hat\theta - \theta^* \;\approx\; -\frac{\lambda}{1-\lambda}\,d \;=\; -49\,d
$$

例：靠港装卸致 $T$ 以 0.02 s/s 的相对速率漂移（$T=10$ s 时 $d = 0.2\% \cdot T$ 每拍），滞后偏差 $\approx 49\times0.2\% \approx 9.8\%\cdot T$——仍在 §9 的 ±25% 漂移门以内，ESC 自治吸收；若漂移更急（航速突变），滞后超限即触发显式重基线。

**(c) 模型失配（结构型误差）**。一阶 Nomoto 是 §1.2 多重近似的产物：被略去的横荡动态、横倾耦合、舵机饱和/死区非线性、风浪外力矩都不在模型内。其效果是最小二乘意义下的"最优拟合参数"随激励频谱移动（低频激励拟合准、高频激励偏差大），且波频艏摇（遭遇频率约 $0.3\sim1.5$ rad/s，部分落在辨识带宽内）被当作噪声甚至信号参与拟合。工程对策都写在代码里：激励门控挑选信噪比最高的频段样本、$valid()$ 物理门限拦截失配产物（下节）、稳定门在应用侧兜底（§9）。

### 8.4 $valid()$ 物理门限的依据

`nomoto_identifier.hpp:119-124`：

$$
\text{valid} \iff \hat K \in (0,\,2]\ \mathrm{s^{-1}} \;\wedge\; \hat T \in [1,\,60]\ \mathrm{s}
$$

依据是 §1.2 的集总参数结构与工程包线：

- $\hat K > 0$、$\hat T \ge 1$：航向稳定船 $\mathcal N>0$ 的直接推论（$K = \mathcal M/\mathcal N$，常规舵布置 $\mathcal M>0$；$T = \mathcal I/\mathcal N$，小艇惯性/阻尼比也在秒级）。$\hat K \le 0$ 或 $\hat T < 1$ 意味着数据已被污染或模型失配到失去物理意义；
- $\hat K \le 2\ \mathrm{s^{-1}}$：覆盖快艇高舵效上限（35° 舵产生 $70°/s$ 转首率已远超实船物理）；
- $\hat T \le 60$ s：覆盖大型满载油轮的最慢响应。

越界即 `valid()=false`，下游整拍拒用（`auto_tuner.hpp:158,191`）——**辨识器自己不做钳位修正**（钳位会把系统性错误伪装成合法值），而是把"不合理"作为信号原样上报，由消费方保守拒绝。

---

## 9. 与下游的衔接：极点配置、稳定门与 ±25% 漂移门

### 9.1 快照接口与两条应用路径

辨识结果以只读快照 `IdSnapshot{excited, valid, K, T}`（`auto_tuner.hpp:31-36`）传递，由调用方每拍组装（`autopilot_core.hpp:176-178`），辨识器与调参器互不持有。快照经两条受控路径生效：

1. **tick 周期评估**（`auto_tuner.hpp:158-161`）：仅当 `excited && valid` 才进入 `applyEstimate_`，且开启漂移门（`driftGate=true`）；
2. **Z 形试验收官** `establishBaseline()`（`:190-194`）：跳过漂移门（试验的明确产出），振荡测量传 0，由代数稳定条件把关，恒重锚 ESC。

### 9.2 $\hat K/\hat T$ 如何进入极点配置

被控对象 $\psi(s)/\delta(s) = \dfrac{K}{s(Ts+1)}$（§1.3），PD 控制律 $\delta = K_p e + K_d\dot e$（$e = \psi_{\text{ref}} - \psi$）。闭环特征多项式：

$$
s(Ts+1) + K(K_d s + K_p) = 0
\;\Longrightarrow\;
Ts^2 + (1 + KK_d)s + KK_p = 0
$$

对照标准二阶 $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$（两边除以 $T$ 后比较系数）：

$$
\frac{KK_p}{T} = \omega_n^2,\qquad \frac{1 + KK_d}{T} = 2\zeta\omega_n
$$

反解得极点配置公式（`AutoTuner::polePlacement`，`auto_tuner.hpp:228-237`，设计目标 $\zeta=0.85$、$\omega_n=0.20$ rad/s，`:84-85`）：

$$
\boxed{\;K_p = \frac{\omega_n^2\,\hat T}{\hat K},\qquad K_d = \frac{2\zeta\omega_n\,\hat T - 1}{\hat K}\;}
$$

（$K_d<0$ 时钳 0——负微分会反向放大振荡，`:235`；$\hat K,\hat T$ 退化到 $10^{-6}$ 以下直接拒绝，`:232`。）**辨识误差到增益的映射是单调有理的**：$\hat K$ 偏小 25% $\Rightarrow K_p,K_d$ 偏大 33%；$\hat T$ 偏大 25% $\Rightarrow K_p$ 偏大 25%、$K_d$ 同向。这是辨识精度要求的定量来源。

### 9.3 稳定门

极点配置给出的增益还要过四条件稳定门（`stabilityCheck`，`auto_tuner.hpp:241-254`）：把候选 $(K_p,K_d)$ 代回实测/估计模型，重算闭环指标

$$
\omega_n = \sqrt{\frac{K_p\hat K}{\hat T}},\qquad
\zeta = \frac{1 + K_d\hat K}{2\sqrt{\hat T K_p \hat K}}
$$

要求 $\zeta \ge 0.6$（`zetaMin`）、$\omega_n \le 0.5$ rad/s（`wnMax`）、实测振荡幅值 $\le 5°$（`oscMax`）（`:98-100`），外加 $K_p>0$ 与有限性检查。数学含义：极点配置是**设计点**的代数反解，稳定门是**鲁棒性包线**——辨识误差把真实闭环拖离设计点时（$\hat K/\hat T$ 与真值不符，上式算出的才是"真实" $\zeta,\omega_n$），门负责拦截。

### 9.4 ±25% 漂移门与重基线的数学含义

漂移门（`applyEstimate_`，`auto_tuner.hpp:308-313`，常量 `:499-502`）：

$$
\text{ESC 自治（不动基线）} \iff
\begin{cases}
|\hat K - K_{\text{base}}| \le 0.25\,\max\bigl(|K_{\text{base}}|,\ 0.05\bigr) \\
|\hat T - T_{\text{base}}| \le 0.25\,\max\bigl(|T_{\text{base}}|,\ 1.0\bigr)
\end{cases}
$$

即**以基线建立时刻的估计为锚的相对误差死区**（分母设下限 $0.05/1.0$，防早期小估计把相对偏差放大失真）。三条数学解读：

1. **死区内的增益摄动有界**。由 §9.2 的映射，$\hat K,\hat T$ 各漂 25% 时 $K_p$ 最坏漂移 $|{\Delta K_p}/{K_p}| \approx |\Delta T/T| + |\Delta K/K| \le 50\%$——在 ESC 局部搜索的修正能力半径内（ESC 以基线为中心做梯度微调，5% 幅值摄动 + 每窗 2% 步长），故把修正权交给 ESC 的连续微调，避免基线频繁跳变；
2. **超死区 = 工况真变**。25% 已显著大于 §8.2 的稳态误差点（$\sim$1% 级）与 §8.3(b) 的正常滞后（$\sim$10% 级），超限大概率是船况/航速真实切换或估计已远离早期未收敛值——此时旧基线对应的 $K_p,K_d$ 已偏离新极点配置解，必须显式重算并重锚；
3. **重锚的清零语义**。重基线时 ESC 积分器清零（`rebaseEsc()`，`:322-327`）：ESC 测得的梯度 $\hat g \approx \partial J/\partial\theta$ 是围绕**旧基线增益与旧模型**的局部量，基线迁移后旧梯度对新点无效，积分状态必须作废，否则相当于把过期梯度施加到新工作点。

---

## 附录 A：公式–代码对照总表

| 公式/参数 | 本文 | 代码位置 |
|---|---|---|
| $T\dot r + r = K\delta$、回归化 $\varphi=[\delta/50,\ -dr]^{\mathsf T}$ | §1–2 | `nomoto_identifier.hpp:3-4, 76` |
| 差分 $dr=(r_k-r_{k-1})/\Delta t$，$\Delta t$ 自测 | §2.1, §7.3 | `nomoto_identifier.hpp:58-67` |
| 归一化/反归一化 $K=\theta_0/50,\ T=\theta_1$ | §2.3 | `nomoto_identifier.hpp:24-26, 112-115` |
| RLS 三式 + 除零保护 | §4.5 | `nomoto_identifier.hpp:79-103` |
| $\lambda=0.98$、$p_0=1$ | §3.1, §4.6 | `nomoto_identifier.hpp:18, 23` |
| $P$ 对角保底 $10^{-3}$ | §5.4 | `nomoto_identifier.hpp:98, 105-106` |
| 激励门控 $3°/0.5°\!/\mathrm{s}/0.2°\!/\mathrm{s}^2$ | §6.3 | `nomoto_identifier.hpp:20-22, 68-69` |
| 入口防御与锚点管理 | §6.4 | `nomoto_identifier.hpp:53-55, 70-72` |
| $valid()$：$K\in(0,2]$、$T\in[1,60]$ s | §8.4 | `nomoto_identifier.hpp:119-124` |
| 极点配置 $K_p=\omega_n^2\hat T/\hat K$ 等（$\zeta=0.85,\ \omega_n=0.20$） | §9.2 | `auto_tuner.hpp:228-237, 84-85` |
| 稳定门 $\zeta\ge0.6,\ \omega_n\le0.5,\ \mathrm{osc}\le5°$ | §9.3 | `auto_tuner.hpp:241-254, 98-100` |
| 漂移门 ±25%（下限 0.05/1.0）、重锚 | §9.4 | `auto_tuner.hpp:308-329, 499-502` |
| 快照组装 | §9.1 | `autopilot_core.hpp:176-178` |

## 附录 B：关键数值速查

| 量 | 值 | 出处 |
|---|---|---|
| 遗忘因子 $\lambda$ | 0.98 | `nomoto_identifier.hpp:18` |
| 有效记忆窗口 $1/(1-\lambda)$ | 50 样本 ≈ 50 s | §5.1 |
| 权重半衰期 / 平均年龄 | 34.3 拍 / 49 拍 | §5.1 |
| $P$ 无激励发散率 | ×1.0204/拍 | §5.3 |
| 稳态误差系数 $(1-\lambda)/(1+\lambda)$ | ≈ 0.0101 | §8.2 |
| 跟踪滞后 | $49\times$ 每拍漂移 | §8.3(b) |
| RLS 拍间隔 / 辨识 Nyquist | 1 s / 0.5 Hz | §7.1 |
| 被辨识带宽 $1/(2\pi T)$ | 0.0027–0.159 Hz | §7.1 |
| 差分降噪（1 Hz vs 50 Hz） | 标准差 ×1/50，SNR ×2500 | §7.2 |
| 仿真实测收敛 | $t\approx13$ s 建基线，$\hat K/\hat T$ 偏差 +18.2%/+18.5% | 配套文档 §6.4 |
