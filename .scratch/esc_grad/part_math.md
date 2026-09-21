# 为什么这样能测出梯度：ESC 梯度测量的数学推导

> 本文是《ESC 代价函数梯度计算详解》的数学推导部分，回答一个核心问题：**只凭实测的窗口代价 $J(t)$、不向模型要任何解析导数，为什么"摄动 → 高通 → 乘载波 → 平均"这条链路输出的恰好是梯度 $\nabla J$？** 推导与 `control/include/auto_tuner.hpp` 的实现逐环节对应。

记号（与代码默认值一致，`auto_tuner.hpp:87-91, 104`）：

| 记号 | 含义 | 默认值 |
|---|---|---|
| $\theta=(K_p,K_d)$ | 被调参数（PD 增益） | — |
| $a$ | 相对摄动幅值 `escAmp` | 0.05 |
| $\omega$ | 载波角频率 `escFreq` | 0.05 rad/s（周期 $2\pi/\omega\approx125.7$ s） |
| $\tau_H$ / $\tau_L$ | 高通/低通时间常数 | 5 s / 10 s |
| $dt$ | 控制周期 | 0.02 s（50 Hz） |
| $T_w$ | 评估窗口 `windowSec` | 60 s（$N=T_w/dt=3000$ 拍） |
| $\eta$ | 梯度下降步长 `escStep` | 0.02 |

## 1. 问题设定：只能实测、不可求导的 $J(\theta)$

被调参数是 PD 增益 $\theta=(K_p,K_d)$，代价是评估窗口内的 ITAE 型积分：

$$J(\theta)=\int_0^{T_w}\Bigl(t\,|e(t)|+w_U|\delta(t)|+w_{Du}|\dot\delta(t)|\Bigr)\,dt$$

离散实现为每拍成本增量（`accumulate`，`auto_tuner.hpp:366-367`，与 `computeJ` :213-223 的被积函数同口径）：

$$c_k=\bigl(t_k|e_k|+w_U|\delta_k|+w_{Du}|\Delta\delta_k|\bigr)\,dt,\qquad J=\sum_k c_k$$

困难在于：$e(t),\delta(t)$ 是**闭环系统在海况与参考机动下的响应**，$J$ 对 $\theta$ 的依赖穿过"增益 → PD 控制律 → 舵机 → 船体 → 航向误差"整条闭环，没有可用的解析表达式；即便对某个工况写出来，海况一变它就失效。因此对 $J(\theta)$ 我们只知道一件事：**给定 $\theta$，跑一个窗口，实测到一个数**。

> 直觉：这是一个"黑箱函数求最小"问题——能试、能测，但不能求导。

无模型优化里梯度只能靠"试"出来。最朴素的是有限差分（两次试探做一次差商），但它对噪声极敏感，而且每走一步要多花一个窗口。ESC 的办法更巧：**让 $\theta$ 自己持续做小幅周期摄动，把梯度信息调制到一个已知频率的载波上，再从 $J(t)$ 里解调出来**。推导依赖一个前提假设：

**静态映射假设（时间尺度分离）**：闭环动态（调节时间 $\sim3/(\zeta\omega_n)\approx18$ s，目标模态 $\zeta=0.85,\ \omega_n=0.2$）远快于 $\theta$ 的变化，因此每个瞬时 $\theta(t)$ 都对应一个"充分收敛后"的代价，$J$ 可以当作 $\theta$ 的静态光滑函数处理。该假设的破坏后果见 §7.4。

## 2. 摄动与泰勒展开：梯度信息藏在一阶项里

### 2.1 注入摄动

先取标量 $\theta$（代表 $K_p$ 或 $K_d$）。经典摄动 ESC 给参数叠加小幅正弦：

$$\theta(t)=\bar\theta+a\sin\omega t,\qquad a\ll\bar\theta$$

> 直觉：在原地小步抖动。若当前位置左低右高，抖动引起的代价起伏就与抖动同相；右低左高则反相——起伏相对载波的"相位方向"就是梯度符号。

代码注入的是**相对（乘性）摄动**（`escDither`，`auto_tuner.hpp:382-388`）：$\theta(t)=\bar\theta\bigl(1+a\sin\omega t\bigr)$，等价于把上式的 $a$ 换成 $a\bar\theta$。本节先用加性形式推导（式子更干净），§2.4 再换回相对形式与代码对齐。

### 2.2 泰勒展开（含二阶项）

$J$ 在 $\bar\theta$ 处做二阶泰勒展开，代入 $\theta(t)-\bar\theta=a\sin\omega t$：

$$J(t)=J(\bar\theta)+J'(\bar\theta)\cdot a\sin\omega t+\frac{1}{2}J''(\bar\theta)\cdot a^2\sin^2\omega t+O(a^3)$$

> 直觉：一阶项是代价对摄动的线性响应，二阶项是曲率带来的二次修正。$a$ 很小，各项按 $a$ 的幂次自动排好大小：一阶 $O(a)$、二阶 $O(a^2)$。

用 $\sin^2\omega t=\dfrac{1-\cos2\omega t}{2}$ 把二阶项拆为直流 + 二倍频，得**完整谐波分解**：

$$\boxed{\;J(t)=\underbrace{J(\bar\theta)+\frac{a^2}{4}J''(\bar\theta)}_{\text{直流 }J_0}\;+\;\underbrace{a\,J'(\bar\theta)\,\sin\omega t}_{\text{基波：携带梯度}}\;-\;\underbrace{\frac{a^2}{4}J''(\bar\theta)\,\cos2\omega t}_{\text{二倍频：干扰}}\;+\;O(a^3)\;}$$

> 直觉：一个纯 $\sin\omega t$ 输入经过非线性系统，产出三种成分——不动的（直流）、同频的（基波）、双倍频的（谐波）。梯度信息**只**在基波里，基波幅值恰好是 $a\,J'(\bar\theta)$。

逐项解读：

- **直流项** $J_0=J(\bar\theta)+\frac{a^2}{4}J''(\bar\theta)$：当前点代价 + 抖动本身的平均代价（dither loss，极小点附近 $J''>0$，该项恒正——即使已停在最优点，抖动也抬高平均代价）。它不含梯度信息，数值上却比基波幅值大几个数量级，必须去除（§3）；
- **基波项** $aJ'(\bar\theta)\sin\omega t$：幅值与梯度成正比，**符号藏在相位里**——$J'>0$（$\bar\theta$ 在最优点右侧）时与载波同相，$J'<0$ 时反相。这就是整条链路要提取的信号；
- **二倍频项** $-\frac{a^2}{4}J''\cos2\omega t$：只含曲率，与 $J'$ 的符号无关，即不含梯度方向信息，是必须被平均掉的干扰（§4）。

### 2.3 线性项与二次项的分工

- **线性（一阶）项负责"测"**：梯度信息全部在 $O(a)$ 基波中。取大 $a$ 信号更强，但破坏小摄动前提、抬高抖动代价；
- **二次项负责"限制精度"**：它制造两样东西——直流里的 dither loss $\frac{a^2}{4}J''$，以及二倍频干扰。越接近极小值，真实梯度信号 $aJ'(\bar\theta)\to0$，而二次项并不消失，信噪比持续恶化。**这就是"用一次项提取梯度"的方案在极小值附近精度受限的数学根源**：估计在 $|\Delta\bar\theta|\lesssim O(a)$ 的邻域内失去方向性，表现为围绕最优点的极限环——ESC 只能收敛到 $\theta^*$ 的 $O(a)$ 邻域而非精确点（§7.4）。

### 2.4 从加性摄动回到代码的相对摄动

把 $a\to a\bar\theta$（相对摄动），基波幅值变为 $a\bar\theta J'(\bar\theta)$。用链式法则看更本质：记无量纲相对摄动 $u=a\sin\omega t$，$\theta=\bar\theta(1+u)$，则

$$\frac{\partial J}{\partial u}=\bar\theta\,\frac{\partial J}{\partial\theta}$$

即代码测到的是"**$J$ 对相对摄动的梯度**"，数值上等于 $\bar\theta\,J'(\bar\theta)$——正是 `escStep` 注释（`auto_tuner.hpp:399`）所写的"∂J/∂(增益相对摄动)"。下文为书写简洁仍用加性记号 $a$，结论换回相对形式只需把 $J'$ 替换为 $\bar\theta J'$。

下图给出几何直观：最优点左侧的摄动引起反相的 $J$ 响应（负相关），右侧引起同相响应（正相关），相关性的符号与大小即梯度信息。

![ESC 梯度估计原理：J 响应与载波的相关性给出梯度符号与大小](../../assets/esc_math_gradient.png)

## 3. 高通滤波：把直流挡在相关器之外

### 3.1 为什么需要 HPF

理论上直流 $J_0$ 乘 $\sin\omega t$ 后在整周期内均值也是零，似乎不必滤波。但实测中 $J_0$ 比基波幅值 $aJ'$ 大几个数量级（仿真窗口 $J\sim O(10^4\!\sim\!10^5)$，而 $aJ'$ 在最优点附近趋于零），只要窗口不严格覆盖整数个载波周期、或 $J$ 带慢漂（海况变化、$K/T$ 漂移、机动），巨大的直流/慢漂就会泄漏进相关器淹没信号。高通滤波在解调**之前**把直流与慢漂挡掉，是相关器正常工作的前提。

### 3.2 一阶高通的频域作用

一阶高通传递函数：

$$H(s)=\frac{s}{s+1/\tau_H}=\frac{\tau_H s}{\tau_H s+1}$$

> 直觉：写成微分方程 $\tau_H\dot y+y=\tau_H\dot c$——输出只跟随输入的"变化率"，输入不动输出归零，直流天然被阻断。

对角频率 $\nu$ 的正弦分量，频率响应的模与相位：

$$|H(j\nu)|=\frac{\nu\tau_H}{\sqrt{1+(\nu\tau_H)^2}},\qquad \angle H(j\nu)=\frac{\pi}{2}-\arctan(\nu\tau_H)$$

> 直觉：$\nu\tau_H$ 是"该频率分量算不算快"的无量纲度量——远快于 $1/\tau_H$ 的几乎全通，远慢的被压掉。

作用于 §2.2 谐波分解的各成分：

- **直流**（$\nu=0$）：$|H|=0$，稳态完全阻断；
- **慢漂**（$\nu\lesssim1/\tau_H$）：按 $\nu\tau_H$ 比例衰减——海况/工况引起的 $J$ 慢变化被抑制；
- **基波**（$\nu=\omega$）：保留，但幅值乘 $|H(j\omega)|$、相位移动 $\varphi_H=\angle H(j\omega)$；
- **二倍频**（$\nu=2\omega$）：$|H(j2\omega)|>|H(j\omega)|$，衰减更小，被保留下来——只能靠下一级解调平均去除。

### 3.3 量化校核

经典设计要求载波远快于高通截止频率：$\omega\tau_H\gg1$。本项目默认参数：

$$\omega\tau_H=0.05\times5=0.25\;\Rightarrow\;|H(j\omega)|=\frac{0.25}{\sqrt{1+0.25^2}}\approx0.2425,\qquad\varphi_H=\frac{\pi}{2}-\arctan0.25\approx76.0^\circ$$

下一节将证明：解调平均后只有基波的**同相分量**留存，梯度估计实际被乘上

$$\operatorname{Re}H(j\omega)=|H(j\omega)|\cos\varphi_H=\frac{(\omega\tau_H)^2}{1+(\omega\tau_H)^2}\approx0.0588$$

> 直觉：HPF 把基波相位转了 $76^\circ$，与载波"对上相"的部分只剩约 6%——默认参数下梯度幅值被衰减约 17 倍。它只改变步长不改变符号（$\operatorname{Re}H>0$），故减慢收敛而不破坏方向。这属于参数整定问题：先定 $T_w$、再取 $\omega$、最后配 $\tau_H$ 使 $\operatorname{Re}H\gtrsim0.9$（§7.2）。

## 4. 乘以同相载波解调：把梯度"搬"到直流

### 4.1 解调做什么

HPF 输出保留基波 + 二倍频（直流已除）：

$$y(t)=A\sin(\omega t+\varphi_H)+B\cos(2\omega t+\varphi_2),\qquad A=a\,J'(\bar\theta)\,|H(j\omega)|$$

将 $y(t)$ 乘以与注入**同相**的载波 $\sin\omega t$。代码用同一相位变量 `escPhase_` 驱动注入（`escDither`，`auto_tuner.hpp:384-386`）与解调（`accumulate`，:373：`demodKpSum_ += hpfY_ * std::sin(escPhase_)`），保证因果同相。

> 直觉：乘同频载波是"频率搬家"——两个 $\omega$ 分量相乘产生差频 $0$（直流）与和频 $2\omega$。只有与载波同频的成分能在直流上留下印记，其余成分全被搬到非零频率，随后被平均清除。

### 4.2 逐项展开（积化和差）

**基波项**（$\sin x\sin y=\frac12[\cos(x-y)-\cos(x+y)]$，取 $x=\omega t+\varphi_H,\ y=\omega t$）：

$$A\sin(\omega t+\varphi_H)\sin\omega t=\frac{A}{2}\cos\varphi_H-\frac{A}{2}\cos(2\omega t+\varphi_H)$$

**二倍频项**（$\cos x\sin y=\frac12[\sin(x+y)-\sin(x-y)]$，取 $x=2\omega t+\varphi_2,\ y=\omega t$）：

$$B\cos(2\omega t+\varphi_2)\sin\omega t=\frac{B}{2}\sin(3\omega t+\varphi_2)-\frac{B}{2}\sin(\omega t+\varphi_2)$$

解调后的频率成分清单：

| 成分 | 来源 | 频率 |
|---|---|---|
| $\dfrac{A}{2}\cos\varphi_H$ | 基波 × 载波（差频） | **0（直流）——梯度信息** |
| $-\dfrac{A}{2}\cos(2\omega t+\varphi_H)$ | 基波 × 载波（和频） | $2\omega$ |
| $-\dfrac{B}{2}\sin(\omega t+\varphi_2)$ | 二倍频 × 载波（差频） | $\omega$ |
| $\dfrac{B}{2}\sin(3\omega t+\varphi_2)$ | 二倍频 × 载波（和频） | $3\omega$ |

**所有非直流成分的频率都不低于 $\omega$**——窗口平均（或低通）即可将其清除，只留下直流项。

### 4.3 窗口平均：三个均值事实

设窗口覆盖整数个载波周期，则：

$$\langle\sin^2\omega t\rangle=\Big\langle\frac{1-\cos2\omega t}{2}\Big\rangle=\frac12,\qquad \langle\cos2\omega t\,\sin\omega t\rangle=\frac12\big\langle\sin3\omega t-\sin\omega t\big\rangle=0,\qquad \langle\sin\omega t\rangle=0$$

> 直觉：正弦在整周期内正负相抵；$\sin^2$ 恒正，留下均值 $1/2$——这个 $1/2$ 正是归一化系数里那个"2"的来历。

对解调结果逐项取窗口均值：

$$\big\langle y\sin\omega t\big\rangle=\frac{A}{2}\cos\varphi_H-\frac{A}{2}\underbrace{\big\langle\cos(2\omega t+\varphi_H)\big\rangle}_{=\,0}+\frac{B}{2}\underbrace{\big\langle\sin(3\omega t+\varphi_2)\big\rangle}_{=\,0}-\frac{B}{2}\underbrace{\big\langle\sin(\omega t+\varphi_2)\big\rangle}_{=\,0}=\frac{A}{2}\cos\varphi_H$$

### 4.4 关键恒等式的逐步推导

把 $A=aJ'(\bar\theta)\,|H(j\omega)|$ 代回：

$$\big\langle y\sin\omega t\big\rangle=\frac{a\,J'(\bar\theta)}{2}\,|H(j\omega)|\cos\varphi_H=\frac{a}{2}\,J'(\bar\theta)\,\operatorname{Re}H(j\omega)$$

> 直觉：相关平均测出了基波"与载波同相的分量"的一半；HPF 的相移让这个分量再打一个 $\cos\varphi_H$ 的折扣。

两边同乘 $\dfrac{2}{a}$：

$$\boxed{\;\hat g=\frac{2}{a}\,\big\langle y\sin\omega t\big\rangle=J'(\bar\theta)\cdot\operatorname{Re}H(j\omega)\;\xrightarrow{\ \omega\tau_H\gg1\ }\;J'(\bar\theta)=\frac{\partial J}{\partial\theta}\bigg|_{\bar\theta}\;}$$

由于直流与载波在整周期内正交（$\langle J_0\sin\omega t\rangle=0$），理想情况下 $\langle J\sin\omega t\rangle=\langle y\sin\omega t\rangle$，所以该恒等式也常写成

$$\hat g=\frac{2}{a}\operatorname{avg}\bigl[J(t)\sin\omega t\bigr]\approx\frac{\partial J}{\partial\theta}$$

——HPF 是工程上的保险，在理想恒等式里它"隐身"。归一化系数 $\dfrac{2}{a}$ 的两个因子各有严格来历（离散式中还有一个 $N$，见 §6）：

- **2**：恰好抵消 $\langle\sin^2\rangle=1/2$；
- **1/a**：除掉注入摄动的幅值，把"单位摄动引起的 $J$ 变化"还原出来。

换成代码的相对摄动（§2.4），同一恒等式给出 $\hat g=\bar\theta\,J'(\bar\theta)\operatorname{Re}H(j\omega)$，即 `escStep` 注释（`auto_tuner.hpp:399`）的 $\hat g=2\sum(y\cdot载波)/(aN)\approx\partial J/\partial(\text{增益相对摄动})$。

下图用合成代价 $J(\theta)=(\theta-\theta^*)^2$ 对这条链路做数值验证：原始 $J(t)$ 的直流 + 基波 + 二倍频结构、HPF 后的幅值衰减与相移、乘以同相载波后均值恒正（$\bar\theta$ 在最优点右侧，梯度为正），以及滑动平均收敛到梯度估计值。

![ESC 解调链路时域验证（合成代价函数）](../../assets/esc_math_demod.png)

## 5. 双参数同时估计：sin/cos 正交解调

### 5.1 双参数二次型与正交载波注入

两参数同时整定时，局部二次模型推广为（$\boldsymbol\theta=(K_p,K_d)^\top$，$H$ 为 Hessian）：

$$J(\boldsymbol\theta)\approx J^*+\frac12(\boldsymbol\theta-\boldsymbol\theta^*)^\top H\,(\boldsymbol\theta-\boldsymbol\theta^*),\qquad H=\begin{pmatrix}H_{11}&H_{12}\\ H_{12}&H_{22}\end{pmatrix}\succ0$$

代码给两个参数注入**同频、相位差 $90^\circ$** 的载波（`escDither`，`auto_tuner.hpp:384-385`）：

$$K_p(t)=\bar K_p\bigl(1+a\sin\omega t\bigr),\qquad K_d(t)=\bar K_d\bigl(1+a\cos\omega t\bigr)$$

记 $s_1=\sin\omega t,\ s_2=\cos\omega t$，瞬时偏差 $\theta_i-\theta_i^*=\Delta\bar\theta_i+\bar\theta_i a s_i$，代入二次型：

$$J(t)=J^*+\frac{H_{11}}{2}\bigl(\Delta\bar\theta_1+\bar\theta_1 a s_1\bigr)^2+\frac{H_{22}}{2}\bigl(\Delta\bar\theta_2+\bar\theta_2 a s_2\bigr)^2+H_{12}\bigl(\Delta\bar\theta_1+\bar\theta_1 a s_1\bigr)\bigl(\Delta\bar\theta_2+\bar\theta_2 a s_2\bigr)$$

收集基波成分（$s_1^2,s_2^2$ 只产生直流与 $\cos2\omega t$；$s_1s_2=\frac12\sin2\omega t$ 只产生二倍频，均不含基波）：

$$J(t)\supset\underbrace{a\bar K_p\bigl(H_{11}\Delta\bar\theta_1+H_{12}\Delta\bar\theta_2\bigr)}_{=\,a\bar K_p\,\partial J/\partial K_p}\sin\omega t+\underbrace{a\bar K_d\bigl(H_{12}\Delta\bar\theta_1+H_{22}\Delta\bar\theta_2\bigr)}_{=\,a\bar K_d\,\partial J/\partial K_d}\cos\omega t$$

> 直觉：同频的两个正交载波像两条互不干扰的"频道"——$K_p$ 的梯度信息挂在 $\sin$ 频道、$K_d$ 的挂在 $\cos$ 频道。各频道系数含 Hessian 交叉耦合项，测到的是真梯度 $\nabla J$，不是逐坐标偏导的拼凑。

### 5.2 正交性证明与两路分离

$K_p$ 路解调（乘 $\sin\omega t$）时，$K_d$ 频道基波的贡献为：

$$\Big\langle a\bar K_d\frac{\partial J}{\partial K_d}\cos\omega t\cdot\sin\omega t\Big\rangle=a\bar K_d\frac{\partial J}{\partial K_d}\cdot\frac12\big\langle\sin2\omega t\big\rangle=0$$

> 直觉：$\sin$ 与 $\cos$ 错开四分之一周期，乘积 $\frac12\sin2\omega t$ 仍是整周期正负相抵的正弦——窗口覆盖整数个载波周期时，两个频道**严格**互不串扰。

二倍频干扰与两路载波同样正交：$\langle\cos2\omega t\,\sin\omega t\rangle=\langle\sin2\omega t\,\cos\omega t\rangle=0$（整数周期时严格成立）。于是两路估计互不污染：

$$\hat g_{K_p}=\frac{2}{a}\big\langle y\sin\omega t\big\rangle=\bar K_p\,\frac{\partial J}{\partial K_p}\operatorname{Re}H(j\omega),\qquad \hat g_{K_d}=\frac{2}{a}\big\langle y\cos\omega t\big\rangle=\bar K_d\,\frac{\partial J}{\partial K_d}\operatorname{Re}H(j\omega)$$

对应代码中两路累积量 `demodKpSum_/demodKdSum_`（`accumulate`，`auto_tuner.hpp:373-374`），窗末共用同一归一化系数（`escStep`，:401-405）。

下图在合成二维二次 Bowl $J(K_p,K_d)$ 上复现双参数搜索（算法参数与代码一致：$a=0.05$、$\omega=0.05$ rad/s、$\eta=0.02$、±10% 限幅）：正交 sin/cos 摄动下，增益轨迹从偏离初值螺旋收敛到最优点附近。

![双参数 ESC 收敛轨迹（合成二次 Bowl）](../../assets/esc_math_2param.png)

## 6. 离散化：从连续相关器到每拍累积

### 6.1 连续相关器的采样化

连续形式的梯度估计是在一个窗口 $T_w$ 上做相关积分：

$$\hat g=\frac{2}{aT_w}\int_0^{T_w}y(t)\sin\omega t\,dt$$

控制环以 $dt=0.02$ s 采样，窗口内 $N=T_w/dt=3000$ 拍，第 $k$ 拍载波相位 $\varphi_k=\omega t_k$。把积分换成黎曼和（$\frac{1}{T_w}\int_0^{T_w}\!\cdot\,dt\;\to\;\frac1N\sum_{k=0}^{N-1}$）：

$$\hat g=\frac{2}{a}\cdot\frac{1}{N}\sum_{k=0}^{N-1}y_k\sin\varphi_k=\frac{2\,\sum_{k}y_k\sin\varphi_k}{a\,N}$$

> 直觉：连续"平均"落到数字系统就是"累加再除以拍数"——系数里的 $N$ 把代码中的累加和换算回均值。

与代码逐字对应（`escStep`，`auto_tuner.hpp:401-405`）：`norm = 2.0/(escAmp*demodCount_)`，`gKp = demodKpSum_*norm`。其中 `demodKpSum_`/`demodKdSum_` 由 `accumulate` 每拍累加（:373-375，$\sin$ 路进 `demodKpSum_`、$\cos$ 路进 `demodKdSum_`、`demodCount_++`），窗末统一归一化后清零（:417-425，非重叠窗）。**每拍只做一次乘法累加、窗末只做一次除法**——相关器的计算量被均摊到每个控制周期，50 Hz 下开销可忽略。

### 6.2 每拍 HPF：后向欧拉离散

连续高通 $\tau_H\dot y+y=\tau_H\dot c$ 两边做后向欧拉（$\dot y\approx\dfrac{y_k-y_{k-1}}{dt}$，$\dot c\approx\dfrac{c_k-c_{k-1}}{dt}$）：

$$\tau_H\frac{y_k-y_{k-1}}{dt}+y_k=\tau_H\frac{c_k-c_{k-1}}{dt}\;\Longrightarrow\;y_k(\tau_H+dt)=\tau_H\bigl(y_{k-1}+c_k-c_{k-1}\bigr)$$

归并 $y_k$：

$$\boxed{\;y_k=\alpha_H\bigl(y_{k-1}+c_k-c_{k-1}\bigr),\qquad \alpha_H=\frac{\tau_H}{\tau_H+dt}=\frac{5}{5.02}\approx0.99602\;}$$

与代码逐字对应（`accumulate`，`auto_tuner.hpp:369-371`）：`alphaH = escHPFtau/(escHPFtau+dt)`，`hpfY_ = alphaH*(hpfY_ + c - hpfCPrev_)`。输入 $c_k$ 是每拍成本增量（:366-367），与 `computeJ` 的被积函数严格同口径——HPF 的输入就是代价被积序列，"差分 $c_k-c_{k-1}$ 再乘 $\alpha_H$"实现了"去直流、留交流"。

### 6.3 窗末：LPF、梯度下降与限幅

窗末每 $T_w$ 秒才来一个新估计 $\hat g_w`，先经一阶 LPF 平滑（`escStep`，:407-409）。由于该滤波器输入的**真实采样间隔是 $T_w$ 而非 $dt$**，后向欧拉系数必须按 $T_w$ 计算：

$$g_w=(1-\alpha_L)\,g_{w-1}+\alpha_L\,\hat g_w,\qquad \alpha_L=\frac{T_w}{\tau_L+T_w}=\frac{60}{10+60}\approx0.857$$

随后梯度下降（:411-412）：$\bar\theta\leftarrow\bar\theta-\eta\,g_w$，$\eta=0.02$；再经 `clampUpdate` 限幅——单窗变动不超过当前值的 ±10%（:413-414，实现 :452-461）——与非负钳位（:415-416）。

> 直觉（P11 整改教训）：ESC 各环节运行在不同节拍上——摄动/解调在每拍（$dt$），LPF/积分在窗末（$T_w$）。**每个滤波器系数必须用它所在环节的真实采样周期计算**；把按 $dt$ 设计的系数用在窗级样本上，等效时间常数被放大 $T_w/dt=3000$ 倍，梯度会被衰减到几乎消失。

### 6.4 三个时间尺度

整条链路能否成立，归结为时间尺度是否拉开：

$$\underbrace{dt=0.02\ \text{s}}_{\text{控制/解调节拍}}\;\ll\;\underbrace{\tau_{cl}\sim\frac{3}{\zeta\omega_n}\approx18\ \text{s}}_{\text{闭环动态}}\;<\;\underbrace{\frac{2\pi}{\omega}\approx125.7\ \text{s}}_{\text{载波周期}}\;\ll\;\underbrace{\frac{T_w}{\eta\,\bar\theta J''}\sim\text{数十分钟}}_{\text{积分器适应}}$$

- **闭环 ≪ 载波**：每个载波周期内闭环近似到达准稳态，静态映射假设（§1）成立；
- **载波 ≪ 适应**：一个载波周期内 $\bar\theta$ 几乎不动，"中心值 + 摄动"的分解才有意义，窗口平均才是在固定工作点上测梯度；
- 窗口 $T_w=60$ s 本身是噪声方差（§7.3）与更新速率（每窗才能更新一次）之间的折中；注意它与默认载波周期 125.7 s 存在张力——单窗装不满半个载波周期，§4.3 的均值事实不再严格成立（§7.1）。

![ESC 时间尺度三分离](../../assets/esc_math_timescale.png)

## 7. 误差来源小结

梯度估计 $\hat g$ 不是真梯度，而是真梯度乘上若干因子、再叠加干扰项。按来源分四类。

### 7.1 窗口长度 $T_w$

- **整数周期假设**：§4.3 的三个均值事实、§5.2 的正交性都只在窗口覆盖整数个载波周期时严格成立。默认参数 $\omega T_w=0.05\times60=3$ rad，窗内仅 $0.477$ 个载波周期——$\langle\sin^2\rangle$ 偏离 $1/2$、sin/cos 频道轻微串扰、二倍频泄漏进估计。代码的载波相位跨窗连续（`escPhase_` 刻意不清零，`auto_tuner.hpp:436-437`），残余误差逐窗轮转、近似零均值涨落——**偏置转化为方差**，由多窗平均部分吸收；
- **方差与速率折中**：窗口越长（$N$ 越大），噪声方差 $\propto1/N$ 越小（§7.3），但每窗才能更新一次，收敛变慢、对工况漂移的跟踪变迟钝。$T_w=60$ s（$N=3000$）是两者的折中。

### 7.2 载波频率 $\omega$

- **HPF 衰减**：乘性因子 $\operatorname{Re}H(j\omega)=\dfrac{(\omega\tau_H)^2}{1+(\omega\tau_H)^2}$，默认参数下仅 $0.059$（§3.3）。只改步长不改符号，但直接拖慢收敛；
- **时间尺度分离**：$\omega$ 必须远慢于闭环带宽（静态映射假设）、远快于适应速率（§6.4）。$\omega$ 过低还会靠近海浪遭遇频率（$0.05\sim0.2$ Hz）的谐波与非线性混频产物——与载波同频的干扰 HPF 挡不住、窗口平均也消不掉；
- **整定顺序**：先定 $T_w$，再取 $\omega=2\pi n/T_w$（每窗恰好 $n$ 个整周期），最后配 $\tau_H$ 使 $\operatorname{Re}H(j\omega)\gtrsim0.9$。

### 7.3 噪声

设每拍成本增量经 HPF 后近似零均值白噪声、方差 $\sigma_y^2$，且与载波不相关，则：

$$\operatorname{Var}\Bigl[\sum_{k=0}^{N-1} y_k\sin\varphi_k\Bigr]=\sum_{k=0}^{N-1}\sigma_y^2\sin^2\varphi_k=\frac{N\sigma_y^2}{2}\;\Longrightarrow\;\boxed{\;\operatorname{Var}[\hat g]=\Bigl(\frac{2}{aN}\Bigr)^2\frac{N\sigma_y^2}{2}=\frac{2\sigma_y^2}{a^2N}\;}$$

- $\propto1/a^2$：加大摄动幅值是压方差最有效的手段，但与抖动代价 $\frac{a^2}{4}J''$（§2.3，$\propto a^2$）正面冲突——$a$ 的取值是经典 ESC 权衡；
- 上式只用到了零均值与不相关假设；与载波**同频**的噪声分量不满足该假设，会原样漏进估计（即 §7.2 的波浪频率问题）；
- 窗内参考机动/海况突变使 $J$ 非平稳，相当于大幅低频噪声，HPF 只能部分抑制，该窗梯度被污染。

### 7.4 非凸性与模型假设

- **局部性**：推导只要求 $J$ 在工作点附近光滑，ESC 收敛到的是**局部**极小。全局非凸（多个极值、鞍点）超出其能力范围——这正是三层调参架构中由调度表/极点配置先给基线、ESC 只做邻域微调的分工依据；
- **静态映射假设的破坏**：闭环未充分收敛时，$J$ 不是 $\theta$ 的静态函数，基波相位不再单纯由梯度符号决定，估计方向可能短时出错（工程上由稳定门 + 回退兜底）；
- **极值点漂移**：海况变化使 $\theta^*$ 移动，积分器会持续跟踪，但跟踪滞后与 §7.1 的窗口折中直接相关；
- **二次项主导区**：$|\Delta\bar\theta|\lesssim O(a)$ 时，二阶谐波泄漏与真实梯度信号同量级（§2.3），估计在最优点附近失去方向性，表现为围绕 $\theta^*$ 的极限环——精度天花板 $O(a)$。

---

**小结**：梯度信息被摄动调制在载波基波上（§2），HPF 去除具淹没性的直流/慢漂（§3），同相解调把基波搬回直流、把干扰留在 $\omega$ 及以上频率（§4），窗口平均加归一化系数 $\frac{2}{aN}$ 完成提取（§4、§6），sin/cos 正交载波让两个参数共用一条链路互不串扰（§5）。整条链路不建模、不差分，只用实测 $J(t)$ 与已知载波的相关性——这就是"为什么这样能测出梯度"。
