# ESC 方案代码事实清单（code_facts.md）

> 本文所有事实以源码为准，引用格式 `文件:行号`。
> 核心文件：`control/include/auto_tuner.hpp`（510 行，namespace `ar`）。
> 配套：`control/include/autopilot_core.hpp`、`pd_controller.hpp`、`nomoto_identifier.hpp`、`gain_schedule.hpp`、`angle_utils.hpp`。
> 测试：`control/tests/test_auto_tuner.cpp`、`control/tests/test_autopilot_core.cpp`。
> 验证：`control/build/tests/test_auto_tuner`（17/17 通过）、`test_autopilot_core`（17/17 通过），2026-08-21 实测。

---

## 1. AutoTuner 公共接口、数据结构与内部状态清单

### 1.1 数据结构

**`IdSnapshot`**（辨识快照，只读投影；AutoTuner 不持有辨识器，由调用方组装传入）— `control/include/auto_tuner.hpp:31-36`

| 字段 | 默认 | 含义 |
|---|---|---|
| `bool excited` | `false` | 最近一次 RLS 拍满足激励条件 |
| `bool valid` | `false` | 估计物理合理（K∈(0,2]、T∈[1,60]） |
| `double K` | `0.0` | Nomoto 增益估计 |
| `double T` | `0.0` | Nomoto 时间常数估计 |

**`TunerInput`**（tick() 每拍输入）— `auto_tuner.hpp:39-48`

| 字段 | 含义 |
|---|---|
| `headingRefDeg` | 目标航向 (°) |
| `headingDeg` | 实际航向 (°) |
| `rudderDeg` | 舵角反馈 (°) |
| `speedKn` | 船速 (kn)，高速去激活门控在内部判定 |
| `dt` | 距上次调用的真实间隔 (s) |
| `schedKp` / `schedKd` | 增益调度名义值，基线未建立时直通 |
| `id` | `IdSnapshot` 辨识快照 |

**`GainCommand`**（tick() 每拍输出，已含 ESC 扰动载波）— `auto_tuner.hpp:51-54`：`double Kp; double Kd;`

**`RingBuffer<T,N>`**（固定容量环形数组，无堆分配，P2 整改）— `auto_tuner.hpp:57-78`
- `push` 覆写最旧元素（`:60-64`）；`size/empty/full/clear`（`:65-68`）；`operator[]` 按写入顺序（最旧→最新）遍历（`:70-73`）。
- 私有：`buf_[N]={}`、`head_=0`、`size_=0`（`:75-77`）。

**`AutoTuner::Params`**（全部默认值）— `auto_tuner.hpp:82-106`

| 参数 | 默认 | 含义 | 行号 |
|---|---|---|---|
| `zeta` | 0.85 | 极点配置目标阻尼比（按船速分段见 §3.10.4） | `:84` |
| `wn` | 0.20 rad/s | 极点配置目标自然频率 | `:85` |
| `escAmp` | 0.05 | ESC 扰动幅值（占增益比例，乘性） | `:87` |
| `escFreq` | 0.05 rad/s | ESC 扰动角频率（周期 ≈125.66 s） | `:88` |
| `escStep` | 0.02 | ESC 积分步长 | `:89` |
| `escHPFtau` | 5.0 s | 高通时间常数，去 J 直流分量 | `:90` |
| `escLPFtau` | 10.0 s | 低通时间常数，平滑梯度估计 | `:91` |
| `wU` | 0.01 | 性能指标中控制能耗权重 | `:93` |
| `wDu` | 0.005 | 控制变化率能耗权重 | `:94` |
| `maxUpdateFrac` | 0.10 | 单次更新幅值 ≤10% | `:96` |
| `absMinUpdate` | 1e-3 | 单次更新绝对下限（P11：old=0 防锁死） | `:97` |
| `zetaMin` | 0.6 | 稳定门：阻尼比下限 | `:98` |
| `wnMax` | 0.5 rad/s | 稳定门：自然频率上限 | `:99` |
| `oscMax` | 5.0 deg | 稳定门：振荡幅值阈值 | `:100` |
| `highSpeedDeactivateKn` | 15.0 | 高速去激活阈值 (kn) | `:102` |
| `windowSec` | 60.0 | 评估窗口长度 (s) | `:104` |
| `dt` | 0.02 | 控制周期 (s)：HPF/ITAE/载波相位/窗口拍数的时基 | `:105` |

### 1.2 公共接口（R1 深化后对外动作仅 3 个：tick / establishBaseline / suspend）

| 接口 | 行号 | 说明 |
|---|---|---|
| `AutoTuner()` / `explicit AutoTuner(const Params&)` | `:108-109` | 构造即 `reset()` |
| `setParams(const Params&)` / `params()` | `:111-112` | 设参并 `reset()` |
| `reset()` | `:114-142` | 全状态清零（见 1.4） |
| `GainCommand tick(const TunerInput&)` | `:150-183` | 每拍主入口，返回本拍增益（含载波） |
| `bool establishBaseline(const IdSnapshot&)` | `:190-194` | 一次性评估并建基线（P14 辨识试验收官/人工标定注入） |
| `void suspend()` | `:199-203` | 切出自动模式时调用：停用 + 清测量状态，保留基线 |
| `bool enabled()` | `:207` | 速度门控结果 |
| `bool baselineEstablished()` | `:209` | 基线状态可观测（R1） |
| `void currentGains(double&,double&)` | `:211` | 当前增益只读投影（不含 ESC 载波） |
| `double computeJ()` | `:214-223` | ITAE 性能指标 J = ∫(t·\|e\| + wU·\|δ\| + wDu·\|dδ\|)dt |
| `static bool polePlacement(K,T,ζ,ωn,Kp&,Kd&)` | `:228-237` | 层2 极点配置，全仓库公式唯一定义处 |
| `bool stabilityCheck(K,T,Kp,Kd,oscAmp)` | `:241-254` | §3.10.7 四条件稳定门 |
| `class OscillationDetector` | `:257-292` | 振荡检测器（嵌套 public 类） |
| `const OscillationDetector& oscDetector()` | `:294` | 只读访问 |

### 1.3 私有方法

| 方法 | 行号 | 说明 |
|---|---|---|
| `setBaseline(Kp,Kd)` | `:300-303` | 写 `baselineKp_/baselineKd_` |
| `applyEstimate_(K,T,oscAmp,driftGate,forceRebase)` | `:308-329` | tick 周期路径与 establishBaseline 一次性路径的共享实现 |
| `settleWindow_(const IdSnapshot&)` | `:333-345` | ESC 窗口结算（稳定门 + 回退） |
| `accumulate(e,delta,oscEligible)` | `:354-376` | 每拍性能样本累积 + 因果解调 |
| `escDither(Kp&,Kd&)` | `:382-388` | 每拍 ESC 扰动注入 |
| `escStep(Kp&,Kd&)` | `:395-426` | 窗末结算：归一化梯度 → LPF → 积分 → 限幅 |
| `revert(Kp&,Kd&)` | `:429-432` | 回退到基线 |
| `rebaseEsc()` | `:438-450` | ESC 积分/解调/窗口样本清零重锚 |
| `clampUpdate(newVal,oldVal)` | `:452-461` | 单次更新幅值约束 |

### 1.4 内部状态变量清单（含默认值与行号）

声明处 `auto_tuner.hpp:463-507`；`reset()` 清零处 `:114-142`。

| 变量 | 默认/初值 | 行号 | 含义 |
|---|---|---|---|
| `Params p_` | 见 1.1 表 | `:463` | 参数组 |
| `double escPhase_` | 0.0 | `:464` | ESC 载波相位 (rad) |
| `double prevDelta_` | 0.0 | `:465` | 上一拍舵角（算 dδ） |
| `double baselineKp_` / `baselineKd_` | 0.0 | `:466-467` | 基线增益（回退目标） |
| `double lastJ_` | 0.0 | `:468` | 上一窗 ITAE 成本（结算时记录） |
| `double gradKp_` / `gradKd_` | 0.0 | `:469-470` | LPF 平滑后的梯度估计（ESC 积分器输入） |
| `double hpfCPrev_` | 0.0 | `:472` | HPF 上一拍成本增量 |
| `double hpfY_` | 0.0 | `:473` | HPF 输出状态 |
| `double demodKpSum_` / `demodKdSum_` | 0.0 | `:474-475` | 解调相关累积量（sin/cos 通道） |
| `uint32_t demodCount_` | 0 | `:476` | 解调累积拍数 |
| `bool enabled_` | false | `:477` | 速度门控结果（每拍刷新） |
| `static constexpr size_t WIN_N` | 3000 | `:479` | 环形缓冲容量（60 s / 0.02 s） |
| `RingBuffer<double,WIN_N> windowErr_` | 空 | `:480` | 窗口 \|e\| 样本 |
| `RingBuffer<double,WIN_N> windowDelta_` | 空 | `:481` | 窗口 \|δ\| 样本 |
| `RingBuffer<double,WIN_N> windowDDelta_` | 空 | `:482` | 窗口 \|dδ\| 样本 |
| `OscillationDetector oscDetector_` | reset | `:483` | 振荡检测器 |
| `bool baselineEstablished_` | false | `:486` | 极点配置基线是否已建立 |
| `double curKp_` / `curKd_` | 0.0 | `:487-488` | 当前增益（基线建立后归 ESC 所有，不随调度表） |
| `double baseK_` / `baseT_` | 0.0 | `:489-490` | 基线对应的 K/T 估计（P12 漂移判据基准） |
| `uint32_t winCounter_` | 0 | `:491` | ESC 窗口节拍计数 |
| `double prevRef_` | 0.0 | `:492` | 航向参考变化检测 |
| `double refAgeSec_` | 0.0 | `:493` | 参考未变累计时长 (s) |
| `static constexpr double kReBaselineTol` | 0.25 | `:499` | K/T 相对漂移重基线阈值 25% |
| `static constexpr double kReBaselineKFloor` | 0.05 | `:501` | K 漂移分母下限 |
| `static constexpr double kReBaselineTFloor` | 1.0 | `:502` | T 漂移分母下限 |
| `static constexpr double kOscRefSettleSec` | 30.0 | `:505` | 参考稳定满 30 s 才喂振荡检测 |
| `static constexpr double PI` | 3.14159265358979323846 | `:507` | 模块内 π（避免依赖 angle_utils 的同名常量歧义） |

**OscillationDetector 内部状态**（`auto_tuner.hpp:287-291`）：`maxPos_/minNeg_`（当前半周正/负极值）、`lastPosPeak_/lastNegPeak_`（上一完整半周峰）、`amp_`（峰-峰幅值估计）、`prevSign_`（上一拍误差符号）。

**reset() 行为**（`:114-142`）：上述全部运行状态清零、`enabled_=false`、`oscDetector_.reset()`、`baselineEstablished_=false`；`p_` 不清（由构造/setParams 管理）。

---

## 2. ESC 信号链逐环节实现细节

### 2.1 `tick()` 每拍主入口 — `auto_tuner.hpp:150-183`

按代码顺序：

1. **速度门控**（`:152`）：`enabled_ = (speedKn <= highSpeedDeactivateKn)`（默认 ≤15 kn 启用）。
2. **基线未建立时跟随调度值**（`:155`）：`if (!baselineEstablished_) { curKp_=schedKp; curKd_=schedKd; }`（P11：否则 ESC 增益每拍被调度表覆盖）。
3. **周期辨识评估**（`:158-161`）：`if (enabled_ && id.excited && id.valid)` → `applyEstimate_(id.K, id.T, oscDetector_.amplitude(), driftGate=true, forceRebase=false)`。
4. **参考稳定度计时**（`:165-166`）：参考变化则 `prevRef_` 更新且 `refAgeSec_=0`，否则 `refAgeSec_ += dt`。
5. **误差与累积**（`:167-168`）：`e = angleDiffDeg(ref, heading)`（`angle_utils.hpp:32-34`，归一化到 (-180,180]）；`accumulate(e, rudderDeg, oscEligible = refAgeSec_ >= kOscRefSettleSec)`（30 s 门限，`:505`）。
6. **窗口结算判定**（`:171-177`）：`winBeats = round(windowSec/dt)`（默认 60/0.02=3000；`dt<=0` 时兜底 3000）；`++winCounter_ >= winBeats` 则清零计数器并 `settleWindow_(id)`。
7. **扰动注入与返回**（`:180-182`）：`Kp=curKp_, Kd=curKd_`；基线已建立才 `escDither(Kp,Kd)`；返回 `{Kp,Kd}`。

### 2.2 `escDither` 扰动注入 — `auto_tuner.hpp:382-388`

```
if (!enabled_) return;
Kp *= 1 + escAmp·sin(escPhase_);   // Kp 用 sin 载波
Kd *= 1 + escAmp·cos(escPhase_);   // Kd 用 cos 载波（与 sin 正交，双参数同时解调）
escPhase_ += escFreq · dt;         // 相位按控制周期推进
if (escPhase_ > 2π) escPhase_ -= 2π;
```

- 乘性相对摄动，幅值 `escAmp=0.05`（±5%），角频率 `escFreq=0.05 rad/s`（周期 ≈125.7 s）。
- 载波**不持久化**：`curKp_/curKd_` 本体不变，只作用于当拍返回值。
- 注意顺序：本拍先以旧相位施加载波，再推进相位（`:384-386`）。

### 2.3 `accumulate` 成本累积 + 每拍因果解调 — `auto_tuner.hpp:354-376`

```
ddelta = delta - prevDelta_
t = windowErr_.size() * p_.dt          // 本拍在窗口内时刻（ITAE 时间权重）
windowErr_.push(|e|); windowDelta_.push(|δ|); windowDDelta_.push(|dδ|)
prevDelta_ = delta
if (oscEligible) oscDetector_.update(e, p_.dt)    // 带符号误差喂振荡检测（P12 门控）
if (!enabled_) return;                            // 以下仅 ESC 启用时
c = (t·|e| + wU·|δ| + wDu·|dδ|) · p_.dt           // 本拍成本增量（与 computeJ 被积函数一致）
alphaH = escHPFtau / (escHPFtau + p_.dt)          // 一阶 HPF 后向欧拉系数（按 0.02 s 设计）
hpfY_ = alphaH·(hpfY_ + c - hpfCPrev_); hpfCPrev_ = c   // 去直流/慢漂
demodKpSum_ += hpfY_·sin(escPhase_)               // 与 Kp 载波同相相关
demodKdSum_ += hpfY_·cos(escPhase_)               // 与 Kd 载波同相相关
++demodCount_
```

- HPF 系数：`alphaH = 5/(5+0.02) ≈ 0.99602`（默认值）。
- 三个环形缓冲**不受 enabled_ 影响**地持续写入（`enabled_` 只门控解调段，`:364`）。
- `t` 由缓冲内样本数推算（`:356`），窗口清零后重新从 0 起计时。

### 2.4 `escStep` 窗末结算 — `auto_tuner.hpp:395-426`

```
if (!enabled_) return;
lastJ_ = computeJ();                                   // 记录本窗 ITAE 成本
norm = 2 / (escAmp · demodCount_)                      // 解调归一化（demodCount_>0 且 escAmp>0 时）
gKp = demodKpSum_·norm;  gKd = demodKdSum_·norm        // ĝ ≈ ∂J/∂(增益相对摄动)
alphaL = windowSec / (escLPFtau + windowSec)           // LPF 系数按窗口周期取（默认 60/70≈0.857）
gradKp_ = (1-alphaL)·gradKp_ + alphaL·gKp              // 梯度 LPF 平滑
gradKd_ = (1-alphaL)·gradKd_ + alphaL·gKd
newKp = Kp - escStep·gradKp_                           // 梯度下降（积分器），步长 escStep=0.02
newKd = Kd - escStep·gradKd_
newKp = clampUpdate(newKp, Kp); newKd = clampUpdate(newKd, Kd)  // 单次更新幅值约束
Kp = max(newKp, 0); Kd = max(newKd, 0)                 // 增益不为负
清空 windowErr_/windowDelta_/windowDDelta_、hpfCPrev_、hpfY_、demodKpSum_/demodKdSum_、demodCount_
```

- 非重叠窗：结算后下一窗从零开始（`:417-425`）。
- 梯度物理意义：ĝ = 2·Σ(y·载波)/(a·N)，即经典摄动 ESC 的梯度估计（注释 `:399`）。

### 2.5 `clampUpdate` 限幅 — `auto_tuner.hpp:452-461`

```
maxDelta = max(|oldVal| · maxUpdateFrac, absMinUpdate)   // 默认 max(10%·|old|, 1e-3)
d = clamp(newVal - oldVal, ±maxDelta)
return oldVal + d
```

P11 整改点：old=0 时纯比例上限为 0 会永久锁死更新（如极点配置把 Kd 钳到 0），故取绝对下限 `absMinUpdate=1e-3`。

### 2.6 `settleWindow_` 窗口结算（稳定门 + 回退）— `auto_tuner.hpp:333-345`

```
if (!enabled_ || !baselineEstablished_) return;
Kp = curKp_; Kd = curKd_;
escStep(Kp, Kd);                                        // 试更新（局部副本）
Khat = id.valid ? id.K : 0;  That = id.valid ? id.T : 0;
if (Khat > 0 && That > 0 && stabilityCheck(Khat, That, Kp, Kd, oscDetector_.amplitude()))
    { curKp_ = Kp; curKd_ = Kd;  return; }              // 接受 ESC 更新（P11：持久化）
revert(curKp_, curKd_);                                 // 否则回退基线
```

注意：快照无效（`valid=false`）时 `Khat=0` → 条件不成立 → 必然 `revert`（本窗 ESC 更新被丢弃）。

### 2.7 `revert` 回退 — `auto_tuner.hpp:429-432`

`Kp = baselineKp_; Kd = baselineKd_;`（恢复极点配置基线，非上次值）。

### 2.8 `rebaseEsc` 重锚 — `auto_tuner.hpp:438-450`

清零：`gradKp_/gradKd_`（积分器）、`hpfCPrev_/hpfY_`（HPF）、`demodKpSum_/demodKdSum_/demodCount_`（解调）、三个窗口环形缓冲、`prevDelta_`。
**不清** `escPhase_`：相位连续避免重锚拍载波突跳（注释 `:436-437`）。

### 2.9 `stabilityCheck` 四条件稳定门 — `auto_tuner.hpp:241-254`

```
if (!isfinite(K) || !isfinite(T)) return false;
if (|K| < 1e-6 || |T| < 1e-6) return false;
if (Kp <= 0) return false;
wn   = sqrt(Kp·K/T)                        // 由 PD+Nomoto 闭环关系反解
zeta = (1 + Kd·K) / (2·sqrt(T·Kp·K))
if (!isfinite(wn) || !isfinite(zeta)) return false;
拒绝条件：zeta < zetaMin(0.6)  |  wn > wnMax(0.5)  |  oscAmplitude > oscMax(5.0°)
```

四个代数/测量条件全满足才接受（注释 `:239`：§3.10.7）。

### 2.10 `OscillationDetector` 振荡检测 — `auto_tuner.hpp:257-292`

基于航向误差**峰-峰幅值**估计（跨完整周期）：
- `update(e, dt)`（`:265-285`）：跟踪误差符号；过零时结算——刚结束正半周则保存 `lastPosPeak_=maxPos_` 并重置 `maxPos_`；刚结束负半周则保存 `lastNegPeak_=minNeg_`、重置 `minNeg_`，并计算 `pp = lastPosPeak_ - lastNegPeak_`：若 `pp > amp_` 直接取大者（保守），否则 `amp_ = 0.9·amp_ + 0.1·pp` 平滑衰减。
- 非过零拍持续刷新当前半周极值（`:282-283`）。
- `reset()`（`:260-264`）清空含历史峰在内的全部状态（P11：防止上一窗口峰值泄漏）。
- `amplitude()` 返回 `amp_`（`:286`）。

### 2.11 `applyEstimate_` 漂移重基线 — `auto_tuner.hpp:308-329`

```
if (driftGate && baselineEstablished_
    && |K - baseK_| <= kReBaselineTol·max(|baseK_|, kReBaselineKFloor)   // 25%, floor 0.05
    && |T - baseT_| <= kReBaselineTol·max(|baseT_|, kReBaselineTFloor))  // 25%, floor 1.0
    return false;                                   // P12：漂移在阈值内 → ESC 自治，不动基线
if (!polePlacement(K, T, p_.zeta, p_.wn, Kp2, Kd2)) return false;   // P6：退化拒绝
if (!stabilityCheck(K, T, Kp2, Kd2, oscAmp)) return false;          // P4/§3.10.7
rebaseline = baselineEstablished_;
setBaseline(Kp2, Kd2); baselineEstablished_ = true;
curKp_ = Kp2; curKd_ = Kd2;  baseK_ = K; baseT_ = T;
if (forceRebase || rebaseline) { rebaseEsc(); winCounter_ = 0; }    // 显式重基线才重锚
return true;
```

两条调用路径：
- tick 周期路径（`:159-160`）：`driftGate=true, forceRebase=false`，osc 取 `oscDetector_.amplitude()`。
- establishBaseline 一次性路径（`:192-193`）：`driftGate=false, forceRebase=true`，osc 恒传 0（机动刚结束振荡测量无效，由 ζ/ωn 代数条件把关）。

### 2.12 `suspend` 挂起 — `auto_tuner.hpp:199-203`

```
enabled_ = false;
rebaseEsc();        // 清测量状态（窗口/解调/HPF/梯度），保留 baselineKp_/baselineKd_
winCounter_ = 0;
```

P9 完整版：旧 `disable()` 只翻标志位，手动操舵期间旧测量会混进下一窗口。由 `AutopilotCore::setMode` 在切到 MANUAL/HOLD 时调用（`autopilot_core.hpp:47`）。

### 2.13 `establishBaseline` — `auto_tuner.hpp:190-194`

```
if (!snap.excited || !snap.valid) return false;
return applyEstimate_(snap.K, snap.T, 0.0, driftGate=false, forceRebase=true);
```

语义：P14 辨识试验收官（`AutopilotCore::finalizeIdentTrial`，`autopilot_core.hpp:184-187`），亦可人工标定注入；恒重锚 ESC，返回是否建立（映射试验 OK/FAILED）。

### 2.14 `computeJ` ITAE 指标 — `auto_tuner.hpp:214-223`

```
J = Σ_i ( t_i·|e_i| + wU·|δ_i| + wDu·|dδ_i| ) · dt,   t_i = i·dt
```

窗口为空返回 0。被积函数与 `accumulate` 的每拍成本增量 `c` 一致（`:366-367`）。

---

## 3. 每拍主流程与窗末结算流程（步骤化，可直接画流程图）

### 3.1 每拍主流程（50 Hz，`AutopilotCore::step` → `headingControlStep` → PD → δ_cmd）

`AutopilotCore::step(s, dt)`（`autopilot_core.hpp:82-136`）按模式分发；AUTO_HEADING 走 `headingControlStep`（`:151-173`）：

1. **首拍舵速基准**（`:153-156`）：`pendingBaseline_` 为真则 `pd_.initBaseline(s.rudderDeg)`（P11：以实际舵角初始化舵速限幅基准，避免舵令从 0° 爬升）。
2. **增益调度查询**（`:159-160`）：`schedule_.query(speedKn, seaState, Kp, Kd)` —— `GainSchedule::query`（`gain_schedule.hpp:43-75`）：按海况筛选分段 → 速度维中点锚定线性插值（P5），边界外取首/末段名义值。默认表 12 个分段（`:22-38`：0-4/4-10/10-18/18-25 kn × 海况 0-3）。
3. **喂辨识器**（`:163`）：`identifier_.update(rudderDeg, rateDegS, dt)` —— `NomotoIdentifier::update`（`nomoto_identifier.hpp:49-110`）：入口范围校验（有限性、\|δ\|≤35°、\|r\|≤30°/s、dt>0，`:53-55`）→ 自降频累计满 `beatSec=1.0 s` 才做一次 RLS 拍（`:58-61`）→ 激励判定（\|δ\|>3° 且（\|r\|>0.5°/s 或 \|dr\|>0.2°/s²），`:68-69`）→ 归一化回归量 φ=[δ/50, −dr/1]（`:76`）→ RLS 更新 θ、P（λ=0.98，P-floor=1e-3，`:79-106`）。
4. **组装快照 + 调参器 tick**（`:166-167`）：`idSnapshot()`（`:176-178`）返回 `{excited(), valid(), K(), T()}`；`tuner_.tick({headingRef_, headingDeg, rudderDeg, speedKn, dt, Kp, Kd, snap})` —— 内部流程见 2.1（速度门控 → 调度直通/周期评估 → accumulate → 窗口结算 → escDither → 返回 GainCommand）。
5. **应用增益并闭环**（`:170-172`）：`pd_.setDt(dt)`；`pd_.setGains(g.Kp, g.Kd)`（P1：轻量设置不清状态，`pd_controller.hpp:37-38`）；`pd_.update(headingRef_, headingDeg)` → δ_cmd：
   - 误差 `e = angleDiffDeg(ref, act)`（`pd_controller.hpp:68`）；
   - 死区 \|e\|<0.3° → `saturate(0)` 经舵速限幅回中（`:71-76`）；
   - 带滤波微分：首拍 0，否则 `(e-prevErr)/dt` 经一阶低通 α=1/(1+τ/dt)（`:79-87`）；
   - 控制律 `out = Kp·e + Kd·deriv`（`:90`）；
   - 舵角限幅 ±35°（`:93-94`）→ 舵速限幅 ±ddMax·dt（`:97-100`）→ 量化 0.1°（`:103`）；
   - 更新 `prevErr_/prevDeriv_/prevOut_`，返回 out（`:105-108`）。
6. `step()` 末尾记录 `lastRudderCmd_`（`:134`，供 HOLD 冻结）。

其他模式要点：MANUAL 透传舵角反馈（`:87`）；AUTO_MANEUVER 由 `sequencer_.update` 出指令，HEADING 段做调度查询+PD（`:111-120`）、RUDDER 段经 `pd_.saturate`（`:121-125`），机动完成拍若是辨识试验则 `finalizeIdentTrial()`，随后转 AUTO_HEADING 并立即闭环（`:96-105`）；HOLD 输出冻结舵角（`:131`）。

### 3.2 每 60 s 窗末结算流程（默认 windowSec=60 s、dt=0.02 s → 3000 拍）

触发：`tick` 内 `++winCounter_ >= winBeats(=3000)` → `winCounter_=0` → `settleWindow_(id)`（`auto_tuner.hpp:171-177`）。

1. **前置门**：`enabled_` 且 `baselineEstablished_`，否则直接返回（`:334`）。
2. **记录成本**：`lastJ_ = computeJ()`（`:398`）。
3. **梯度归一化**：`ĝ = 2·Σ(hpfY·载波)/(escAmp·N)`，得 `gKp/gKd`（`:400-405`）。
4. **梯度 LPF**：`αL = windowSec/(escLPFtau+windowSec) = 60/70 ≈ 0.857`；`grad = (1−αL)·grad + αL·g`（`:407-409`）。
5. **积分更新**：`new = old − escStep·grad`（步长 0.02）（`:411-412`）。
6. **限幅**：`clampUpdate`（max(10%·\|old\|, 1e-3)）（`:413-414`）→ 负值钳 0（`:415-416`）。
7. **清窗**：清三个环形缓冲 + HPF 状态 + 解调累积（`:418-425`）。
8. **稳定门**：取 `Khat/That`（快照无效则为 0）；`Khat>0 && That>0 && stabilityCheck(Khat,That,newKp,newKd,oscDetector_.amplitude())`（`:337-340`）。
9. **接受或回退**：通过 → `curKp_/curKd_` 持久化为试更新值（`:341`）；否则 `revert` 回 `baselineKp_/baselineKd_`（`:344`）。
10. 下一拍起 `escDither` 继续在新（或回退后的）`curKp_/curKd_` 上叠加载波。

### 3.3 基线建立/重基线流程（applyEstimate_）

- 每拍周期路径：快照 `excited&&valid` 且 `enabled_` → 漂移检查（25% 阈值内直接返回 false，ESC 自治）→ 极点配置 → 稳定门 → 写基线/当前增益/baseK_/baseT_ → 若已有旧基线则 `rebaseEsc()` + `winCounter_=0`（`auto_tuner.hpp:308-329`）。
- 一次性路径（辨识试验收官）：`finalizeIdentTrial`（`autopilot_core.hpp:184-187`）→ `establishBaseline` → 同一路径但跳过漂移检查、osc=0、恒重锚；返回值映射 OK/FAILED。

---

## 4. polePlacement 极点配置的代码-公式对应

代码（`auto_tuner.hpp:228-237`）：

```
if (!isfinite(K) || !isfinite(T)) return false;
if (|K| < 1e-6 || |T| < 1e-6) return false;          // P6：退化拒绝
Kp = wn² · T / K                                     // :233
kdRaw = (2·ζ·wn·T − 1) / K                           // :234
Kd = max(kdRaw, 0)                                   // :235 负 Kd 会反向放大振荡，clamp
```

公式推导对应（PD + Nomoto 一阶模型闭环）：
- 对象 `T·ṙ + r = K·δ`，PD 律 `δ = Kp·e + Kd·ė`（e = ref − ψ，r = ψ̇），闭环特征多项式 `T·s² + (1 + K·Kd)·s + K·Kp`。
- 与标准二阶 `s² + 2ζωn·s + ωn²` 对比得：`ωn² = Kp·K/T`，`2ζωn = (1 + K·Kd)/T`。
- 反解即代码两式：`Kp = ωn²·T/K`，`Kd = (2ζωn·T − 1)/K`。

`stabilityCheck`（`:247-248`）正是该关系的正向验证：`wn = sqrt(Kp·K/T)`、`zeta = (1+Kd·K)/(2·sqrt(T·Kp·K))`（把 `2ζωn=(1+KdK)/T` 与 `ωn` 联立消元）。

文件头注释（`:3`）亦标明：`Kp = ωn²·T / K, Kd = (2ζωn·T − 1) / K`，且 `:227` 注释强调"公式全仓库唯一定义处（仿真场景安装调度表亦应调此处）"。

---

## 5. ESC/自动调参相关测试用例清单与断言要点

### 5.1 `control/tests/test_auto_tuner.cpp`（14 个用例）

| 用例 | 行号 | 断言要点 |
|---|---|---|
| `PolePlacementProducesTargetZetaWn` | `:27-35` | K=0.2,T=8,ζ=0.85,ωn=0.25 配置出的 Kp/Kd 反算 ωn 误差 <1e-6、ζ 误差 <1e-3 |
| `PolePlacementRejectsDegenerate` | `:37-41` | K=0 或 T=0 返回 false |
| `PolePlacementClampsNegativeKd` | `:43-48` | 小 ωn(0.05) 使 Kd_raw<0 时钳到 ≥0 |
| `StabilityCheckAcceptsHealthy` | `:50-55` | 健康配置（osc=0）通过稳定门 |
| `StabilityCheckRejectsLowZeta` | `:57-61` | Kd=0 → 低阻尼被拒 |
| `StabilityCheckRejectsHighWn` | `:63-69` | Kp×10 → ωn 超限被拒 |
| `OscillationDetectorMeasuresAmplitude` | `:72-79` | ±3° 正弦误差（200 拍）→ 峰-峰幅值 ≈6°±1 |
| `StabilityCheckRejectsOscillation` | `:81-87` | osc=8° > oscMax 5° 被拒 |
| `RingBufferNoHeap` | `:90-97` | 容量 16 推 100 个：size 恒 16，最旧 84 最新 99 |
| `OscillationDetectorResetClearsPeaks` | `:100-113` | reset 后幅值归零；复位后过零结算得 pp=1（验证历史峰不泄漏） |
| `EscDitherModulatesGains` | `:118-137` | 基线未建立时调度增益原样直通（无载波）；建立基线 (1.6, 8.6) 后 2 个整载波周期内 Kp 覆盖 ±escAmp 相对调制（峰-峰 ≈2·escAmp·1.6±0.02），Kd 同样在 ±escAmp 内 |
| `WindowSettleUpdateIsClamped` | `:141-172` | 10 个窗口（500 拍/窗）逐窗断言：更新受 clampUpdate 约束（≤max(10%·\|old\|,1e-3)）**或**被稳定门拒绝后精确回退基线（1e-9）；增益始终有限 |
| `EscTunesKpTowardOptimum` | `:177-216` | 端到端闭环（tick→PD→Nomoto 对象 K=0.5/T=2）：故意偏低基线 Kp=0.15，8 窗后 KpEnd > 0.18（向增大方向）且 Jlast < Jfirst |
| `EscUpdateUnsticksFromZeroKd` | `:219-245` | 基线 Kd=0 时，注入与 cos 载波负相关的成本，3 窗后 KdEnd > 0（解锁）且 ≤3·absMinUpdate+1e-12 |
| `SettleRevertsToBaselineWhenEstimateInvalid` | `:248-265` | 快照 valid=false → 结算不过稳定门 → 精确 revert 回基线 (1.6, 8.6)（1e-9） |
| `SettleAcceptsUpdateWhenEstimateHealthy` | `:267-285` | 快照健康 → Kd 按负相关成本梯度上移（>8.6）且有界（≤8.6+max(8.6·10%,1e-3)） |
| `RebaselineClearsIntegratorState` | `:289-312` | 半窗解调累积后显式重基线 B：`computeJ()==0`；重锚后跑零成本整窗结算，增益精确等于新基线（DOUBLE_EQ，证明旧解调累积已作废） |

（注：`EscTunesKpTowardOptimum` 等测试用 `makeSnap(K,T)` 使极点配置恰好产出目标基线，如 `:184` 的 K=40/31、T=150/31 → (0.15, 0.5)。）

### 5.2 `control/tests/test_autopilot_core.cpp` 中调参链路相关用例（8 个）

| 用例 | 行号 | 断言要点 |
|---|---|---|
| `HighSpeedDisablesTuner` | `:71-78` | 20 kn > 15 kn 阈值 → `tuner().enabled()==false` |
| `LowSpeedEnablesTuner` | `:80-87` | 10 kn → `enabled()==true` |
| `EscDitherReachesPdGainsAfterBaseline` | `:142-191` | 合成 Z 形驱动在线辨识（K=0.2/T=8）→ 极点配置基线 Kp≈1.6（±0.2）建立；之后 2 个载波周期内 PD 实际增益峰-峰 > 0.5·2·escAmp·1.6（载波真实到达 PD） |
| `NoBaselinePassesScheduleGainsThrough` | `:193-206` | 无激励时 100 拍后 PD 增益恒为调度值 (1.2, 0.5)（1e-9），基线不建立 |
| `IdentUpdatesDoNotOverwriteEscGains` | `:215-275` | P12 回归：T 从 8 漂到 12 期间，漂移 +13%（<25%）时增益钉在旧基线 (≈1.588, 8.30)；漂移越阈后显式重基线跳到新配置 (≈2.12, 12.54)，Kd 增量 >3 |
| `EscFallbackRestoresBaselineOnOscillation` | `:282-354` | 先注恒正成本（与 sin 载波负相关，不触发振荡检测）→ Kp 上移 >4% 被接受；再注零均值 ±6° 振荡（峰-峰 12°>5°）→ 结算被拒，增益回退基线（Kp±0.03 / Kd±0.20） |
| `OscDetectorStarvedDuringReferenceMotion` | `:357-378` | 参考每 15 s 翻转时振荡检测幅值恒 0（不喂入）；参考稳定 30 s 后恢复喂入，±8° 摆动幅值 >8° |
| `IdentTrialIdentifiesAndAppliesGains` | `:401-428` | P14 端到端：自动 Z 形试验收官 → identTrialState==OK、辨识 K≈0.2±0.05/T≈8±2、PD 增益 ≈ 极点配置 (1.6, 8.6)（Kp±0.8/Kd±4.0）、模式回 AUTO_HEADING |

验证结果：`control/build/tests/test_auto_tuner` 17/17 PASSED、`test_autopilot_core` 17/17 PASSED（2026-08-21 在本机 control/build 下构建并运行）。

---

## 6. 存疑与观察（供后续文档/PPT 决策）

1. **解调载波与施加载波存在一拍相位差**【存疑】：`tick` 中 `accumulate`（用当前 `escPhase_` 解调，`:373-374`）先于 `escDither`（用同一相位施加载波后再推进相位，`:384-386`）。即第 k 拍施加的载波相位为 φ_k，而第 k+1 拍测量成本时以 φ_{k+1} 解调——严格说有 1 拍（0.02 s）错位。默认 ω=0.05 rad/s 时相位差仅 0.001 rad，影响可忽略；高频测试载波（如 2π rad/s，0.126 rad/拍）下相关增益略有衰减但测试仍通过。是否为有意设计（对齐"施加后下一拍才见成本"的因果延迟）源码无注释说明。
2. **梯度 LPF 系数偏大**：`alphaL = windowSec/(escLPFtau+windowSec)`，默认 60/70≈0.857，即每窗新梯度样本权重 85.7%，平滑作用弱（形式上接近"按窗口周期离散化的一阶惯性"，但 tau=10 s ≪ windowSec=60 s 时平滑名不副实）。`lastJ_` 记录后未被任何逻辑读取（只写不读）【存疑：疑似遗留调试字段】。
3. **窗口缓冲与 enabled_ 解耦**：`accumulate` 中三个环形缓冲在 `enabled_==false`（高速）时仍持续写入（`:357-359` 在 `:364` 的 enabled 检查之前），仅解调段被门控；窗口拍数计数器 `winCounter_` 也不受 enabled 影响地推进并重置（`:174-177`），只是 `settleWindow_` 内部直接返回。长时高速后窗口内容是全速段数据，恢复低速首个窗口的 `computeJ` 会混入高速段样本（解调累积则只含低速段）。【存疑：是否有意】
4. **默认载波周期 > 窗口长度**：escFreq=0.05 rad/s → 周期 ≈125.7 s，而 windowSec=60 s，每窗不足半个载波周期，单窗内 sin/cos 相关不具有整周期正交性，梯度估计依赖 LPF 跨窗平均。测试中为获得整周期均改用更快载波（如 1 s 周期）。这是参数设计事实，非 bug，但 PPT 讲解"正交解调"时需注意默认参数下单窗不正交。
5. **快照无效 ⇒ 必回退**：`settleWindow_` 中 `id.valid==false` 时 Khat=0 → 稳定门不通过 → revert（`:337-344`）。即辨识长期无效时，ESC 每窗试更新都被丢弃，增益钉在极点配置基线。属安全设计，但意味着"辨识失效时 ESC 实际停摆"。
6. **`kOscRefSettleSec=30 s` 门控用 `in.dt` 累加**（`:166`），而 `escDither`/`accumulate` 内部用 `p_.dt`（固定 0.02）作时基——真实调用周期与 `p_.dt` 不一致时（调用方 dt 变化），HPF 系数、载波相位推进、ITAE 时间权重仍按 0.02 s 假设，refAgeSec_ 却按真实 dt。混合时基是既有设计（`p_.dt` 注释 `:105` 自称"内部系数时基"），非错误，但文档中应明确说明。
