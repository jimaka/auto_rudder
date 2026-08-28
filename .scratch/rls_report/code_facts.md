# RLS 在线辨识方案——代码事实清单（code_facts.md）

> 本文是《RLS 方案数学推导与算法实现详解》的事实底座：一切结论以源码为准，引用格式 `文件:行号`。
> 核心文件：`control/include/nomoto_identifier.hpp`（辨识器本体，136 行，header-only，namespace `ar`）。
> 关联文件：`control/include/autopilot_core.hpp`（调用方）、`control/include/auto_tuner.hpp`（消费方）、
> `control/include/maneuver_sequencer.hpp`（Z 形机动生成）、`control/tests/test_nomoto_identifier.cpp` 等。
> 验证环境：cmake 4.2.3 + g++ 15.2.0，构建目录 `control/build`（已存在，直接增量构建）。

---

## 1. NomotoIdentifier 公共接口与内部状态

### 1.1 类定位与数学模型（文件头注释 `control/include/nomoto_identifier.hpp:1-8`）

- 对应设计文档 §3.9.4（Z 形试验辨识）与 §3.10.3（RLS 在线辨识）。
- 模型：一阶 Nomoto `T·dr + r = K·δ`，离散回归式 `r_k = K·δ_k − (T/Δt)·(r_k − r_{k−1})`。
- 回归定义：`φ_k = [δ_k, −(r_k − r_{k−1})/Δt]ᵀ`，`y_k = r_k`，`θ = [K, T]ᵀ`（注释，`nomoto_identifier.hpp:3-4`）。
- 注意：代码中 θ 存的是**归一化尺度下**的参数（θ₀ = K·dScale，θ₁ = T·drScale），物理量经 `K()`/`T()` 反归一化得到（见 2.7）。

### 1.2 公共接口一览

| 接口 | 行号 | 说明 |
|---|---|---|
| `struct Params` | `nomoto_identifier.hpp:17-27` | 参数组（默认值见 1.3） |
| `NomotoIdentifier()` | `:29` | 默认构造，`reset()` |
| `explicit NomotoIdentifier(const Params&)` | `:30` | 带参构造，`reset()` |
| `void setParams(const Params&)` | `:32` | 整组设参并 `reset()` |
| `const Params& params() const` | `:33` | 只读参数 |
| `void reset()` | `:35-44` | 全状态复位（初值见 1.4） |
| `bool update(double delta, double r, double dtNow)` | `:49-110` | 主入口；返回"本拍 RLS 更新是否被接受" |
| `double K() const` | `:114` | 物理量 K = θ₀/dScale |
| `double T() const` | `:115` | 物理量 T = θ₁/drScale |
| `bool excited() const` | `:116` | 最近一次判定是否满足激励条件 |
| `bool valid() const` | `:119-124` | 物理合理性门限（P6 整改） |

`update()` 输入契约（注释 `:46-48`）：`delta` 舵角 (deg)、`r` 转向率 (deg/s)、`dtNow` 距上次调用的真实间隔 (s)；
可任意频率调用，内部累计满 `beatSec` 才做一次 RLS 拍（点采样：取拍时刻样本）；未到期/被拒一律返回 `false`。

### 1.3 Params 全部字段与默认值（`:17-27`）

| 字段 | 默认值 | 行号 | 语义 |
|---|---|---|---|
| `lambda` | 0.98 | `:18` | 遗忘因子 λ |
| `beatSec` | 1.0 | `:19` | RLS 拍间隔 (s)（P3：降频减轻 ECU 负担、保证差分信噪比） |
| `deltaMin` | 3.0 | `:20` | 激励阈值：舵角 (deg) |
| `rMin` | 0.5 | `:21` | 激励阈值：转向率 (deg/s) |
| `drMin` | 0.2 | `:22` | 激励阈值：转向加速度 (deg/s²)，保留过零瞬态以辨识 T |
| `p0` | 1.0 | `:23` | 协方差初值（归一化后回归量 O(1)，p0 取 1 量级） |
| `dScale` | 50.0 | `:25` | 舵角归一化尺度 (deg) |
| `drScale` | 1.0 | `:26` | 转向加速度归一化尺度 (deg/s²) |

### 1.4 内部状态变量清单（私有段 `:126-133` + `reset()` `:35-44`）

| 变量 | 类型 | 声明行 | 默认值（声明处） | reset() 值 | 语义 |
|---|---|---|---|---|---|
| `p_` | `Params` | `:127` | — | 不复位 | 参数副本 |
| `theta_[2]` | `double[2]` | `:128` | — | `[0.0, 0.0]`（`:36-37`） | 归一化参数估计 [θ₀, θ₁] |
| `P_[2][2]` | `double[2][2]` | `:129` | — | `diag(p0, p0)`（`:38-39`） | RLS 协方差阵 |
| `prevR_` | `double` | `:130` | 0.0 | 0.0（`:40`） | 上一次 RLS 拍时刻的转向率（差分锚点） |
| `elapsed_` | `double` | `:131` | 0.0 | 0.0（`:41`） | 距上一次 RLS 拍的累计时间 (s)（自降频） |
| `first_` | `bool` | `:132` | true | true（`:42`） | 首个拍标志（首拍无差分） |
| `excited_` | `bool` | `:133` | false | false（`:43`） | 最近一拍是否过激励门 |

---

## 2. `update()` 实现细节逐环节（`:49-110`）

总伪代码（与行号一一对应）：

```
bool update(δ, r, dtNow):
    # (a) 入口防御                                   :53-55
    if !isfinite(δ) || !isfinite(r):  return false   # 不写 prevR_、不累计 elapsed_
    if |δ| > 35.0 || |r| > 30.0:      return false
    if !(dtNow > 0):                  return false
    # (b) 自降频                                     :58-61
    elapsed_ += dtNow
    if elapsed_ < beatSec: return false
    beatDt = elapsed_; elapsed_ = 0
    # (c) 差分                                       :65-67
    dr = first_ ? 0 : (r − prevR_)/beatDt;  first_ = false
    # (d) 激励门控                                   :68-73
    excited_ = (|δ| > deltaMin) && (|r| > rMin || |dr| > drMin)
    if !excited_: prevR_ = r; return false           # 未激励：写锚点、不递推
    # (e) 回归向量（归一化）                          :76
    φ = [δ/dScale, −dr/drScale]
    # (f) 增益                                       :79-85
    Pφ = P·φ;  denom = λ + φᵀPφ
    if |denom| < 1e-12: prevR_ = r; return false
    Kgain = Pφ / denom
    # (g) 残差与参数更新                              :88-94
    err = r − φᵀθ
    θ += Kgain · err
    # (h) 协方差更新 + 对角保底                       :97-106
    P ← (1/λ)(P − Kgain·(Pφ)ᵀ)
    P[0][0] = max(P[0][0], 1e-3);  P[1][1] = max(P[1][1], 1e-3)
    prevR_ = r; return true                          :108-109
```

### 2.1 入口范围检查与 NaN 防御（P6/P11，`:50-55`）

- `:53` `!std::isfinite(delta) || !std::isfinite(r)` → 丢弃：NaN/Inf 防御。
- `:54` `|δ| > 35.0 || |r| > 30.0` → 丢弃：物理范围检查（舵角 35°、转向率 30°/s，与最大舵角/典型转艏率包线一致）。
- `:55` `!(dtNow > 0.0)` → 丢弃：非法采样周期（防除零/时间倒流；同时挡住 NaN 的 dtNow）。
- **关键语义**：这三个早退都在 `elapsed_ += dtNow`（`:58`）之前，也不写 `prevR_`——即 P11 整改（注释 `:51-52`）：被拒绝的拍不会成为下一拍差分锚点，否则 NaN/毛刺经 `dr=(r−prevR_)/Δt` 污染 φ，永久毒化 θ/P。

### 2.2 自降频机制（R1，`:57-61` + 头注释 `:5-8`）

- `elapsed_ += dtNow`（`:58`）；`elapsed_ < p_.beatSec` 则仅累计不递推（`:59`）。
- 到期：`beatDt = elapsed_`（`:60`，自测拍长 ≈ 1 s，与调用频率无关），`elapsed_ = 0`（`:61`）。
- 设计动机（头注释 `:5-8`）：旧设计把"50 Hz × 1 s"降频约定放在调用方（计数器 + 改写 Params.dt），调用频率一变差分即静默出错；现 Δt 由模块自测（两次 RLS 拍之间的真实间隔），`Params.dt` 语义陷阱已删除。
- 采样方式：点采样——取拍时刻的 (δ, r) 样本，非拍内平均（注释 `:47`）。

### 2.3 差分与首拍处理（`:65-67`）

- `first_` 为真时 `dr = 0` 并清标志（`:66`）；否则 `dr = (r − prevR_)/beatDt`（`:67`，后向差分，Δt 用自测拍长）。
- 注意 `first_` 在**首个通过降频的拍**即被消耗，与该拍是否过激励门无关；未激励分支会写 `prevR_`（`:71`），故第二拍起差分锚点始终有效。

### 2.4 激励门控条件（§3.10.3，`:63-73`）

- 判据（`:68-69`）：`|δ| > deltaMin(3.0°)` **且** `(|r| > rMin(0.5°/s)` **或** `|dr| > drMin(0.2°/s²))`。
- 设计理由（注释 `:63-64`）：保留 r 过零附近的高 dr 瞬态，否则 T 欠定（r 换向瞬间 |r| 小但 |dr| 大，是辨识 T 的信息来源）。
- 未过门：`prevR_ = r; return false`（`:70-73`）——**注意与 2.1 的区别**：未激励拍会写差分锚点（数据合法只是信息量不足），仅入口被拒的拍不写。
- `excited_` 每个降频拍都刷新（`:68`），未到期调用不刷新。

### 2.5 回归向量构造与归一化（`:75-76`）

- `phi[2] = { delta / p_.dScale, −dr / p_.drScale }`（`:76`）。
- 归一化目的（注释 `:24, :75`）：消除 δ（数十度）与 dr（零点几度/s²）量级差导致的 P 矩阵病态；归一化后回归量 O(1)，与 `p0 = 1.0` 量级匹配。

### 2.6 RLS 三式（`:78-106`）

- **增益**（`:79-85`）：`Pφ = P·φ`；`denom = λ + φᵀPφ`（`:83`）；分母保护 `|denom| < 1e-12` → 写锚点后返回 false（`:84`）；`gainK = Pφ/denom`（`:85`）。即 `K_k = P·φ/(λ + φᵀPφ)`。
- **残差与参数更新**（`:87-94`）：`y = r`（`:88`）；`ŷ = φᵀθ`（`:89`）；`err = y − ŷ`（`:90`）；`θ ← θ + gainK·err`（`:93-94`）。
- **协方差更新**（`:96-103`）：`invL = 1/λ`（`:97`）；双循环 `P_[i][j] = invL·(P_[i][j] − gainK[i]·Pphi[j])`（`:99-103`），即 `P ← (1/λ)(P − K·φᵀ·P)`（利用 P 对称，`φᵀP = (Pφ)ᵀ`）。
- **P 对角保底**（`:98, :104-106`）：`pFloor = 1e-3`（`:98`）；`P[0][0]`、`P[1][1]` 低于 1e-3 即钳到 1e-3——防协方差塌陷、保持长期自适应能力（注释 `:96`）。仅钳对角元，不动非对角元。

### 2.7 物理量反归一化与 valid() 物理门（`:112-124`）

- 推导注释（`:112-113`）：`r = K·δ − T·dr = θ₀·(δ/dScale) + θ₁·(−dr/drScale)` ⇒ `K = θ₀/dScale`，`T = θ₁/drScale`。
- `K()` `:114`、`T()` `:115` 实时反归一化，无缓存。
- `valid()`（P6 整改，`:118-124`）：用物理量判定——`isfinite(K) && isfinite(T) && K > 0.0 && K <= 2.0 && T >= 1.0 && T <= 60.0`，即 **K∈(0, 2]、T∈[1, 60] s**。

---

## 3. 每拍数据流（50 Hz 主循环 → RLS → 极点配置/稳定门）

### 3.1 常规航向保持路径（AUTO_HEADING）

1. `AutopilotCore::step(s, dt)`（`autopilot_core.hpp:82`）按模式分发，`AUTO_HEADING` → `headingControlStep(s, dt)`（`:91`）。
2. `headingControlStep`（`:151-173`）依次：
   - 首拍舵速限幅基准初始化（P11，`:153-156`，与辨识无关）。
   - 增益调度查询 `schedule_.query(speedKn, seaState, Kp, Kd)`（`:160`）。
   - **喂辨识器** `identifier_.update(s.rudderDeg, s.rateDegS, dt)`（`:163`）——每拍无条件调用，返回值被忽略（自降频内部消化节拍）。
   - **组装快照并 tick 调参器** `tuner_.tick({headingRef_, headingDeg, rudderDeg, speedKn, dt, Kp, Kd, idSnapshot()})`（`:166-167`）；`idSnapshot()`（`:176-178`）= `{identifier_.excited(), identifier_.valid(), identifier_.K(), identifier_.T()}`，对应 `IdSnapshot` 结构（`auto_tuner.hpp:31-36`，字段 excited/valid/K/T 及默认值在行内）。
   - `pd_.setDt(dt); pd_.setGains(g.Kp, g.Kd); return pd_.update(...)`（`:170-172`）。
3. `AutoTuner::tick(in)`（`auto_tuner.hpp:150-183`）内部：
   - 速度门控 `enabled_ = (in.speedKn <= p_.highSpeedDeactivateKn)`（`:152`，默认 15 kn，`:102`）。
   - 基线未建立时 `curKp_/curKd_` 跟随调度名义值（`:155`）。
   - **周期辨识评估**：`if (enabled_ && in.id.excited && in.id.valid) applyEstimate_(in.id.K, in.id.T, oscDetector_.amplitude(), driftGate=true, forceRebase=false)`（`:158-161`）。
   - 振荡检测门控 + 成本/解调累积（`:165-168`，参考稳定满 `kOscRefSettleSec=30 s` 才喂振荡检测，常量 `:505`）。
   - ESC 窗口结算 `settleWindow_(in.id)`（`:171-177`；窗口拍数 = `windowSec/dt`，`:171-173`）。
   - ESC 扰动注入并返回增益（`:180-182`）。
4. `applyEstimate_(K, T, oscAmp, driftGate, forceRebase)`（`:308-329`）：
   - **漂移门**（P12，`driftGate=true` 且已有基线时，`:309-313`）：`|K−baseK_| ≤ 0.25·max(|baseK_|, 0.05)` 且 `|T−baseT_| ≤ 0.25·max(|baseT_|, 1.0)` 则不动基线（ESC 自治）。常量 `kReBaselineTol=0.25`（`:499`）、`kReBaselineKFloor=0.05`（`:501`）、`kReBaselineTFloor=1.0`（`:502`）。
   - **极点配置** `polePlacement(K, T, ζ=0.85, ωn=0.20, Kp2, Kd2)`（`:315`；实现 `:228-237`）：非有限或 `|K|<1e-6 || |T|<1e-6` → false（`:231-232`）；`Kp = ωn²·T/K`（`:233`）；`Kd = (2ζωnT − 1)/K`，负值 clamp 到 0（`:234-235`）。
   - **稳定门** `stabilityCheck(K, T, Kp2, Kd2, oscAmp)`（`:316`；实现 `:241-254`）：非有限/退化/Kp≤0 → false（`:244-246`）；反算 `ωn=√(KpK/T)`、`ζ=(1+KdK)/(2√(TKpK))`（`:247-248`）；要求 `ζ ≥ zetaMin(0.6)`、`ωn ≤ wnMax(0.5)`、`oscAmp ≤ oscMax(5.0°)`（`:250-252`；参数默认 `:98-100`）。
   - 通过则 `setBaseline(Kp2,Kd2)`、`baselineEstablished_=true`、`curKp_/curKd_` 接管、`baseK_/baseT_` 记录基准（`:317-321`）；`forceRebase || rebaseline` 时 `rebaseEsc()` + 窗口计数清零（`:322-327`）。
5. ESC 窗口结算中的辨识消费（`settleWindow_` `:333-345`）：`Khat = id.valid ? id.K : 0.0`（`:337-338`），仅当 `Khat>0 && That>0` 且过稳定门才接受 ESC 更新，否则回退基线（`:339-344`）。

### 3.2 辨识试验路径（AUTO_MANEUVER + P14）

- `step()` 的 `AUTO_MANEUVER` 分支（`autopilot_core.hpp:94-127`）：
  - 机动未完成且 `identTrial_ == RUNNING` → `identifier_.update(s.rudderDeg, s.rateDegS, dt)`（`:106-110`；注释明确：自降频任意频率均正确；试验显式发起，不受调参器速度门控影响——注意此处不调 `tuner_.tick`）。
  - 机动完成拍（`sequencer_.done()`，`:96`）：`identTrial_ == RUNNING` → `finalizeIdentTrial()`（`:98`）；随后切 `AUTO_HEADING` 并以当前航向为参考闭环本拍（`:101-104`）。
- `finalizeIdentTrial()`（`:184-187`）：`ok = tuner_.establishBaseline(idSnapshot())`；`identTrial_ = ok ? OK : FAILED`。

---

## 4. 辨识激励的两种来源与 establishBaseline 衔接

### 4.1 来源一：P14 自动 Z 形辨识试验 `startIdentifyTrial`（`autopilot_core.hpp:69-77`）

- 签名 `startIdentifyTrial(double dz = 20.0, double phiz = 20.0, int cycles = 4)`（`:72`）。
- 流程：`identifier_.reset()`（`:73`，重新辨识不被历史污染）→ `sequencer_.setLegs(ManeuverSequencer::zigzag(0.0, dz, phiz, cycles, false))`（`:74`）→ `identTrial_ = RUNNING`（`:75`）→ `mode_ = AUTO_MANEUVER`（`:76`）。
- Z 形腿生成（`maneuver_sequencer.hpp:125-145`）：共 `2N` 段 RUDDER 段，交替 `target=±dz`（舵角），触发条件 `HEADING_REACHED` `threshold=±phiz`（艏向到达即反向）；`returnToStart=false` 时无回航段。
- 状态机 `IdentTrialState {NONE, RUNNING, OK, FAILED}`（`autopilot_core.hpp:34`）；`setMode`（`:45`）与 `startManeuver`（`:64`）会取消进行中的试验。
- 收官衔接：`finalizeIdentTrial()` → `AutoTuner::establishBaseline(snap)`（`auto_tuner.hpp:185-194`）：`!snap.excited || !snap.valid` → false（`:191`）；否则 `applyEstimate_(snap.K, snap.T, 0.0, driftGate=false, forceRebase=true)`（`:192-193`）——语义差异（注释 `:186-188`）：不做漂移检查（试验的明确产出）；振荡测量在机动刚结束时无效，osc 以 0 传入由 ζ/ωn 代数条件把关；恒重锚 ESC。

### 4.2 来源二：常规机动/航向保持的在线辨识

- `headingControlStep` 每拍 `identifier_.update(...)`（`autopilot_core.hpp:163`）：正常操舵（航向改变、抗风流通航）产生的舵角/转向率只要过激励门即被辨识利用。
- 消费路径为 `tuner_.tick` 的周期评估（`auto_tuner.hpp:158-161`，带漂移门）与 ESC 窗口结算的稳定门（`:337-340`），不直接改增益——先过极点配置 + 稳定门 + 漂移门三重闸。
- MANUAL/HOLD 模式不喂辨识器（`step()` `:86-88, :129-132` 无 update 调用）；普通 AUTO_MANEUVER（非试验）也不喂（`:108` 的门条件是 `identTrial_ == RUNNING`）。

---

## 5. 辨识相关测试与仿真场景清单（含运行验证结果）

### 5.1 单元/集成测试

**`control/tests/test_nomoto_identifier.cpp`（9 例，构建运行全部 PASS）**

| 用例 | 行号 | 断言要点 |
|---|---|---|
| `NomotoIdentifier.ConvergesOnNomotoData` | `:16-36` | 真值 K=0.2/T=8，dt=1.0（每次调用即一拍），±25° 非对称 Z 形（8/3 拍保持），放宽门限 rMin=0.1/deltaMin=2.0；400 拍后 excited、valid，`\|K̂−0.2\|≤0.05`、`\|T̂−8\|≤2.0` |
| `NomotoIdentifier.SelfDecimatesAtAnyCallRate` | `:40-59` | R1 自降频：50 Hz 调用（dt=0.02）共 20000 次，激励按拍翻转；同样收敛容差 |
| `NomotoIdentifier.NotExcitedWhenIdle` | `:61-65` | 小幅输入 (δ=1°, r=0.1) × 50 拍 → `excited()==false` |
| `NomotoIdentifier.RejectsOutOfRangeInput` | `:67-76` | P6：δ=50°(>35)、r=50°/s(>30) 被丢弃；θ 保持初值（K、T ≈ 0，1e-9 容差） |
| `NomotoIdentifier.RecoversAfterNaNInput` | `:80-107` | P11 回归：第 200 拍注入单拍 NaN，与干净参考辨识器对比——结果有限、valid，`\|ΔK\|≤0.02`、`\|ΔT\|≤1.0` |
| `NomotoIdentifier.InvalidWhenOutOfRange` | `:109-125` | 正常辨识后 `valid()==true`（K=0.2∈(0,2]、T=8∈[1,60]） |
| `PolePlacement.ReturnsFalseOnDegenerate` | `:127-131` | K=0 或 T=0 → 返回 false |
| `PolePlacement.ClampsNegativeKd` | `:133-140` | ωn=0.05 使 2ζωnT<1 → Kd 钳 0 且返回 true |
| `PolePlacement.ProducesTargetZetaWn` | `:142-151` | 反算闭环 ωn=0.25±1e-6、ζ=0.85±1e-3 |

测试辅助：`nomotoStep()`（`:12-14`）用后向欧拉离散 `r_k=(K·δ·dt+T·r_{k−1})/(dt+T)` 生成与回归式自洽的数据。

**`control/tests/test_autopilot_core.cpp`（辨识相关 2 例，均 PASS）**

- `IdentUpdatesDoNotOverwriteEscGains`（`:215-269+`）：合成 Nomoto 数据 (K=0.2,T=8) 驱动 200 s，基线链收敛到 placement≈(1.588, 8.30)（断言 `:253-254`）；t=200 s 切 T=12，t≈270 s 时 T̂>8.6（漂移 +13%~19% < 25% 阈值）增益钉住旧基线（`:261-265`）；t≈284 s 漂移越阈触发显式重基线（`:267-269`）。
- `IdentTrialIdentifiesAndAppliesGains`（`:401-428`）：P14 回归——未标定冷启动（调度表 1.0/0.5）、ESC 去激活；`startIdentifyTrial(20,20,4)` → RUNNING；K=0.2/T=8 的 Nomoto 船模闭环跑完试验 → `IdentTrialState::OK`、`valid()`、K̂=0.2±0.05、T̂=8±2.0；试验后增益 = 极点配置 (ζ=0.85, ωn=0.2)：Kp≈1.6±0.8、Kd≈8.6±4.0；模式转 AUTO_HEADING。

**`control/tests/test_auto_tuner.cpp`（17 例全 PASS，其中辨识衔接相关）**

- 辅助 `makeSnap(K,T)`（`:12-13`）构造 excited=valid=true 的 `IdSnapshot`。
- `establishBaseline` 驱动的行为测试：`:128`（建基线）、`:146-148`（基线 (1.6, 8.6) 与回退断言）、`:184-185`、`:225-226`、`:253`、`:272-273`、`:288-307`（重基线清 ESC 积分状态）等；`:122` 用默认 `IdSnapshot`（excited=false）验证不触发周期评估。

### 5.2 仿真场景（`control/sim/`）

- `--ident`（`main.cpp:146-160` → `simulator.hpp:159-204` `runIdent`）：AUTO_HEADING + 大幅航向阶跃（±100°，`:176, :186`）等效 Z 形激励，调度表装极点配置增益保持闭环；PASS 判据 `valid && |K̂−K|<0.05 && |T̂−T|<2.0`（`main.cpp:156`）。ctest 名 `sim_ident`（`sim/CMakeLists.txt:13`）。
- `--trial`（`main.cpp:163-182` → `simulator.hpp:552-600` `runIdentTrial`）：冷启动增益 1.0/0.5、调参器强制去激活（`highSpeedDeactivateKn=-1`，`:569-570`）；`startIdentifyTrial(20,20,4)`，收官即停（`:593`）；PASS 判据 `applied && |errK|≤25% && |errT|≤25%`（`main.cpp:179`）。ctest 名 `sim_trial`（`sim/CMakeLists.txt:15`）。
- 另：`--esc` 长期场景（`simulator.hpp:206-295`）注释 `:207-212` 记录了激励门设计教训——旧 ±10°/30 s 阶跃多数样本过不了门导致辨识有偏、基线永不建立的假阳性；现用 Z 形 ±40°/15 s 参考。

### 5.3 本次实际构建运行结果（2026-08-21，control/build 增量构建）

- `test_nomoto_identifier`：**9/9 PASSED**。
- `test_autopilot_core --gtest_filter='*Ident*'`：**2/2 PASSED**。
- `test_auto_tuner`：**17/17 PASSED**。
- `ar_sim --ident`：`K̂=0.2293`（真值 0.20）、`T̂=9.0723`（真值 8.00），valid=1 excited=1 → **PASS**（K 相对偏差 +14.7%，T +13.4%，在 0.05/2.0 绝对容差内）。
- `ar_sim --trial`：`K̂=0.2067 (+3.4%)`、`T̂=8.13 (+1.6%)`，valid=1，用时 175 s；增益 1.00/0.50 → Kp=1.5723、Kd=8.5277，applied=1 → **PASS**。

---

## 6. 存疑与备注

- 【备注】`update()` 返回值的"被接受"仅指过了激励门且完成 RLS 递推；未到期与各类被拒都返回 false，调用方（`autopilot_core.hpp:163`）实际忽略返回值。
- 【备注】入口被拒拍（`:53-55`）连 `elapsed_` 都不累计——NaN 拍不推进降频时钟；分母保护早退（`:84`）与未激励早退（`:70-73`）都会写 `prevR_`。
- 【备注】非对角 P 元素无保底，仅靠 P 更新的对称结构维持；`P_[0][1]`/`P_[1][0]` 理论上保持对称（初值对称、更新式对称），代码未做显式对称化。【存疑】长期浮点累积下对称性漂移未防护，但 9 例测试与仿真未暴露问题。
- 【备注】`lambda=0.98` 作用在 1 s 拍上，等效时间常数约 1/(1−λ)=50 拍 ≈ 50 s（渐近记忆长度），与 `--ident` 场景约 60~120 s 收敛表现一致。
- 【存疑】`beatSec` 到期判定用 `elapsed_ < beatSec`（严格小于）：dtNow=0.02 时第 50 次调用累计 1.00（浮点可能 0.9999999…），极端情况下拍间隔可能漂移一拍；测试 `SelfDecimatesAtAnyCallRate` 通过表明实际无影响。
- 【备注】`gain_schedule.hpp` 与辨识无直接耦合——调度表只提供名义增益（`autopilot_core.hpp:160`），基线建立后增益所有权移交 AutoTuner（`auto_tuner.hpp:155` 注释）。
