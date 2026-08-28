## 在线辨识与自动调参

> 素材来源：`control/include/nomoto_identifier.hpp`、`control/include/auto_tuner.hpp`、`control/include/param_store.hpp`，及 `docs/auto_tuning/辨识流程原理推导与工作流程.md`、`docs/auto_tuning/自动调参流程图文详解.md`。
> 本节覆盖三层调参结构中的层2（辨识 + 极点配置）与层3（ESC 极值搜索），以及参数持久化；层1 增益调度表（`gain_schedule.hpp`）在调度章节展开。

### 总体结构：三层递进式调参

| 层 | 机制 | 源码 | 作用 |
|---|---|---|---|
| 层1 | 增益调度表 | `gain_schedule.hpp` | 按船速段×海况给名义 `Kp/Kd`，兜底 |
| 层2 | RLS 在线辨识 + 极点配置 | `nomoto_identifier.hpp` + `auto_tuner.hpp` | 辨识 Nomoto 模型 `K/T`，解析算出增益，建立基线 |
| 层3 | ESC 极值搜索 | `auto_tuner.hpp` | 在基线附近注入微小载波扰动，实测性能梯度，连续微调 |

设计直觉：**调度表保证"不会太差"；辨识+极点配置把增益一次性拉到理论最优附近（基线）；ESC 在基线周围做无模型精细寻优**，吃掉真实船舶与 Nomoto 一阶模型之间的模型误差。

### Nomoto 一阶模型与 RLS 在线辨识

#### 模型与回归化

艏摇动力学用 Nomoto 一阶模型近似（舵角 δ 输入，转向率 r=ψ̇ 输出）：

```
T·ṙ + r = K·δ        →        r(s)/δ(s) = K / (T·s + 1)
```

- `K`：旋回性指数，单位舵角产生的稳态转向率，物理合理范围 `(0, 2]`（代码 `valid()` 强制）
- `T`：追随性指数，转向惯性时间常数 (s)，合理范围 `[1, 60]`

对 ṙ 取后向差分 `dr_k = (r_k − r_{k−1})/Δt` 离散化，模型变为标准线性回归（`nomoto_identifier.hpp:3-4` 注释与 `update()` 逐行对应）：

```
r_k = K·δ_k − T·dr_k
y_k = r_k,  φ_k = [δ_k, −dr_k]ᵀ,  θ = [K, T]ᵀ
```

**可辨识性关键点**：稳态直航时 `dr≈0`，回归量第二维消失，只能定出 K——**T 的信息只藏在转向反向（r 过零）附近的高 dr 瞬态里**。这是激励门控保留高 dr 样本、以及 Z 形辨识试验反复换舵的设计依据。

#### RLS 递推（带遗忘因子）

目标函数 `J_LS = Σ λ^(k−i)·(y_i − φ_iᵀθ)²`，遗忘因子 `lambda = 0.98`（约 34 拍后旧样本权重衰减到 50%）。代码实现的标准 RLS 三件套：

```
增益  g_k = P·φ / (λ + φᵀPφ)
残差  e_k = y_k − φᵀθ
更新  θ ← θ + g_k·e_k
      P ← (1/λ)·(P − g_k·φᵀ·P)，对角元保底 pFloor = 1e-3 防协方差塌陷
```

工程化处理（全部在 `NomotoIdentifier::update()` 内）：

| 机制 | 实现 | 参数 |
|---|---|---|
| 归一化 | `φ = [δ/50, −dr/1]`，估计反归一化 `K=θ0/50, T=θ1/1`，消除量级差导致的 P 矩阵病态 | `dScale=50.0`、`drScale=1.0` |
| 激励门控 | 仅当 `\|δ\|>3° 且 (\|r\|>0.5°/s 或 \|dr\|>0.2°/s²)` 才递推；直航时 φ≈0，若更新则 P 每拍放大 1/λ 导致协方差爆炸 | `deltaMin=3.0`、`rMin=0.5`、`drMin=0.2`（保留过零瞬态以辨识 T） |
| 坏数据防御 | 非有限值、`\|δ\|>35°`、`\|r\|>30°/s`、`dt≤0` 一律丢弃；**被拒的拍不写 `prevR_`**，否则毛刺成为下一拍差分锚点、永久毒化 θ/P（P11 整改） | 硬编码阈值 |
| 物理合理性 | `K∈(0,2] 且 T∈[1,60]s` 才 `valid()=true`，调参器保守拒绝越界估计（P6 整改） | `valid()` |

#### 自降频（self-decimation）

`update(δ, r, dtNow)` 可被任意频率调用（主循环 50 Hz），内部累计满 `beatSec = 1.0 s` 才做一次 RLS 拍；差分 Δt 取**两次 RLS 拍之间的真实间隔自测**（`beatDt = elapsed_`），与调用频率无关。收益：减轻 ECU 负担、保证差分信噪比。旧设计把"50 Hz × 1 s"降频约定放在调用方，调用频率一变差分即静默出错——该语义陷阱已随 `Params.dt` 一并删除（见头文件注释）。

#### IdSnapshot：辨识结果的只读投影

辨识器是独立模块，`AutoTuner` 不持有辨识器；由 `AutopilotCore` 每拍喂数据并组装快照传入：

```cpp
struct IdSnapshot {
    bool   excited;  // 最近一次 RLS 拍满足激励条件
    bool   valid;    // 估计物理合理（K∈(0,2]、T∈[1,60]）
    double K, T;     // 当前估计
};
```

辨识结果经两条受控路径进入调参器（共享同一评估实现 `AutoTuner::applyEstimate_()`）：

1. **tick 周期评估**：每拍检查快照，K̂/T̂ 相对建基线时的估计漂移 ≤25% 交给 ESC 自治；漂移超阈值才显式重基线并重锚 ESC（P12）。
2. **辨识试验收官**（P14）：`AutopilotCore::startIdentifyTrial(dz=20, phiz=20, cycles=4)` 自动执行 Z 形机动，期间辨识器 `reset()` 后密集喂数；机动结束调 `establishBaseline()` 一次性评估建基线——不做漂移检查，振荡测量在机动刚结束时无效，osc 以 0 传入，由 ζ/ωn 代数条件把关。

### 极点配置建基线

辨识得到 K̂/T̂ 后，层2 用极点配置解析反解 PD 增益。闭环特征方程（Nomoto + 积分环节 + PD）：

```
T·s² + (1 + K·Kd)·s + K·Kp = 0   ⇔   s² + 2ζωn·s + ωn² = 0
```

逐项对照系数反解，得公式（**全仓库唯一定义在 `AutoTuner::polePlacement()`**，`auto_tuner.hpp:228`；仿真装表等场景也应调此处）：

```
Kp = ωn²·T / K
Kd = (2ζωn·T − 1) / K
```

- 默认目标 `zeta = 0.85`（小超调、快稳定）、`wn = 0.20 rad/s`（`AutoTuner::Params`）。
- 退化保护：`K/T` 非有限或 |K|、|T| < 1e-6 返回 false；**Kd clamp 到 ≥0**——`2ζωnT < 1`（小 T 快船）时公式给出负值，负 Kd 与阻尼项符号相反，会反向放大振荡。

稳定门 `stabilityCheck()` 用同一组关系的**正向形式**反解任意 `(Kp, Kd)` 对应的闭环极点参数，四条件全过才接受新增益：

```
ωn = √(Kp·K/T),   ζ = (1 + Kd·K) / (2·√(T·Kp·K))
要求：Kp > 0，ζ ≥ zetaMin(0.6)，ωn ≤ wnMax(0.5 rad/s)，振荡幅值 ≤ oscMax(5°)
```

### ESC 极值搜索在线微调

ESC 是无模型在线优化：不需要 J 对增益的解析梯度，通过"载波注入 → 因果解调 → 梯度下降"估出梯度方向，沿负梯度走向谷底。

#### 性能指标 J（ITAE 型）

`computeJ()` 在评估窗口上累计：

```
J = ∫ ( t·|e| + wU·|δ| + wDu·|dδ| ) dt
```

- `t·|e|`：ITAE，误差权重随时间线性增长，重罚收敛慢；
- `wU = 0.01`：舵机能耗惩罚；
- `wDu = 0.005`：舵角变化惩罚（保护舵机、抑制抖振）。

#### 信号链：HPF → 解调 → LPF → 积分器

经典摄动 ESC 三环在代码中一个不少，且**每个滤波器用各自环节的真实采样周期**（P11 整改的核心教训）：

1. **扰动注入**（`escDither()`，每控制拍）：`Kp *= 1 + a·sin(ωt)`、`Kd *= 1 + a·cos(ωt)`，`a = escAmp = 5%`、`ω = escFreq = 0.05 rad/s`。Kp 用 sin、Kd 用 cos（正交，由 `sin·cos` 均值为 0 保证双参数解调互不串扰）。载波只存在于返回给 PD 的增益里，**不持久化**到 `curKp_`。
2. **因果解调**（`accumulate()`，每控制拍）：计算本拍成本增量 `c_k`（与 `computeJ` 被积函数一致），经一阶 HPF 去直流/慢漂（后向欧拉，`αH = τH/(τH+dt)`，`escHPFtau = 5 s`），再与同相载波相乘累加：`demodKpSum += hpfY·sin(phase)`、`demodKdSum += hpfY·cos(phase)`。
3. **窗口结算**（`escStep()`，每 `windowSec = 60 s` 一次）：归一化梯度 `ĝ = 2·Σ(y·载波)/(a·N)`（系数 2 与 a 恰好归一掉推导中的 `a/2`）；LPF 平滑梯度（系数按窗口周期折算 `αL = Tw/(τL+Tw)`，`escLPFtau = 10 s`）；积分器梯度下降 `θ ← θ − escStep·ĝ`（`escStep = 0.02`）；**单次更新 ≤ 旧值 10%**（`maxUpdateFrac = 0.10`），并有绝对下限 `absMinUpdate = 1e-3`（防止旧值为 0 时比例上限锁死更新）；增益不为负；清空本窗样本，下一窗从零开始（非重叠窗）。

时间尺度三分离是 ESC 成立的前提：**闭环动态（秒级）≪ 载波 1/ω（百秒级）≪ 积分器适应（多窗口）**。本工程 ω=0.05 rad/s（周期约 126 s）、窗口 60 s。

> P11 整改教训（头文件注释）：旧实现扰动实际没注入（窗口内增益恒定，"梯度"没有因果来源），且按 dt=0.02 s 设计的 HPF/LPF 系数被用在 60 s 才来一个的样本上，梯度被衰减 2~3 个数量级——调参结果与初值逐位相同。现结构每拍注入、每拍解调、窗末结算。

### 编排逻辑：全部收在 `AutoTuner::tick()`

R1 深化后，节拍计数、ESC 窗口、速度门控、增益所有权、漂移重基线、振荡门控全部内收进 `AutoTuner`，对外只剩三个动作：`tick()` / `establishBaseline()` / `suspend()`（P11/P12/P14 的 bug 全部诞生在编排散于调用方的旧时代）。

#### tick() 每拍执行顺序

输入 `TunerInput{headingRefDeg, headingDeg, rudderDeg, speedKn, dt, schedKp, schedKd, id}`，输出 `GainCommand{Kp, Kd}`（已含 ESC 载波，直接转发 PD）。内部依次：

1. **速度门控**：`enabled_ = (speedKn ≤ highSpeedDeactivateKn)`，默认 15 kn——高速时调参整体去激活。
2. **基线未建立**：当前增益跟随调度名义值（否则 ESC 增益会每拍被调度表覆盖，P11）。
3. **周期辨识评估**：`enabled_ && id.excited && id.valid` 时调 `applyEstimate_(K, T, osc幅值, driftGate=true)`。
4. **性能样本累积 + 振荡门控**：航向参考变化则 `refAgeSec_` 清零；**参考稳定满 30 s（`kOscRefSettleSec`）才喂入振荡检测**——参考机动期间的误差摆动是指令响应而非闭环振荡，混入会误判（P12）。成本累积与 ESC 解调不受该门控影响。
5. **ESC 窗口结算**：窗口拍数按 `windowSec/dt` 换算（默认 60 s / 0.02 s = 3000 拍，`winCounter_` 自维护），满窗调 `settleWindow_()`。
6. **ESC 扰动注入**：基线已建立则对输出增益叠加载波（不持久化），返回本拍增益。

#### 窗口结算、稳定门与回退

`settleWindow_()`：仅 `enabled_ && baselineEstablished_` 时执行。先 `escStep()` 算出新增益，再用快照的 K̂/T̂（无效则置 0）过 `stabilityCheck()`——**通过才接受 ESC 更新（持久化到 `curKp_/curKd_`）；不通过（含快照无效）则 `revert()` 回退到基线**。基线永远是"已验证安全"的锚点。

#### 漂移门与重锚（rebase）

`applyEstimate_()` 内置 25% 漂移阈值（`kReBaselineTol = 0.25`，分母带下限 `kReBaselineKFloor=0.05` / `kReBaselineTFloor=1.0`，防早期小估计把相对偏差放大失真）：

- 新 K̂/T̂ 相对**建基线时**的 `baseK_/baseT_` 漂移 ≤25% → 不动基线，ESC 自治（新旧极点配置差异在 ESC 自身修正能力内）；
- 漂移 >25% → 船况/航速真正改变，显式重基线（极点配置 → 稳定门 → 新基线），并调 `rebaseEsc()` **重锚**：ESC 积分器/解调累积/HPF 状态/窗口样本全部清零，窗口节拍重对齐——旧梯度与旧窗口成本是围绕旧增益、旧模型测得的，不得施加到新基线。载波相位 `escPhase_` 不清，避免重锚拍载波突跳。

#### 生命周期钩子 suspend()

切出自动模式（MANUAL/HOLD）时调用：停用并清掉全部测量状态（窗口样本、解调累积、HPF、梯度），但**保留基线增益与基线 K/T**——避免手动操舵的旧数据混进下一窗口，回到自动模式后从基线无缝续上。

### 增益所有权规则

- **基线建立前**：增益归增益调度表，`tick()` 每拍直通 `schedKp/schedKd`。
- **基线建立后**：增益归 `AutoTuner` 所有（ESC 微调 + 载波注入），**不再每拍被调度表覆写**；调度名义值仅在速度门控等场景作为参考输入。
- 外部只读观测经 `currentGains(Kp, Kd)`（不含载波）；基线状态经 `baselineEstablished()` 观测——替代了仿真中"增益偏离 1e-9 反推"的迂回断言。
- 每次重基线必伴随 ESC 重锚：增益所有权转移的边界清晰，陈旧梯度不跨基线。

### 参数持久化：param_store

`param_store.hpp` 负责控制参数的落盘与校验（纯算法层，实际 EEPROM/Flash 读写由 BSP 层经 `ReadFn/WriteFn` 注入，时钟经 `ClockFn` 注入）。

`ControlParams` 与 EEPROM 区块 A~D 对应：

| 区块 | 内容 | 字段 |
|---|---|---|
| A | 舵角标定 | `rudderZero/rudderLeft/rudderRight`、`ddotMax`（舵速限幅，1~30 deg/s） |
| B | 模型参数 | `K`、`T` |
| C | PD 增益 | `Kp`（>0）、`Kd`（≥0） |
| D | 增益调度表 | `schedTable`（`GainPoint`：speedMin/speedMax/seaState/Kp/Kd） |
| 元信息 | — | `version`、`timestamp`、`crc` |

关键机制（P8 整改后）：

- **序列化**：逐字段显式小端写入（`writeLEf64/writeLEu32`），消除结构体填充字节依赖；格式 `[8×double][version][timestamp][schedCount][sched 表]`。
- **CRC32**（IEEE 0xEDB88320）：`updateCrc()` 对 `serialize()` 全部字节计算（不含 `crc` 字段自身）；落盘格式为 `[payload][crc u32]` 追加在末尾。
- **save()**：注入时钟填 `timestamp` → `version += 1` → 算 CRC → 写。
- **load()**：读入缓冲 → 反序列化 payload → 读末尾 CRC → `verifyCrc()` 校验；之后应再过 `validateRange()` 物理范围检查（`K∈(0,2]`、`T∈[1,60]`、`ddotMax∈[1,30]` 等，与辨识/调参侧的合理性边界一致）。

### 备注与待确认

- `AutoTuner::Params` 注释称极点配置目标"按船速分段（见 §3.10.4）"，但结构体只有单一 `zeta/wn` 一对，代码中未见分段实现（待确认：是文档先行还是分段已移除）。
- 代码中 `wDu` 乘的是每拍舵角增量 `|Δδ|`（未除以 dt），与文档 §4.1 符号 `|δ̇|` 存在量纲差异；两项机制意图一致（惩罚舵角变动），表述以代码为准（待确认）。
- 载波周期约 126 s（ω=0.05 rad/s）大于评估窗口 60 s，即单个窗口不足一个完整载波周期；文档 §4.2 将此表述为时间尺度权衡的有意取值，梯度估计靠 LPF 跨窗平滑（备注，非矛盾）。
- `ParamStore::load()` 固定读 4096 字节缓冲（上限），要求注入的 `ReadFn` 能返回满缓冲数据（备注：BSP 适配约束）。
- 辨识的已知局限（来自调研文档 §8）：辨识质量强依赖激励设计，慢船 Z 形翻转周期建议 ≥2T；λ=0.98 跟踪带宽有限，航速突变建议显式触发重基线；高海况下量测未预滤波，可评估对 r 做一阶低通后再喂 RLS。
