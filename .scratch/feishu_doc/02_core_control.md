## 核心控制算法

> 素材依据：`control/include/autopilot_core.hpp`、`pd_controller.hpp`、`gain_schedule.hpp`、`angle_utils.hpp`，以及 `control/控制算法代码说明文档.md`（下称"代码说明文档"）对应章节。控制闭环在航向层完成，系统唯一输出为舵角指令 `δ_cmd`。

### 每拍数据流：从传感器到舵角指令

`AutopilotCore::step(const SensorInput& s, double dt)` 是主循环唯一入口，设计调用频率 50 Hz（`dt = 0.02 s`），输出目标舵角（deg），由外部驱动层转换为 PWM/CAN 指令。每拍输入为 `SensorInput`：

| 字段 | 含义 | 来源 |
|------|------|------|
| `headingDeg` | 航向 ψ (°) | AHRS |
| `rateDegS` | 转向率 r (°/s) | AHRS |
| `rudderDeg` | 舵角反馈 δ (°) | 舵机反馈 |
| `speedKn` | 船速 (kn) | — |
| `seaState` | 海况 0..3 | — |

`AUTO_HEADING` 模式下每拍执行 `headingControlStep()`，数据流严格按以下顺序（`autopilot_core.hpp:151-173`）：

1. **首拍初始化**：切入 `AUTO_HEADING` 的第一拍，用实际舵角 `s.rudderDeg` 调用 `pd_.initBaseline()` 初始化舵速限幅基准，避免舵令从 0° 爬升产生跳变（真实舵面并不在 0°，P11 整改）。
2. **增益调度查询**：`schedule_.query(s.speedKn, s.seaState, Kp, Kd)` 得到本拍名义增益（详见"增益调度表"一节）。
3. **喂辨识器**：`identifier_.update(s.rudderDeg, s.rateDegS, dt)` 每拍无条件喂入；辨识器自降频，内部累计满 `beatSec`（1 s）才做一次 RLS 拍，调用频率任意。
4. **调参器拍**：`tuner_.tick({...})` 传入 `TunerInput`（目标/实际航向、舵角、船速、dt、调度名义增益、辨识快照 `IdSnapshot`），返回本拍应施加的 `GainCommand`（详见"增益转发"一节）。
5. **下发增益**：`pd_.setDt(dt)` + `pd_.setGains(g.Kp, g.Kd)` —— 轻量接口，不清控制器内部状态（P1 整改，替代会 `reset()` 的 `setParams`）。
6. **PD 闭环**：`pd_.update(headingRef_, s.headingDeg)` 输出本拍舵角指令。

每拍末尾 `lastRudderCmd_` 记录本拍输出，供 `HOLD` 模式冻结使用。

### PD 控制器（`pd_controller.hpp`）

#### 控制律

```text
δ_k = Kp·e_k + Kd·ė_k
```

其中航向误差先做角度归一化（wrap 处理）：

```cpp
const double e = angleDiffDeg(headingRef, headingAct);  // 落在 (-180, 180]
```

`angleDiffDeg(ref, act)` 计算最小带符号差 `ref - act` 并归一化到 `(-180, 180]`（`angle_utils.hpp`）。不做归一化时，359° 与 1° 的误差会被误算为 358° 而非 -2°，跨 0°/360° 边界时舵令会反向打满。

#### 微分来源与滤波

- 微分**不是**直接取 AHRS 的转向率 r，而是对航向误差 e 做差分：`derivRaw = (e - prevErr) / dt`；首拍（`first_`）`derivRaw = 0`，避免启动微分冲击。
- 差分结果经一阶低通滤波：`alpha = 1 / (1 + tau/dt)`，`deriv = alpha·derivRaw + (1-alpha)·prevDeriv`，默认 `tau = 0.2 s`。

#### 死区、限幅与量化

`update()` 内的处理链按固定顺序执行：

| 环节 | 实现 | 默认值 |
|------|------|--------|
| 误差死区 | `\|e\| < deadzone` 时目标为 0，但经 `saturate(0.0)` 舵速限幅平滑回中，而非直接跳 0（P11 整改：直接跳 0 会绕过舵速限幅，如 35°→0° 瞬时跳变） | 0.3° |
| PD 控制律 | `out = Kp·e + Kd·deriv` | — |
| 舵角限幅 | `±dMax` | ±35° |
| 舵速限幅 | 每拍增量 ≤ `ddMax·dt`（50 Hz 下即 0.2°/拍），基准为上一拍输出 `prevOut_` | 10°/s |
| 输出量化 | `round(v/quant)·quant` | 0.1° |

`PdController::Params` 全量参数：

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `Kp` | 1.0 | — | 比例增益 |
| `Kd` | 0.5 | — | 微分增益 |
| `dMax` | 35.0 | deg | 舵角限幅 |
| `ddMax` | 10.0 | deg/s | 舵速限幅 |
| `deadzone` | 0.3 | deg | 误差死区 |
| `tau` | 0.2 | s | 微分滤波时间常数 |
| `quant` | 0.1 | deg | 输出量化步长 |
| `dt` | 0.02 | s | 控制周期（50 Hz） |

#### 状态管理与接口语义

- `setParams()`：全量设置并 `reset()` 清空内部状态（`prevErr_/prevDeriv_/prevOut_/first_`）；对非法值有防护——`dt ≤ 0` 钳到 `kMinDt = 1e-3`（防微分除零产生 Inf），`quant ≤ 0` 钳到 `kMinQuant = 1e-3`（防量化除零产生 NaN）。
- `setGains(Kp, Kd)` / `setDt(dt)`：轻量设置，**不清状态**，供增益调度/调参器每拍调用；`setDt` 拒绝非法 dt。
- `initBaseline(rudderAct)`：用实际舵角初始化舵速限幅基准（上电/切模式时调用）。
- `saturate(v)`：公开的限幅工具（舵角限幅 + 舵速限幅 + 量化），供机动序列 RUDDER 段复用；非 const，每次调用推进 `prevOut_` 基准，保证连续调用时限幅基准连续。

### 模式状态机（`AutopilotCore`）

#### 四种模式

`enum class Mode { MANUAL, AUTO_HEADING, AUTO_MANEUVER, HOLD }`，默认上电为 `MANUAL`。

| 模式 | 每拍行为 |
|------|----------|
| `MANUAL` | 透传舵角反馈：`rudderCmd = s.rudderDeg` |
| `AUTO_HEADING` | 航向保持闭环：`headingControlStep()`（PD + 增益调度 + 自动优化） |
| `AUTO_MANEUVER` | 机动航段序列执行：HEADING 段走 PD 闭环，RUDDER 段经 `pd_.saturate(cmd.rudder)` 施加幅值/舵速限幅后直接输出 |
| `HOLD` | 冻结上一拍舵角指令 `holdRudder_`（而非输出 0，P9 整改） |

机动 `AUTO_MANEUVER` 内的两个细节：

- HEADING 段补齐了增益调度查询（`schedule_.query` → `pd_.setGains`，P13 整改：否则全程停留默认增益，慢船直航段目标航向丢失）；机动中不调用 `tuner_.tick()`，增益直接取自调度表（代码注释称"辨识/ESC 在机动中仍保持挂起"）。
- 机动完成拍不再输出 0（那是绕过舵速限幅的瞬时跳变，P11 整改），而是立即转入 `AUTO_HEADING`、以当前航向为 `headingRef_`，当拍即闭环运行。

#### 模式切换的清理逻辑（`setMode()`，P9 整改）

切入新模式前统一清理上一模式残留状态，避免跨模式污染：

- 一律 `pd_.reset()`，并将进行中的辨识试验置为 `IdentTrialState::NONE`（切模式取消辨识试验）。
- 切出到 `MANUAL` 或 `HOLD` 时调用 `tuner_.suspend()`（见下）。
- 切入 `HOLD`：冻结当前舵角作为保持指令 `holdRudder_`。
- 切入 `AUTO_HEADING`：置 `pendingBaseline_ = true`，首拍以实际舵角初始化舵速限幅基准。

另外，`startManeuver(legs)` 与 `startIdentifyTrial(dz, phiz, cycles)` 会直接将模式置为 `AUTO_MANEUVER`（普通机动会抢占进行中的辨识试验）。

#### `suspend()` 语义

`suspend()` 是 `AutoTuner` 唯一的模式生命周期钩子，仅在切出自动模式（`MANUAL`/`HOLD`）时由 core 调用。其实现（`auto_tuner.hpp:199-203`）：

```cpp
void suspend() {
    enabled_ = false;
    rebaseEsc();      // 清测量状态（窗口/解调/HPF/梯度），保留 baselineKp_/baselineKd_
    winCounter_ = 0;
}
```

一句话：**清测量状态、保留基线**——ESC 的积分/解调/窗口样本全部清零（旧梯度不得施加到之后的基线上），但已建立的基线增益 `baselineKp_/baselineKd_` 保留，切回自动模式后从原基线继续。

### 增益调度表（`gain_schedule.hpp`）

#### 表结构

```cpp
struct GainPoint {
    double speedMin, speedMax;  // 船速段 (kn)
    int    seaState;            // 海况 0..3
    double Kp, Kd;              // 该段名义增益
};
```

内置默认表（§3.10.4，代码按"速度段 × 海况"逐行共 14 行；海况 0 与 1、2 与 3 数值相同，代码说明文档合并为 7 行呈现）：

| 船速区间 | 海况 | Kp | Kd |
|----------|------|-----|-----|
| < 4 kn | 0~1 | 2.0 | 0.8 |
| 4~10 kn | 0~1 | 1.5 | 0.6 |
| 4~10 kn | 2~3 | 1.0 | 0.4 |
| 10~18 kn | 0~1 | 1.2 | 0.5 |
| 10~18 kn | 2~3 | 0.9 | 0.4 |
| 18~25 kn | 0~1 | 1.0 | 0.4 |
| 18~25 kn | 2~3 | 0.7 | 0.3 |

#### 查询算法（`query(speedKn, seaState, Kp&, Kd&)`）

- **海况维：精确匹配，不插值。** 先将 `seaState` 钳位到 0..3，再从表中筛出该海况的全部速度段（最多 8 段）；查不到任何段时兜底返回 `Kp=1.0, Kd=0.5`。
- **速度维：段中点锚定线性插值**（P5 整改，消除段切换处的增益跳变）。名义增益锚在每段中点 `m_i = (speedMin+speedMax)/2`——默认表四个中点为 2 / 7 / 14 / 21.5 kn：
  - `speedKn ≤ m_0` → 取首段名义值；
  - `speedKn ≥ m_{n-1}` → 取末段名义值；
  - `speedKn ∈ [m_i, m_{i+1}]` → 按 `f = (v − m_i)/(m_{i+1} − m_i)` 对 Kp、Kd 分别线性插值。

整表可经 `setTable()` 替换（仿真/标定场景注入自定义调度表，经 `AutopilotCore::scheduleMut()` 暴露）。

### 每拍增益转发：`GainCommand` → PD

自动优化的编排（节拍/窗口/速度门控/增益所有权/稳定门/回退/重锚）已收进 `AutoTuner::tick()`（R1 深化），core 只负责"传输入、转输出"。每拍接口结构体（`auto_tuner.hpp`）：

```cpp
struct TunerInput {
    double headingRefDeg, headingDeg, rudderDeg, speedKn, dt;
    double schedKp, schedKd;   // 增益调度名义值——基线未建立时直通
    IdSnapshot id;             // 辨识快照 {excited, valid, K, T}
};
struct GainCommand { double Kp, Kd; };  // 本拍应施加的增益（已含 ESC 扰动载波）
```

`tick()` 内与本节相关的两个关键点：

1. **速度门控**：`enabled_ = (speedKn ≤ highSpeedDeactivateKn)`，高速去激活（代码说明文档给出阈值 > 15 kn）。
2. **基线未建立时跟随调度**：`if (!baselineEstablished_) { curKp_ = in.schedKp; curKd_ = in.schedKd; }`——即无基线时 `GainCommand` 直通调度表名义值；基线建立后增益归 `AutoTuner` 所有（ESC 微调 + 载波），不再每拍被调度表覆写（增益所有权）。

core 侧的转发只有两行（`autopilot_core.hpp:170-172`）：

```cpp
pd_.setDt(dt);
pd_.setGains(g.Kp, g.Kd);        // GainCommand 直接下发给 PD
return pd_.update(headingRef_, s.headingDeg);
```

`setGains` 不清 PD 内部状态，因此每拍更新增益不会扰动微分滤波与舵速限幅基准。当前增益（去载波投影）可经 `tuner().currentGains(Kp&, Kd&)` 只读观测。

### 代码与文档不一致 / 待确认

- **死区行为**：代码说明文档 §3.2 特性表写死区"输出 0"，实际代码是 `return saturate(0.0)`——目标为 0 但仍经舵速限幅平滑回中（P11 整改后的行为），文档表述未跟上代码。
- **HOLD 冻结值**：`setMode()` 中 `holdRudder_ = pd_.params().dt > 0 ? lastRudderCmd_ : 0.0;`——由于 `setParams`/`setDt` 已保证 `dt` 恒为正，该条件实际恒真，`: 0.0` 分支不可达（待确认是否为历史防护残留）。
- **ESC 窗口 60 s**：代码说明文档 §4.3 给出"ESC 窗口结算 60 s"，该默认值定义在 `AutoTuner::Params.windowSec`（本节指定阅读的四个头文件之外，数值待确认）。
- **机动中"挂起"的措辞**：`autopilot_core.hpp` 注释称机动中"辨识/ESC 仍保持挂起"，实际机制是机动分支不调用 `tuner_.tick()`（tuner 不更新），而非显式调用 `suspend()`；措辞与机制略有出入。
