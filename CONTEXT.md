# CONTEXT.md — auto_rudder 领域模型

本仓库的统一语言（ubiquitous language）。命名领域概念（issue 标题、重构提案、测试名、文档）时使用这里的术语，不要漂移使用同义词。

## 领域术语

| 术语 | 含义 |
|------|------|
| **自动舵（Autopilot）** | 本系统：船用航向保持与机动控制软件，控制闭环在航向层完成 |
| **航向（heading, ψ）** | 艏向角 (°)；`headingRef` 为目标航向 |
| **舵角（rudder, δ）** | 舵面角 (°)；`δ_cmd` 为舵角指令，系统唯一输出 |
| **转向率（rate, r）** | 艏向角速度 (°/s)，来自 AHRS |
| **海况（sea state）** | 0..3 级，增益调度与扰动强度的输入 |
| **Nomoto 模型（K/T）** | 船舶转向一阶模型 `T·ṙ + r = K·δ`；K=增益，T=时间常数 |
| **增益调度（gain schedule）** | 按船速段×海况查名义 Kp/Kd，段中点线性插值 |
| **极点配置（pole placement）** | 由 K̂/T̂ 反推 Kp/Kd：`Kp=ωn²T/K, Kd=(2ζωnT−1)/K`，公式唯一定义在 `AutoTuner::polePlacement` |
| **ESC（极值搜索）** | 扰动-解调式在线调参：载波注入 → 因果解调 → 梯度下降 |
| **基线（baseline）** | 经极点配置建立、由稳定门背书的工作增益；ESC 从基线出发微调 |
| **重锚（rebase）** | 显式重基线时清零 ESC 积分/解调/窗口样本，旧梯度不得施加到新基线 |
| **回退（revert）** | 窗口结算未过稳定门时增益恢复到基线 |
| **机动（maneuver）** | 预置航段序列（circles/williamson/u-turn/zigzag/cloverleaf/search） |
| **航段（Leg）** | 机动的最小单元：模式（HEADING/RUDDER）+ 目标 + 触发条件 + next |
| **辨识试验（ident trial）** | P14 自动 Z 形试验：机动中辨识，收官自动建基线 |

## R1 深化定型的接口术语（2026-08，架构评审候选 1）

自动优化的编排（节拍/窗口/速度门控/增益所有权/稳定门/回退/重锚）收进 `AutoTuner`；`AutopilotCore` 回归"模式状态机 + 机动编排 + 每拍转发"。

| 术语 | 含义 |
|------|------|
| **`tick()`** | `AutoTuner` 的每拍单入口：输入观测与调度名义增益，返回本拍应施加的增益 |
| **`TunerInput`** | `tick()` 的每拍输入包：目标/实际航向、舵角、船速、dt、调度增益、辨识快照 |
| **`GainCommand`** | `tick()` 的返回：本拍增益 {Kp, Kd}（已含 ESC 载波），直接转发给 PD |
| **`IdSnapshot`** | 辨识快照：辨识器当前估计的只读投影 {excited, valid, K, T}；辨识器保持独立模块，core 每拍喂数据并组装快照 |
| **`establishBaseline()`** | 一次性评估并建基线（辨识试验收官/人工标定注入）；与 tick 共享内部评估路径，但不做漂移检查、振荡条件以 0 把关 |
| **`suspend()`** | 唯一的模式生命周期钩子：切出自动模式时清测量状态、保留基线 |
| **自降频（self-decimation）** | `NomotoIdentifier::update(δ, r, dtNow)` 可任意频率调用，内部满 `beatSec`（1 s）才做 RLS 拍，Δt 自测 |
| **增益所有权（gain ownership）** | 基线建立后增益归 AutoTuner 所有（ESC 微调+载波），不再每拍被调度表覆写；只读投影经 `currentGains()` 观测 |

## 实船调试接口术语（2026-09）

`TunerDebugApi` 把 `AutoTuner` 的公有动作与私有机制摊平为可单独调用的调试方法（经 `friend` 暴露，不改任何在线行为），配套命令行与脚本工具在船上/岸上驱动。安全红线：仅用于调试/标定，不得接入正常航行控制回路；`force*` 系列旁路稳定门。

| 术语 | 含义 |
|------|------|
| **实船调试接口（TunerDebugApi）** | `AutoTuner` 的调试门面（facade）：每个方法恰对应一个功能，互不影响、可任意顺序做单环节激励-观测 |
| **ar_tuner_debug** | 调试接口的逐行命令行封装：每行一条命令恰调一个功能，输出 `key=value`（成功 `OK`、失败 `ERR`），可经 ssh/串口在船上使用 |
| **窗末裁决（settleWindow）** | 窗口结束时的裁决步：窗末梯度结算 → 稳定门 → 接受/回退，即 `tick()` 每窗自动执行的 `settleWindow_` |
| **验收脚本（tuner_debug_accept.py）** | 驱动 `ar_tuner_debug` 的端到端验收：9 个功能组 49 项断言（数值经手工核算），全过退出码 0，可接 CI |
| **监控脚本（tuner_debug_monitor.py）** | 周期下发 `status` 解析快照，实时绘制增益/梯度/J/振荡/载波相位曲线；`--demo` 内置激励，无实船即可演示 |

## 架构决策的落点

- ADR 目录：`docs/adr/`（尚无条目；有分量的否决/权衡出现时按 `docs/agents/domain.md` 的规则惰性创建）
- 工单与试验记录：`.scratch/`（本地 markdown 工单系统，见 `docs/agents/issue-tracker.md`）
