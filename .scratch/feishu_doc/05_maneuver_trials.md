## 机动序列与辨识试验

> 素材依据：`control/include/maneuver_sequencer.hpp`、`control/sim/scenarios.hpp`、`control/include/autopilot_core.hpp`、`control/include/auto_tuner.hpp`、`control/sim/simulator.hpp`、`control/sim/main.cpp`、`docs/auto_tuning/自动调参测试用例.md`，以及《自动舵定位与控制设计文档》§4、《控制算法开发文档》§7。

### 机动概念：航段序列 + 触发驱动的状态机

机动（maneuver）在本系统中统一抽象为**预置航段序列**：每个机动 = 一串航段（Leg），由机动编排器 `ManeuverSequencer` 按触发条件逐段推进。设计前提是**仅航向（AHRS/罗经）、无定位**的降级工况——机动必须能由"航向角触发 + 航向变化率"实现，不依赖绝对位置（设计文档 §4.1）。

可行性结论（设计 §4.1 / 开发文档 §7.4.1）：

| 类别 | 机动 | 说明 |
|---|---|---|
| 闭环可行（4 类） | Circles / Williamson / U-Turn / Zigzag | 纯航向触发即可正确完成 |
| 开环可行（2 类） | Cloverleaf / Search | 无位置闭合，风流水致漂移不可消除，形态正确但图案不闭合 |
| 不可行 | Orbit 绕点 | 需位置闭环，未实现 |

### 预置机动类型与各自用途

六种预置机动均为 `ManeuverSequencer` 的静态工厂函数，返回 `std::vector<Leg>`：

| 机动 | 工厂函数（缺省参数） | 段数 | 模式组合 | 用途 |
|---|---|---|---|---|
| Circles 绕圈 | `circles(dCmd=15, totalDeg=360)` | 1 | RUDDER | 回转试验/绕圈；恒舵角等效恒转向率，累计转满 `totalDeg` 完成 |
| Williamson 威廉逊掉头 | `williamson(psi0, dMax=25)` | 3 | RUDDER×2 + HEADING | **人落水紧急返点机动**，本就为"仅罗经"场景设计 |
| U-Turn U 型掉头 | `uTurn(psi0, right=true, dMax=25)` | 2 | RUDDER + HEADING | 单次 180° 掉头，方向由 `right` 指定（舵角符号强制） |
| Zigzag Z 字形 | `zigzag(psi0, dz=10, phiz=10, N=5, returnToStart=false)` | 2N（+1 保持段） | RUDDER（+HEADING） | 操纵性 Z 形试验；**兼作辨识激励发生器**（见下文 P14） |
| Cloverleaf 三叶草 | `cloverleaf(psi0, tLeg=30, dMax=20)` | 8 | RUDDER 转向 + HEADING 直航 | 4×(转向 270° + 直航 `tLeg` 秒)，开环 |
| Search 搜索 | `search(psi0, t0=30, dt=10, cycles=6, dMax=20)` | 2×cycles | HEADING 直航 + RUDDER 转向 | 扩张方搜，边长逐次递增（`t0 + c·dt`），每边后右转 90°，开环 |

各机动的关键实现要点（代码注释与开发文档 §7.4）：

- **Williamson**：`+dMax` 至累计航向变化 `+60°` → `-dMax` 至累计 `-180°` → HEADING 保持 `ψ0+180°`（NONE 触发长期保持）。缺省 `dMax=25°` 而非满舵 35°，留出段间超调的修正裕量；反舵段阈值取负（−180°），与本段舵向一致。
- **Zigzag**：±`dz` 交替打舵，每段以带符号阈值 ±`phiz` 触发；`returnToStart=true` 时末尾追加一段 HEADING 保持段，目标为起始航向 `psi0`（P12 整改：旧实现硬编码 0°）。
- **Circles**：设计 §4.3 原为"转向率闭环（`r_cmd=3°/s`）"，当前实现以**单段 RUDDER 恒舵角等效**（Nomoto 稳态下恒舵角即恒转向率）；若海试发现风流下圆度不佳，预留升级为转向率外环（开发文档 §7.4.4）。
- **Cloverleaf**：四个转向段**必须同向**（`dMax` 符号决定右旋/左旋四叶）。总航向变化 4×270°=1080°，净回 ψ0，四叶图案几何闭合；若转向方向交替，出转航向只有 ψ0 与 ψ0+270° 两个取值，直航目标与实际航向脱节，飞不出四叶形。直航段目标航向 = `ψ0 + dir·270°·(i+1)`（mod 360°）。
- **Search**：直航段为 HEADING + TIME 定时，转向段恒右舵 +90°；开环，覆盖率随漂移下降，设计建议配合雷达/视觉辅助。注意其形参 `dt` 是**边长时长增量（s）**，与控制周期 dt 同名易混淆（待确认是否为有意命名）。

历史缺陷（已关闭，值得在文档中说明其整改价值）：旧实现 Cloverleaf/Search 的转向段用 HEADING 模式 + 270°/90° 阈值，但 PD 跟踪目标航向走**最短路径**（+270° 等价 −90°），阈值永不满足、序列卡死（§10.5-P7）。整改方案：转向段一律改为 RUDDER 恒舵角，达到累计角度后切 HEADING 段收尾。

### 航段 Leg 结构

`Leg` 是机动的最小单元（`control/include/maneuver_sequencer.hpp:13`）：

```cpp
struct Leg {
    enum class Mode { HEADING, RUDDER };
    enum class Trigger { HEADING_REACHED, TIME, RATE, NONE };
    Mode     mode;       // HEADING: PD 跟踪目标航向 / RUDDER: 固定舵角
    double   target;     // HEADING: 目标航向(deg) / RUDDER: 目标舵角(deg)
    Trigger  trigger;    // 切段触发条件
    double   threshold;  // HEADING_REACHED: 累计航向变化(deg) / TIME: 时长(s) / RATE: 转向率(deg/s)
    int      next;       // 下一段索引，-1 表示序列结束
};
```

四类触发条件的判定式与适用场景（开发文档 §7.3.2）：

| Trigger | 判定式 | 适用 | 注意 |
|---|---|---|---|
| `HEADING_REACHED` | `cumHeading·threshold ≥ threshold²`（方向敏感，P12） | 转向段 | 增量语义、切段清零；**阈值带符号，须与本段舵向一致** |
| `TIME` | `legElapsed ≥ threshold` | 直航定时段（Cloverleaf/Search） | 精度受控制周期 dt 粒度限制 |
| `RATE` | `abs(rateAct) ≤ threshold` | "转向率衰减到位"类收尾段 | 阈值过小会因噪声永不触发，建议 ≥ 0.5°/s |
| `NONE` | 永不触发 | 末段长期保持（Williamson Leg3、U-Turn 保持段） | 由操作员或上层退出 |

方向敏感判定（P12 整改）的意义：旧逻辑 `|cum| ≥ |thr|` 无方向——反舵后船舶因惯性仍沿旧转向顺漂，累计量继续按旧方向增长，会在真正转向新方向之前把航段**"预触发"**。现要求累计量沿阈值符号方向达到幅值才切段。

每拍 `update(headingAct, rateAct, dt)` 做三件事：**累计状态量 → 判定触发 → 输出当前段指令**：

- 累计航向变化：`cumHeading += angleDiffDeg(ψ_k, ψ_{k-1})`，逐拍取最小带符号差，可正确表达"累计转过 270°/360°"这类超一圈语义；首拍用当前航向初始化基准（`first_`），避免把 `ψ−0` 误计入。
- 接口约束：逐拍差分要求相邻两拍航向变化 < 180°，50 Hz 下裕量充足；低频调用（如 1 Hz）会折叠错误，**调用周期须 ≥ 10 Hz**。
- 输出 `Cmd{isHeading, heading, rudder}`：HEADING 段给目标航向由 PD 闭环跟踪；RUDDER 段直接给舵角指令，但**不旁路安全限幅**——`AutopilotCore` 中经 `pd_.saturate(cmd.rudder)` 施加幅值/舵速限幅（P7 整改，设计 §4.10）。

### 机动编排器的状态机逻辑

`ManeuverSequencer` 内部状态：`legs_`（航段表）、`idx_`（当前段索引）、`legElapsed_`（段内计时）、`cumHeading_`（段内累计航向变化）、`lastHeading_`/`first_`（差分基准）、`done_`。

状态流转：

```text
setLegs(legs) 非空 → 复位全部状态，idx_=0，进入执行
执行中：每拍 update() 累计 → 判定触发
  未触发        → 持续输出当前段指令
  触发且 next 有效 → legElapsed_/cumHeading_ 清零，idx_=next，切下一段
  触发且 next=-1 或越界 → done_=true，序列结束
done 后 update() 返回安全零值；setLegs() 重新装载可再次执行
```

`AutopilotCore` 侧的编排（`autopilot_core.hpp`）：

- `startManeuver(legs)`：装载序列并切入 `Mode::AUTO_MANEUVER`；会抢占进行中的辨识试验（`identTrial_=NONE`）。
- `AUTO_MANEUVER` 每拍：调 `sequencer_.update()` → HEADING 段做增益调度查询（`schedule_.query(speedKn, seaState, Kp, Kd)`，P13 整改：否则机动直航段全程停留默认增益，慢船会丢目标航向）后 PD 闭环；RUDDER 段经 `pd_.saturate()` 限幅直通。机动期间辨识器与 ESC 保持挂起（辨识试验除外，见下节）。
- 序列 `done()` 当拍：不再输出 0（那是绕过舵速限幅的瞬时跳变，P11 整改），立即转 `AUTO_HEADING` 并**以当前航向为保持目标**闭环运行本拍。
- `setMode()` 切模式时清理残留：`pd_.reset()`、取消进行中的辨识试验；切出到 MANUAL/HOLD 时 `tuner_.suspend()`（清测量状态、保留基线）。

### P14 自动 Z 形辨识试验

定位：把传统的 Z 形操纵性试验**全自动化**——一键发起，自动执行 Z 形机动，机动中在线辨识 Nomoto 模型 K̂/T̂（`T·ṙ + r = K·δ`），收官自动经极点配置建立基线增益（`Kp=ωn²T/K`，`Kd=(2ζωnT−1)/K`），无需人工标定介入。对应设计 §3.9.4 Z 形试验辨识的自动化。

**接口与状态**（`autopilot_core.hpp`）：

```cpp
void startIdentifyTrial(double dz = 20.0, double phiz = 20.0, int cycles = 4);
enum class IdentTrialState { NONE, RUNNING, OK, FAILED };
```

**流程**：

1. **发起**：`identifier_.reset()` 清空历史辨识数据；装载 `ManeuverSequencer::zigzag(0.0, dz, phiz, cycles, false)`（2×cycles 段 ±dz 交替、±phiz 触发的 Z 形序列）；`identTrial_=RUNNING`，切入 `AUTO_MANEUVER`。
2. **机动中辨识**：试验期间每拍 `identifier_.update(δ, r, dt)` 喂辨识器——`NomotoIdentifier` 自降频，任意频率调用内部满 1 s（`beatSec`）才做一拍 RLS；试验显式发起，**不受调参器速度门控影响**。
3. **收官建基线**：序列 `done()` 当拍调 `finalizeIdentTrial()`，一行映射到 `tuner_.establishBaseline(idSnapshot())`：
   - 前置条件：辨识快照 `excited && valid`（物理合理性 K̂∈(0,2]、T̂∈[1,60]），否则返回 false，**保持原增益**；
   - 通过则极点配置（默认目标 ζ=0.85、ωn=0.20 rad/s）→ 稳定检查 → 设基线并应用；
   - 与 `tick()` 周期评估共享内部路径 `applyEstimate_()`，但**不做漂移检查**（试验是明确产出）、振荡条件以 0 传入由 ζ/ωn 代数条件把关（机动刚结束时振荡测量无效）、**恒重锚** ESC（清零积分/解调/窗口样本）；
   - 结果映射 `IdentTrialState::OK / FAILED`。
4. **善后**：无论成败，当拍转 `AUTO_HEADING` 保持当前航向闭环运行；基线建立后增益归 `AutoTuner` 所有，ESC 可从该基线继续微调。
5. **取消/抢占**：`setMode()` 切模式取消试验；`startManeuver()` 普通机动抢占试验。

**仿真验证场景**（`ar_sim --trial`，`Simulator::runIdentTrial`）：模拟未标定冷启动——调度表给保守默认增益（Kp=1.0/Kd=0.5 覆盖全海况行），调参器强制去激活（无 ESC 载波纹波），海况 0 无扰动；收官即停。PASS 判据：收官应用成功（`applied`）且辨识误差 |ΔK|、|ΔT| ≤ 25%。默认参数：K=0.2/T=8、10 kn、上限 300 s。

实测结果（测试用例文档 §6.2）：**PASS**——K̂=0.2067（+3.4%）、T̂=8.13（+1.6%）、175 s 完成并应用增益；试验后增益 = 极点配置结果本身（Kp=1.6、Kd=8.6 量级，ζ=0.85/ωn=0.2）。

### 测试用例覆盖点

**单元测试 `test_maneuver_sequencer`（14 用例，全部 PASS）**：

| 用例 | 覆盖点 |
|---|---|
| `EmptyIsDone` | 空序列 `done()==true`，输出安全值 |
| `TimeTriggerAdvances` / `HeadingReachedTriggerAdvances` | TIME / HEADING_REACHED 触发切段 |
| `HeadingReachedIsDirectionAware` | 方向敏感触发（P12），反舵顺漂不预触发 |
| `WilliamsonStructure` / `UTurnUsesRudderTurn` / `CirclesStructure` | 各预置机动的段数/模式/阈值结构 |
| `ZigzagStructure` / `ZigzagReturnToStart` / `ZigzagReturnTargetsPsi0` | Zigzag 2N 段计数、可选保持段、保持目标=ψ0（P12） |
| `CloverleafTurnsUseRudder` / `CloverleafLegsAreGeometricallyConsistent` / `CloverleafGeometryClosesWithIdealShip` | 三叶草转向段 RUDDER 化（P7）、四段同向几何一致、理想船几何闭合 |
| `SearchTurnsUseRudder` | 方搜转向段 RUDDER 化（P7） |

**单元测试 `test_autopilot_core` 中相关用例（17 用例中的机动/P14 部分）**：

- `ManeuverCompletesToAutoHeading`（P11）：机动完成拍输出 PD 闭环结果（非 0 瞬跳），自动回 `AUTO_HEADING` 并保持当前航向；
- `ManeuverHeadingLegAppliesScheduledGains`（P13）：机动 HEADING 段应用调度表增益；
- `IdentTrialIdentifiesAndAppliesGains`（P14）：完整 Z 形试验闭环——K=0.2/T=8 模型当船，断言收官 `OK`、`identifier().valid()`、K̂/T̂ 收敛（±0.05/±2 s）、试验后增益≈极点配置值（Kp≈1.6、Kd≈8.6）、模式回到 `AUTO_HEADING`。

**仿真场景回归**（`control/sim/scenarios.hpp` 定义场景结构，`ar_sim` 执行）：

| 场景 | 命令 | 判据 | 实测 |
|---|---|---|---|
| 六机动形态 | `ar_sim --maneuver <name>`（williamson/uturn/zigzag/circles/cloverleaf/search） | 序列真实执行完毕（`done()` 或进入 NONE 保持末段）+ 全程数值有限 + `maxRudder ≤ 36°`（限幅生效含余量） | zigzag PASS（§6.2） |
| 自动 Z 形辨识试验 | `ar_sim --trial` | 见上节 | PASS（K̂ +3.4%、T̂ +1.6%，175 s） |
| 调参收敛试验的 Z 形激励 | `ar_sim --tune --K .. --T .. --leg ..` | RLS 收敛 + 基线 + ESC 窗口平台（U24~U26） | 典型船/快船 converged=1；慢船需 `--leg 60`（激励 ≥2T）才辨识收敛（U06/U26，已知限制） |

补充说明：各场景 Z 形参数取值不同，引用时注意区分——工厂缺省 `zigzag(ψ0, 10, 10, 5)`；P14 试验缺省 `dz=20/phiz=20/cycles=4`；`--maneuver zigzag` 场景用 `dz=10/phiz=15/N=3`；`--tune` 的 Z 形参考激励 leg 缺省 15 s（慢船需按 T 缩放至 ≥2T，如 T=20 s 船用 60 s）。

**（待确认）代码与设计文档的两处口径差异**：

1. Circles：设计 §4.3 为"转向率闭环 `r_cmd=3°/s`"，实现为恒舵角等效（缺省 +15°）。开发文档 §7.4.4 已声明该简化合理，但设计文档未同步更新。
2. U-Turn 完成判据：设计 §4.5 要求 `|e|<1°` 持续 2 s；当前实现末段为 NONE 触发长期保持，该判据"需上层实现"（开发文档 §7.4.4），`AutopilotCore` 中未见对应逻辑——由谁实现待确认。
