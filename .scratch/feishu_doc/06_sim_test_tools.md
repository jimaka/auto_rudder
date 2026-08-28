## 仿真验证、测试与工具链

本节覆盖四块内容：桌面闭环仿真器（`control/sim/`）、GoogleTest 单元测试（`control/tests/`）、Python 绘图/视频工具链（`control/tools/`）、调试上位机（`control/ui/debug_ui.html`）。

### 仿真器架构（control/sim/）

仿真器是纯桌面的 50 Hz 闭环 runner：被控对象 + 扰动模型 + 真实控制算法头文件（`autopilot_core.hpp`，经 `ar_headers` INTERFACE 目标引入）串成完整闭环，**跑的就是实船同一份控制代码**，不是算法副本。四个头文件职责如下。

#### 被控对象：`NomotoPlant`（nomoto_plant.hpp）

- 船舶动态用 Nomoto 一阶模型：`T·ṙ + r = K·δ_eff`，其中 `δ_eff = δ_act + disturb`（扰动折算为等效舵角叠加在输入端）。
- 舵机建模为一阶惯性 + 物理限幅：`τS=0.3 s`（一阶时间常数）、`ddotMax=10 °/s`（舵速限幅）、`dMax=35°`（舵角限幅）。
- 默认船参：`K=0.2 (1/s)`（旋回性指数）、`T=8.0 s`（追随性指数）；状态为 `r`（转向率 °/s）、`psi`（航向 °）、`deltaAct`（实际舵角 °）。
- 积分方式为显式欧拉（`r += dr·dt; psi += r·dt`），航向归一化到 `(-180, 180]`。

#### 扰动模型：`Disturbances`（disturbances.hpp）

三类扰动，幅值统一按海况缩放：

| 扰动 | 模型 | 海况 1 基准幅值 |
|------|------|----------------|
| 恒值风致 | 等效恒值舵角 `windDeg` | 2.0° |
| 波浪 | 0.1 Hz 正弦等效舵角 `waveAmp·sin(ωt+φ)` | 1.0° |
| 量测噪声 | ψ / r 加性噪声 | σ 分别 0.2° / 0.1°/s |

- 海况映射（`seaFactor`，唯一定义处）：sea 0 = ×0（平静）、sea 1 = ×1（基准）、sea 2 = ×2、sea 3 及以上 = ×3。
- 伪随机用自带 LCG（`seed=12345`），不依赖 `<random>`，保证跨平台确定性复现；注释特别说明：无论海况因子如何都推进 LCG 序列，保持随机序列一致。

#### 闭环运行器：`Simulator`（simulator.hpp）

固定步长 `dt=0.02 s`（50 Hz），每拍序为：组装 `SensorInput`（可叠加噪声）→ `ap.step(s, dt)` 得舵角指令 → `plant.step(cmd, dt, disturb)` 推进对象 → 按 `recordEvery` 记录 `Sample`（含 t/ψ/ψ_ref/δ_cmd/δ_act/r/e/Kp/Kd/mode）。

内置 6 种运行模式，覆盖从单元验证到全链路收敛的完整验证梯度：

| 方法 | 场景 | 关键设计 |
|------|------|----------|
| `runStep` | 航向阶跃（先保持 `holdSec`，再阶跃 `stepDeg`） | M1 纯 PD 验证：用已知 K/T 做极点配置增益铺调度表（默认 `ζ=0.85`、`ωn=0.25`，公式 `Kp=ωn²T/K`、`Kd=(2ζωnT−1)/K`），禁用自动优化（`highSpeedDeactivateKn=-1`）；扰动可选 |
| `runIdent` | RLS 闭环辨识 | Z 形 ±20°/10 s 舵角激励（实现上以 AUTO_HEADING + 大幅航向目标等效注入，代码注释说明 MANUAL 返回 0 无法直接注入），验证 K̂/T̂ 收敛到真值 |
| `runEsc` | ESC 长期场景 | Z 形 ±40°/15 s 航向参考激励（注释记载选型理由：旧 ±10°/30 s 激励过不了辨识激励门导致假阳性 PASS），跑通「辨识收敛 → 极点配置建基线 → ESC 窗口调参」 |
| `runTuneConvergence` | 自动调参收敛试验（R1 全链路） | 同 ESC 激励，每 1 s 记一行富采样（含载波增益、去载波持久增益 `KpCenter/KdCenter`、K̂/T̂、窗口 J、结算事件）；收敛判据见下 |
| `runManeuver` | 六机动形态 | 跑预置航段序列，判据：序列真实执行完毕（`done()` 或进入末段保持段）+ 全程数值有限 |
| `runIdentTrial` | P14 自动 Z 形辨识试验 | 从未标定增益 (1.0/0.5) 冷启动，`ap.startIdentifyTrial(dz, phiz, cycles)` 自动机动 + 1 s 节拍辨识，收官自动建基线，收官即停 |
| `runValidation` | 全验证矩阵 + 实时性 | 带扰动阶跃，额外测单拍最大耗时 `maxStepMicros`（PASS 要求 < 5000 μs，50 Hz 周期 20 ms 留 4 倍余量） |

阶跃类场景的性能指标与 PASS 阈值（`computeMetrics`，对应开发文档 §1.5）：

| 指标 | 定义 | 阈值 |
|------|------|------|
| 超调 `overshootPct` | 符号感知，只统计阶跃后 | ≤ 10% |
| 调节时间 `settlingTime` | 5% 带，从阶跃时刻起算 | ≤ 30 s |
| 稳态误差 1σ `ess1Sigma` | 最后 20 s 误差标准差 | ≤ 1.0° |
| 稳态偏置 `essBias` | 最后 20 s 误差 \|mean\|（暴露 PD 无积分在恒风下的系统偏置） | ≤ 1.0° |
| 操舵频次 `rudderFreq` | 舵令方向翻转计数（幅值 >0.1° 才计） | ≤ 6 次/分钟 |

`runTuneConvergence` 的收敛判据（代码注释原文）：(a) 基线建立且辨识物理合理；(b) 基线后至少结算 4 个窗口，末 3 窗 J 相对散布 ≤ 10%（平台）；(c) 末窗 J ≤ 首窗 ×1.02，且 K̂/T̂ 相对真值偏差 ≤ 25%；(d) 全程数值有界。

CSV 导出入口：`exportCsv` / `exportLastCsv`（11 列通用格式）与 `exportTuneCsv`（收敛试验专用 19 列，含 `KpCenter/KdCenter/Khat/That/identValid/baseline/Jwindow/winIndex/settle`）。

#### 场景定义（scenarios.hpp）

- `defaultMatrix()`：三组船参的 30° 阶跃——典型 `K=0.2/T=8`、快船 `K=0.5/T=3`、慢船 `K=0.1/T=20`（慢船时长 180 s，其余 120 s），海况均取 1。
- 另有 `IdentScenario` / `EscScenario` / `ManeuverSimScenario` / `ValidationScenario` 结构体承载对应模式的参数。

### 构建与运行

#### CMake 结构（control/CMakeLists.txt + sim/tests 子目录）

- C++17，默认 `CMAKE_BUILD_TYPE=Release`；算法库为 header-only 的 `ar_headers` INTERFACE 目标。
- 选项：`AR_BUILD_TESTS`（默认 ON，GoogleTest v1.14.0 经 FetchContent 拉取）、`AR_BUILD_SIM`（默认 ON）。
- 产物：仿真器 `build/sim/ar_sim`；单元测试每文件一个可执行目标（便于独立调试），经 `gtest_discover_tests` 注册，并注入 `ASAN_OPTIONS`/`LSAN_OPTIONS` 环境（配合 `build-asan/` 的 ASan 构建）。

```bash
cd control
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

#### ar_sim 命令行

```bash
./build/sim/ar_sim --scenario step_typical --out out/step.csv   # 单场景阶跃
./build/sim/ar_sim --all                                        # 默认矩阵 3 场景
./build/sim/ar_sim --ident / --trial / --esc / --tune           # 辨识 / 辨识试验 / ESC / 收敛试验
./build/sim/ar_sim --maneuver zigzag                            # 六机动之一
./build/sim/ar_sim --validate                                   # 全验证矩阵 + 实时性
./build/sim/ar_sim --list                                       # 列出内置场景
```

覆盖参数：`--K --T --speed --sea(0-3) --seconds --leg --dist --out`。退出码语义：任一 FAIL → 非零退出，可直接进 CI。机动默认时长：`cloverleaf` 480 s、`search` 360 s、其余 300 s；`--tune` 默认 1200 s，`--esc` 默认 480 s。

各模式判 PASS 的核心条件：

- `--ident`：辨识 `valid` 且 |K̂−K|<0.05、|T̂−T|<2.0（默认船参下即 ±25%）。
- `--trial`：收官应用成功（`applied`）且 K/T 辨识误差 ≤ 25%。
- `--esc`：辨识 valid + 基线建立（`baselineEstablished()` 只读 getter 观测，非迂回反推）+ 增益相对起点移动 >1% + 数值有界——ESC 从未启动时增益逐位不动，必然 FAIL（防假阳性）。
- `--tune`：上文四段收敛判据，不收敛即非零退出。
- `--maneuver`：`completed`（序列真实跑完）且 `maxRudder ≤ 36°`（限幅生效含余量）。
- `--validate`：固定 4 个 case（典型无扰动/典型带扰动/快船/慢船），指标达标且单拍耗时 < 5 ms，全过才 ALL PASS。

CTest 冒烟（`AR_BUILD_TESTS` 时注册，`TIMEOUT 120`）：`sim_all`、`sim_validate`、`sim_ident`、`sim_esc`、`sim_trial`、`sim_maneuver_zigzag`。注意 `--tune` 与其余五种机动未注册为 ctest（仅手动/脚本跑）（待确认：是否有意为之）。

### 单元测试覆盖清单（control/tests/）

共 8 个测试文件、87 个用例（按 TEST 宏计数）。逐文件列出测试点：

#### test_angle_utils.cpp — 角度归一化与差值（7 例）

- `NormalizePositiveBoundary`：+180 保留为 +180（区间左开右闭 `(−180, 180]`）。
- `NormalizeNegativeBoundary`：−180 折叠为 +180。
- `NormalizePositiveFold`：540 → 180。
- `NormalizeNegativeFold`：−541 → 179。
- `DiffAcrossZeroPositive` / `DiffAcrossZeroNegative`：跨零差值 ±2°（1 与 359）。
- `RadDegConsistency`：度→弧度归一化一致性。

#### test_pd_controller.cpp — PD 航向控制器（11 例）

- `CrossZeroError`：ref=1/act=359 跨零误差为 +2，输出正舵。
- `DeadzoneOutputsZero`：|e|=0.2 < 死区 0.3 → 输出 0。
- `RudderSaturation`：大误差限幅到 ±35°。
- `RateLimit`：舵速限幅 10°/s → 每拍最多 +0.2°，逐拍爬坡。
- `FirstBeatNoDerivativeKick`：首拍无微分冲击（Kd=100 放大暴露）。
- `Quantization`：输出按 `quant=0.1°` 量化。
- `LowpassAttenuation`：5 Hz 噪声下微分低通（τ=0.2）衰减 > 80%。
- `SetGainsPreservesState`（P1 回归）：`setGains` 不清 `prevErr_`，微分历史保留。
- `InitBaselinePreventsJump`（P11）：`initBaseline(20°)` 后死区内目标 0 也经舵速限幅回中（20→19.8），不瞬跳。
- `DeadzoneExitRespectsRateLimit`（P11）：35° 饱和后进死区，逐拍回中 |Δ|≤0.2°/拍，最终回 0。
- `SetDtRejectsNonPositive` / `SetParamsClampsInvalidDtQuant`（P11）：非法 dt/quant 被拒或钳制，输出恒有限。

#### test_gain_schedule.cpp — 增益调度（7 例）

- `NominalAtMidpoint`：段中点取名义值（v=7、海况 0 → Kp=1.5/Kd=0.6）。
- `InterpolationContinuityAtBoundary`（P5 插值回归）：v=14 段交界两侧极限连续，中点锚定名义值。
- `HighSeaState`：海况 2 行正确命中（Kp=1.0）。
- `SeaStateClamp`：海况越界（9）按 3 处理。
- `SpeedAboveMax` / `SpeedBelowMin` / `NoCrashOnNegativeSpeed`：超程/低速/负速分别取末段、首段名义值，不崩溃。

#### test_nomoto_identifier.cpp — Nomoto 在线辨识 + 极点配置（9 例）

- `ConvergesOnNomotoData`：1 s 节拍、非对称 Z 形 ±25°（8/3 拍保持）激励下收敛到 K=0.2±0.05 / T=8±2.0。
- `SelfDecimatesAtAnyCallRate`（R1 自降频）：50 Hz 任意频率调用 `update(δ, r, dtNow)`，内部满 `beatSec=1 s` 才做 RLS 拍，收敛性不变。
- `NotExcitedWhenIdle`：小幅值输入不过激励门，`excited()=false`。
- `RejectsOutOfRangeInput`（P6）：δ>35° 或 r>30°/s 的样本被丢弃，不污染 θ。
- `RecoversAfterNaNInput`（P11）：单拍 NaN 不永久毒化辨识器，恢复后与干净参考运行结果相当。
- `InvalidWhenOutOfRange`：物理合理范围（K∈(0,2]、T∈[1,60]）内 `valid()=true`。
- `PolePlacement.ReturnsFalseOnDegenerate`：K=0 或 T=0 返回 false。
- `PolePlacement.ClampsNegativeKd`：小 ωn 使 `2ζωnT<1` 时 Kd 钳到 0。
- `PolePlacement.ProducesTargetZetaWn`：反算闭环 `ωn=√(Kp·K/T)`、`ζ=(1+Kd·K)/(2√(T·Kp·K))` 命中目标 0.25/0.85。

#### test_auto_tuner.cpp — 自动优化器（15 例，R1 起一律经公有接口 `tick()`/`establishBaseline()` 驱动）

- 极点配置三例（同上等价覆盖）：`PolePlacementProducesTargetZetaWn` / `PolePlacementRejectsDegenerate` / `PolePlacementClampsNegativeKd`。
- 稳定门：`StabilityCheckAcceptsHealthy`（健康增益通过）、`StabilityCheckRejectsLowZeta`（Kd≈0 低阻尼拒绝）、`StabilityCheckRejectsHighWn`（Kp×10 使 ωn 超限拒绝）、`StabilityCheckRejectsOscillation`（振荡幅值 8° > `oscMax` 5° 拒绝）。
- 振荡检测器：`OscillationDetectorMeasuresAmplitude`（±3° 正弦误差 → 峰-峰 ≈6°）、`OscillationDetectorResetClearsPeaks`（P11：`reset` 清历史峰，不泄漏进下一窗口）。
- `RingBufferNoHeap`（P2）：编译期容量环形数组，满后不增长、无堆分配。
- `EscDitherModulatesGains`（P11）：基线未建立时调度增益原样直通（无载波）；建立后完整载波周期内实际增益覆盖 ±`escAmp` 相对调制。
- `WindowSettleUpdateIsClamped`（P2/P11）：逐窗校验 ESC 更新受 `clampUpdate` 约束（≤ `max(|old|·maxUpdateFrac, absMinUpdate)`），或被稳定门拒绝时回退基线。
- `EscTunesKpTowardOptimum`（P11 端到端）：Nomoto 对象 + 故意偏低 Kp 起点，8 个窗口后 Kp 向增大方向移动且窗口成本 J 下降。
- `EscUpdateUnsticksFromZeroKd`（P11）：Kd 被钳 0 后，绝对下限 `absMinUpdate` 允许 ESC 继续调整解锁。
- `SettleRevertsToBaselineWhenEstimateInvalid`（§3.10.7）：快照无效 → 窗口结算过不了稳定门 → 回退（revert）到基线 (1.6, 8.6)。
- `SettleAcceptsUpdateWhenEstimateHealthy`：快照健康 → 结算被接受，Kd 按梯度有界上移。
- `RebaselineClearsIntegratorState`（P12 重锚）：显式 `establishBaseline` 重基线清零 ESC 积分/解调/窗口样本（`computeJ()=0`），旧梯度不得施加到新基线。

#### test_maneuver_sequencer.cpp — 机动序列框架（14 例）

- `EmptyIsDone`：空序列即 done。
- 结构断言：`WilliamsonStructure`（3 段：+δ_max 转 60° → −δ_max 转 −180° → HEADING 保持；P12 反舵段阈值带负号）、`UTurnUsesRudderTurn`（P7：转向段为 RUDDER 恒舵角 ±dMax 转 ±180°）、`ZigzagStructure`（阈值符号与本段舵向一致、交替）、`ZigzagReturnToStart` / `ZigzagReturnTargetsPsi0`（P7/P12：returnToStart 追加 HEADING 保持段，目标=传入 ψ0 而非硬编码 0°）、`CloverleafTurnsUseRudder`（8 段，转向段 RUDDER、直航段 TIME 触发）、`SearchTurnsUseRudder`（直航 HEADING + 右转 90° RUDDER 交替）、`CirclesStructure`（单段 RUDDER 360°）。
- `CloverleafLegsAreGeometricallyConsistent`：四叶几何自洽——四转向段同向、直航目标=前段出转航向、累计 ±1080° 净回 ψ0、四个直航目标互异；负 dMax 镜像为左转四叶同样自洽。
- `CloverleafGeometryClosesWithIdealShip`：理想化航向模型走完整序列，验证按段顺序推进、切段航向连续、累计 +1080°、净回 ψ0 图案闭合。
- 触发器行为：`TimeTriggerAdvances`（1 s 后切段）、`HeadingReachedTriggerAdvances`（累计 90° 切段）、`HeadingReachedIsDirectionAware`（P12：反舵段顺漂不得预触发，须真正反向过阈值才切段）。

#### test_param_store.cpp — 参数存储（7 例，P8 整改）

- `SerializeRoundtrip`：序列化/反序列化全字段逐位一致（含调度表逐行）。
- `CrcDetectsCorruption`：篡改一字节后 CRC 校验失败。
- `CrcSelfExclusion`：CRC 范围不含 crc 字段自身。
- `ValidateRangeRejectsBad`：K>0、T≥1、ddotMax≤30、Kd≥0 等范围检查。
- `SaveLoadRoundtrip`：内存模拟 EEPROM，自动填时间戳、版本号 +1，加载后 CRC/范围双过。
- `LoadDetectsCorruption`：存储内容被篡改 → 加载失败。
- `LittleEndianPortable`：显式小端字节序，跨平台一致（rudderZero=1.0 → `3F F0 00…`）。

#### test_autopilot_core.cpp — 核心集成（17 例）

- 模式机：`DefaultModeIsManual`、`ManualPassesRudderThrough`（透传舵角）、`AutoHeadingProducesNonzeroRudder`、`SetModeResetsState`（P9：切模式清 PD 状态，首拍从 0 基准限幅爬坡）、`HoldFreezesLastRudder`（P9：HOLD 冻结上一拍舵角而非输出 0）、`HoldThenManualClears`。
- 速度门控：`HighSpeedDisablesTuner`（20 kn > 15 kn 阈值去激活）、`LowSpeedEnablesTuner`。
- 机动集成：`ManeuverCompletesToAutoHeading`（完成后回 AUTO_HEADING；P11：完成拍输出 PD 闭环结果 14.8° 而非 0 瞬跳）、`ManeuverHeadingLegAppliesScheduledGains`（P13：机动 HEADING 段必须应用增益调度）。
- `EnterAutoHeadingSeedsRudderBaseline`（P11）：切入自动的首拍以实际舵角为舵速限幅基准，舵令不从 0° 爬升。
- 自动优化集成：`EscDitherReachesPdGainsAfterBaseline`（P11：合成辨识驱动建基线后，载波真实叠加到 PD 增益）、`NoBaselinePassesScheduleGainsThrough`（R1：无基线时调度增益直通）、`IdentUpdatesDoNotOverwriteEscGains`（P12：增益所有权——基线建立后辨识更新不盲目覆写，T 漂移超 25% 阈值才显式重锚跳到新配置）、`EscFallbackRestoresBaselineOnOscillation`（P12：振荡超 `oscMax` → 结算被拒 → 回退基线）、`OscDetectorStarvedDuringReferenceMotion`（P12：参考机动期间振荡检测不喂入，稳定后恢复）。
- `IdentTrialIdentifiesAndAppliesGains`（P14）：自动 Z 形辨识试验端到端——冷启动 (1.0/0.5)，机动中辨识收敛到 K̂≈0.2/T̂≈8，收官应用极点配置增益（ζ=0.85、ωn=0.2 → Kp≈1.6、Kd≈8.6），模式回 AUTO_HEADING。

### Python 绘图/视频工具链（control/tools/）

依赖：`matplotlib>=3.5`、`numpy>=1.20`（系统包 `python3-matplotlib`/`python3-numpy`），视频编码需 `ffmpeg`；`requirements.txt` 可供虚拟环境安装。三个脚本都自动检测 CJK 字体（Noto Sans CJK / 文泉驿等）避免中文乱码。

#### plot_sim.py — 多子图分析图

读取 `ar_sim` 导出的 11 列 CSV，生成 5 子图（航向 ψ/参考、舵指令 δ_cmd/实际 δ_act 含 ±35° 限幅虚线、转向率 r、误差 e、增益 Kp/Kd 双 y 轴），模式切换处画绿色点划竖线。用法：

```bash
python3 tools/plot_sim.py out/step.csv -o out/step.png --metrics   # 单文件 + 指标表
python3 tools/plot_sim.py a.csv b.csv -o cmp.png                   # 多文件叠加对比
python3 tools/plot_sim.py out/validate_*.csv --grid -o grid.png    # 网格平铺
python3 tools/plot_sim.py out/will.csv --polar -o will.png         # 追加机动极坐标轨迹（<stem>_polar.png）
```

- `--metrics` 打印指标表（超调/调节时间/稳态 1σ/操舵频次，脚本内独立重算）。
- 极坐标图以航向 ψ 为极角、累计转首角为半径，零度朝北、顺时针（航海习惯），标起点（绿）/终点（红）。
- 环境变量 `PLOT_SIM_AGG=1` 强制 Agg 后端（无 GUI 仅保存）。

#### animate_sim.py — 动画视频

同一 CSV 生成时间演化动画：左侧 5 子图 + 可选右侧机动极坐标轨迹；按目标 fps 下采样控制总帧数（上限 1200 帧）。

```bash
python3 tools/animate_sim.py out/step.csv -o out/step.mp4 --fps 25 --speed 2.0
python3 tools/animate_sim.py out/will.csv -o out/will.mp4 --polar
python3 tools/animate_sim.py out/step.csv -o out/step.gif   # 无 ffmpeg 时用 PillowWriter 出 GIF
```

`.mp4` 走 `FFMpegWriter`，`.gif` 走 `PillowWriter`；环境变量 `ANIM_SIM_AGG=1` 强制 Agg。

#### plot_convergence.py — 自动调参收敛效果图

读取 `--tune` 导出的收敛 CSV（19 列），每个输入产出三张图：

1. `*_rls.png`：K̂/T̂ 随时间逼近真值（真值虚线 + 基线建立时刻竖线）。
2. `*_esc.png`：去载波持久增益 `KpCenter/KdCenter` 轨迹 + 调度起点水平线 + 每窗口结算竖线 + 窗口 ITAE J 柱状图。
3. `*_overview.png`：全链路概览（航向/舵角/误差/含载波增益/辨识估计）。

```bash
./build/sim/ar_sim --tune --K 0.2 --T 8 --seconds 1200 --out out/tune.csv
python3 tools/plot_convergence.py out/tune.csv --trueK 0.2 --trueT 8 -o out/figures
```

真值也可从文件名 `k<K>_t<T>`（如 `k0p2_t8`）自动解析；默认输出目录 `docs/auto_tuning/figures`。慢船激励适配：`ar_sim --leg 60` 把 Z 形参考翻转周期从 15 s 放大（建议 ≥ 2T，否则 K̂/T̂ 系统性偏大）。

#### 批量 shell 脚本

- `gen_all_videos.sh`：一键跑全部场景（step/ident/esc/六机动/validate 4 case）→ CSV → MP4，输出到 `control/out/`，已存在的跳过（增量）。
- `gen_maneuver_matrix.sh`：全工况机动视频矩阵——3 船型（typical/fast/slow）× 6 机动 × 2 干扰（无/带 `--dist`）= 36 个视频，输出 `out/matrix/<船型>_<机动>[_dist].mp4`；每个工况时长按实测序列完成时间标定（如 slow+cloverleaf 850 s、slow+search 540 s）。

### 调试上位机（control/ui/debug_ui.html）

单文件 Web 应用（HTML+CSS+JS，无第三方依赖），浏览器直接打开；内部 JS 实现与 `control/include/` 的 C++ 模块一一对应（PD/调度/辨识/ESC/机动序列/参数存储均有对应 JS 移植），仅用于调试可视化，不可用于实船控制。

#### 功能布局

- **顶栏**：模式切换、航向目标、仿真启停、ECU 连接（地址 + 状态灯）、数据导出。
- **左侧参数面板 4 标签**：PD（Kp/Kd/dMax/ddMax/deadzone/tau/quant/dt）、调度（调度表可增删行）、辨识（λ/dt/δ激励/r激励/P0）、优化（ζ/ωn、ESC 参数、性能权重 wU/wDu、安全约束 ζmin/ωnMax/oscMax/高速去激活/窗口）；顶部另有 Kp/Kd/航向目标/船速/海况 5 个实时滑块。
- **中部三路 Canvas 实时曲线**：航向（蓝实际/红目标，±180°）、舵角（绿指令/黄实际，±35°）、误差/转向率（紫/青，±20）。
- **右侧**：8 项实时状态；机动触发 7 按钮（Williamson、U-Turn 左/右、Zigzag、Circles、Cloverleaf、Search）+ 停止，显示当前段推进；自动优化面板（K̂/T̂/J/ζ̂/ω̂n/激励 + 启用/禁用/ESC 一步/回退基线）；参数存储（版本/CRC/时间戳 + 保存/加载，仅内存）。
- **底部日志**；数据导出支持 CSV（原始时间序列）与 JSON（元信息 + 参数快照 + 全部历史），文件名带时间戳。

#### 双模式运行

- **独立运行**：内置 Nomoto 一阶仿真器（`T·ṙ + r = K·δ`，默认 K=0.18、T=8.5；舵角一阶滞后 0.5 s；注入小幅噪声），点"启动"即以 PD 周期 `dt`（默认 20 ms → 50 Hz）跑闭环。
- **对接真实 ECU**：WebSocket（默认 `ws://localhost:8080`），连接后内置仿真器自动旁路，UI 只显示 ECU 上报。协议要点：
  - UI→ECU 指令 `{"type":"cmd","rudder":…,"mode":"AUTO_HEADING","headingRef":…}`；
  - ECU→UI 传感器上报 `{"type":"sensor","sensor":{headingDeg,rateDegS,rudderDeg,speedKn,seaState}}`；
  - 参数下发 `{"type":"set_params","scope":"pd|sched|id|tuner|store|mode", …}`（应用参数与滑块拖动时自动下发）；
  - ECU 回 `params_ack`（含 version/accepted）与 `status`（故障码列表）。

README 中附有完整的 UI 参数 ↔ C++ 字段映射表（`PdController::Params`、`NomotoIdentifier::Params`、`AutoTuner::Params` 各项默认值对照），此处不重复罗列。

### 代码与文档不一致 / 待确认点

- **仿真增益目标 ωn 两套值**：`Simulator` 默认 `zetaTarget=0.85 / wnTarget=0.25`（M1 验证用），而生产默认（P14 辨识试验收官、`AutoTuner::Params`）为 ζ=0.85、ωn=0.20（Kp=ωn²T/K=1.6 @ K=0.2/T=8）。属有意区分（仿真注释注明"M1 验证 PD 本身"），但文档引用数值时需说明上下文（待确认对外文档统一口径）。
- **UI 辨识参数仍含 `dt`**：R1 已删除 `NomotoIdentifier::Params.dt`（改自降频 `beatSec`，见 test_nomoto_identifier.cpp 注释"Params.dt 语义陷阱已删除"），但 `debug_ui.html` 的辨识标签与 `set_params`（scope=id）仍带 `dt` 字段（待确认 ECU 端是否忽略该字段）。
- **UI README 写"ESC 频率默认 0.05"**：C++ 侧 `escFreq` 为角频率（测试中取 `2·π` 即 1 Hz 载波），UI 默认 0.05 的量纲/语义与 C++ 侧是否一致（待确认）。
- **`--tune` 与 5/6 种机动未注册 ctest**：sim/CMakeLists.txt 只注册 `sim_maneuver_zigzag` 一个机动用例（待确认是否有意控制 CI 时长——`--tune` 默认 1200 s 仿真时长）。
