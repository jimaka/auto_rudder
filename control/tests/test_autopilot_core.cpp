// test_autopilot_core.cpp — 核心集成（对应开发文档 §5 + P9 整改回归）
#include "autopilot_core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
using namespace ar;

TEST(AutopilotCore, DefaultModeIsManual) {
    AutopilotCore ap;
    EXPECT_EQ(ap.mode(), Mode::MANUAL);
}

TEST(AutopilotCore, ManualPassesRudderThrough) {
    AutopilotCore ap;
    ap.setMode(Mode::MANUAL);
    SensorInput s{0, 0, 7.5, 10, 0};
    EXPECT_NEAR(ap.step(s, 0.02), 7.5, 1e-9);
}

TEST(AutopilotCore, AutoHeadingProducesNonzeroRudder) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    SensorInput s{0, 0, 0, 10, 0};
    double cmd = ap.step(s, 0.02);
    EXPECT_GT(std::abs(cmd), 0.0);
}

TEST(AutopilotCore, SetModeResetsState) {
    // P9 整改：切模式应清理状态
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    SensorInput s{0, 0, 0, 10, 0};
    for (int k = 0; k < 50; ++k) ap.step(s, 0.02);  // 累积状态
    // 切回 MANUAL 再切 AUTO_HEADING，PD 状态应被清
    ap.setMode(Mode::MANUAL);
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    double cmd = ap.step(s, 0.02);  // 首拍，应从 0 基准开始
    // 首拍舵速限幅：cmd 应 ≤ 0.2°（从 prevOut=0 起算）
    EXPECT_LE(std::abs(cmd), 0.2 + 1e-9);
}

TEST(AutopilotCore, HoldFreezesLastRudder) {
    // P9 整改：HOLD 保持上一拍舵角，而非输出 0
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    SensorInput s{0, 0, 0, 10, 0};
    double lastCmd = 0;
    for (int k = 0; k < 100; ++k) { lastCmd = ap.step(s, 0.02); }
    // 切 HOLD
    ap.setMode(Mode::HOLD);
    double holdCmd = ap.step(s, 0.02);
    EXPECT_NEAR(holdCmd, lastCmd, 1e-9);  // 冻结上一拍
}

TEST(AutopilotCore, HoldThenManualClears) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    SensorInput s{0, 0, 0, 10, 0};
    for (int k = 0; k < 50; ++k) ap.step(s, 0.02);
    ap.setMode(Mode::HOLD);
    ap.step(s, 0.02);
    ap.setMode(Mode::MANUAL);
    SensorInput s2{0, 0, 5.0, 10, 0};
    EXPECT_NEAR(ap.step(s2, 0.02), 5.0, 1e-9);  // MANUAL 透传
}

TEST(AutopilotCore, HighSpeedDisablesTuner) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    SensorInput s{0, 0, 0, 20.0, 0};  // 20 kn > 15 kn 去激活阈值
    ap.step(s, 0.02);
    EXPECT_FALSE(ap.tuner().enabled());
}

TEST(AutopilotCore, LowSpeedEnablesTuner) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    SensorInput s{0, 0, 0, 10.0, 0};  // 10 kn < 15 kn
    ap.step(s, 0.02);
    EXPECT_TRUE(ap.tuner().enabled());
}

TEST(AutopilotCore, ManeuverCompletesToAutoHeading) {
    AutopilotCore ap;
    auto legs = ManeuverSequencer::circles(15.0, 90.0);
    ap.startManeuver(legs);
    EXPECT_EQ(ap.mode(), Mode::AUTO_MANEUVER);
    SensorInput s{0, 0, 0, 10, 0};
    double hdg = 0.0;
    double prevCmd = 0.0, completionCmd = 0.0;
    bool gotCompletion = false;
    for (int k = 0; k < 2000 && ap.mode() == Mode::AUTO_MANEUVER; ++k) {
        s.headingDeg = hdg;
        s.rateDegS = 5.0;
        s.rudderDeg = prevCmd;  // 舵面跟随指令
        hdg += 5.0 * 0.02;
        const double cmd = ap.step(s, 0.02);
        if (ap.mode() == Mode::AUTO_HEADING && !gotCompletion) {
            completionCmd = cmd;      // 机动完成拍的输出
            gotCompletion = true;
        }
        prevCmd = cmd;
    }
    EXPECT_EQ(ap.mode(), Mode::AUTO_HEADING);  // 完成后回到 AUTO_HEADING
    ASSERT_TRUE(gotCompletion);
    // P11 整改：完成拍必须输出 PD 闭环结果（经舵速限幅），而非 0 初始值瞬跳。
    // 本场景：完成时舵角 15°，完成拍 e=0 进入死区，输出限幅回中 15-0.2=14.8
    EXPECT_GT(completionCmd, 0.0);
    EXPECT_NEAR(completionCmd, 14.8, 1e-9);
    EXPECT_LE(std::abs(completionCmd - 15.0), 10.0 * 0.02 + 1e-9);  // ddMax·dt
}

// P11 整改回归：切入 AUTO_HEADING 的首拍以实际舵角为舵速限幅基准，
//               舵令不得从 0° 爬升
TEST(AutopilotCore, EnterAutoHeadingSeedsRudderBaseline) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(30.0);
    SensorInput s{0, 0, 0, 10, 0};
    // 先把舵角指令跑到 10°（0.2°/拍 × 50 拍）
    double lastCmd = 0.0;
    for (int k = 0; k < 50; ++k) lastCmd = ap.step(s, 0.02);
    EXPECT_NEAR(lastCmd, 10.0, 1e-9);
    // HOLD 冻结舵角，实际舵面保持在 10°
    ap.setMode(Mode::HOLD);
    s.rudderDeg = 10.0;
    ap.step(s, 0.02);
    // 切回 AUTO_HEADING：首拍输出应从 10° 基准出发（≤ ddMax·dt），而非从 0 爬升
    ap.setMode(Mode::AUTO_HEADING);
    const double cmd = ap.step(s, 0.02);
    EXPECT_NEAR(cmd, 10.0, 10.0 * 0.02 + 1e-9);
    EXPECT_GT(cmd, 1.0);  // 明确不在 0 附近
}

// P11 整改回归：ESC 基线建立后，扰动载波真实叠加到 PD 实际增益
TEST(AutopilotCore, EscDitherReachesPdGainsAfterBaseline) {
    AutopilotCore ap;
    // 加快载波便于观察（1 s 周期）
    auto tp = ap.tunerMut().params();
    tp.escFreq = 2.0 * M_PI;
    ap.tunerMut().setParams(tp);
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    SensorInput s{0, 0, 0, 10, 0};

    // 合成 Nomoto 数据驱动在线辨识：±25° Z 形（8/3 拍保持），K=0.2 T=8
    // 辨识器每 50 拍（1 s）采样一次，收敛后极点配置建立基线（Kp≈1.6）
    const double K = 0.2, T = 8.0;
    double r = 0.0;
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    double schedKp = 0.0;
    bool baselineSeen = false;
    for (int k = 0; k < 400 * 50 && !baselineSeen; ++k) {
        if (k % 50 == 0) {  // 每秒推进一次合成激励
            const double d = dval[phase];
            r = (K * d * 1.0 + T * r) / (1.0 + T);
            s.rudderDeg = d;
            s.rateDegS = r;
            if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
        }
        ap.step(s, 0.02);
        const double kpNow = ap.pd().params().Kp;
        if (k == 0) schedKp = kpNow;  // 调度表增益（10 kn 海况 0 → 1.2）
        // 极点配置目标 Kp = 0.2²·8/0.2 = 1.6，与调度值 1.2 可区分
        // （阈值放宽到 ±0.2：同拍窗口结算可能先钳一步 ±10%）
        if (std::abs(kpNow - 1.6) < 0.2) baselineSeen = true;
    }
    ASSERT_TRUE(baselineSeen) << "极点配置基线未建立（schedKp=" << schedKp << "）";
    // 基线建立后：一个完整载波周期内，实际增益应覆盖 ±escAmp 相对调制
    double mn = 1e9, mx = -1e9;
    for (int k = 0; k < 100; ++k) {  // 2 s = 2 个整周期
        if (k % 50 == 0) {
            const double d = dval[phase];
            r = (K * d * 1.0 + T * r) / (1.0 + T);
            s.rudderDeg = d;
            s.rateDegS = r;
            if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
        }
        ap.step(s, 0.02);
        const double kpNow = ap.pd().params().Kp;
        mn = std::min(mn, kpNow); mx = std::max(mx, kpNow);
    }
    EXPECT_GT(mx - mn, 0.5 * 2.0 * tp.escAmp * 1.6);  // 至少一半理论峰-峰
}

TEST(AutopilotCore, RunEscTuningNoBaselineNoop) {
    // P9 整改：基线未建立时 runEscTuning 不应崩溃也不改增益
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    double Kp0 = ap.pd().params().Kp, Kd0 = ap.pd().params().Kd;
    ap.runEscTuning();  // 基线未建立，应 no-op
    EXPECT_NEAR(ap.pd().params().Kp, Kp0, 1e-9);
    EXPECT_NEAR(ap.pd().params().Kd, Kd0, 1e-9);
}

// P12 整改回归：ESC 拥有增益后，辨识更新不再盲目覆写；
//               K/T 漂移超阈值时才走显式重基线（增益跳到新配置）
// 场景（与探针核对过的确定性时序）：合成辨识驱动 (K=0.2,T=8) → 基线链收敛到
//   placement≈(1.588, 8.30)；t=200s 切换 T=12（K 不变）。RLS 缓慢再收敛期间
//   （t≈270s，T̂≈9.1，较基线估计 +19% < 25% 阈值）增益必须仍钉在旧基线——
//   旧实现此时每秒都被覆写到 placement≈(2.0, 12+)；t≈284s 漂移越阈触发显式
//   重基线，增益跳到新 placement≈(2.12, 12.54) 并保持。
TEST(AutopilotCore, IdentUpdatesDoNotOverwriteEscGains) {
    AutopilotCore ap;
    auto tp = ap.tunerMut().params();
    tp.escFreq = 2.0 * M_PI;   // 1 s 载波：50 拍均值恰好整周期去抖
    tp.windowSec = 10.0;       // 缩短 ESC 窗口加快测试
    ap.tunerMut().setParams(tp);
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    SensorInput s{0, 0, 0, 10, 0};

    // 合成 Nomoto 数据驱动在线辨识：±25° Z 形（8/3 拍保持）
    double K = 0.2, T = 8.0;
    double r = 0.0;
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    auto step1s = [&]() {  // 跑 50 拍（1 s），推进一次合成激励
        const double d = dval[phase];
        r = (K * d * 1.0 + T * r) / (1.0 + T);
        s.rudderDeg = d;
        s.rateDegS = r;
        if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
        for (int k = 0; k < 50; ++k) ap.step(s, 0.02);
    };
    // 50 拍（载波整周期）PD 增益均值 = 去抖后的 ESC 持久增益
    auto gainCenter = [&](double& kpC, double& kdC) {
        kpC = 0.0; kdC = 0.0;
        for (int k = 0; k < 50; ++k) {
            ap.step(s, 0.02);
            kpC += ap.pd().params().Kp;
            kdC += ap.pd().params().Kd;
        }
        kpC /= 50.0; kdC /= 50.0;
    };

    // 阶段1：(K=0.2,T=8) 驱动 200 s，基线链收敛到 placement≈(1.588, 8.30)
    for (int sec = 0; sec < 199; ++sec) step1s();
    double KpB, KdB;
    gainCenter(KpB, KdB);  // 第 200 s
    EXPECT_NEAR(KpB, 1.588, 0.10);
    EXPECT_NEAR(KdB, 8.30, 0.50);

    // 阶段2：切换 T=12（K 不变），RLS 缓慢再收敛
    T = 12.0;
    for (int sec = 0; sec < 70; ++sec) step1s();  // t≈270 s
    // 检查点1：估计已明显移动（T̂>8.6，较基线估计 +13% 以上），但 25% 阈值未到，
    //          增益必须仍钉在旧基线（盲目覆写实现此时已跳到 placement≈(2.0,12+)）
    EXPECT_GT(ap.identifier().T(), 8.6) << "辨识未跟上新对象，测试前提不成立";
    double Kp1, Kd1;
    gainCenter(Kp1, Kd1);
    EXPECT_NEAR(Kp1, KpB, 0.06) << "Kp 被辨识更新覆写";
    EXPECT_NEAR(Kd1, KdB, 0.40) << "Kd 被辨识更新覆写";

    // 阶段3：继续再收敛，t≈284 s 漂移越阈 → 显式重基线
    for (int sec = 0; sec < 118; ++sec) step1s();  // t≈389 s
    double Kp2, Kd2;
    gainCenter(Kp2, Kd2);
    // 增益必须已跳到新 placement≈(2.12, 12.54)（显式重基线路径）
    EXPECT_NEAR(Kp2, 2.12, 0.15);
    EXPECT_NEAR(Kd2, 12.54, 0.80);
    EXPECT_GT(Kd2 - KdB, 3.0) << "未发生显式重基线";
}

// P12 整改回归：稳定门回退路径保留——ESC 更新被四条件检查拒绝时，
//               增益必须回到极点配置基线（而不是停在 ESC 试出界的位置）
// 时序：基线收敛后先注入与 Kp 载波负相关的成本（e 恒正，不触发振荡检测），
//   ESC 更新被接受、Kp 上移；再改注零均值振荡误差（峰-峰 12° > oscMax 5°），
//   窗口结算被稳定门拒绝 → revert 回基线。
TEST(AutopilotCore, EscFallbackRestoresBaselineOnOscillation) {
    AutopilotCore ap;
    auto tp = ap.tunerMut().params();
    tp.escFreq = 2.0 * M_PI;   // 1 s 载波（50 拍整周期）
    tp.windowSec = 10.0;       // 500 拍窗口
    ap.tunerMut().setParams(tp);
    ap.setMode(Mode::AUTO_HEADING);
    ap.setHeadingRef(0.0);
    SensorInput s{0, 0, 0, 10, 0};

    double K = 0.2, T = 8.0;
    double r = 0.0;
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    int k0 = -1;              // 基线建立拍（载波相位原点）
    double schedKp = 0.0;
    long kGlobal = 0;
    // 跑 50 拍（1 s）；eFn 为空则 heading=0（e=0），否则按拍注入航向误差
    auto run1s = [&](double (*eFn)(long, int)) {
        const double d = dval[phase];
        r = (K * d * 1.0 + T * r) / (1.0 + T);
        s.rudderDeg = d;
        s.rateDegS = r;
        if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
        for (int k = 0; k < 50; ++k, ++kGlobal) {
            if (eFn) s.headingDeg = -eFn(kGlobal, k0);  // e = -heading（ref=0）
            ap.step(s, 0.02);
            if (kGlobal == 0) schedKp = ap.pd().params().Kp;
            if (k0 < 0 && std::abs(ap.pd().params().Kp - schedKp) > 0.01 * schedKp)
                k0 = kGlobal;  // 首次极点配置接管的拍
        }
    };
    auto gainCenter = [&](double& kpC, double& kdC) {
        kpC = 0.0; kdC = 0.0;
        for (int k = 0; k < 50; ++k, ++kGlobal) {
            ap.step(s, 0.02);
            kpC += ap.pd().params().Kp;
            kdC += ap.pd().params().Kd;
        }
        kpC /= 50.0; kdC /= 50.0;
    };

    // 阶段1：驱动 200 s 收敛基线（e=0）
    for (int sec = 0; sec < 199; ++sec) run1s(nullptr);
    ASSERT_GE(k0, 0) << "极点配置基线未建立";
    double KpB, KdB;
    gainCenter(KpB, KdB);
    EXPECT_NEAR(KpB, 1.588, 0.10);
    EXPECT_NEAR(KdB, 8.30, 0.50);

    // 阶段2：注入 e = 5-3·sin(载波相位)（恒正 → 振荡检测无过零，幅值保持 0），
    //        成本与 Kp 载波负相关 → 梯度使 Kp 上移，更新被接受
    for (int sec = 0; sec < 25; ++sec) {
        run1s([](long k, int base) {
            return 5.0 - 3.0 * std::sin(2.0 * M_PI * static_cast<double>(k - base) / 50.0);
        });
    }
    double KpUp, KdUp;
    gainCenter(KpUp, KdUp);
    EXPECT_GT(KpUp, KpB * 1.04) << "ESC 未能在稳定门放行时上调 Kp";

    // 阶段3：改注 e = 6·sin(载波相位)（零均值过零振荡，峰-峰 12° > oscMax 5°），
    //        窗口结算被稳定门拒绝 → revert 回极点配置基线
    for (int sec = 0; sec < 25; ++sec) {
        run1s([](long k, int base) {
            return 6.0 * std::sin(2.0 * M_PI * static_cast<double>(k - base) / 50.0);
        });
    }
    double KpR, KdR;
    gainCenter(KpR, KdR);
    EXPECT_NEAR(KpR, KpB, 0.03) << "回退未恢复 Kp 基线";
    EXPECT_NEAR(KdR, KdB, 0.20) << "回退未恢复 Kd 基线";
}

// P12 整改回归：振荡检测在参考机动期间不喂入（测量无效），参考稳定后恢复
TEST(AutopilotCore, OscDetectorStarvedDuringReferenceMotion) {
    AutopilotCore ap;
    ap.setMode(Mode::AUTO_HEADING);
    SensorInput s{0, 0, 0, 10, 0};
    // 参考每 15 s 翻转（--esc Z 形节拍），航向误差 ±8° 摆动：
    // 若检测器照常喂入，幅值会顶到 16° 并锁死稳定门
    double maxAmp = 0.0;
    for (int k = 0; k < 60 * 50; ++k) {  // 60 s
        ap.setHeadingRef((k / 750) % 2 == 0 ? 40.0 : -40.0);
        s.headingDeg = 8.0 * std::sin(2.0 * M_PI * k / 250.0);
        ap.step(s, 0.02);
        maxAmp = std::max(maxAmp, ap.tuner().oscDetector().amplitude());
    }
    EXPECT_DOUBLE_EQ(maxAmp, 0.0);  // 参考机动期间检测器不喂入
    // 参考稳定 30 s 后恢复喂入：±8° 摆动的峰-峰幅值 ≈16°
    ap.setHeadingRef(0.0);
    for (int k = 0; k < 45 * 50; ++k) {
        s.headingDeg = 8.0 * std::sin(2.0 * M_PI * k / 250.0);
        ap.step(s, 0.02);
    }
    EXPECT_GT(ap.tuner().oscDetector().amplitude(), 8.0);
}

// P13 整改回归：机动 HEADING 段必须应用增益调度（此前直接 pd_.update，
// PD 全程停留在默认增益 Kp=1/Kd=0.5，慢船机动闭环形同虚设）
TEST(AutopilotCore, ManeuverHeadingLegAppliesScheduledGains) {
    AutopilotCore ap;
    // 安装具有辨识度的调度表增益（区别于默认值 1.0/0.5）
    ap.scheduleMut().setTable({{0.0, 100.0, 0, 7.25, 33.5}});
    // 单段 HEADING 保持机动（trigger=NONE，永不推进，便于观察稳态增益）
    Leg hold;
    hold.mode = Leg::Mode::HEADING;
    hold.target = 20.0;
    hold.trigger = Leg::Trigger::NONE;
    hold.threshold = 0.0;
    hold.next = -1;
    ap.startManeuver({hold});
    SensorInput s{0, 0, 0, 10, 0};
    ap.step(s, 0.02);
    EXPECT_DOUBLE_EQ(ap.pd().params().Kp, 7.25);
    EXPECT_DOUBLE_EQ(ap.pd().params().Kd, 33.5);
}

// P14 回归：自动 Z 形辨识试验——机动期间辨识器被喂入，收官自动辨识并应用极点配置增益
TEST(AutopilotCore, IdentTrialIdentifiesAndAppliesGains) {
    AutopilotCore ap;
    ap.scheduleMut().setTable({{0.0, 100.0, 0, 1.0, 0.5}});  // 未标定冷启动
    auto tp = ap.tunerMut().params(); tp.highSpeedDeactivateKn = -1.0;  // 无 ESC 载波
    ap.tunerMut().setParams(tp);
    ap.startIdentifyTrial(20.0, 20.0, 4);
    EXPECT_EQ(ap.mode(), Mode::AUTO_MANEUVER);
    EXPECT_EQ(ap.identTrialState(), IdentTrialState::RUNNING);
    // 用 K=0.2/T=8 的 Nomoto 模型当船（与 sim/nomoto_plant.hpp 相同方程）
    double r = 0.0, psi = 0.0, deltaAct = 0.0;
    for (int k = 0; k < 300 * 50 && ap.identTrialState() == IdentTrialState::RUNNING; ++k) {
        SensorInput s{psi, r, deltaAct, 10.0, 0};
        const double cmd = ap.step(s, 0.02);
        double ddot = (cmd - deltaAct) / 0.3;
        ddot = std::clamp(ddot, -10.0, 10.0);
        deltaAct = std::clamp(deltaAct + ddot * 0.02, -35.0, 35.0);
        r += (0.2 * deltaAct - r) / 8.0 * 0.02;
        psi += r * 0.02;
    }
    ASSERT_EQ(ap.identTrialState(), IdentTrialState::OK);
    EXPECT_TRUE(ap.identifier().valid());
    EXPECT_NEAR(ap.identifier().K(), 0.2, 0.05);
    EXPECT_NEAR(ap.identifier().T(), 8.0, 2.0);
    // 试验后增益 = 极点配置 (ζ=0.85, ωn=0.2)：Kp=ωn²T/K=1.6，Kd=(2ζωnT−1)/K=8.6
    EXPECT_NEAR(ap.pd().params().Kp, 1.6, 0.8);
    EXPECT_NEAR(ap.pd().params().Kd, 8.6, 4.0);
    EXPECT_EQ(ap.mode(), Mode::AUTO_HEADING);
}
