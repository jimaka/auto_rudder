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
