// test_tuner_debug_api.cpp
// 实船调试接口（TunerDebugApi）单元测试：
//   每个功能可单独调用、行为与 AutoTuner 在线路径一致、force* 系列确属旁路。
#include "tuner_debug_api.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace ar;

namespace {

// 典型船参数（与 test_auto_tuner.cpp 惯例一致）
constexpr double K_SHIP = 0.2;
constexpr double T_SHIP = 8.0;

TEST(TunerDebugApi, PolePlacementPassThrough) {
    AutoTuner t;
    TunerDebugApi api(t);
    double kp1, kd1, kp2, kd2;
    EXPECT_TRUE(api.polePlacement(K_SHIP, T_SHIP, 0.85, 0.20, kp1, kd1));
    EXPECT_TRUE(AutoTuner::polePlacement(K_SHIP, T_SHIP, 0.85, 0.20, kp2, kd2));
    EXPECT_DOUBLE_EQ(kp1, kp2);
    EXPECT_DOUBLE_EQ(kd1, kd2);
    // 退化输入同样拒绝
    EXPECT_FALSE(api.polePlacement(0.0, T_SHIP, 0.85, 0.20, kp1, kd1));
}

TEST(TunerDebugApi, StabilityCheckPassThrough) {
    AutoTuner t;
    TunerDebugApi api(t);
    double kp, kd;
    ASSERT_TRUE(AutoTuner::polePlacement(K_SHIP, T_SHIP, 0.85, 0.20, kp, kd));
    EXPECT_TRUE(api.stabilityCheck(K_SHIP, T_SHIP, kp, kd, 0.0));
    // 逐条违反：高振荡 / 高带宽（Kp 拉大）/ 负 Kp
    EXPECT_FALSE(api.stabilityCheck(K_SHIP, T_SHIP, kp, kd, 99.0));
    EXPECT_FALSE(api.stabilityCheck(K_SHIP, T_SHIP, kp * 100.0, kd, 0.0));
    EXPECT_FALSE(api.stabilityCheck(K_SHIP, T_SHIP, -kp, kd, 0.0));
}

TEST(TunerDebugApi, EstablishBaselineThenSnapshotConsistent) {
    AutoTuner t;
    TunerDebugApi api(t);
    ASSERT_TRUE(api.establishBaseline(K_SHIP, T_SHIP));
    double kpExp, kdExp;
    ASSERT_TRUE(AutoTuner::polePlacement(K_SHIP, T_SHIP, 0.85, 0.20, kpExp, kdExp));
    const auto s = api.snapshot();
    EXPECT_TRUE(s.baselineEstablished);
    EXPECT_DOUBLE_EQ(s.curKp, kpExp);
    EXPECT_DOUBLE_EQ(s.curKd, kdExp);
    EXPECT_DOUBLE_EQ(s.baselineKp, kpExp);
    EXPECT_DOUBLE_EQ(s.baseK, K_SHIP);
    EXPECT_DOUBLE_EQ(s.baseT, T_SHIP);
}

TEST(TunerDebugApi, EvaluateEstimateDriftGateSemantics) {
    AutoTuner t;
    TunerDebugApi api(t);
    ASSERT_TRUE(api.establishBaseline(K_SHIP, T_SHIP));
    // 漂移在 25% 阈值内：driftGate=true 拒绝（ESC 自治）
    EXPECT_FALSE(api.evaluateEstimate(K_SHIP * 1.10, T_SHIP * 1.10, 0.0,
                                      /*driftGate=*/true, /*forceRebase=*/false));
    // 漂移超阈值：接受并重基线
    EXPECT_TRUE(api.evaluateEstimate(K_SHIP * 1.40, T_SHIP * 1.40, 0.0,
                                     /*driftGate=*/true, /*forceRebase=*/false));
    const auto s = api.snapshot();
    EXPECT_DOUBLE_EQ(s.baseK, K_SHIP * 1.40);
}

TEST(TunerDebugApi, DitherOnceRequiresEnabledAndAdvancesPhase) {
    AutoTuner t;
    TunerDebugApi api(t);
    double kp = 1.0, kd = 1.0;
    api.ditherOnce(kp, kd);  // enabled_=false：不动
    EXPECT_DOUBLE_EQ(kp, 1.0);
    EXPECT_DOUBLE_EQ(kd, 1.0);

    api.forceEnabled(true);
    const double ph0 = api.snapshot().escPhase;
    api.ditherOnce(kp, kd);  // 相位 0：sin=0, cos=1 → Kp 不变、Kd +5%
    EXPECT_DOUBLE_EQ(kp, 1.0);
    EXPECT_NEAR(kd, 1.05, 1e-12);
    EXPECT_GT(api.snapshot().escPhase, ph0);
}

TEST(TunerDebugApi, AccumulateAndEscStepIndividuallyCallable) {
    AutoTuner t;
    TunerDebugApi api(t);
    ASSERT_TRUE(api.establishBaseline(K_SHIP, T_SHIP));
    api.forceEnabled(true);
    // 单独喂样本（不经过 tick 编排）
    for (int i = 0; i < 100; ++i) api.accumulateOnce(1.0, 2.0, true);
    auto s = api.snapshot();
    EXPECT_EQ(s.windowSamples, 100u);
    EXPECT_EQ(s.demodCount, 100u);
    EXPECT_GT(api.currentJ(), 0.0);
    // 单独做窗末结算：窗口与解调累积清空
    double kp = s.curKp, kd = s.curKd;
    api.escStepOnce(kp, kd);
    s = api.snapshot();
    EXPECT_EQ(s.windowSamples, 0u);
    EXPECT_EQ(s.demodCount, 0u);
}

TEST(TunerDebugApi, ClampUpdateLimitsAndAbsFloor) {
    AutoTuner t;
    TunerDebugApi api(t);
    // 比例上限：old=1.0 → ±0.1
    EXPECT_DOUBLE_EQ(api.clampUpdate(2.0, 1.0), 1.1);
    EXPECT_DOUBLE_EQ(api.clampUpdate(0.0, 1.0), 0.9);
    // 绝对下限：old=0 时仍能走 1e-3（P11 锁死回归）
    EXPECT_DOUBLE_EQ(api.clampUpdate(1.0, 0.0), 1e-3);
}

TEST(TunerDebugApi, OscFeedAndReset) {
    AutoTuner t;
    TunerDebugApi api(t);
    // 喂两个完整正弦周期（幅值 2°，峰-峰 4°）
    for (int i = 0; i < 200; ++i) {
        const double ph = 2.0 * 3.14159265358979 * i / 100.0;
        api.oscFeed(2.0 * std::sin(ph));
    }
    EXPECT_NEAR(api.oscAmplitude(), 4.0, 0.5);
    api.oscReset();
    EXPECT_DOUBLE_EQ(api.oscAmplitude(), 0.0);
}

TEST(TunerDebugApi, RevertRebaseSuspend) {
    AutoTuner t;
    TunerDebugApi api(t);
    ASSERT_TRUE(api.establishBaseline(K_SHIP, T_SHIP));
    api.forceEnabled(true);
    api.forceGradient(0.5, -0.5);
    for (int i = 0; i < 10; ++i) api.accumulateOnce(1.0, 1.0, true);

    api.rebaseEsc();  // 清梯度与窗口，保留基线
    auto s = api.snapshot();
    EXPECT_DOUBLE_EQ(s.gradKp, 0.0);
    EXPECT_EQ(s.windowSamples, 0u);
    EXPECT_TRUE(s.baselineEstablished);

    double kp = 0, kd = 0;
    api.revertToBaseline(kp, kd);
    EXPECT_DOUBLE_EQ(kp, s.baselineKp);
    EXPECT_DOUBLE_EQ(kd, s.baselineKd);

    api.suspend();  // 清测量、留基线
    s = api.snapshot();
    EXPECT_FALSE(s.enabled);
    EXPECT_TRUE(s.baselineEstablished);
    EXPECT_EQ(s.winCounter, 0u);
}

TEST(TunerDebugApi, ForceGainsBypassesGate_DebugOnly) {
    AutoTuner t;
    TunerDebugApi api(t);
    ASSERT_TRUE(api.establishBaseline(K_SHIP, T_SHIP));
    // 这组增益必然不过稳定门（Kp<0），force 路径照样写入——确属旁路
    api.forceGains(-1.0, 999.0);
    const auto s = api.snapshot();
    EXPECT_DOUBLE_EQ(s.curKp, -1.0);
    EXPECT_DOUBLE_EQ(s.curKd, 999.0);
    // 基线未被污染
    double kpExp, kdExp;
    ASSERT_TRUE(AutoTuner::polePlacement(K_SHIP, T_SHIP, 0.85, 0.20, kpExp, kdExp));
    EXPECT_DOUBLE_EQ(s.baselineKp, kpExp);
}

TEST(TunerDebugApi, TickThroughApiMatchesDirectTick) {
    AutoTuner t1, t2;
    TunerDebugApi api(t1);
    TunerInput in;
    in.headingRefDeg = 10.0; in.headingDeg = 9.0; in.rudderDeg = 1.0;
    in.speedKn = 12.0; in.dt = 0.02; in.schedKp = 1.0; in.schedKd = 2.0;
    in.id.excited = true; in.id.valid = true; in.id.K = K_SHIP; in.id.T = T_SHIP;
    for (int i = 0; i < 50; ++i) {
        const GainCommand g1 = api.tick(in);
        const GainCommand g2 = t2.tick(in);
        EXPECT_DOUBLE_EQ(g1.Kp, g2.Kp);
        EXPECT_DOUBLE_EQ(g1.Kd, g2.Kd);
    }
    EXPECT_TRUE(api.snapshot().baselineEstablished);
}

} // namespace
