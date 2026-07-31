// test_gain_schedule.cpp — 增益调度（对应开发文档 §4.6 + P5 插值回归）
#include "gain_schedule.hpp"
#include <gtest/gtest.h>
using namespace ar;

TEST(GainSchedule, NominalAtMidpoint) {
    GainSchedule sched;
    double Kp, Kd;
    // [4,10) 段中点 m=7，海况 0 → 名义 Kp=1.5
    sched.query(7.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 1.5, 1e-9);
    EXPECT_NEAR(Kd, 0.6, 1e-9);
}

TEST(GainSchedule, InterpolationContinuityAtBoundary) {
    GainSchedule sched;
    // 中点锚定：段中点处增益应等于名义值，且跨中点连续
    // v=14 是 [7,14] 与 [14,21.5] 区间交界，两侧极限都应为 seg2 名义 Kp=1.2
    double KpAt, KdAt, KpBelow, KpAbove, KdBelow, KdAbove;
    sched.query(14.0, 0, KpAt, KdAt);
    sched.query(14.0 - 1e-9, 0, KpBelow, KdBelow);
    sched.query(14.0 + 1e-9, 0, KpAbove, KdAbove);
    EXPECT_NEAR(KpAt, 1.2, 1e-6);          // 中点处 = 名义值
    EXPECT_NEAR(KpBelow, KpAbove, 1e-6);  // 跨交界连续
    EXPECT_NEAR(KdBelow, KdAbove, 1e-6);
}

TEST(GainSchedule, HighSeaState) {
    GainSchedule sched;
    double Kp, Kd;
    // [4,10) 海况 2 → 名义 Kp=1.0
    sched.query(7.0, 2, Kp, Kd);
    EXPECT_NEAR(Kp, 1.0, 1e-9);
    EXPECT_NEAR(Kd, 0.4, 1e-9);
}

TEST(GainSchedule, SeaStateClamp) {
    GainSchedule sched;
    double Kp, Kd;
    sched.query(7.0, 9, Kp, Kd);  // 越界 → 按 sea=3
    double Kp3, Kd3;
    sched.query(7.0, 3, Kp3, Kd3);
    EXPECT_NEAR(Kp, Kp3, 1e-9);
    EXPECT_NEAR(Kd, Kd3, 1e-9);
}

TEST(GainSchedule, SpeedAboveMax) {
    GainSchedule sched;
    double Kp, Kd;
    // 30 kn 超过最高段 [18,25)，取末段名义值（中点 21.5 处 Kp=1.0）
    sched.query(30.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 1.0, 1e-9);
    EXPECT_NEAR(Kd, 0.4, 1e-9);
}

TEST(GainSchedule, SpeedBelowMin) {
    GainSchedule sched;
    double Kp, Kd;
    // 0 kn 低于首段中点 → 取首段名义 Kp=2.0
    sched.query(0.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 2.0, 1e-9);
    EXPECT_NEAR(Kd, 0.8, 1e-9);
}

TEST(GainSchedule, NoCrashOnNegativeSpeed) {
    GainSchedule sched;
    double Kp, Kd;
    sched.query(-1.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 2.0, 1e-9);  // 取首段
}
