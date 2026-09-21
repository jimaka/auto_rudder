// test_gain_schedule.cpp — 增益调度（对应开发文档 §4.6 + P5 插值回归）
#include "gain_schedule.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
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
    EXPECT_NEAR(KdAt, 0.5, 1e-6);
    EXPECT_NEAR(KpBelow, KpAbove, 1e-6);  // 跨交界连续
    EXPECT_NEAR(KdBelow, KdAbove, 1e-6);
}

TEST(GainSchedule, InterpolatesInsideSpeedInterval) {
    GainSchedule sched;
    double Kp, Kd;
    // v=10.5 是中点 7 与 14 的正中，增益也应位于两锚点正中。
    sched.query(10.5, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 1.35, 1e-9);
    EXPECT_NEAR(Kd, 0.55, 1e-9);
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
    double KpHigh, KdHigh, Kp3, Kd3;
    sched.query(7.0, 9, KpHigh, KdHigh);  // 上越界 → 按 sea=3
    sched.query(7.0, 3, Kp3, Kd3);
    EXPECT_NEAR(KpHigh, Kp3, 1e-9);
    EXPECT_NEAR(KdHigh, Kd3, 1e-9);

    double KpLow, KdLow, Kp0, Kd0;
    sched.query(7.0, -1, KpLow, KdLow);    // 下越界 → 按 sea=0
    sched.query(7.0, 0, Kp0, Kd0);
    EXPECT_NEAR(KpLow, Kp0, 1e-9);
    EXPECT_NEAR(KdLow, Kd0, 1e-9);
}

TEST(GainSchedule, SpeedAboveMax) {
    GainSchedule sched;
    double Kp, Kd;
    // 30 kn 超过最高段 [18,25)，取末段名义值（中点 21.5 处 Kp=1.0）
    sched.query(30.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 1.0, 1e-9);
    EXPECT_NEAR(Kd, 0.4, 1e-9);
}

TEST(GainSchedule, SpeedClampsFromLastMidpoint) {
    GainSchedule sched;
    double KpAtMid, KdAtMid, KpAtUpperEdge, KdAtUpperEdge;
    sched.query(21.5, 0, KpAtMid, KdAtMid);
    sched.query(25.0, 0, KpAtUpperEdge, KdAtUpperEdge);
    EXPECT_NEAR(KpAtMid, 1.0, 1e-9);
    EXPECT_NEAR(KdAtMid, 0.4, 1e-9);
    EXPECT_NEAR(KpAtUpperEdge, KpAtMid, 1e-9);
    EXPECT_NEAR(KdAtUpperEdge, KdAtMid, 1e-9);
}

TEST(GainSchedule, SpeedBelowMin) {
    GainSchedule sched;
    double Kp, Kd;
    // 0 kn 低于首段中点 → 取首段名义 Kp=2.0
    sched.query(0.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 2.0, 1e-9);
    EXPECT_NEAR(Kd, 0.8, 1e-9);
}

TEST(GainSchedule, NegativeSpeedClampsToFirstGainPoint) {
    GainSchedule sched;
    double Kp, Kd;
    sched.query(-1.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 2.0, 1e-9);  // 取首段
    EXPECT_NEAR(Kd, 0.8, 1e-9);
}

TEST(GainSchedule, EmptyTableUsesFallbackGains) {
    GainSchedule sched;
    sched.setTable({});
    double Kp, Kd;
    sched.query(10.0, 0, Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 1.0);
    EXPECT_DOUBLE_EQ(Kd, 0.5);
}

TEST(GainSchedule, MissingSeaStateUsesFallbackGains) {
    GainSchedule sched;
    sched.setTable({{0.0, 10.0, 1, 4.0, 2.0}});
    double Kp, Kd;
    sched.query(5.0, 0, Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 1.0);
    EXPECT_DOUBLE_EQ(Kd, 0.5);
}

TEST(GainSchedule, CustomTableIsInterpolated) {
    GainSchedule sched;
    sched.setTable({
        {0.0, 10.0, 0, 4.0, 2.0},   // midpoint 5
        {10.0, 20.0, 0, 2.0, 1.0},  // midpoint 15
    });
    double Kp, Kd;
    sched.query(10.0, 0, Kp, Kd);
    EXPECT_NEAR(Kp, 3.0, 1e-9);
    EXPECT_NEAR(Kd, 1.5, 1e-9);
}

TEST(GainSchedule, CustomTableSupportsMoreThanEightSegments) {
    GainSchedule sched;
    std::vector<GainPoint> table;
    for (int i = 0; i < 9; ++i) {
        table.push_back({2.0 * i, 2.0 * i + 2.0, 0,
                         10.0 - i, 5.0 - 0.5 * i});
    }
    sched.setTable(table);

    double Kp, Kd;
    sched.query(17.0, 0, Kp, Kd);  // 第九段中点
    EXPECT_NEAR(Kp, 2.0, 1e-9);
    EXPECT_NEAR(Kd, 1.0, 1e-9);
}

TEST(GainSchedule, NonFiniteSpeedsReturnFiniteGains) {
    GainSchedule sched;
    double Kp, Kd;

    sched.query(-std::numeric_limits<double>::infinity(), 0, Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 2.0);
    EXPECT_DOUBLE_EQ(Kd, 0.8);

    sched.query(std::numeric_limits<double>::infinity(), 0, Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 1.0);
    EXPECT_DOUBLE_EQ(Kd, 0.4);

    sched.query(std::numeric_limits<double>::quiet_NaN(), 0, Kp, Kd);
    EXPECT_TRUE(std::isfinite(Kp));
    EXPECT_TRUE(std::isfinite(Kd));
}
