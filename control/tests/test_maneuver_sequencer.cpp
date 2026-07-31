// test_maneuver_sequencer.cpp — 机动序列框架（对应开发文档 §4 + P7/P10 整改回归）
#include "maneuver_sequencer.hpp"
#include <gtest/gtest.h>
using namespace ar;

static ManeuverSequencer::Cmd step(ManeuverSequencer& s, double& hdg, double rate, double dt) {
    // 简单积分航向
    hdg += rate * dt;
    while (hdg > 180.0) hdg -= 360.0;
    while (hdg <= -180.0) hdg += 360.0;
    return s.update(hdg, rate, dt);
}

TEST(ManeuverSequencer, EmptyIsDone) {
    ManeuverSequencer s;
    EXPECT_TRUE(s.done());
}

TEST(ManeuverSequencer, WilliamsonStructure) {
    auto legs = ManeuverSequencer::williamson(0.0);
    ASSERT_EQ(legs.size(), 3u);
    EXPECT_EQ(legs[0].mode, Leg::Mode::RUDDER);
    EXPECT_GT(legs[0].target, 0.0);  // +δ_max
    EXPECT_EQ(legs[0].trigger, Leg::Trigger::HEADING_REACHED);
    EXPECT_NEAR(legs[0].threshold, 60.0, 1e-9);
    EXPECT_LT(legs[1].target, 0.0);  // -δ_max
    EXPECT_NEAR(legs[1].threshold, 180.0, 1e-9);
    EXPECT_EQ(legs[2].mode, Leg::Mode::HEADING);
}

TEST(ManeuverSequencer, UTurnUsesRudderTurn) {
    // P7 整改：U-Turn 转向段应为 RUDDER 恒舵角
    auto legs = ManeuverSequencer::uTurn(0.0, true);
    ASSERT_EQ(legs.size(), 2u);
    EXPECT_EQ(legs[0].mode, Leg::Mode::RUDDER);
    EXPECT_GT(legs[0].target, 0.0);  // 右转 +dMax
    EXPECT_EQ(legs[0].trigger, Leg::Trigger::HEADING_REACHED);
    EXPECT_NEAR(legs[0].threshold, 180.0, 1e-9);
    EXPECT_EQ(legs[1].mode, Leg::Mode::HEADING);
    EXPECT_NEAR(legs[1].target, 180.0, 1e-9);

    auto legsL = ManeuverSequencer::uTurn(10.0, false);  // 用非边界航向避免 ±180 归一化歧义
    EXPECT_LT(legsL[0].target, 0.0);  // 左转 -dMax
    EXPECT_NEAR(legsL[1].target, normalizeAngleDeg(10.0 - 180.0), 1e-9);  // -170°
}

TEST(ManeuverSequencer, ZigzagStructure) {
    auto legs = ManeuverSequencer::zigzag(0.0, 10.0, 15.0, 3);
    ASSERT_EQ(legs.size(), 6u);
    for (size_t i = 0; i < legs.size(); ++i) {
        EXPECT_EQ(legs[i].mode, Leg::Mode::RUDDER);
        EXPECT_EQ(legs[i].trigger, Leg::Trigger::HEADING_REACHED);
        EXPECT_NEAR(legs[i].threshold, 15.0, 1e-9);
    }
    EXPECT_GT(legs[0].target, 0.0);
    EXPECT_LT(legs[1].target, 0.0);
}

TEST(ManeuverSequencer, ZigzagReturnToStart) {
    // P7 整改：returnToStart 加一个 HEADING 保持段
    auto legs = ManeuverSequencer::zigzag(0.0, 10.0, 15.0, 2, true);
    ASSERT_EQ(legs.size(), 5u);  // 2*2 + 1
    EXPECT_EQ(legs.back().mode, Leg::Mode::HEADING);
    EXPECT_EQ(legs.back().next, -1);
}

TEST(ManeuverSequencer, CloverleafTurnsUseRudder) {
    // P7 整改：cloverleaf 转向段应为 RUDDER
    auto legs = ManeuverSequencer::cloverleaf(0.0, 30.0, 20.0);
    ASSERT_EQ(legs.size(), 8u);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(legs[2*i].mode, Leg::Mode::RUDDER) << "turn leg " << 2*i;
        EXPECT_NEAR(legs[2*i].threshold, 270.0, 1e-9);
        EXPECT_EQ(legs[2*i+1].mode, Leg::Mode::HEADING) << "straight leg " << 2*i+1;
        EXPECT_EQ(legs[2*i+1].trigger, Leg::Trigger::TIME);
    }
}

TEST(ManeuverSequencer, SearchTurnsUseRudder) {
    // P7 整改：search 转向段应为 RUDDER
    auto legs = ManeuverSequencer::search(0.0, 30.0, 10.0, 3);
    ASSERT_EQ(legs.size(), 6u);  // 3 cycles × 2
    for (int c = 0; c < 3; ++c) {
        EXPECT_EQ(legs[2*c].mode, Leg::Mode::HEADING);      // 直航
        EXPECT_EQ(legs[2*c+1].mode, Leg::Mode::RUDDER);   // 右转
        EXPECT_NEAR(legs[2*c+1].threshold, 90.0, 1e-9);
        EXPECT_GT(legs[2*c+1].target, 0.0);                 // +dMax 右舵
    }
}

TEST(ManeuverSequencer, CirclesStructure) {
    auto legs = ManeuverSequencer::circles(15.0, 360.0);
    ASSERT_EQ(legs.size(), 1u);
    EXPECT_EQ(legs[0].mode, Leg::Mode::RUDDER);
    EXPECT_NEAR(legs[0].target, 15.0, 1e-9);
    EXPECT_EQ(legs[0].trigger, Leg::Trigger::HEADING_REACHED);
    EXPECT_NEAR(legs[0].threshold, 360.0, 1e-9);
}

TEST(ManeuverSequencer, TimeTriggerAdvances) {
    ManeuverSequencer s;
    std::vector<Leg> legs = {
        {Leg::Mode::HEADING, 10.0, Leg::Trigger::TIME, 1.0, 1},
        {Leg::Mode::HEADING, 20.0, Leg::Trigger::NONE, 0.0, -1},
    };
    s.setLegs(legs);
    double hdg = 0.0;
    double dt = 0.1;
    ManeuverSequencer::Cmd cmd;
    for (int k = 0; k < 15 && !s.done(); ++k) {
        cmd = step(s, hdg, 0.0, dt);
    }
    EXPECT_EQ(s.currentIndex(), 1);  // 1 s 后切到段 1
    EXPECT_NEAR(cmd.heading, 20.0, 1e-9);
}

TEST(ManeuverSequencer, HeadingReachedTriggerAdvances) {
    ManeuverSequencer s;
    std::vector<Leg> legs = {
        {Leg::Mode::RUDDER, 20.0, Leg::Trigger::HEADING_REACHED, 90.0, 1},
        {Leg::Mode::HEADING, 0.0, Leg::Trigger::NONE, 0.0, -1},
    };
    s.setLegs(legs);
    double hdg = 0.0;
    double dt = 0.1;
    ManeuverSequencer::Cmd cmd;
    for (int k = 0; k < 200 && !s.done(); ++k) {
        cmd = step(s, hdg, 5.0, dt);  // 5°/s 转向，18 s 转 90°
    }
    EXPECT_EQ(s.currentIndex(), 1);  // 累计 90° 后切段
}
