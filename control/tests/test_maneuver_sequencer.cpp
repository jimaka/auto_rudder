// test_maneuver_sequencer.cpp — 机动序列框架（对应开发文档 §4 + P7/P10 整改回归）
#include "maneuver_sequencer.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <set>
#include <vector>
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

TEST(ManeuverSequencer, CloverleafLegsAreGeometricallyConsistent) {
    // 几何整改回归：四个转向段同向，直航段目标 = 前一段出转航向，
    // 总转向 4×270° = 1080°，净航向回 ψ0（四叶闭合）
    for (double psi0 : {0.0, 40.0, -120.0}) {
        auto legs = ManeuverSequencer::cloverleaf(psi0, 30.0, 20.0);
        ASSERT_EQ(legs.size(), 8u);
        double hdg = psi0;      // 理想船期望航向（无超调）
        double totalTurn = 0.0;
        for (int i = 0; i < 8; ++i) {
            const Leg& l = legs[i];
            EXPECT_EQ(l.next, (i < 7) ? (i + 1) : -1) << "leg " << i;
            if (l.mode == Leg::Mode::RUDDER) {
                EXPECT_GT(l.target, 0.0) << "turn leg " << i << " 四段转向必须同向(+)";
                const double dpsi = (l.target > 0 ? 1.0 : -1.0) * l.threshold;
                hdg = normalizeAngleDeg(hdg + dpsi);
                totalTurn += dpsi;
            } else {
                // 直航段目标航向必须等于上一段出转后的实际航向
                EXPECT_NEAR(angleDiffDeg(l.target, hdg), 0.0, 1e-9)
                    << "straight leg " << i << " psi0=" << psi0;
            }
        }
        EXPECT_NEAR(totalTurn, 1080.0, 1e-9) << "psi0=" << psi0;
        EXPECT_NEAR(angleDiffDeg(hdg, psi0), 0.0, 1e-9) << "psi0=" << psi0;
        // 四个直航目标互不相同（四叶指向四个不同方向）
        std::set<int> targets;
        for (int i = 1; i < 8; i += 2)
            targets.insert(static_cast<int>(std::lround(legs[i].target)));
        EXPECT_EQ(targets.size(), 4u) << "psi0=" << psi0;
    }
    // dMax 取负：整体镜像为左转四叶，几何同样自洽
    auto legsL = ManeuverSequencer::cloverleaf(10.0, 30.0, -20.0);
    double hdg = 10.0, totalTurn = 0.0;
    for (const Leg& l : legsL) {
        if (l.mode == Leg::Mode::RUDDER) {
            EXPECT_LT(l.target, 0.0);
            const double dpsi = (l.target > 0 ? 1.0 : -1.0) * l.threshold;
            hdg = normalizeAngleDeg(hdg + dpsi);
            totalTurn += dpsi;
        } else {
            EXPECT_NEAR(angleDiffDeg(l.target, hdg), 0.0, 1e-9);
        }
    }
    EXPECT_NEAR(totalTurn, -1080.0, 1e-9);
    EXPECT_NEAR(angleDiffDeg(hdg, 10.0), 0.0, 1e-9);
}

TEST(ManeuverSequencer, CloverleafGeometryClosesWithIdealShip) {
    // 几何整改回归：用理想化航向模型走完整序列——
    //   RUDDER 段：按舵角符号以恒速率转向；HEADING 段：一阶趋近目标航向。
    // 验证按段顺序推进、切段时航向与计划出段航向连续、
    // 累计航向变化 ≈ +1080°（四叶）、末段结束后航向回 ψ0。
    const double psi0 = 30.0;
    auto legs = ManeuverSequencer::cloverleaf(psi0, 5.0, 20.0);
    ASSERT_EQ(legs.size(), 8u);
    // 计划进段航向：转向段出口 += sign(δ)·270°，直航段不变
    std::vector<double> planEntry = {psi0};
    for (size_t i = 0; i + 1 < legs.size(); ++i) {
        double exitH = planEntry.back();
        if (legs[i].mode == Leg::Mode::RUDDER)
            exitH += (legs[i].target > 0 ? 1.0 : -1.0) * legs[i].threshold;
        planEntry.push_back(normalizeAngleDeg(exitH));
    }

    ManeuverSequencer s;
    s.setLegs(legs);
    const double dt = 0.1, maxRate = 3.0;
    double hdg = psi0, lastH = psi0, cumHdg = 0.0, rate = 0.0;
    ManeuverSequencer::Cmd cmd{false, 0.0, 0.0};
    std::vector<int> visitOrder = {0};
    for (int k = 0; k < 20000 && !s.done(); ++k) {
        hdg = normalizeAngleDeg(hdg + rate * dt);   // 理想船积分航向
        cumHdg += angleDiffDeg(hdg, lastH);
        lastH = hdg;
        const int prev = s.currentIndex();
        cmd = s.update(hdg, rate, dt);
        if (s.currentIndex() != prev && !s.done()) {  // 切段：航向须与计划进段航向连续
            const int cur = s.currentIndex();
            visitOrder.push_back(cur);
            ASSERT_LT(cur, (int)planEntry.size());
            EXPECT_NEAR(angleDiffDeg(planEntry[cur], hdg), 0.0, 1.5)
                << "enter leg " << cur;
            if (legs[cur].mode == Leg::Mode::HEADING)
                EXPECT_NEAR(angleDiffDeg(legs[cur].target, hdg), 0.0, 1.5)
                    << "straight target mismatch at leg " << cur;
        }
        if (cmd.isHeading) {                        // 一阶趋近目标航向
            const double diff = angleDiffDeg(cmd.heading, hdg);
            rate = std::max(-maxRate, std::min(maxRate, 2.0 * diff));
        } else {                                    // 恒舵角 → 恒转向率
            rate = (cmd.rudder > 0.0) ? maxRate : -maxRate;
        }
    }
    EXPECT_TRUE(s.done());
    EXPECT_EQ(visitOrder, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
    EXPECT_NEAR(cumHdg, 1080.0, 8.0);               // 四叶总航向变化
    EXPECT_NEAR(angleDiffDeg(hdg, psi0), 0.0, 1.5); // 净回 ψ0，图案闭合
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
