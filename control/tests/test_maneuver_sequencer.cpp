// test_maneuver_sequencer.cpp — 机动序列框架（对应开发文档 §4 + P7/P10/P12 整改回归）
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
    EXPECT_NEAR(legs[1].threshold, -180.0, 1e-9);  // P12：反舵段阈值为负，与舵向一致
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
    EXPECT_NEAR(legsL[0].threshold, -180.0, 1e-9);  // P12：左转阈值为负
    EXPECT_NEAR(legsL[1].target, normalizeAngleDeg(10.0 - 180.0), 1e-9);  // -170°
}

TEST(ManeuverSequencer, ZigzagStructure) {
    auto legs = ManeuverSequencer::zigzag(0.0, 10.0, 15.0, 3);
    ASSERT_EQ(legs.size(), 6u);
    for (size_t i = 0; i < legs.size(); ++i) {
        EXPECT_EQ(legs[i].mode, Leg::Mode::RUDDER);
        EXPECT_EQ(legs[i].trigger, Leg::Trigger::HEADING_REACHED);
        // P12：阈值带符号且与本段舵向一致，符号交替
        const double expect = (i % 2 == 0) ? 15.0 : -15.0;
        EXPECT_NEAR(legs[i].threshold, expect, 1e-9) << "leg " << i;
        EXPECT_GT(legs[i].target * legs[i].threshold, 0.0) << "leg " << i << " 阈值符号须与舵向一致";
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

TEST(ManeuverSequencer, ZigzagReturnTargetsPsi0) {
    // P12 整改：returnToStart 保持段目标 = 传入的 psi0（旧实现硬编码 0°）
    auto legs = ManeuverSequencer::zigzag(40.0, 10.0, 15.0, 2, true);
    ASSERT_EQ(legs.size(), 5u);
    EXPECT_EQ(legs.back().mode, Leg::Mode::HEADING);
    EXPECT_NEAR(legs.back().target, 40.0, 1e-9);
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
                const double dpsi = l.threshold;  // P12：阈值带符号，直接即航向增量
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
            EXPECT_LT(l.threshold, 0.0);  // P12：左转段阈值为负，与舵向一致
            const double dpsi = l.threshold;
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
            exitH += legs[i].threshold;  // P12：阈值带符号，直接即计划航向增量
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

TEST(ManeuverSequencer, HeadingReachedIsDirectionAware) {
    // P12 整改回归：反舵段的顺漂预触发。
    // 段1 右舵转 +30°；段2 反舵、阈值 -20°。进入段2 后船因惯性仍沿旧方向
    // 顺漂（cumHeading 继续正向增长，甚至远超 |-20°|），触发器不得切段；
    // 直到船真正反向、累计航向变化过 -20° 才允许切段。
    ManeuverSequencer s;
    std::vector<Leg> legs = {
        {Leg::Mode::RUDDER, +20.0, Leg::Trigger::HEADING_REACHED, +30.0, 1},
        {Leg::Mode::RUDDER, -20.0, Leg::Trigger::HEADING_REACHED, -20.0, 2},
        {Leg::Mode::HEADING, 0.0, Leg::Trigger::NONE, 0.0, -1},
    };
    s.setLegs(legs);
    double hdg = 0.0;
    const double dt = 0.1;
    // 段1：+5°/s 右转，累计 +30° 后切到段2
    for (int k = 0; k < 200 && s.currentIndex() == 0; ++k) step(s, hdg, 5.0, dt);
    ASSERT_EQ(s.currentIndex(), 1);
    const double hdgAtLeg2Entry = hdg;  // ≈ +30°
    // 顺漂阶段：舵已反，航向仍 +5°/s 增长 8 s（累计 +40°。
    // 旧逻辑 |cum|>=|thr| 在顺漂 +20° 时就会误切段）
    for (int k = 0; k < 80; ++k) {
        step(s, hdg, 5.0, dt);
        ASSERT_EQ(s.currentIndex(), 1) << "顺漂阶段不得触发（k=" << k << "）";
    }
    // 船真正反向：-5°/s，从 +40° 回摆，净变化过 -20° 后应切段
    bool advanced = false;
    for (int k = 0; k < 300 && !advanced; ++k) {
        step(s, hdg, -5.0, dt);
        if (s.currentIndex() == 2) {
            advanced = true;
            // 切段点相对段2入口的真实航向变化必须已达 -20°（容差一步 0.5°）
            EXPECT_LE(angleDiffDeg(hdg, hdgAtLeg2Entry), -19.5);
        }
    }
    EXPECT_TRUE(advanced) << "真正反向过阈值后必须切段";
}
