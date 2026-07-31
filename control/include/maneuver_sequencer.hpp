// maneuver_sequencer.hpp
// 机动航段序列框架（对应文档 §4.2 通用机动调度框架）
//   支持航向跟踪 / 固定舵角两种模式
//   触发条件：航向到达 / 时间 / 转向率
//   预置：Circles / Williamson / U-Turn / Zigzag / Cloverleaf / Search
#pragma once
#include "angle_utils.hpp"
#include <vector>
#include <functional>

namespace ar {

struct Leg {
    enum class Mode { HEADING, RUDDER };
    enum class Trigger { HEADING_REACHED, TIME, RATE, NONE };

    Mode     mode = Mode::HEADING;
    double   target = 0.0;       // HEADING: 目标航向(deg) / RUDDER: 目标舵角(deg)
    Trigger  trigger = Trigger::NONE;
    double   threshold = 0.0;    // HEADING_REACHED: 累计航向变化(deg) / TIME: 时长(s) / RATE: 转向率(deg/s)
    int      next = -1;          // 下一段索引，-1 表示结束
};

class ManeuverSequencer {
public:
    void setLegs(const std::vector<Leg>& legs) {
        legs_ = legs;
        idx_ = 0;
        legElapsed_ = 0.0;
        cumHeading_ = 0.0;
        lastHeading_ = 0.0;
        first_ = true;
        done_ = legs.empty();
    }

    bool done() const { return done_; }
    int  currentIndex() const { return idx_; }
    const std::vector<Leg>& legs() const { return legs_; }

    // 输入：实际航向(deg)、实际转向率(deg/s)、dt(s)
    // 输出：当前段指令 [期望航向(deg), 期望舵角(deg)]
    //       mode==HEADING 时返回 (target, NaN) 由 PD 跟踪
    //       mode==RUDDER  时返回 (NaN, target) 直接输出舵角
    struct Cmd { bool isHeading; double heading; double rudder; };

    Cmd update(double headingAct, double rateAct, double dt) {
        if (done_ || idx_ < 0 || idx_ >= (int)legs_.size()) {
            return {false, 0.0, 0.0};
        }

        if (first_) { lastHeading_ = headingAct; first_ = false; }

        // 累计航向变化（带符号最小差）
        const double dh = angleDiffDeg(headingAct, lastHeading_);
        cumHeading_ += dh;
        lastHeading_ = headingAct;
        legElapsed_ += dt;

        const Leg& cur = legs_[idx_];
        bool advance = false;
        switch (cur.trigger) {
            case Leg::Trigger::HEADING_REACHED:
                if (std::abs(cumHeading_) >= std::abs(cur.threshold)) advance = true;
                break;
            case Leg::Trigger::TIME:
                if (legElapsed_ >= cur.threshold) advance = true;
                break;
            case Leg::Trigger::RATE:
                if (std::abs(rateAct) <= cur.threshold) advance = true;
                break;
            case Leg::Trigger::NONE:
                break;
        }

        Cmd cmd;
        if (cur.mode == Leg::Mode::HEADING) {
            cmd.isHeading = true;
            cmd.heading = cur.target;
            cmd.rudder = 0.0;
        } else {
            cmd.isHeading = false;
            cmd.heading = 0.0;
            cmd.rudder = cur.target;
        }

        if (advance) {
            // 切段时清零累计量
            legElapsed_ = 0.0;
            cumHeading_ = 0.0;
            idx_ = cur.next;
            if (idx_ < 0 || idx_ >= (int)legs_.size()) done_ = true;
        }
        return cmd;
    }

    // ===== 预置机动序列 =====

    // §4.4 Williamson Turn：+δ_max 至 Δψ=60° → -δ_max 至 Δψ=180° → 保持 ψ0+180°
    static std::vector<Leg> williamson(double psi0, double dMax = 25.0) {
        return {
            {Leg::Mode::RUDDER,   +dMax,           Leg::Trigger::HEADING_REACHED, 60.0,  1},
            {Leg::Mode::RUDDER,   -dMax,           Leg::Trigger::HEADING_REACHED, 180.0, 2},
            {Leg::Mode::HEADING,  normalizeAngleDeg(psi0 + 180.0), Leg::Trigger::NONE, 0.0, -1},
        };
    }

    // §4.5 U-Turn：P7 整改——转向段用 RUDDER 恒舵角，转满 180° 后保持目标航向
    static std::vector<Leg> uTurn(double psi0, bool right = true, double dMax = 25.0) {
        const double tgt = normalizeAngleDeg(psi0 + (right ? 180.0 : -180.0));
        return {
            {Leg::Mode::RUDDER,   right ? +dMax : -dMax, Leg::Trigger::HEADING_REACHED, 180.0, 1},
            {Leg::Mode::HEADING,  tgt,                  Leg::Trigger::NONE,          0.0,   -1},
        };
    }

    // §4.6 Zigzag：±δ_z 与 ±φ_z 交替 N 次
    // P10 整改：清理未用 cum/psi0；P7 整改：加 returnToStart 可选返回起始航向
    static std::vector<Leg> zigzag(double /*psi0*/, double dz = 10.0, double phiz = 10.0,
                                    int N = 5, bool returnToStart = false) {
        std::vector<Leg> legs;
        bool positive = true;
        const int totalLegs = 2 * N + (returnToStart ? 1 : 0);
        for (int i = 0; i < 2 * N; ++i) {
            Leg l;
            l.mode = Leg::Mode::RUDDER;
            l.target = positive ? +dz : -dz;
            l.trigger = Leg::Trigger::HEADING_REACHED;
            l.threshold = phiz;
            l.next = (i + 1 < totalLegs) ? (i + 1) : -1;
            legs.push_back(l);
            positive = !positive;
        }
        if (returnToStart) {
            Leg hold;
            hold.mode = Leg::Mode::HEADING;
            hold.target = 0.0;  // 由调用方在 setLegs 后按需调整
            hold.trigger = Leg::Trigger::NONE;
            hold.threshold = 0.0;
            hold.next = -1;
            legs.push_back(hold);
        }
        return legs;
    }

    // §4.3 Circles：恒定转向率闭环（这里以 RUDDER 等效恒舵角实现）
    static std::vector<Leg> circles(double dCmd = 15.0, double totalDeg = 360.0) {
        return { {Leg::Mode::RUDDER, dCmd, Leg::Trigger::HEADING_REACHED, totalDeg, -1} };
    }

    // §4.7 Cloverleaf：4×(转向 270° + 直航 t_leg)
    // P7 整改：转向段改 RUDDER 恒舵角 + 显式 270° 航向变化阈值
    static std::vector<Leg> cloverleaf(double psi0, double tLeg = 30.0, double dMax = 20.0) {
        std::vector<Leg> legs;
        for (int i = 0; i < 4; ++i) {
            // 转向段：恒舵角，累计航向变化达 270° 切下一段
            Leg turn;
            turn.mode = Leg::Mode::RUDDER;
            turn.target = (i % 2 == 0) ? +dMax : -dMax;  // 交替转向方向形成四叶
            turn.trigger = Leg::Trigger::HEADING_REACHED;
            turn.threshold = 270.0;
            turn.next = 2 * i + 1;
            legs.push_back(turn);

            // 直航段：保持当前航向 t_leg 秒
            Leg straight;
            straight.mode = Leg::Mode::HEADING;
            straight.target = normalizeAngleDeg(psi0 + 270.0 * (i + 1));
            straight.trigger = Leg::Trigger::TIME;
            straight.threshold = tLeg;
            straight.next = (i < 3) ? (2 * i + 2) : -1;
            legs.push_back(straight);
        }
        return legs;
    }

    // §4.8 Search：扩张方搜，边长递增，每边后右转 90°
    // P7 整改：转向段改 RUDDER 恒舵角 + 显式 90° 航向变化阈值
    static std::vector<Leg> search(double psi0, double t0 = 30.0, double dt = 10.0,
                                    int cycles = 6, double dMax = 20.0) {
        std::vector<Leg> legs;
        double hdg = psi0;
        int idx = 0;
        for (int c = 0; c < cycles; ++c) {
            const double t = t0 + c * dt;
            // 直航
            Leg straight;
            straight.mode = Leg::Mode::HEADING;
            straight.target = hdg;
            straight.trigger = Leg::Trigger::TIME;
            straight.threshold = t;
            straight.next = idx + 1;
            legs.push_back(straight); ++idx;
            // 右转 90°：恒右舵
            hdg = normalizeAngleDeg(hdg + 90.0);
            Leg turn;
            turn.mode = Leg::Mode::RUDDER;
            turn.target = +dMax;
            turn.trigger = Leg::Trigger::HEADING_REACHED;
            turn.threshold = 90.0;
            turn.next = (c < cycles - 1) ? (idx + 1) : -1;
            legs.push_back(turn); ++idx;
        }
        return legs;
    }

private:
    std::vector<Leg> legs_;
    int    idx_ = 0;
    double legElapsed_ = 0.0;
    double cumHeading_ = 0.0;
    double lastHeading_ = 0.0;
    bool   first_ = true;
    bool   done_ = true;
};

} // namespace ar
