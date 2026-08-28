// test_auto_tuner.cpp — 自动优化器（对应开发文档 §3.10 + P2/P4 整改回归）
// R1 深化：编排收进模块后，状态机制测试一律经公有接口 tick()/establishBaseline()
//           驱动——测试与调用方（AutopilotCore）跨同一条 seam，无测试专用后门。
#include "auto_tuner.hpp"
#include "pd_controller.hpp"
#include <gtest/gtest.h>
using namespace ar;

// ---- tick 驱动的测试夹具 ----

// 兴奋且物理合理的辨识快照
static IdSnapshot makeSnap(double K, double T) {
    IdSnapshot s; s.excited = true; s.valid = true; s.K = K; s.T = T; return s;
}

// 组装一拍 tick 输入（dt 固定 0.02 s，与生产 50 Hz 约定一致）
static TunerInput mkInput(double ref, double heading, double rudder, double speed,
                          double schedKp, double schedKd, const IdSnapshot& snap) {
    TunerInput in;
    in.headingRefDeg = ref; in.headingDeg = heading; in.rudderDeg = rudder;
    in.speedKn = speed; in.dt = 0.02;
    in.schedKp = schedKp; in.schedKd = schedKd;
    in.id = snap;
    return in;
}

TEST(AutoTuner, PolePlacementProducesTargetZetaWn) {
    double Kp, Kd;
    ASSERT_TRUE(AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.25, Kp, Kd));
    const double K = 0.2, T = 8.0;
    const double wn = std::sqrt(Kp * K / T);
    const double zeta = (1.0 + Kd * K) / (2.0 * std::sqrt(T * Kp * K));
    EXPECT_NEAR(wn, 0.25, 1e-6);
    EXPECT_NEAR(zeta, 0.85, 1e-3);
}

TEST(AutoTuner, PolePlacementRejectsDegenerate) {
    double Kp, Kd;
    EXPECT_FALSE(AutoTuner::polePlacement(0.0, 8.0, 0.85, 0.25, Kp, Kd));
    EXPECT_FALSE(AutoTuner::polePlacement(0.2, 0.0, 0.85, 0.25, Kp, Kd));
}

TEST(AutoTuner, PolePlacementClampsNegativeKd) {
    double Kp, Kd;
    // 小 wn 使 Kd_raw 为负
    ASSERT_TRUE(AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.05, Kp, Kd));
    EXPECT_GE(Kd, 0.0);
}

TEST(AutoTuner, StabilityCheckAcceptsHealthy) {
    AutoTuner t;
    double Kp, Kd;
    AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.25, Kp, Kd);
    EXPECT_TRUE(t.stabilityCheck(0.2, 8.0, Kp, Kd, 0.0));
}

TEST(AutoTuner, StabilityCheckRejectsLowZeta) {
    AutoTuner t;
    // 极小 Kd → 低阻尼
    EXPECT_FALSE(t.stabilityCheck(0.2, 8.0, 1.0, 0.0, 0.0));
}

TEST(AutoTuner, StabilityCheckRejectsHighWn) {
    AutoTuner t;
    double Kp, Kd;
    AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.25, Kp, Kd);
    // 放大 Kp 使 wn 超限
    EXPECT_FALSE(t.stabilityCheck(0.2, 8.0, Kp * 10, Kd, 0.0));
}

// P4 整改：振荡检测器
TEST(AutoTuner, OscillationDetectorMeasuresAmplitude) {
    AutoTuner::OscillationDetector det;
    // 模拟 ±3° 振荡误差
    for (int k = 0; k < 200; ++k) {
        det.update(3.0 * std::sin(2 * M_PI * k / 50), 0.02);
    }
    EXPECT_NEAR(det.amplitude(), 6.0, 1.0);  // 峰-峰 ≈ 6°
}

TEST(AutoTuner, StabilityCheckRejectsOscillation) {
    AutoTuner t;
    double Kp, Kd;
    AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.25, Kp, Kd);
    // 振荡幅值 8° > oscMax 5°
    EXPECT_FALSE(t.stabilityCheck(0.2, 8.0, Kp, Kd, 8.0));
}

// P2 整改：环形数组无堆分配（编译期容量）
TEST(AutoTuner, RingBufferNoHeap) {
    RingBuffer<double, 16> rb;
    for (int i = 0; i < 100; ++i) rb.push(static_cast<double>(i));
    EXPECT_EQ(rb.size(), 16u);  // 满后不增长
    // 最旧应是 84，最新应是 99
    EXPECT_DOUBLE_EQ(rb[0], 84.0);
    EXPECT_DOUBLE_EQ(rb[15], 99.0);
}

// P11 整改：OscillationDetector::reset 清空历史峰，不泄漏进下一窗口
TEST(AutoTuner, OscillationDetectorResetClearsPeaks) {
    AutoTuner::OscillationDetector det;
    // 先制造 ±3° 振荡，积累历史峰
    for (int k = 0; k < 200; ++k) {
        det.update(3.0 * std::sin(2 * M_PI * k / 50), 0.02);
    }
    EXPECT_GT(det.amplitude(), 4.0);
    det.reset();
    EXPECT_DOUBLE_EQ(det.amplitude(), 0.0);
    // 复位后先走负半周再过零：若上次正峰（≈3°）泄漏，幅值会被虚报为 ≈3-(-1)=4°
    for (int k = 0; k < 10; ++k) det.update(-1.0, 0.02);
    det.update(0.5, 0.02);  // 过零结算：lastPosPeak_ 已清零 → pp = 0-(-1)=1
    EXPECT_DOUBLE_EQ(det.amplitude(), 1.0);
}

// ---- 以下为 R1 改写的行为测试（tick/establishBaseline 驱动） ----

// P11 整改：ESC 扰动载波真实叠加到输出增益（原 escDither 单元测试的 tick 版）
TEST(AutoTuner, EscDitherModulatesGains) {
    AutoTuner::Params p;
    p.escFreq = 2.0 * M_PI;  // 1 s 载波周期
    AutoTuner t(p);
    const IdSnapshot neutral;  // 不触发周期评估
    // 基线未建立：调度增益原样直通（无载波）
    GainCommand g = t.tick(mkInput(0, 0, 0, 10, 1.0, 2.0, neutral));
    EXPECT_DOUBLE_EQ(g.Kp, 1.0);
    EXPECT_DOUBLE_EQ(g.Kd, 2.0);
    // 建立基线 (1.6, 8.6)（K=0.2/T=8 的极点配置值）
    ASSERT_TRUE(t.establishBaseline(makeSnap(0.2, 8.0)));
    // 完整载波周期内，输出增益应覆盖 ±escAmp 相对调制
    double mn = 1e9, mx = -1e9;
    for (int k = 0; k < 100; ++k) {  // 2 s = 2 个整周期
        g = t.tick(mkInput(0, 0, 0, 10, 0.0, 0.0, neutral));  // 基线已建立，调度值不被使用
        mn = std::min(mn, g.Kp); mx = std::max(mx, g.Kp);
        EXPECT_NEAR(g.Kd, 8.6, p.escAmp * 8.6 + 1e-9);  // Kd 同样在 ±escAmp 内
    }
    EXPECT_NEAR(mx - mn, 2.0 * p.escAmp * 1.6, 0.02);
}

// P2/P11 整改：ESC 标准 HPF+解调+LPF 结构不发散，且逐窗更新幅值受约束
// （原 EscStepDoesNotDiverge 的 tick 版：断言打到窗口结算后的持久增益上）
TEST(AutoTuner, WindowSettleUpdateIsClamped) {
    AutoTuner::Params p;
    p.windowSec = 10.0;              // 500 拍窗口
    p.escFreq = 2.0 * M_PI / 5.0;    // 载波 5 s 周期，每窗 2 个整周期
    AutoTuner t(p);
    const IdSnapshot snap = makeSnap(0.2, 8.0);
    ASSERT_TRUE(t.establishBaseline(snap));  // 基线 (1.6, 8.6)
    double KpB, KdB;  // 与 establishBaseline 内部同一计算，取精确值供回退断言
    ASSERT_TRUE(AutoTuner::polePlacement(0.2, 8.0, p.zeta, p.wn, KpB, KdB));
    double Kp = KpB, Kd = KdB;
    // 累积 10 个窗口的样本（误差渐增）；逐窗校验：
    //   ESC 更新受 clampUpdate 约束（≤ max(|old|·frac, absMin)），
    //   或被稳定门拒绝时 revert 跳回基线（安全机制，幅值不受梯度钳制）
    for (int w = 0; w < 10; ++w) {
        for (int k = 0; k < 500; ++k) {
            const double e = 1.0 + 0.1 * k;
            t.tick(mkInput(0.0, -e, 5.0, 10.0, 0.0, 0.0, snap));  // e = angleDiffDeg(0, -e)
        }
        double KpNow, KdNow;
        t.currentGains(KpNow, KdNow);
        EXPECT_TRUE(std::isfinite(KpNow));
        EXPECT_TRUE(std::isfinite(KdNow));
        const bool clamped =
            std::abs(KpNow - Kp) <= std::max(std::abs(Kp) * p.maxUpdateFrac, p.absMinUpdate) + 1e-12 &&
            std::abs(KdNow - Kd) <= std::max(std::abs(Kd) * p.maxUpdateFrac, p.absMinUpdate) + 1e-12;
        const bool reverted =
            std::abs(KpNow - KpB) <= 1e-9 && std::abs(KdNow - KdB) <= 1e-9;
        EXPECT_TRUE(clamped || reverted)
            << "Kp " << Kp << "→" << KpNow << ", Kd " << Kd << "→" << KdNow;
        Kp = KpNow; Kd = KdNow;
    }
}

// P11 整改：端到端证明调参器真的在调——Nomoto 对象 + 偏低 Kp，
//           若干窗口后 Kp 应向增大（性能改善）方向移动且窗口成本下降。
//           R1 版：整闭环经 tick 驱动（生产拍序），舵角反馈用上拍 PD 输出。
TEST(AutoTuner, EscTunesKpTowardOptimum) {
    AutoTuner::Params p;
    p.windowSec = 60.0;
    p.escFreq = 2.0 * M_PI / 60.0;  // 每窗 1 个载波周期（慢于闭环响应，拟稳态成立）
    p.escAmp = 0.10; p.escHPFtau = 30.0; p.escLPFtau = 10.0; p.escStep = 0.02;
    AutoTuner t(p);
    // 故意偏低的起点基线 (0.15, 0.5)：快照取 K=40/31, T=150/31 使极点配置恰好产出之
    const IdSnapshot snap = makeSnap(40.0 / 31.0, 150.0 / 31.0);
    ASSERT_TRUE(t.establishBaseline(snap));
    PdController::Params pp;
    pp.Kp = 0.15; pp.Kd = 0.5; pp.dt = 0.02; pp.deadzone = 0.0;
    pp.ddMax = 100.0; pp.quant = 0.01;
    PdController pd(pp);
    const double K = 0.5, T = 2.0, dt = 0.02;
    double heading = 0.0, rate = 0.0, deltaPrev = 0.0;
    double Jfirst = 0.0, Jlast = 0.0;
    const int winBeats = static_cast<int>(p.windowSec / dt);
    for (int w = 0; w < 8; ++w) {
        for (int k = 0; k < winBeats; ++k) {
            const double tt = k * dt;
            const double ref = 20.0 * std::sin(2.0 * M_PI * tt / 30.0);  // 窗口整数倍周期
            // 生产拍序：tick（用上拍舵角反馈）→ 应用增益 → PD → 对象
            GainCommand g = t.tick(mkInput(ref, heading, deltaPrev, 10.0, 0.0, 0.0, snap));
            pd.setGains(g.Kp, g.Kd);
            const double delta = pd.update(ref, heading);
            rate = (K * delta * dt + T * rate) / (T + dt);  // Nomoto 一阶：T·ṙ+r=K·δ
            heading += rate * dt;
            deltaPrev = delta;
            if (k == winBeats - 2) {  // 窗末结算前一拍读 J（结算后窗口清零）
                const double J = t.computeJ();
                if (w == 0) Jfirst = J;
                Jlast = J;
            }
        }
    }
    double KpEnd, KdEnd;
    t.currentGains(KpEnd, KdEnd);
    EXPECT_GT(KpEnd, 0.15 + 0.03);  // 向改善方向（增大）移动
    EXPECT_LT(Jlast, Jfirst);       // 窗口成本下降
}

// P11 整改：Kd 被钳到 0 后 clampUpdate 的绝对下限允许 ESC 继续调整
TEST(AutoTuner, EscUpdateUnsticksFromZeroKd) {
    AutoTuner::Params p;
    p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 2.5;  // 2.5 s 载波周期，每窗 4 个整周期
    AutoTuner t(p);
    // 基线 (1.0, 0.0)：快照 K=2/17, T=50/17 使极点配置恰好产出
    const IdSnapshot snap = makeSnap(2.0 / 17.0, 50.0 / 17.0);
    ASSERT_TRUE(t.establishBaseline(snap));
    // 注入与 Kd 载波（cos）负相关的成本序列 → 梯度指向增大 Kd
    // （phase 与内部载波同步：基线建立后每拍推进 escFreq·dt）
    long beat = 0;
    for (int w = 0; w < 3; ++w) {
        for (int k = 0; k < 500; ++k, ++beat) {
            const double phase = std::fmod(beat * p.escFreq * p.dt, 2.0 * M_PI);
            const double e = 5.0 - 3.0 * std::cos(phase);
            t.tick(mkInput(0.0, -e, 0.0, 10.0, 0.0, 0.0, snap));
        }
        double KpNow, KdNow;
        t.currentGains(KpNow, KdNow);
        EXPECT_TRUE(std::isfinite(KdNow));
    }
    double KpEnd, KdEnd;
    t.currentGains(KpEnd, KdEnd);
    EXPECT_GT(KdEnd, 0.0);  // 从 0 解锁
    // 且每窗移动不超过绝对下限（梯度大时钳到 absMinUpdate）
    EXPECT_LE(KdEnd, 3.0 * p.absMinUpdate + 1e-12);
}

// §3.10.7：窗口结算的回退与接受（原 RevertRestoresBaseline 的行为版）
TEST(AutoTuner, SettleRevertsToBaselineWhenEstimateInvalid) {
    AutoTuner::Params p;
    p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 2.5;
    AutoTuner t(p);
    ASSERT_TRUE(t.establishBaseline(makeSnap(0.2, 8.0)));  // 基线 (1.6, 8.6)
    // 注入与 Kd 载波负相关的成本使梯度非零；快照无效（valid=false）→
    // 窗口结算无法通过稳定门 → revert 回基线
    for (int k = 0; k < 500; ++k) {
        const double phase = std::fmod(k * p.escFreq * p.dt, 2.0 * M_PI);
        const double e = 5.0 - 3.0 * std::cos(phase);
        t.tick(mkInput(0.0, -e, 0.0, 10.0, 0.0, 0.0, IdSnapshot{}));
    }
    double KpNow, KdNow;
    t.currentGains(KpNow, KdNow);
    EXPECT_NEAR(KpNow, 1.6, 1e-9);
    EXPECT_NEAR(KdNow, 8.6, 1e-9);
}

TEST(AutoTuner, SettleAcceptsUpdateWhenEstimateHealthy) {
    AutoTuner::Params p;
    p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 2.5;
    AutoTuner t(p);
    const IdSnapshot snap = makeSnap(0.2, 8.0);
    ASSERT_TRUE(t.establishBaseline(snap));  // 基线 (1.6, 8.6)
    // 同样的激励但快照健康 → 结算被接受：Kd 按 ESC 梯度上移（有界）
    for (int k = 0; k < 500; ++k) {
        const double phase = std::fmod(k * p.escFreq * p.dt, 2.0 * M_PI);
        const double e = 5.0 - 3.0 * std::cos(phase);
        t.tick(mkInput(0.0, -e, 0.0, 10.0, 0.0, 0.0, snap));
    }
    double KpNow, KdNow;
    t.currentGains(KpNow, KdNow);
    EXPECT_GT(KdNow, 8.6);  // cos 负相关成本 → Kd 上移
    EXPECT_LE(KdNow, 8.6 + std::max(8.6 * p.maxUpdateFrac, p.absMinUpdate) + 1e-12);
    EXPECT_TRUE(std::isfinite(KpNow));
}

// P12 整改：显式重基线清零 ESC 积分/解调/窗口样本——旧梯度不得施加到新基线
// （原 RebaseEscClearsIntegratorState 的行为版：重锚经 establishBaseline 触发）
TEST(AutoTuner, RebaselineClearsIntegratorState) {
    AutoTuner::Params p;
    p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 2.5;  // 2.5 s 载波，每窗 4 个整周期
    AutoTuner t(p);
    ASSERT_TRUE(t.establishBaseline(makeSnap(0.2, 8.0)));   // 基线 A (1.6, 8.6)
    // 半个窗口的 cos 负相关成本 → 解调累积非零（未结算）
    for (int k = 0; k < 250; ++k) {
        const double phase = std::fmod(k * p.escFreq * p.dt, 2.0 * M_PI);
        t.tick(mkInput(0.0, -(5.0 - 3.0 * std::cos(phase)), 0.0, 10.0, 0.0, 0.0, IdSnapshot{}));
    }
    // 显式重基线 B：窗口样本与解调累积必须作废
    ASSERT_TRUE(t.establishBaseline(makeSnap(0.5, 3.0)));
    EXPECT_DOUBLE_EQ(t.computeJ(), 0.0);  // 旧窗口成本围绕旧增益测得，不得保留
    double KpB, KdB;  // 与 establishBaseline 内部同一计算，取精确值（避免字面量 ULP 差）
    ASSERT_TRUE(AutoTuner::polePlacement(0.5, 3.0, p.zeta, p.wn, KpB, KdB));
    // 重锚后跑满一个零成本窗口并结算：若旧解调累积残留，结算会再次施加它
    for (int k = 0; k < 500; ++k) {
        t.tick(mkInput(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, IdSnapshot{}));
    }
    double KpNow, KdNow;
    t.currentGains(KpNow, KdNow);
    EXPECT_DOUBLE_EQ(KpNow, KpB);
    EXPECT_DOUBLE_EQ(KdNow, KdB);
}
