// test_auto_tuner.cpp — 自动优化器（对应开发文档 §3.10 + P2/P4 整改回归）
#include "auto_tuner.hpp"
#include "pd_controller.hpp"
#include <gtest/gtest.h>
using namespace ar;

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

// P2 整改：ESC 标准 HPF+解调+LPF 结构不发散
// P11 整改：改为真实每拍路径（escDither 注入 + accumulate 因果解调），
//           并逐窗校验更新幅值约束，而非仅断言有界
TEST(AutoTuner, EscStepDoesNotDiverge) {
    AutoTuner::Params p;
    p.dt = 0.02; p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 5.0;  // 载波 5 s 周期，每窗 2 个整周期
    AutoTuner t(p);
    t.enable(10.0);
    t.setBaseline(1.6, 8.6);
    double Kp = 1.6, Kd = 8.6;
    // 累积 10 个窗口的样本并调用 escStep
    for (int w = 0; w < 10; ++w) {
        for (int k = 0; k < 500; ++k) {  // 10 s 窗口 = 500 拍
            double KpA = Kp, KdA = Kd;
            t.escDither(KpA, KdA);  // 扰动注入
            // 载波幅值约束：实际增益不超出 ±escAmp 调制
            EXPECT_NEAR(KpA, Kp, p.escAmp * Kp + 1e-9);
            t.accumulate(1.0 + 0.1 * k, 5.0);  // 误差渐增
        }
        const double KpPrev = Kp, KdPrev = Kd;
        t.escStep(Kp, Kd);
        EXPECT_TRUE(std::isfinite(Kp));
        EXPECT_TRUE(std::isfinite(Kd));
        // 单窗更新幅值受 max(|old|·maxUpdateFrac, absMinUpdate) 约束
        EXPECT_LE(std::abs(Kp - KpPrev),
                  std::abs(KpPrev) * p.maxUpdateFrac + p.absMinUpdate + 1e-12);
        EXPECT_LE(std::abs(Kd - KdPrev),
                  std::abs(KdPrev) * p.maxUpdateFrac + p.absMinUpdate + 1e-12);
    }
}

// P11 整改：escDither 载波注入单元测试
TEST(AutoTuner, EscDitherModulatesGains) {
    AutoTuner::Params p;
    p.dt = 0.02; p.escFreq = 2.0 * M_PI;  // 1 s 载波周期
    AutoTuner t(p);
    // 未启用：增益原样返回
    double Kp = 1.0, Kd = 2.0;
    t.escDither(Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 1.0);
    EXPECT_DOUBLE_EQ(Kd, 2.0);
    // 启用后：完整周期内 Kp 应覆盖 ±escAmp 相对调制
    t.enable(10.0);
    double mn = 1e9, mx = -1e9;
    for (int k = 0; k < 100; ++k) {  // 2 s = 2 个整周期
        double a = 1.0, b = 2.0;
        t.escDither(a, b);
        mn = std::min(mn, a); mx = std::max(mx, a);
        EXPECT_NEAR(b, 2.0, p.escAmp * 2.0 + 1e-9);  // Kd 同样在 ±escAmp 内
    }
    EXPECT_NEAR(mx - mn, 2.0 * p.escAmp, 0.02);
}

// P11 整改：端到端证明调参器真的在调——Nomoto 对象 + 偏低 Kp，
//           若干窗口后 Kp 应向增大（性能改善）方向移动且窗口成本下降
TEST(AutoTuner, EscTunesKpTowardOptimum) {
    AutoTuner::Params p;
    p.dt = 0.02; p.windowSec = 60.0;
    p.escFreq = 2.0 * M_PI / 60.0;  // 每窗 1 个载波周期（慢于闭环响应，拟稳态成立）
    p.escAmp = 0.10; p.escHPFtau = 30.0; p.escLPFtau = 10.0; p.escStep = 0.02;
    AutoTuner t(p);
    t.enable(10.0);
    double Kp = 0.15, Kd = 0.5;  // Kp 明显偏低：增大 Kp 可减小跟踪误差
    t.setBaseline(Kp, Kd);
    PdController::Params pp;
    pp.Kp = Kp; pp.Kd = Kd; pp.dt = 0.02; pp.deadzone = 0.0;
    pp.ddMax = 100.0; pp.quant = 0.01;
    PdController pd(pp);
    const double K = 0.5, T = 2.0, dt = 0.02;
    double heading = 0.0, rate = 0.0;
    double Jfirst = 0.0, Jlast = 0.0;
    const int winBeats = static_cast<int>(p.windowSec / dt);
    for (int w = 0; w < 8; ++w) {
        for (int k = 0; k < winBeats; ++k) {
            const double tt = k * dt;
            const double ref = 20.0 * std::sin(2.0 * M_PI * tt / 30.0);  // 窗口整数倍周期
            double KpA = Kp, KdA = Kd;
            t.escDither(KpA, KdA);
            pd.setGains(KpA, KdA);
            const double delta = pd.update(ref, heading);
            rate = (K * delta * dt + T * rate) / (T + dt);  // Nomoto 一阶：T·ṙ+r=K·δ
            heading += rate * dt;
            t.accumulate(angleDiffDeg(ref, heading), delta);
        }
        const double J = t.computeJ();
        if (w == 0) Jfirst = J;
        Jlast = J;
        t.escStep(Kp, Kd);
    }
    EXPECT_GT(Kp, 0.15 + 0.03);  // 向改善方向（增大）移动
    EXPECT_LT(Jlast, Jfirst);    // 窗口成本下降
}

// P11 整改：Kd 被钳到 0 后 clampUpdate 的绝对下限允许 ESC 继续调整
TEST(AutoTuner, EscUpdateUnsticksFromZeroKd) {
    AutoTuner::Params p;
    p.dt = 0.02; p.windowSec = 10.0;
    p.escFreq = 2.0 * M_PI / 2.5;  // 2.5 s 载波周期，每窗 4 个整周期
    AutoTuner t(p);
    t.enable(10.0);
    t.setBaseline(1.0, 0.0);
    double Kp = 1.0, Kd = 0.0;  // Kd=0：旧 clampUpdate 的 maxDelta=|0|·frac=0 会永久锁死
    // 注入与 Kd 载波（cos）负相关的成本序列 → 梯度指向增大 Kd
    for (int w = 0; w < 3; ++w) {
        for (int k = 0; k < 500; ++k) {  // 10 s 窗口
            const double phase = std::fmod(k * p.escFreq * p.dt, 2.0 * M_PI);
            t.accumulate(5.0 - 3.0 * std::cos(phase), 0.0);
            double a = Kp, b = Kd;
            t.escDither(a, b);
        }
        t.escStep(Kp, Kd);
        EXPECT_TRUE(std::isfinite(Kd));
    }
    EXPECT_GT(Kd, 0.0);  // 从 0 解锁
    // 且每窗移动不超过绝对下限（梯度大时钳到 absMinUpdate）
    EXPECT_LE(Kd, 3.0 * p.absMinUpdate + 1e-12);
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

TEST(AutoTuner, RevertRestoresBaseline) {
    AutoTuner t;
    t.setBaseline(1.6, 8.6);
    double Kp = 2.0, Kd = 10.0;
    t.revert(Kp, Kd);
    EXPECT_DOUBLE_EQ(Kp, 1.6);
    EXPECT_DOUBLE_EQ(Kd, 8.6);
}
