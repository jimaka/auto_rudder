// test_pd_controller.cpp — PD 航向控制器（对应开发文档 §3.6 + P1 整改回归）
#include "pd_controller.hpp"
#include <gtest/gtest.h>
using namespace ar;

static PdController::Params defaultParams() {
    PdController::Params p;
    p.Kp = 1.0; p.Kd = 0.5;
    p.dMax = 35.0; p.ddMax = 10.0;
    p.deadzone = 0.3; p.tau = 0.2; p.quant = 0.1; p.dt = 0.02;
    return p;
}

TEST(PdController, CrossZeroError) {
    PdController pd(defaultParams());
    // ref=1, act=359 → e=+2，输出应为正（右舵）
    double out = pd.update(1.0, 359.0);
    EXPECT_GT(out, 0.0);
}

TEST(PdController, DeadzoneOutputsZero) {
    PdController pd(defaultParams());
    double out = pd.update(100.0, 99.8);  // |e|=0.2 < 0.3
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(PdController, RudderSaturation) {
    auto p = defaultParams();
    p.Kp = 10.0; p.ddMax = 2000.0;  // 抬高舵速以观察幅值饱和（单拍可达 35°）
    PdController pd(p);
    double out = pd.update(170.0, 0.0);  // e=170
    EXPECT_NEAR(out, 35.0, 1e-9);  // 限幅到 ±35°
}

TEST(PdController, RateLimit) {
    auto p = defaultParams();
    p.Kp = 10.0; p.ddMax = 10.0; p.dMax = 35.0;
    PdController pd(p);
    double o0 = pd.update(30.0, 0.0);  // 首拍
    // 首拍 deriv=0，out = Kp*e = 300，限幅 35，再受舵速：prevOut=0 → +0.2
    EXPECT_NEAR(o0, 0.2, 1e-9);
    double o1 = pd.update(30.0, 0.0);
    EXPECT_NEAR(o1, 0.4, 1e-9);  // 再 +0.2
    EXPECT_LE(std::abs(o1 - o0), 0.2 + 1e-9);
}

TEST(PdController, FirstBeatNoDerivativeKick) {
    auto p = defaultParams();
    p.Kp = 1.0; p.Kd = 100.0;  // 放大微分系数以暴露首拍冲击
    PdController pd(p);
    double out = pd.update(30.0, 0.0);  // 首拍 deriv=0
    // 首拍无微分尖峰：out = Kp*e = 30，限幅 35，舵速 0.2 → 0.2
    EXPECT_NEAR(out, 0.2, 1e-9);
}

TEST(PdController, Quantization) {
    PdController pd(defaultParams());
    for (int i = 0; i < 10; ++i) {
        double out = pd.update(5.0 + i * 0.01, 0.0);
        double r = std::round(out / 0.1) * 0.1;
        EXPECT_NEAR(out, r, 1e-9);
    }
}

TEST(PdController, LowpassAttenuation) {
    // 叠加 5 Hz 噪声，微分分量应显著衰减（> 80%）
    // 对比有滤波(tau=0.2)与无滤波(tau≈0)两种控制器的微分贡献
    auto pf = defaultParams(); pf.Kp = 0.0; pf.Kd = 1.0; pf.tau = 0.2; pf.ddMax = 1e6; pf.dMax = 1e6;
    auto pu = pf; pu.tau = 1e-6;  // 几乎无滤波
    PdController pdf(pf), pdu(pu);
    double sumF = 0.0, sumU = 0.0;
    for (int k = 0; k < 200; ++k) {
        double e = 1.0 + 0.5 * std::sin(2 * M_PI * 5 * k * pf.dt);  // 5 Hz 噪声
        double of = pdf.update(e, 0.0);   // Kp=0，out 即微分贡献
        double ou = pdu.update(e, 0.0);
        sumF += std::abs(of);
        sumU += std::abs(ou);
    }
    // 滤波后微分贡献应 < 无滤波的 20%（衰减 > 80%）
    EXPECT_LT(sumF, 0.2 * sumU);
}

// P1 整改回归：setGains 不清状态
TEST(PdController, SetGainsPreservesState) {
    auto p = defaultParams();
    p.Kp = 1.0; p.Kd = 0.0; p.ddMax = 1000.0; p.dMax = 100.0;  // 关闭微分与限幅干扰
    PdController pd(p);
    (void)pd.update(10.0, 0.0);  // 建立历史
    // setGains 不应清空 prevErr_，下一拍微分应基于上一拍误差
    pd.setGains(2.0, 0.5);
    (void)pd.update(10.0, 0.0);  // e 恒为 10，deriv 应为 0（e 未变）
    // 改用 Kd>0 + 变化 e 验证状态保留
    pd.reset();
    (void)pd.update(10.0, 0.0);
    pd.setGains(1.0, 1.0);
    double out2 = pd.update(11.0, 0.0);  // e 从 10→11，deriv=(1)/0.02=50（带低通）
    // 状态保留时 prevErr_=10，derivRaw=(11-10)/0.02=50；若被清 first_=true 则 deriv=0
    EXPECT_GT(out2, 1.0 * 11.0);  // 微分项有贡献
}

TEST(PdController, InitBaselinePreventsJump) {
    auto p = defaultParams();
    p.dMax = 35.0; p.ddMax = 10.0;
    PdController pd(p);
    pd.initBaseline(20.0);  // 当前舵角 20°
    double out = pd.update(0.0, 0.0);  // e=0，死区内
    // P11 整改：死区内目标 0 也经舵速限幅——从 20° 每拍最多回中 0.2°，而非瞬跳到 0
    EXPECT_NEAR(out, 19.8, 1e-9);
    // 退出死区时，舵速限幅基于回中后的基准（≈19.8）而非 0
    double out2 = pd.update(30.0, 0.0);
    EXPECT_LE(out2, 20.0 + 0.2 + 1e-9);  // 从 ~20 起最多 +0.2
    EXPECT_GT(out2, 19.0);               // 明确不是从 0 爬升
}

// P11 整改回归：进入死区不得绕过舵速限幅
TEST(PdController, DeadzoneExitRespectsRateLimit) {
    auto p = defaultParams();
    p.Kp = 10.0; p.ddMax = 10.0; p.dMax = 35.0;  // ddMax*dt = 0.2°/拍
    PdController pd(p);
    // 先把输出跑到大舵角（误差 30°，持续至限幅饱和）
    double prev = 0.0;
    for (int k = 0; k < 300; ++k) prev = pd.update(30.0, 0.0);
    EXPECT_NEAR(prev, 35.0, 1e-9);  // 已到正限幅
    // 误差突然进入死区：输出应经舵速限幅逐拍回中，|Δ| ≤ 0.2°/拍
    double out = prev;
    for (int k = 0; k < 300 && out != 0.0; ++k) {
        double next = pd.update(0.1, 0.0);  // |e|=0.1 < deadzone 0.3
        EXPECT_LE(std::abs(next - out), 0.2 + 1e-9)
            << "beat " << k << ": 死区回中跳变超限 " << out << " -> " << next;
        out = next;
    }
    EXPECT_DOUBLE_EQ(out, 0.0);  // 最终回到 0
}

// P11 整改回归：非法 dt / quant 防护
TEST(PdController, SetDtRejectsNonPositive) {
    PdController pd(defaultParams());
    pd.setDt(0.0);
    EXPECT_DOUBLE_EQ(pd.params().dt, 0.02);  // 非法值被忽略
    pd.setDt(-1.0);
    EXPECT_DOUBLE_EQ(pd.params().dt, 0.02);
    pd.setDt(0.01);
    EXPECT_DOUBLE_EQ(pd.params().dt, 0.01);  // 合法值生效
}

TEST(PdController, SetParamsClampsInvalidDtQuant) {
    auto p = defaultParams();
    p.dt = 0.0;    // 旧行为：微分除零 → Inf
    p.quant = 0.0; // 旧行为：量化除零 → NaN
    PdController pd(p);
    EXPECT_GT(pd.params().dt, 0.0);
    EXPECT_GT(pd.params().quant, 0.0);
    double out = pd.update(30.0, 0.0);
    EXPECT_TRUE(std::isfinite(out));
}
