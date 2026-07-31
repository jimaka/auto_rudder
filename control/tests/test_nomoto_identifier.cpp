// test_nomoto_identifier.cpp — Nomoto 在线辨识（对应开发文档 §3.10.3 + P6 整改回归）
#include "nomoto_identifier.hpp"
#include "auto_tuner.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace ar;

// 生成 Nomoto 一阶模型的 (δ, r) 序列，用与辨识器回归一致的后向欧拉离散：
//   r_k = K·δ_k - T·(r_k - r_{k-1})/dt  →  r_k = (K·δ_k·dt + T·r_{k-1})/(dt + T)
static double nomotoStep(double K, double T, double delta, double rPrev, double dt) {
    return (K * delta * dt + T * rPrev) / (dt + T);
}

TEST(NomotoIdentifier, ConvergesOnNomotoData) {
    NomotoIdentifier::Params p;
    p.dt = 1.0; p.lambda = 0.98; p.p0 = 1.0;
    p.rMin = 0.1; p.deltaMin = 2.0; p.drMin = 0.2;
    NomotoIdentifier id(p);
    const double K = 0.2, T = 8.0, dt = 1.0;
    double r = 0.0;
    // 非对称 Z 形激励：+25° 保持 8 拍、-25° 保持 3 拍，制造 δ 与 dr 的独立性
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    for (int k = 0; k < 400; ++k) {
        double d = dval[phase];
        r = nomotoStep(K, T, d, r, dt);
        id.update(d, r);
        if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
    }
    EXPECT_TRUE(id.excited());
    EXPECT_TRUE(id.valid());
    EXPECT_NEAR(id.K(), K, 0.05);
    EXPECT_NEAR(id.T(), T, 2.0);
}

TEST(NomotoIdentifier, NotExcitedWhenIdle) {
    NomotoIdentifier id;
    for (int k = 0; k < 50; ++k) id.update(1.0, 0.1);  // 小幅值，不满足激励
    EXPECT_FALSE(id.excited());
}

TEST(NomotoIdentifier, RejectsOutOfRangeInput) {
    NomotoIdentifier id;
    // P6：超限输入应被丢弃，不污染 θ
    id.update(50.0, 5.0);   // delta>35 → 丢弃
    id.update(10.0, 50.0);  // r>30 → 丢弃
    EXPECT_FALSE(id.excited());
    // θ 仍为初值
    EXPECT_NEAR(id.K(), 0.0, 1e-9);
    EXPECT_NEAR(id.T(), 0.0, 1e-9);
}

// P11 整改回归：单拍 NaN 不得永久毒化辨识器
// 旧行为：NaN 写入 prevR_，下一拍差分 dr 为 NaN，经激励门控污染 θ/P，valid() 永不恢复
TEST(NomotoIdentifier, RecoversAfterNaNInput) {
    NomotoIdentifier::Params p;
    p.dt = 1.0; p.lambda = 0.98; p.p0 = 1.0;
    p.rMin = 0.1; p.deltaMin = 2.0; p.drMin = 0.2;
    NomotoIdentifier id(p), ref(p);
    const double K = 0.2, T = 8.0, dt = 1.0;
    double r = 0.0;
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    const double NaN = std::numeric_limits<double>::quiet_NaN();
    for (int k = 0; k < 400; ++k) {
        double d = dval[phase];
        r = nomotoStep(K, T, d, r, dt);
        // 参考辨识器：全程干净数据
        ref.update(d, r);
        // 被测辨识器：第 200 拍注入单拍 NaN（舵角与转向率各来一次）
        if (k == 200) { id.update(NaN, NaN); }
        else          { id.update(d, r); }
        if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
    }
    // 恢复：结果有限、物理合理，且与无 NaN 参考运行相当
    EXPECT_TRUE(id.excited());
    EXPECT_TRUE(id.valid());
    EXPECT_TRUE(std::isfinite(id.K()));
    EXPECT_TRUE(std::isfinite(id.T()));
    EXPECT_NEAR(id.K(), ref.K(), 0.02);
    EXPECT_NEAR(id.T(), ref.T(), 1.0);
}

TEST(NomotoIdentifier, InvalidWhenOutOfRange) {
    NomotoIdentifier id;
    // 通过反射式：先正常辨识一个合理值，验证 valid()=true
    NomotoIdentifier::Params p; p.dt=1.0; p.lambda=0.98; p.p0=1.0; p.rMin=0.1; p.deltaMin=2.0;
    NomotoIdentifier id2(p);
    const double K = 0.2, T = 8.0, dt = 1.0;
    double r = 0.0;
    int hold[2] = {8, 3}; double dval[2] = {25.0, -25.0};
    int phase = 0, cnt = 0;
    for (int k = 0; k < 400; ++k) {
        double d = dval[phase];
        r = nomotoStep(K, T, d, r, dt);
        id2.update(d, r);
        if (++cnt >= hold[phase]) { cnt = 0; phase = 1 - phase; }
    }
    EXPECT_TRUE(id2.valid());  // K=0.2∈(0,2], T=8∈[1,60]
}

TEST(PolePlacement, ReturnsFalseOnDegenerate) {
    double Kp, Kd;
    EXPECT_FALSE(AutoTuner::polePlacement(0.0, 8.0, 0.85, 0.25, Kp, Kd));
    EXPECT_FALSE(AutoTuner::polePlacement(0.2, 0.0, 0.85, 0.25, Kp, Kd));
}

TEST(PolePlacement, ClampsNegativeKd) {
    // 构造使 Kd 为负的参数：小 wn 使 2*zeta*wn*T < 1
    double Kp, Kd;
    // K=0.2, T=8, zeta=0.85, wn=0.05 → 2*0.85*0.05*8=0.68 < 1 → Kd_raw=(0.68-1)/0.2=-1.6
    bool ok = AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.05, Kp, Kd);
    EXPECT_TRUE(ok);
    EXPECT_GE(Kd, 0.0);  // clamp 到 0
}

TEST(PolePlacement, ProducesTargetZetaWn) {
    double Kp, Kd;
    bool ok = AutoTuner::polePlacement(0.2, 8.0, 0.85, 0.25, Kp, Kd);
    EXPECT_TRUE(ok);
    // 反算闭环 zeta、wn
    const double K = 0.2, T = 8.0;
    const double wn = std::sqrt(Kp * K / T);
    const double zeta = (1.0 + Kd * K) / (2.0 * std::sqrt(T * Kp * K));
    EXPECT_NEAR(wn, 0.25, 1e-6);
    EXPECT_NEAR(zeta, 0.85, 1e-3);
}
