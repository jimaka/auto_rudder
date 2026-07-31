// disturbances.hpp — 仿真扰动注入（对应开发文档 §10.2）
//   恒值风致力矩（等效舵角）+ 0.1 Hz 波浪 + ψ/r 白噪声
#pragma once
#include <cmath>
#include <cstdint>

namespace ar::sim {

// 简易确定性伪随机（避免依赖 <random>，便于跨平台复现）
inline double lcg(uint32_t& s) {
    s = s * 1664525u + 1013904223u;
    return (s >> 8) * (1.0 / 16777216.0) - 0.5;  // [-0.5, 0.5)
}

struct Disturbances {
    double windDeg = 2.0;       // 等效恒值风致力矩（舵角 deg），海况 1 基准幅值
    double waveAmp = 1.0;        // 波浪幅值（等效舵角 deg），海况 1 基准幅值
    double waveFreq = 0.1 * 2 * M_PI;  // 0.1 Hz
    double headingNoiseSigma = 0.2;   // ψ 量测噪声 σ (deg)，海况 1 基准幅值
    double rateNoiseSigma = 0.1;      // r 量测噪声 σ (deg/s)，海况 1 基准幅值
    uint32_t seed = 12345u;

    double wavePhase = 0.0;

    // 海况 → 扰动强度映射（唯一定义处）：
    //   sea 0 = 平静，扰动幅值 ×0（近零扰动）
    //   sea 1 = 基准 ×1（= 历史固定幅值，既有默认运行结果保持可比）
    //   sea 2 = ×2，sea 3 及以上 = ×3
    static double seaFactor(int seaState) {
        if (seaState <= 0) return 0.0;
        if (seaState == 1) return 1.0;
        if (seaState == 2) return 2.0;
        return 3.0;
    }

    // 由仿真器在每次运行前按场景海况调用
    void setSeaState(int seaState) { seaFactor_ = seaFactor(seaState); }

    // 等效扰动舵角（叠加到本船输入）
    double disturbAngle(double t) const {
        return seaFactor_ * (windDeg + waveAmp * std::sin(waveFreq * t + wavePhase));
    }

    // 给量测加噪（注意：无论海况因子如何都推进 LCG，保持随机序列确定性一致）
    double noisyHeading(double psiTrue) {
        return psiTrue + seaFactor_ * headingNoiseSigma * lcg(seed) * 2.0;
    }
    double noisyRate(double rTrue) {
        return rTrue + seaFactor_ * rateNoiseSigma * lcg(seed) * 2.0;
    }

private:
    double seaFactor_ = 1.0;  // 未显式设置时按海况 1 基准，保持历史默认行为
};

} // namespace ar::sim
