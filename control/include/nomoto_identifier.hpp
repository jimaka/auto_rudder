// nomoto_identifier.hpp
// Nomoto 一阶模型在线辨识（对应文档 §3.9.4 Z 形试验辨识 / §3.10.3 RLS 在线辨识）
//   模型： T·dr + r = K·δ  →  r_k = K·δ_k - (T/Δt)·(r_k - r_{k-1})
//   回归： φ_k = [δ_k, -(r_k - r_{k-1})/Δt]^T,  y_k = r_k,  θ = [K, T]^T
#pragma once
#include <array>
#include <cmath>

namespace ar {

class NomotoIdentifier {
public:
    struct Params {
        double lambda = 0.98;     // 遗忘因子
        double dt = 0.02;         // 采样周期 (s)
        double deltaMin = 3.0;    // 激励阈值：舵角 (deg)
        double rMin = 0.5;        // 激励阈值：转向率 (deg/s)
        double drMin = 0.2;       // 激励阈值：转向加速度 (deg/s²)，保留过零瞬态以辨识 T
        double p0 = 1.0;         // 协方差初值（归一化后回归量 O(1)，p0 取 1 量级）
        // 回归量归一化尺度（消除 δ 与 dr 量级差导致的 P 矩阵病态）
        double dScale = 50.0;     // 舵角归一化（deg）
        double drScale = 1.0;      // 转向加速度归一化（deg/s²）
    };

    NomotoIdentifier() { reset(); }
    explicit NomotoIdentifier(const Params& p) : p_(p) { reset(); }

    void setParams(const Params& p) { p_ = p; reset(); }
    const Params& params() const { return p_; }

    void reset() {
        theta_[0] = 0.0;  // K
        theta_[1] = 0.0;  // T
        P_[0][0] = p_.p0; P_[0][1] = 0.0;
        P_[1][0] = 0.0;   P_[1][1] = p_.p0;
        prevR_ = 0.0;
        first_ = true;
        excited_ = false;
    }

    // 输入：舵角 δ (deg)、转向率 r (deg/s)
    // 输出：是否本次更新被接受（满足激励条件）
    bool update(double delta, double r) {
        // P6 整改：入口范围校验，超限丢弃该拍（不污染 θ）
        // P11 整改：被拒绝的拍不再写入 prevR_——否则 NaN/毛刺会成为下一拍差分锚点，
        //           dr=(r-prevR_)/dt 被污染后经激励门控进入 φ，永久毒化 θ/P
        if (!std::isfinite(delta) || !std::isfinite(r)) return false;
        if (std::abs(delta) > 35.0 || std::abs(r) > 30.0) return false;

        // §3.10.3 激励条件：舵角足够大，且转向率或转向加速度足够大
        // （保留 r 过零附近的高 dr 瞬态，否则 T 欠定）
        double dr = 0.0;
        if (first_) { first_ = false; }
        else        { dr = (r - prevR_) / p_.dt; }
        excited_ = (std::abs(delta) > p_.deltaMin &&
                    (std::abs(r) > p_.rMin || std::abs(dr) > p_.drMin));
        if (!excited_) {
            prevR_ = r;
            return false;
        }

        // φ = [δ/dScale, -dr/drScale]（归一化，消除量级差导致的病态）
        const double phi[2] = { delta / p_.dScale, -dr / p_.drScale };

        // 增益 K_k = P·φ / (λ + φᵀPφ)
        double Pphi[2] = {
            P_[0][0]*phi[0] + P_[0][1]*phi[1],
            P_[1][0]*phi[0] + P_[1][1]*phi[1]
        };
        const double denom = p_.lambda + phi[0]*Pphi[0] + phi[1]*Pphi[1];
        if (std::abs(denom) < 1e-12) { prevR_ = r; return false; }
        const double gainK[2] = { Pphi[0]/denom, Pphi[1]/denom };

        // 残差 e = y - φᵀθ
        const double y = r;
        const double yhat = phi[0]*theta_[0] + phi[1]*theta_[1];
        const double err = y - yhat;

        // θ ← θ + K·e
        theta_[0] += gainK[0] * err;
        theta_[1] += gainK[1] * err;

        // P ← (1/λ)(P - K·φᵀ·P)，加 P-floor 防止协方差塌陷（保持长期自适应）
        const double invL = 1.0 / p_.lambda;
        const double pFloor = 1e-3;  // 协方差下界
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P_[i][j] = invL * (P_[i][j] - gainK[i]*Pphi[j]);
            }
        }
        // 对角元保底
        if (P_[0][0] < pFloor) P_[0][0] = pFloor;
        if (P_[1][1] < pFloor) P_[1][1] = pFloor;

        prevR_ = r;
        return true;
    }

    // 物理量：r = K·δ - T·dr = θ0·(δ/dScale) + θ1·(-dr/drScale)
    //   → K = θ0 / dScale, T = θ1 / drScale
    double K() const { return theta_[0] / p_.dScale; }
    double T() const { return theta_[1] / p_.drScale; }
    bool   excited() const { return excited_; }

    // P6 整改：物理合理性检查，辨识结果应满足 K∈(0,2]、T∈[1,60]s
    bool   valid() const {
        const double kHat = K(), tHat = T();  // 用物理量（已反归一化）
        return std::isfinite(kHat) && std::isfinite(tHat)
            && kHat > 0.0 && kHat <= 2.0
            && tHat >= 1.0 && tHat <= 60.0;
    }

private:
    Params p_;
    double theta_[2];
    double P_[2][2];
    double prevR_ = 0.0;
    bool   first_ = true;
    bool   excited_ = false;
};

} // namespace ar
