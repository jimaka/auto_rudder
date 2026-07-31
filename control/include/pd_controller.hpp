// pd_controller.hpp
// PD 航向控制器（对应文档 §3.2~§3.7）
//   §3.2 控制律 δ = Kp·e + Kd·de/dt
//   §3.3 输出限幅 / 舵速限幅
//   §3.7 微分低通滤波 / 误差死区 / 输出量化
#pragma once
#include "angle_utils.hpp"

namespace ar {

class PdController {
public:
    struct Params {
        double Kp = 1.0;        // 比例增益
        double Kd = 0.5;        // 微分增益
        double dMax = 35.0;     // 舵角限幅 (deg)
        double ddMax = 10.0;    // 舵速限幅 (deg/s)
        double deadzone = 0.3;  // 误差死区 (deg)
        double tau = 0.2;       // 微分滤波时间常数 (s)
        double quant = 0.1;     // 输出量化步长 (deg)
        double dt = 0.02;       // 控制周期 (s) = 50 Hz
    };

    PdController() = default;
    explicit PdController(const Params& p) { setParams(p); }

    // P11 整改：dt/quant 非法值防护（dt≤0 → 微分除零产生 Inf；quant≤0 → 量化除零产生 NaN）
    void setParams(const Params& p) {
        p_ = p;
        if (p_.dt <= 0.0) p_.dt = kMinDt;
        if (p_.quant <= 0.0) p_.quant = kMinQuant;
        reset();
    }
    const Params& params() const { return p_; }

    // P1 整改：轻量增益/周期设置，不清内部状态（供增益调度每周期调用）
    void setGains(double Kp, double Kd) { p_.Kp = Kp; p_.Kd = Kd; }
    void setDt(double dt) { if (dt > 0.0) p_.dt = dt; }  // P11 整改：拒绝非法 dt

    // 用实际舵角初始化舵速限幅基准（上电/切模式时调用，避免从 0 爬升产生跳变）
    void initBaseline(double rudderAct) { prevOut_ = rudderAct; }

    // 公开限幅工具（供机动序列 RUDDER 段复用，P7 整改）
    // 非 const：更新 prevOut_ 以保证连续调用时舵速限幅基准连续推进
    double saturate(double v) {
        if (v >  p_.dMax) v =  p_.dMax;
        if (v < -p_.dMax) v = -p_.dMax;
        const double maxDelta = p_.ddMax * p_.dt;
        const double dv = v - prevOut_;
        if (dv >  maxDelta) v = prevOut_ + maxDelta;
        if (dv < -maxDelta) v = prevOut_ - maxDelta;
        const double q = quantize(v);
        prevOut_ = q;  // 推进基准
        return q;
    }

    void reset() {
        prevErr_ = 0.0;
        prevDeriv_ = 0.0;
        prevOut_ = 0.0;
        first_ = true;
    }

    // 输入：期望航向、实际航向（度）
    // 输出：目标舵角（度）
    double update(double headingRef, double headingAct) {
        // §3.2 角度归一化的航向误差
        const double e = angleDiffDeg(headingRef, headingAct);

        // §3.7 误差死区
        if (std::abs(e) < p_.deadzone) {
            // 死区内目标 0，但仍经 saturate() 舵速限幅回中——
            // P11 整改：直接跳 0 会绕过 ddMax（如 35°→0° 瞬时跳变）
            prevErr_ = e;
            return saturate(0.0);
        }

        // §3.7 带滤波的微分（一阶低通）
        double derivRaw;
        if (first_) {
            derivRaw = 0.0;
            first_ = false;
        } else {
            derivRaw = (e - prevErr_) / p_.dt;
        }
        const double alpha = 1.0 / (1.0 + p_.tau / p_.dt);
        const double deriv = alpha * derivRaw + (1.0 - alpha) * prevDeriv_;

        // §3.2 PD 控制律
        double out = p_.Kp * e + p_.Kd * deriv;

        // §3.3 舵角限幅
        if (out >  p_.dMax) out =  p_.dMax;
        if (out < -p_.dMax) out = -p_.dMax;

        // §3.3 舵速限幅
        const double maxDelta = p_.ddMax * p_.dt;
        const double delta = out - prevOut_;
        if (delta >  maxDelta) out = prevOut_ + maxDelta;
        if (delta < -maxDelta) out = prevOut_ - maxDelta;

        // §3.7 输出量化
        out = quantize(out);

        prevErr_ = e;
        prevDeriv_ = deriv;
        prevOut_ = out;
        return out;
    }

private:
    double quantize(double v) const {
        return std::round(v / p_.quant) * p_.quant;
    }

    // P11 整改：非法参数钳位下限
    static constexpr double kMinDt    = 1e-3;
    static constexpr double kMinQuant = 1e-3;

    Params p_;
    double prevErr_ = 0.0;
    double prevDeriv_ = 0.0;
    double prevOut_ = 0.0;
    bool   first_ = true;
};

} // namespace ar
