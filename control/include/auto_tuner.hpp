// auto_tuner.hpp
// 控制参数自动优化（对应文档 §3.10）
//   层2 极点配置：Kp = ωn²·T / K,  Kd = (2ζωn·T - 1) / K
//   层3 ESC 性能寻优：每拍正弦扰动注入 → 每拍因果解调（HPF→×载波）→ 窗末 LPF → 积分器
//   §3.10.7 稳定性监控与回退
//   P2 整改：std::deque → 固定环形数组（无堆分配，ECU 友好）；ESC 升级为标准结构
//   P4 整改：新增 OscillationDetector，stabilityCheck 用实测振荡幅值
//   P11 整改：ESC 真正注入扰动——旧实现增益在窗口内恒定且 HPF/LPF 系数用错采样周期
//            （按 dt=0.02s 设计却每 windowSec=60s 才执行一次），梯度被衰减 2~3 个量级，
//            调参结果与初值逐位相同。现改为：每控制拍 escDither() 叠加 a·sin(ωt)，
//            accumulate() 内按拍累积 成本增量×载波（因果解调），窗末 escStep()
//            用按 windowSec 计算的 LPF 平滑梯度并积分更新。
#pragma once
#include "nomoto_identifier.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ar {

// P2 整改：固定容量环形数组（无堆分配）
template <typename T, size_t N>
class RingBuffer {
public:
    void push(const T& v) {
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full()  const { return size_ == N; }
    void clear() { head_ = 0; size_ = 0; }
    // 按写入顺序遍历（最旧→最新）
    T operator[](size_t i) const {
        const size_t start = (size_ < N) ? 0 : head_;
        return buf_[(start + i) % N];
    }
private:
    T    buf_[N] = {};
    size_t head_ = 0;
    size_t size_ = 0;
};

class AutoTuner {
public:
    struct Params {
        // 极点配置目标（按船速分段，见 §3.10.4）
        double zeta = 0.85;
        double wn   = 0.20;        // rad/s
        // ESC 参数（标准结构）
        double escAmp   = 0.05;    // 扰动幅值（占增益比例）
        double escFreq  = 0.05;    // 扰动频率 (rad/s)
        double escStep  = 0.02;    // 积分步长
        double escHPFtau = 5.0;     // 高通时间常数 (s)，去除 J 直流分量
        double escLPFtau = 10.0;    // 低通时间常数 (s)，平滑梯度估计
        // 性能指标权重
        double wU  = 0.01;         // 控制能耗
        double wDu = 0.005;        // 控制变化率能耗
        // 安全约束
        double maxUpdateFrac = 0.10;  // 单次更新幅值 ≤ 10%
        double absMinUpdate  = 1e-3;  // P11：单次更新绝对下限（old=0 时比例上限为 0 会锁死更新）
        double zetaMin  = 0.6;
        double wnMax    = 0.5;        // rad/s
        double oscMax   = 5.0;       // 振荡幅值阈值 (deg)
        // 高速去激活
        double highSpeedDeactivateKn = 15.0;
        // 评估窗口
        double windowSec = 60.0;
        double dt = 0.02;
    };

    AutoTuner() { reset(); }
    explicit AutoTuner(const Params& p) : p_(p) { reset(); }

    void setParams(const Params& p) { p_ = p; reset(); }
    const Params& params() const { return p_; }

    void reset() {
        escPhase_ = 0.0;
        windowErr_.clear();
        windowDelta_.clear();
        windowDDelta_.clear();
        prevDelta_ = 0.0;
        baselineKp_ = 0.0;
        baselineKd_ = 0.0;
        lastJ_ = 0.0;
        gradKp_ = 0.0;
        gradKd_ = 0.0;
        // P11：每拍解调累积状态
        hpfCPrev_ = 0.0;
        hpfY_ = 0.0;
        demodKpSum_ = 0.0;
        demodKdSum_ = 0.0;
        demodCount_ = 0;
        enabled_ = false;
        oscDetector_.reset();
    }

    // 启用/去激活（§3.10.6 触发逻辑）
    void enable(double speedKn) {
        if (speedKn > p_.highSpeedDeactivateKn) { enabled_ = false; return; }
        enabled_ = true;
    }
    void disable() { enabled_ = false; }
    bool enabled() const { return enabled_; }

    // 设置基线增益（来自极点配置或人工标定）
    void setBaseline(double Kp, double Kd) {
        baselineKp_ = Kp;
        baselineKd_ = Kd;
    }

    // 层2：极点配置重算（§3.10.4）
    // P6 整改：K/T 退化时返回 false；Kd clamp 到 ≥ 0
    static bool polePlacement(double K, double T,
                               double zeta, double wn,
                               double& Kp, double& Kd) {
        if (!std::isfinite(K) || !std::isfinite(T)) return false;
        if (std::abs(K) < 1e-6 || std::abs(T) < 1e-6) return false;
        Kp = wn * wn * T / K;
        double kdRaw = (2.0 * zeta * wn * T - 1.0) / K;
        Kd = kdRaw < 0.0 ? 0.0 : kdRaw;  // 负 Kd 会反向放大振荡，clamp
        return true;
    }

    // 累积性能指标样本（每控制周期调用）
    // 输入：航向误差 e (deg)、舵角 δ (deg)
    // P11 整改：启用 ESC 时同步做因果解调累积——每拍计算成本增量 c_k，
    //           经按控制周期 dt 设计的一阶 HPF 去直流后乘以当前扰动载波 sin/cos，
    //           窗末由 escStep() 归一化为梯度估计
    void accumulate(double e, double delta) {
        const double ddelta = delta - prevDelta_;
        const double t = windowErr_.size() * p_.dt;  // 本拍在窗口内的时刻（ITAE 时间权重）
        windowErr_.push(std::abs(e));
        windowDelta_.push(std::abs(delta));
        windowDDelta_.push(std::abs(ddelta));
        prevDelta_ = delta;
        // P4：振荡检测器同步累积原始误差（带符号）
        oscDetector_.update(e, p_.dt);

        if (!enabled_) return;
        // 本拍成本增量（与 computeJ 的被积函数一致）
        const double c = (t * std::abs(e) + p_.wU * std::abs(delta)
                        + p_.wDu * std::abs(ddelta)) * p_.dt;
        // 一阶高通（后向欧拉离散，α 按真实采样周期 dt 计算）去除直流/慢漂
        const double alphaH = p_.escHPFtau / (p_.escHPFtau + p_.dt);
        hpfY_ = alphaH * (hpfY_ + c - hpfCPrev_);
        hpfCPrev_ = c;
        // 因果解调：与 escDither() 本拍施加的载波同相相关
        demodKpSum_ += hpfY_ * std::sin(escPhase_);
        demodKdSum_ += hpfY_ * std::cos(escPhase_);
        ++demodCount_;
    }

    // 计算 ITAE 性能指标 J = ∫(t|e| + wU|δ| + wDu|dδ|)dt
    double computeJ() const {
        if (windowErr_.empty()) return 0.0;
        double J = 0.0;
        double t = 0.0;
        const size_t n = windowErr_.size();
        for (size_t i = 0; i < n; ++i, t += p_.dt) {
            J += (t * windowErr_[i] + p_.wU * windowDelta_[i] + p_.wDu * windowDDelta_[i]) * p_.dt;
        }
        return J;
    }

    // 层3：ESC 扰动注入（每控制周期调用一次）—— P11 整改新增
    //   经典摄动 ESC 要求在测量窗口内把 a·sin(ωt) 叠加到被调参数上，
    //   否则增益恒定，窗末"梯度"没有因果来源。
    //   调用方（AutopilotCore）每拍把基准增益传入，取回叠加载波后的实际增益。
    //   Kp 用 sin 载波、Kd 用 cos 载波（正交，便于双参数同时解调）。
    void escDither(double& Kp, double& Kd) {
        if (!enabled_) return;
        Kp *= 1.0 + p_.escAmp * std::sin(escPhase_);
        Kd *= 1.0 + p_.escAmp * std::cos(escPhase_);
        escPhase_ += p_.escFreq * p_.dt;  // 载波相位按控制周期推进
        if (escPhase_ > 2.0 * PI) escPhase_ -= 2.0 * PI;
    }

    // 层3：ESC 窗口结算（每评估窗口调用一次）
    //   每拍解调累积量 → 归一化梯度 → LPF（按窗口周期取系数）→ 积分器 + 幅值约束
    //   P2 整改：用标准 ESC 替代朴素 dJ·sin 相关
    //   P11 整改：梯度改为来自 accumulate() 的每拍因果解调；旧实现把按 dt 设计的
    //            HPF/LPF 用在每 windowSec 才来一个的样本上，梯度被衰减 2~3 个量级
    void escStep(double& Kp, double& Kd) {
        if (!enabled_) return;

        lastJ_ = computeJ();
        // 解调累积量归一化：ĝ = 2·Σ(y·载波)/(a·N) ≈ ∂J/∂(增益相对摄动)
        double gKp = 0.0, gKd = 0.0;
        if (demodCount_ > 0 && p_.escAmp > 0.0) {
            const double norm = 2.0 / (p_.escAmp * static_cast<double>(demodCount_));
            gKp = demodKpSum_ * norm;
            gKd = demodKdSum_ * norm;
        }
        // LPF 平滑梯度估计：输入每 windowSec 到达一次，系数按 windowSec 计算
        const double alphaL = p_.windowSec / (p_.escLPFtau + p_.windowSec);
        gradKp_ = (1.0 - alphaL) * gradKp_ + alphaL * gKp;
        gradKd_ = (1.0 - alphaL) * gradKd_ + alphaL * gKd;
        // 梯度下降（积分器）+ 更新幅值约束；增益不为负
        double newKp = Kp - p_.escStep * gradKp_;
        double newKd = Kd - p_.escStep * gradKd_;
        newKp = clampUpdate(newKp, Kp);
        newKd = clampUpdate(newKd, Kd);
        Kp = newKp > 0.0 ? newKp : 0.0;
        Kd = newKd > 0.0 ? newKd : 0.0;
        // 窗口结算：清空本窗样本与解调累积，下一窗从零开始（非重叠窗）
        windowErr_.clear();
        windowDelta_.clear();
        windowDDelta_.clear();
        hpfCPrev_ = 0.0;
        hpfY_ = 0.0;
        demodKpSum_ = 0.0;
        demodKdSum_ = 0.0;
        demodCount_ = 0;
    }

    // §3.10.7 稳定性监控：四条件全满足才接受
    // P4 整改：oscAmplitude 由 OscillationDetector 实测，调用方可传 oscDetector().amplitude()
    bool stabilityCheck(double K, double T,
                        double Kp, double Kd,
                        double oscAmplitude) const {
        if (!std::isfinite(K) || !std::isfinite(T)) return false;
        if (std::abs(K) < 1e-6 || std::abs(T) < 1e-6) return false;
        if (Kp <= 0.0) return false;
        const double wn = std::sqrt(Kp * K / T);
        const double zeta = (1.0 + Kd * K) / (2.0 * std::sqrt(T * Kp * K));
        if (!std::isfinite(wn) || !std::isfinite(zeta)) return false;
        if (zeta < p_.zetaMin) return false;
        if (wn > p_.wnMax) return false;
        if (oscAmplitude > p_.oscMax) return false;
        return true;
    }

    // P4：振荡检测器——基于航向误差的峰-峰幅值估计（跨完整周期）
    class OscillationDetector {
    public:
        // P11 整改：同时清空历史峰，避免上一窗口的峰值泄漏进下一窗口
        void reset() {
            maxPos_ = 0.0; minNeg_ = 0.0;
            lastPosPeak_ = 0.0; lastNegPeak_ = 0.0;
            amp_ = 0.0; prevSign_ = 0;
        }
        void update(double e, double /*dt*/) {
            const int sign = (e > 0) ? 1 : ((e < 0) ? -1 : 0);
            if (sign != 0 && sign != prevSign_ && prevSign_ != 0) {
                // 过零：结算
                if (prevSign_ > 0) {
                    // 刚结束正半周期，保存正峰
                    lastPosPeak_ = maxPos_;
                    maxPos_ = e;  // 重置
                } else {
                    // 刚结束负半周期，保存负峰并计算峰-峰
                    lastNegPeak_ = minNeg_;
                    minNeg_ = e;
                    const double pp = lastPosPeak_ - lastNegPeak_;
                    if (pp > amp_) amp_ = pp;           // 取较大者（保守）
                    else amp_ = 0.9 * amp_ + 0.1 * pp;   // 否则平滑衰减
                }
            }
            if (sign >= 0 && e > maxPos_) maxPos_ = e;
            if (sign <= 0 && e < minNeg_) minNeg_ = e;
            if (sign != 0) prevSign_ = sign;
        }
        double amplitude() const { return amp_; }
    private:
        double maxPos_ = 0.0, minNeg_ = 0.0;
        double lastPosPeak_ = 0.0, lastNegPeak_ = 0.0;
        double amp_ = 0.0;
        int    prevSign_ = 0;
    };

    const OscillationDetector& oscDetector() const { return oscDetector_; }

    // 回退：恢复基线
    void revert(double& Kp, double& Kd) const {
        Kp = baselineKp_;
        Kd = baselineKd_;
    }

private:
    double clampUpdate(double newVal, double oldVal) const {
        // P11 整改：old==0 时比例上限为 0 会永久锁死更新（如极点配置把 Kd 钳到 0），
        //           取 max(|old|·frac, absMin)
        const double maxDelta = std::max(std::abs(oldVal) * p_.maxUpdateFrac,
                                         p_.absMinUpdate);
        double d = newVal - oldVal;
        if (d >  maxDelta) d =  maxDelta;
        if (d < -maxDelta) d = -maxDelta;
        return oldVal + d;
    }

    Params p_;
    double escPhase_ = 0.0;
    double prevDelta_ = 0.0;
    double baselineKp_ = 0.0;
    double baselineKd_ = 0.0;
    double lastJ_ = 0.0;
    double gradKp_ = 0.0;
    double gradKd_ = 0.0;
    // P11：每拍因果解调状态（HPF 输出/上一拍成本增量/载波相关累积）
    double hpfCPrev_ = 0.0;
    double hpfY_ = 0.0;
    double demodKpSum_ = 0.0;
    double demodKdSum_ = 0.0;
    uint32_t demodCount_ = 0;
    bool   enabled_ = false;
    // P2 整改：固定容量环形数组（60 s / 0.02 s = 3000 点）
    static constexpr size_t WIN_N = 3000;
    RingBuffer<double, WIN_N> windowErr_;
    RingBuffer<double, WIN_N> windowDelta_;
    RingBuffer<double, WIN_N> windowDDelta_;
    OscillationDetector oscDetector_;

    static constexpr double PI = 3.14159265358979323846;
};

} // namespace ar
