// auto_tuner.hpp
// 控制参数自动优化（对应文档 §3.10）
//   层2 极点配置：Kp = ωn²·T / K,  Kd = (2ζωn·T - 1) / K
//   层3 ESC 性能寻优：每拍正弦扰动注入 → 每拍因果解调（HPF→×载波）→ 窗末 LPF → 积分器
//   §3.10.7 稳定性监控与回退
//   P2 整改：std::deque → 固定环形数组（无堆分配，ECU 友好）；ESC 升级为标准结构
//   P4 整改：新增 OscillationDetector，stabilityCheck 用实测振荡幅值
//   P11 整改：ESC 真正注入扰动——旧实现增益在窗口内恒定且 HPF/LPF 系数用错采样周期
//            （按 dt=0.02s 设计却每 windowSec=60s 才执行一次），梯度被衰减 2~3 个量级，
//            调参结果与初值逐位相同。现改为：每控制拍扰动叠加 a·sin(ωt)，
//            每拍累积 成本增量×载波（因果解调），窗末用按 windowSec 计算的 LPF
//            平滑梯度并积分更新。
//   P12 整改：ESC 增益所有权——仅当 K/T 估计相对基线估计漂移超阈值时才显式重基线
//            （增益=新配置 + ESC 积分器重锚）；振荡检测仅在航向参考稳定后喂入。
//   R1 深化：编排权从 AutopilotCore 收回本模块——节拍计数、ESC 窗口、速度门控、
//            增益所有权、漂移重基线、振荡门控全部内收。对外只剩两个动作：
//            tick()（每拍，返回本拍增益）与 establishBaseline()（一次性评估并建基线，
//            供辨识试验收官）；suspend() 为唯一的模式生命周期钩子。
//            P11/P12/P14 的 bug 全部诞生在旧编排层，本模块即为根治。
#pragma once
#include "nomoto_identifier.hpp"
#include "angle_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ar {

// 辨识快照：NomotoIdentifier 当前估计的只读投影（辨识器保持独立模块，
// 由调用方每拍喂数据并组装快照传入，本模块不持有辨识器）
struct IdSnapshot {
    bool   excited = false;  // 最近一次 RLS 拍满足激励条件
    bool   valid   = false;  // 估计物理合理（K∈(0,2]、T∈[1,60]）
    double K = 0.0;
    double T = 0.0;
};

// tick() 每拍输入
struct TunerInput {
    double headingRefDeg;  // 目标航向 (°)（误差与参考稳定度由本模块自算）
    double headingDeg;     // 实际航向 (°)
    double rudderDeg;      // 舵角反馈 (°)
    double speedKn;        // 船速 (kn)（高速去激活门控在内部判定）
    double dt;             // 距上次调用的真实间隔 (s)
    double schedKp;        // 增益调度名义值——基线未建立时直通
    double schedKd;
    IdSnapshot id;         // 辨识快照
};

// tick() 每拍输出：应施加到 PD 控制器的增益（已含 ESC 扰动载波）
struct GainCommand {
    double Kp;
    double Kd;
};

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
        double dt = 0.02;           // 控制周期 (s)：内部系数（HPF/ITAE/载波相位/窗口拍数）的时基
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
        // R1：随迁的编排状态
        baselineEstablished_ = false;
        curKp_ = 0.0;
        curKd_ = 0.0;
        baseK_ = 0.0;
        baseT_ = 0.0;
        winCounter_ = 0;
        prevRef_ = 0.0;
        refAgeSec_ = 0.0;
    }

    // ==================== 对外动作接口（R1 深化后仅 3 个） ====================

    // 每拍：给出本拍应施加的增益。内部依次完成：
    //   速度门控 → 基线未建立时跟随调度值 → 辨识评估（漂移检查+极点配置+稳定门）
    //   → 性能样本累积（含振荡检测门控与 ESC 因果解调）→ ESC 窗口结算（稳定门+回退）
    //   → ESC 扰动注入 → 返回增益
    GainCommand tick(const TunerInput& in) {
        // 速度门控（§3.10.6，R1 内收：门控阈值是 Params，判定不再依赖调用方）
        enabled_ = (in.speedKn <= p_.highSpeedDeactivateKn);

        // 基线未建立：当前增益跟随调度名义值（P11：否则 ESC 增益每拍被调度表覆盖）
        if (!baselineEstablished_) { curKp_ = in.schedKp; curKd_ = in.schedKd; }

        // 周期辨识评估（P6：仅物理合理才配置；P12：漂移超阈值才显式重基线）
        if (enabled_ && in.id.excited && in.id.valid) {
            applyEstimate_(in.id.K, in.id.T, oscDetector_.amplitude(),
                           /*driftGate=*/true, /*forceRebase=*/false);
        }

        // P12 振荡检测门控（R1 随迁）：参考稳定满 kOscRefSettleSec 才喂入——
        // 参考机动期间误差摆动是指令响应而非闭环振荡
        if (in.headingRefDeg != prevRef_) { prevRef_ = in.headingRefDeg; refAgeSec_ = 0.0; }
        else                              { refAgeSec_ += in.dt; }
        const double e = angleDiffDeg(in.headingRefDeg, in.headingDeg);
        accumulate(e, in.rudderDeg, refAgeSec_ >= kOscRefSettleSec);

        // ESC 窗口结算（R1 内收：窗口拍数按 windowSec/dt 换算，计数器自维护）
        const uint32_t winBeats = (p_.dt > 0.0)
            ? static_cast<uint32_t>(std::max(1.0, p_.windowSec / p_.dt + 0.5))
            : 3000u;
        if (++winCounter_ >= winBeats) {
            winCounter_ = 0;
            settleWindow_(in.id);
        }

        // ESC 扰动注入（P11：窗口运行期间实际施加到增益的载波；不持久化）
        double Kp = curKp_, Kd = curKd_;
        if (baselineEstablished_) escDither(Kp, Kd);
        return {Kp, Kd};
    }

    // 一次性评估并建立基线（P14 辨识试验收官；亦可供人工标定注入）。
    // 与 tick() 的周期评估共享同一私有路径；语义差异：
    //   不做漂移检查（试验的明确产出）；振荡测量在机动刚结束时无效，osc 以 0 传入，
    //   由 ζ/ωn 代数条件把关；恒重锚 ESC。
    // 返回：基线是否建立（映射试验 OK/FAILED）
    bool establishBaseline(const IdSnapshot& snap) {
        if (!snap.excited || !snap.valid) return false;
        return applyEstimate_(snap.K, snap.T, 0.0,
                              /*driftGate=*/false, /*forceRebase=*/true);
    }

    // 模式生命周期钩子（P9 的完整版）：切出自动模式时调用——停用并清空测量状态
    // （窗口/解调累积/HPF/梯度），保留基线增益与基线估计。旧 disable() 只翻标志位，
    // 手动操舵期间的旧测量会混进下一窗口，此处根治。
    void suspend() {
        enabled_ = false;
        rebaseEsc();      // 清测量状态（保留 baselineKp_/baselineKd_）
        winCounter_ = 0;
    }

    // ==================== 只读查询 ====================

    bool enabled() const { return enabled_; }
    // R1：基线状态可观测（替代仿真中"增益偏离 1e-9 反推"的迂回断言）
    bool baselineEstablished() const { return baselineEstablished_; }
    // R1：当前增益的只读投影（不含 ESC 载波）——测试与遥测观测增益所有权用
    void currentGains(double& Kp, double& Kd) const { Kp = curKp_; Kd = curKd_; }

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

    // 层2：极点配置重算（§3.10.4）
    // P6 整改：K/T 退化时返回 false；Kd clamp 到 ≥ 0
    // 公式全仓库唯一定义处（仿真场景安装调度表亦应调此处）
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

private:
    // ==================== 内部机制（R1 起为模块私有，经 tick/establishBaseline 驱动） ====================

    // 设置基线增益（来自极点配置或人工标定）
    void setBaseline(double Kp, double Kd) {
        baselineKp_ = Kp;
        baselineKd_ = Kd;
    }

    // R1：评估并应用辨识估计——tick 周期路径与 establishBaseline 一次性路径的共享实现
    // driftGate=true 时执行 P12 漂移检查（漂移在阈值内交给 ESC 连续调参，不动基线）
    // forceRebase=true 时恒重锚；否则仅在"重基线"（已有旧基线）时重锚
    bool applyEstimate_(double K, double T, double oscAmp, bool driftGate, bool forceRebase) {
        if (driftGate && baselineEstablished_
            && std::abs(K - baseK_) <= kReBaselineTol * std::max(std::abs(baseK_), kReBaselineKFloor)
            && std::abs(T - baseT_) <= kReBaselineTol * std::max(std::abs(baseT_), kReBaselineTFloor)) {
            return false;  // P12：漂移在阈值内，ESC 自治
        }
        double Kp2, Kd2;
        if (!polePlacement(K, T, p_.zeta, p_.wn, Kp2, Kd2)) return false;  // P6
        if (!stabilityCheck(K, T, Kp2, Kd2, oscAmp)) return false;         // P4/§3.10.7
        const bool rebaseline = baselineEstablished_;
        setBaseline(Kp2, Kd2);
        baselineEstablished_ = true;
        curKp_ = Kp2; curKd_ = Kd2;
        baseK_ = K; baseT_ = T;  // P12：记录本次基线对应的 K/T 估计（漂移判据基准）
        if (forceRebase || rebaseline) {
            // P12：显式重基线——ESC 积分器清零重锚（旧梯度围绕旧增益旧模型测得，
            //      不得施加到新基线），窗口节拍重对齐
            rebaseEsc();
            winCounter_ = 0;
        }
        return true;
    }

    // ESC 窗口结算（原 AutopilotCore::escTrigger，R1 内收）：
    // 仅在启用且基线已建立时执行；稳定性检查不通过则回退基线
    void settleWindow_(const IdSnapshot& id) {
        if (!enabled_ || !baselineEstablished_) return;
        double Kp = curKp_, Kd = curKd_;
        escStep(Kp, Kd);
        const double Khat = id.valid ? id.K : 0.0;
        const double That = id.valid ? id.T : 0.0;
        if (Khat > 0.0 && That > 0.0 &&
            stabilityCheck(Khat, That, Kp, Kd, oscDetector_.amplitude())) {
            curKp_ = Kp; curKd_ = Kd;  // 接受 ESC 更新（P11：持久化）
            return;
        }
        revert(curKp_, curKd_);  // 回退基线
    }

    // 累积性能指标样本（每控制周期由 tick() 调用）
    // 输入：航向误差 e (deg)、舵角 δ (deg)
    // P11 整改：启用 ESC 时同步做因果解调累积——每拍计算成本增量 c_k，
    //           经按控制周期 dt 设计的一阶 HPF 去直流后乘以当前扰动载波 sin/cos，
    //           窗末由 escStep() 归一化为梯度估计
    // P12 整改：oscEligible=false 时跳过振荡检测喂入（参考机动期间误差摆动是
    //           指令响应而非闭环振荡）；成本累积与 ESC 解调不受影响。
    void accumulate(double e, double delta, bool oscEligible) {
        const double ddelta = delta - prevDelta_;
        const double t = windowErr_.size() * p_.dt;  // 本拍在窗口内的时刻（ITAE 时间权重）
        windowErr_.push(std::abs(e));
        windowDelta_.push(std::abs(delta));
        windowDDelta_.push(std::abs(ddelta));
        prevDelta_ = delta;
        // P4：振荡检测器同步累积原始误差（带符号）
        if (oscEligible) oscDetector_.update(e, p_.dt);

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

    // 层3：ESC 扰动注入（每控制周期一次）—— P11 整改新增
    //   经典摄动 ESC 要求在测量窗口内把 a·sin(ωt) 叠加到被调参数上，
    //   否则增益恒定，窗末"梯度"没有因果来源。
    //   Kp 用 sin 载波、Kd 用 cos 载波（正交，便于双参数同时解调）。
    void escDither(double& Kp, double& Kd) {
        if (!enabled_) return;
        Kp *= 1.0 + p_.escAmp * std::sin(escPhase_);
        Kd *= 1.0 + p_.escAmp * std::cos(escPhase_);
        escPhase_ += p_.escFreq * p_.dt;  // 载波相位按控制周期推进
        if (escPhase_ > 2.0 * PI) escPhase_ -= 2.0 * PI;
    }

    // 层3：ESC 窗口结算（每评估窗口一次）
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

    // 回退：恢复基线
    void revert(double& Kp, double& Kd) const {
        Kp = baselineKp_;
        Kd = baselineKd_;
    }

    // P12 整改：显式重基线（K/T 估计漂移超阈值）时触发——ESC 积分/解调/
    //   窗口样本清零重锚。旧梯度与旧窗口成本是围绕旧增益、旧模型测得的，重锚后
    //   继续积分会把陈旧梯度施加到新基线上。载波相位 escPhase_ 不清：相位连续
    //   可避免重锚拍载波突跳。
    void rebaseEsc() {
        gradKp_ = 0.0;
        gradKd_ = 0.0;
        hpfCPrev_ = 0.0;
        hpfY_ = 0.0;
        demodKpSum_ = 0.0;
        demodKdSum_ = 0.0;
        demodCount_ = 0;
        windowErr_.clear();
        windowDelta_.clear();
        windowDDelta_.clear();
        prevDelta_ = 0.0;
    }

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

    // ---- R1：从 AutopilotCore 随迁的编排状态 ----
    bool     baselineEstablished_ = false;  // 极点配置是否成功建立基线
    double   curKp_ = 0.0;     // 当前增益（P11：基线建立后 ESC 拥有，不再跟随调度表）
    double   curKd_ = 0.0;
    double   baseK_ = 0.0;     // P12：基线/重基线时的 K/T 估计（漂移判据基准）
    double   baseT_ = 0.0;
    uint32_t winCounter_ = 0;  // ESC 窗口节拍计数（原 escCounter_）
    double   prevRef_ = 0.0;      // P12：航向参考变化检测（振荡检测门控）
    double   refAgeSec_ = 0.0;    // P12：参考未变累计时长 (s)

    // P12 策略常量（R1 随迁，保持 constexpr：行为不变，不新增可调面）：
    // K/T 估计相对漂移超过 25% 才显式重基线——小于该幅度时新旧极点配置差异
    // 在 ESC 自身修正能力范围内，交给 ESC 连续调参；超过则意味着船况/航速
    // 真正改变（或估计已远离早期未收敛值），必须显式重锚。
    static constexpr double kReBaselineTol = 0.25;
    // 相对漂移分母下限（防早期小估计把相对偏差放大到失真；K∈(0,2]、T∈[1,60]）
    static constexpr double kReBaselineKFloor = 0.05;
    static constexpr double kReBaselineTFloor = 1.0;
    // 参考稳定满 30 s 才喂入振荡检测——覆盖典型船闭环调节时间（T≤20 s 船
    // 约 4T），短于此的参考机动期间 osc 测量无效、不参与稳定门
    static constexpr double kOscRefSettleSec = 30.0;

    static constexpr double PI = 3.14159265358979323846;
};

} // namespace ar
