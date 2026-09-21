// tuner_debug_api.hpp
// 实船调试接口：把 AutoTuner 的每个功能以"可单独调用"的形式暴露出来。
//
// 设计意图：
//   实船调试时需要绕开 tick() 的固定编排，对单一环节做激励-观测——例如只验
//   极点配置数值、只打一拍载波看执行器响应、只喂误差看振荡检测读数、只把
//   当前窗口的 J 读出来。本门面（facade）把 AutoTuner 的公有动作与私有机制
//   全部摊平为独立方法，每个方法对应一个且仅一个功能，互不影响、任意顺序
//   调用。配套命令行工具见 control/tools/tuner_debug_cli.cpp。
//
// 使用约束（与实船安全直接相关，务必遵守）：
//   1. 本接口只用于调试/标定，不得接入正常航行控制回路——forceGains /
//      forceBaseline 旁路稳定门，正常路径的安全保证全部失效；
//   2. 除 force* 系列外，其余方法与其在 tick() 内部的行为逐字一致
//      （同一私有函数、同一参数），观测结果可代表在线行为；
//   3. 所有方法均无堆分配、无 IO，可在 ECU 上经串行/以太网命令通道驱动。
#pragma once
#include "auto_tuner.hpp"

namespace ar {

class TunerDebugApi {
public:
    explicit TunerDebugApi(AutoTuner& tuner) : t_(tuner) {}

    // ==================== 状态观测（只读投影，一次调用取一类状态） ====================

    // 全量快照：一次调用取全部内部状态（遥测打包用）
    struct Snapshot {
        // 增益与基线
        double curKp, curKd;             // 当前增益（不含载波）
        double baselineKp, baselineKd;   // 基线增益（回退目标）
        double baseK, baseT;             // 基线对应的 K/T 估计
        bool   baselineEstablished;
        bool   enabled;
        // ESC 状态
        double escPhase;                 // 载波相位 (rad)
        double gradKp, gradKd;           // LPF 后的梯度估计
        double lastJ;                    // 上窗代价
        double hpfY;                     // HPF 当前输出
        double demodKpSum, demodKdSum;   // 本窗解调累积
        uint32_t demodCount;             // 本窗已累积拍数
        // 窗口与编排
        uint32_t winCounter;             // 本窗已过拍数
        size_t   windowSamples;          // 环形缓冲内样本数
        double   refAgeSec;              // 航向参考未变累计时长
        // 实测振荡
        double oscAmplitude;             // 峰-峰幅值估计 (deg)
    };
    Snapshot snapshot() const {
        return snapshotOf(t_);
    }
    // 只读仿真/遥测可以接受 const 调参器，不必获取可写调试门面。
    static Snapshot snapshotOf(const AutoTuner& tuner) {
        Snapshot s;
        s.curKp = tuner.curKp_;  s.curKd = tuner.curKd_;
        s.baselineKp = tuner.baselineKp_;  s.baselineKd = tuner.baselineKd_;
        s.baseK = tuner.baseK_;  s.baseT = tuner.baseT_;
        s.baselineEstablished = tuner.baselineEstablished_;
        s.enabled = tuner.enabled_;
        s.escPhase = tuner.escPhase_;
        s.gradKp = tuner.gradKp_;  s.gradKd = tuner.gradKd_;
        s.lastJ = tuner.lastJ_;
        s.hpfY = tuner.hpfY_;
        s.demodKpSum = tuner.demodKpSum_;  s.demodKdSum = tuner.demodKdSum_;
        s.demodCount = tuner.demodCount_;
        s.winCounter = tuner.winCounter_;
        s.windowSamples = tuner.windowErr_.size();
        s.refAgeSec = tuner.refAgeSec_;
        s.oscAmplitude = tuner.oscDetector_.amplitude();
        return s;
    }
    double currentJ() const { return t_.computeJ(); }        // 当前窗口 ITAE 代价
    double oscAmplitude() const { return t_.oscDetector_.amplitude(); }

    // ==================== 层 2：极点配置与稳定门（无状态单功能） ====================

    // 极点配置解析解：与 AutoTuner::polePlacement 同一函数
    bool polePlacement(double K, double T, double zeta, double wn,
                       double& Kp, double& Kd) const {
        return AutoTuner::polePlacement(K, T, zeta, wn, Kp, Kd);
    }
    // 稳定门四条件：与在线判定同一函数（oscAmplitude 由调用方给，便于注入测试值）
    bool stabilityCheck(double K, double T, double Kp, double Kd,
                        double oscAmplitude) const {
        return t_.stabilityCheck(K, T, Kp, Kd, oscAmplitude);
    }

    // ==================== 基线管理（事件驱动，单功能） ====================

    // 一次性建基线（辨识试验收官语义：不做漂移检查、osc 以 0 传入、恒重锚）
    bool establishBaseline(double K, double T) {
        IdSnapshot snap; snap.excited = true; snap.valid = true; snap.K = K; snap.T = T;
        return t_.establishBaseline(snap);
    }
    // 辨识评估原语：applyEstimate_ 原样暴露，driftGate/forceRebase 由调用方控制
    bool evaluateEstimate(double K, double T, double oscAmp,
                          bool driftGate, bool forceRebase) {
        return t_.applyEstimate_(K, T, oscAmp, driftGate, forceRebase);
    }

    // ==================== ESC 各环节（每拍/窗末原语，单功能） ====================

    // 只打一拍载波：对给定增益叠加 sin/cos 摄动并推进相位（等价 escDither）
    void ditherOnce(double& Kp, double& Kd) { t_.escDither(Kp, Kd); }
    // 只喂一拍样本：成本累积 + 振荡检测（门控可选）+ 因果解调（等价 accumulate）
    void accumulateOnce(double e, double delta, bool oscEligible) {
        t_.accumulate(e, delta, oscEligible);
    }
    // 只做一次窗末梯度结算：归一化 → LPF → 梯度下降 → 限幅 → 清窗（等价 escStep）
    void escStepOnce(double& Kp, double& Kd) { t_.escStep(Kp, Kd); }
    // 只做一次窗末裁决：escStep → 稳定门 → 接受/回退（等价 settleWindow_，
    // 即 tick() 每 3000 拍自动执行的那一步；辨识估计由调用方给，便于注入测试值）
    void settleWindowOnce(bool idValid, double K, double T) {
        IdSnapshot id; id.excited = idValid; id.valid = idValid; id.K = K; id.T = T;
        t_.settleWindow_(id);
    }
    // 只算一次更新限幅（无状态）
    double clampUpdate(double newVal, double oldVal) const {
        return t_.clampUpdate(newVal, oldVal);
    }

    // ==================== 振荡检测（单功能） ====================

    void oscFeed(double e) { t_.oscDetector_.update(e, t_.p_.dt); }
    void oscReset() { t_.oscDetector_.reset(); }

    // ==================== 回退与重锚（单功能） ====================

    void revertToBaseline(double& Kp, double& Kd) const { t_.revert(Kp, Kd); }
    void rebaseEsc() { t_.rebaseEsc(); }

    // ==================== 主流程与生命周期（与在线路径同一函数） ====================

    GainCommand tick(const TunerInput& in) { return t_.tick(in); }
    void suspend() { t_.suspend(); }
    void reset() { t_.reset(); }

    // ==================== 参数读写 ====================

    const AutoTuner::Params& params() const { return t_.params(); }
    void setParams(const AutoTuner::Params& p) { t_.setParams(p); }

    // ==================== 强制注入（旁路稳定门，仅调试！） ====================

    // 直接改写当前增益——不过稳定门、不动基线。仅用于执行器响应试验等
    // 受控场景；调用方自行保证增益安全。正常航行严禁使用。
    void forceGains(double Kp, double Kd) { t_.curKp_ = Kp; t_.curKd_ = Kd; }
    // 直接改写基线并置基线标志——旁路极点配置与稳定门，仅人工标定注入用。
    // 注意：不记录 baseK_/baseT_（保持 0），此后 driftGate=true 的漂移判据会
    // 把任何 K/T 都视为超阈值漂移（分母走 floor），即下一次评估恒重基线——
    // 这是保守的安全方向，但调用方应知情。
    void forceBaseline(double Kp, double Kd) {
        t_.setBaseline(Kp, Kd);
        t_.curKp_ = Kp; t_.curKd_ = Kd;
        t_.baselineEstablished_ = true;
    }
    // 直接改写载波相位（载波同步试验用）
    void forceEscPhase(double phase) { t_.escPhase_ = phase; }
    // 直接改写梯度积分器（积分器响应试验用）
    void forceGradient(double gKp, double gKd) { t_.gradKp_ = gKp; t_.gradKd_ = gKd; }
    // 直接置/清速度门控标志——dither/accumulate/escStep 等单环节调用依赖
    // enabled_，单独调试这些环节时用它绕开速度门控（不影响其他状态）
    void forceEnabled(bool en) { t_.enabled_ = en; }

private:
    AutoTuner& t_;
};

} // namespace ar
