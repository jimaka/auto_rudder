// autopilot_core.hpp
// 自动舵核心集成（对应文档 §5 定位与控制联合工作流）
//   将 PD 控制器 + 增益调度 + 在线辨识 + 自动优化 + 机动序列 整合
//   仅算法层，IO 由外部注入
//   P12 整改：ESC 增益所有权——基线建立后辨识触发的极点配置不再每秒盲目覆写
//            escKp_/escKd_（旧逻辑把 ESC 窗口间的积分成果全部冲掉，回退基线也被
//            拖到未收敛估计上）；仅当 K/T 估计相对基线估计漂移超阈值时才走显式
//            重基线（增益=新配置 + ESC 积分器 rebaseEsc 重锚）。同时振荡检测改为
//            仅在航向参考稳定后喂入——参考机动期间误差摆动是指令响应，旧逻辑在
//            Z 形激励下 oscAmp≈100°+ 把稳定门永久锁死，基线被钉死在早期坏估计上。
#pragma once
#include "pd_controller.hpp"
#include "gain_schedule.hpp"
#include "nomoto_identifier.hpp"
#include "auto_tuner.hpp"
#include "maneuver_sequencer.hpp"
#include "param_store.hpp"
#include <algorithm>

namespace ar {

struct SensorInput {
    double headingDeg;     // AHRS 艏向
    double rateDegS;       // AHRS 转向率
    double rudderDeg;     // 舵角反馈
    double speedKn;        // 船速
    int    seaState;       // 海况 0..3
};

enum class Mode { MANUAL, AUTO_HEADING, AUTO_MANEUVER, HOLD };

class AutopilotCore {
public:
    AutopilotCore() {
        // P3 整改：辨识器按 1 s 节拍调用，dt 配 1.0（与调用周期一致）
        NomotoIdentifier::Params ip = identifier_.params();
        ip.dt = 1.0;
        identifier_.setParams(ip);
    }

    // P9 整改：setMode 做状态清理，避免跨模式状态污染
    void setMode(Mode m) {
        if (m == mode_) return;
        // 切入新模式前清理上一模式的残留状态
        pd_.reset();
        identCounter_ = 0;
        escCounter_ = 0;
        // 切出 AUTO_HEADING/AUTO_MANEUVER 时去激活自动优化
        if (m == Mode::MANUAL || m == Mode::HOLD) tuner_.disable();
        // HOLD 模式：冻结当前舵角作为保持指令
        if (m == Mode::HOLD) {
            holdRudder_ = pd_.params().dt > 0 ? lastRudderCmd_ : 0.0;
        }
        // P11 整改：切入 AUTO_HEADING 时，首拍以实际舵角初始化舵速限幅基准，
        //           避免舵令从 0° 爬升（真实舵面并不在 0°）
        if (m == Mode::AUTO_HEADING) pendingBaseline_ = true;
        mode_ = m;
    }
    Mode mode() const { return mode_; }

    // 设置航向保持目标
    void setHeadingRef(double ref) { headingRef_ = ref; }

    // 启动机动
    void startManeuver(const std::vector<Leg>& legs) {
        sequencer_.setLegs(legs);
        mode_ = Mode::AUTO_MANEUVER;
    }

    // 主循环：50 Hz 调用
    // 输出：目标舵角 (deg)
    double step(const SensorInput& s, double dt) {
        double rudderCmd = 0.0;

        switch (mode_) {
            case Mode::MANUAL:
                rudderCmd = s.rudderDeg;  // 透传
                break;

            case Mode::AUTO_HEADING:
                rudderCmd = headingControlStep(s, dt);
                break;

            case Mode::AUTO_MANEUVER: {
                ManeuverSequencer::Cmd cmd = sequencer_.update(s.headingDeg, s.rateDegS, dt);
                if (sequencer_.done()) {
                    // P11 整改：完成拍不再输出 0（那是绕过舵速限幅的瞬时跳变），
                    //           立即转入航向保持并闭环运行本拍
                    mode_ = Mode::AUTO_HEADING;
                    headingRef_ = s.headingDeg;  // 保持当前航向
                    rudderCmd = headingControlStep(s, dt);
                    break;
                }
                if (cmd.isHeading) {
                    headingRef_ = cmd.heading;
                    // P13 整改：机动 HEADING 段此前直接 pd_.update，从不查调度表，
                    //           PD 全程停留在默认增益 Kp=1/Kd=0.5（慢船几乎拉不动舵，
                    //           直航段目标航向完全丢失，cloverleaf 严重畸变）。
                    //           此处补齐增益调度查询；辨识/ESC 在机动中仍保持挂起。
                    double Kp, Kd;
                    schedule_.query(s.speedKn, s.seaState, Kp, Kd);
                    pd_.setGains(Kp, Kd);
                    // P1 整改：用 setDt 替代 setParams，避免切段 reset 清状态
                    pd_.setDt(dt);
                    rudderCmd = pd_.update(headingRef_, s.headingDeg);
                } else {
                    // P7 整改：RUDDER 段经 PD 的 saturate() 施加幅值/舵速限幅
                    pd_.setDt(dt);
                    rudderCmd = pd_.saturate(cmd.rudder);
                }
                break;
            }

            case Mode::HOLD:
                // P9 整改：HOLD 保持上一拍舵角指令（冻结），而非输出 0
                rudderCmd = holdRudder_;
                break;
        }
        lastRudderCmd_ = rudderCmd;  // P9: 记录用于 HOLD 冻结
        return rudderCmd;
    }

    // 外部触发 ESC 微调（每窗口周期调用一次）
    // P9 整改：revert 基线保护——仅在基线已建立时执行，否则保持现状
    // P11 整改：与 step() 内联触发共用统一入口 escTrigger()；从持久化的 ESC 增益
    //           出发（pd_ 里的值含抖动载波）并写回，两个入口行为一致
    void runEscTuning() {
        if (!baselineEstablished_) return;
        double Kp = escKp_, Kd = escKd_;
        escTrigger(Kp, Kd);
        escKp_ = Kp; escKd_ = Kd;
        pd_.setGains(Kp, Kd);
    }

    const PdController& pd() const { return pd_; }
    const NomotoIdentifier& identifier() const { return identifier_; }
    const AutoTuner& tuner() const { return tuner_; }
    const ManeuverSequencer& sequencer() const { return sequencer_; }
    // 仿真/标定场景注入自定义调度表（开发文档 §4.4）
    GainSchedule& scheduleMut() { return schedule_; }
    AutoTuner& tunerMut() { return tuner_; }

private:
    // AUTO_HEADING 每拍闭环（P11 整改：从 step() 抽取，供机动完成拍复用）
    double headingControlStep(const SensorInput& s, double dt) {
        // P11 整改：切入 AUTO_HEADING 的首拍以实际舵角作为舵速限幅基准
        if (pendingBaseline_) {
            pendingBaseline_ = false;
            pd_.initBaseline(s.rudderDeg);
        }

        // §3.6 增益调度
        double Kp, Kd;
        schedule_.query(s.speedKn, s.seaState, Kp, Kd);
        if (!baselineEstablished_) { escKp_ = Kp; escKd_ = Kd; }

        // §3.10 自动优化（高速去激活）
        if (s.speedKn <= tuner_.params().highSpeedDeactivateKn) {
            tuner_.enable(s.speedKn);
            // P3 整改：辨识每 1 s 更新一次（每 50 拍），降低 ECU 负担并保证差分信噪比
            if (++identCounter_ >= 50) {
                identCounter_ = 0;
                identifier_.update(s.rudderDeg, s.rateDegS);
                // P6 整改：仅在辨识结果物理合理时才送极点配置
                if (identifier_.excited() && identifier_.valid()) {
                    const double Khat = identifier_.K();
                    const double That = identifier_.T();
                    // P12 整改：基线建立后 ESC 拥有增益。K/T 估计漂移在阈值内时跳过
                    //           重配置（旧逻辑每秒覆写 escKp_/escKd_，ESC 窗口间的积分
                    //           成果全被冲掉）；漂移超阈值（船况/航速真正改变，或估计
                    //           已显著远离早期未收敛值）才走显式重基线。
                    const bool driftSmall = baselineEstablished_
                        && std::abs(Khat - escBaseK_)
                           <= kReBaselineTol * std::max(std::abs(escBaseK_), kReBaselineKFloor)
                        && std::abs(That - escBaseT_)
                           <= kReBaselineTol * std::max(std::abs(escBaseT_), kReBaselineTFloor);
                    if (!driftSmall) {
                        const bool rebaseline = baselineEstablished_;  // P12: 本次为显式重基线
                        double Kp2, Kd2;
                        // P6 整改：polePlacement 返回 bool，失败保持原增益
                        if (AutoTuner::polePlacement(Khat, That, tuner_.params().zeta,
                                                     tuner_.params().wn, Kp2, Kd2)) {
                            // 稳定性监控（P4 整改：oscAmplitude 由 OscillationDetector 实测）
                            if (tuner_.stabilityCheck(Khat, That, Kp2, Kd2,
                                                      tuner_.oscDetector().amplitude())) {
                                tuner_.setBaseline(Kp2, Kd2);
                                baselineEstablished_ = true;
                                escKp_ = Kp2; escKd_ = Kd2;
                                // P12：记录本次基线对应的 K/T 估计，作为漂移判据基准
                                escBaseK_ = Khat; escBaseT_ = That;
                                if (rebaseline) {
                                    // P12：显式重基线——ESC 积分器清零重锚（旧梯度不得
                                    //      施加到新基线），窗口节拍重对齐，使下一窗在
                                    //      新增益附近完整测量
                                    tuner_.rebaseEsc();
                                    escCounter_ = 0;
                                }
                            }
                            // P12：配置/检查失败时不更新 escBaseK_/escBaseT_，
                            //      下一秒估计若仍超阈值会重试；ESC 继续在旧基线上运行
                        }
                    }
                }
            }
        } else {
            tuner_.disable();
        }

        // 累积性能指标样本（P11：内部同步做每拍因果解调）
        const double e = angleDiffDeg(headingRef_, s.headingDeg);
        // P12 整改：振荡检测仅在航向参考稳定满 kOscRefSettleSec 后喂入——
        //           参考机动（如辨识 Z 形 ±40°/15 s）期间误差大幅摆动是指令响应
        //           而非闭环振荡，喂入检测器会把 oscAmp 顶到远超 oscMax（实测
        //           ≈100°+），稳定门从此永久封锁一切配置/ESC 更新，基线被钉死在
        //           早期未收敛估计上。参考机动期间由 ζ/ωn 代数条件把关；
        //           参考平稳后 osc 条件恢复实测参与，回退语义不变。
        if (headingRef_ != prevHeadingRef_) { prevHeadingRef_ = headingRef_; refAgeSec_ = 0.0; }
        else                                { refAgeSec_ += dt; }
        tuner_.accumulate(e, s.rudderDeg, refAgeSec_ >= kOscRefSettleSec);

        // P11 整改：ESC 增益持久化——基线建立后以 ESC 结果为基准，
        //           否则每拍被调度表覆盖，调参永远无效
        if (baselineEstablished_) { Kp = escKp_; Kd = escKd_; }

        // ESC 窗口结算（P11 整改：统一入口；窗口拍数按调参器参数换算）
        const double tdt = tuner_.params().dt;
        const uint32_t winBeats = (tdt > 0.0)
            ? static_cast<uint32_t>(std::max(1.0, tuner_.params().windowSec / tdt + 0.5))
            : 3000u;
        if (++escCounter_ >= winBeats) {
            escCounter_ = 0;
            escTrigger(Kp, Kd);
            escKp_ = Kp; escKd_ = Kd;  // 持久化（未触发更新时即原值）
        }

        // P11 整改：ESC 扰动注入——窗口运行期间（基线已建立）实际施加到 PD 的
        //           增益叠加载波；未建立基线时 ESC 不会结算，无需扰动
        if (baselineEstablished_) tuner_.escDither(Kp, Kd);

        // P1 整改：用 setGains/setDt 替代 setParams，避免每周期 reset 清空状态
        pd_.setDt(dt);
        pd_.setGains(Kp, Kd);
        return pd_.update(headingRef_, s.headingDeg);
    }

    // ESC 窗口结算统一入口（P11 整改：消除 step() 与 runEscTuning() 的重复逻辑）
    // 仅在调参器启用且基线已建立时执行；稳定性检查不通过则回退基线
    void escTrigger(double& Kp, double& Kd) {
        if (!tuner_.enabled() || !baselineEstablished_) return;
        tuner_.escStep(Kp, Kd);
        const double Khat = identifier_.valid() ? identifier_.K() : 0.0;
        const double That = identifier_.valid() ? identifier_.T() : 0.0;
        if (Khat > 0.0 && That > 0.0 &&
            tuner_.stabilityCheck(Khat, That, Kp, Kd,
                                  tuner_.oscDetector().amplitude())) {
            return;  // 接受 ESC 更新
        }
        tuner_.revert(Kp, Kd);  // 回退基线
    }

    PdController       pd_;
    GainSchedule       schedule_;
    NomotoIdentifier   identifier_;
    AutoTuner          tuner_;
    ManeuverSequencer  sequencer_;
    Mode               mode_ = Mode::MANUAL;
    double             headingRef_ = 0.0;
    uint32_t           identCounter_ = 0;  // P3: 辨识 1 s 降频计数器
    uint32_t           escCounter_ = 0;    // P2: ESC 窗口触发计数器
    bool               baselineEstablished_ = false;  // 极点配置是否成功建立基线
    bool               pendingBaseline_ = false;      // P11: 首拍以实际舵角初始化舵速基准
    double             escKp_ = 0.0;       // P11: ESC 持久化增益（基线建立后接管调度值）
    double             escKd_ = 0.0;
    double             escBaseK_ = 0.0;    // P12: 基线/重基线时的 K/T 估计（漂移判据基准）
    double             escBaseT_ = 0.0;
    double             prevHeadingRef_ = 0.0;  // P12: 航向参考变化检测（振荡检测门控）
    double             refAgeSec_ = 0.0;       // P12: 参考未变累计时长 (s)
    double             holdRudder_ = 0.0;     // P9: HOLD 模式冻结舵角
    double             lastRudderCmd_ = 0.0;   // P9: 上一拍舵角指令

    // P12: K/T 估计相对漂移超过 25% 才显式重基线——小于该幅度时新旧极点配置差异
    //      在 ESC 自身修正能力范围内，交给 ESC 连续调参；超过则意味着船况/航速
    //      真正改变（或估计已远离早期未收敛值），必须显式重锚。
    static constexpr double kReBaselineTol = 0.25;
    // P12: 相对漂移分母下限（防早期小估计把相对偏差放大到失真；K∈(0,2]、T∈[1,60]）
    static constexpr double kReBaselineKFloor = 0.05;
    static constexpr double kReBaselineTFloor = 1.0;
    // P12: 参考稳定满 30 s 才喂入振荡检测——覆盖典型船闭环调节时间（T≤20 s 船
    //      约 4T），短于此的参考机动期间 osc 测量无效、不参与稳定门
    static constexpr double kOscRefSettleSec = 30.0;
};

} // namespace ar
