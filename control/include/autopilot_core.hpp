// autopilot_core.hpp
// 自动舵核心集成（对应文档 §5 定位与控制联合工作流）
//   将 PD 控制器 + 增益调度 + 在线辨识 + 自动优化 + 机动序列 整合
//   仅算法层，IO 由外部注入
//   P12 整改：ESC 增益所有权与漂移重基线；振荡检测仅在航向参考稳定后喂入
//            （R1 起这些编排逻辑已迁入 AutoTuner 内部，见 auto_tuner.hpp）
//   P14 新增：自动 Z 形辨识试验（startIdentifyTrial）——自动执行 Z 形机动，
//            期间喂辨识器，机动收官经 AutoTuner::establishBaseline 应用增益
//   R1 深化：自动优化的编排（节拍/窗口/增益所有权/稳定门/回退/重锚）全部收进
//            AutoTuner::tick()；本类回归本职——模式状态机 + 机动编排 + 每拍转发。
//            辨识器自降频（任意频率喂），core 不再维护任何调参计数器/增益副本。
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

// P14：自动 Z 形辨识试验状态（§3.9.4 Z 形试验辨识的自动化）
enum class IdentTrialState { NONE, RUNNING, OK, FAILED };

class AutopilotCore {
public:
    AutopilotCore() = default;

    // P9 整改：setMode 做状态清理，避免跨模式状态污染
    void setMode(Mode m) {
        if (m == mode_) return;
        // 切入新模式前清理上一模式的残留状态
        pd_.reset();
        identTrial_ = IdentTrialState::NONE;  // P14：切模式取消进行中的辨识试验
        // 切出 AUTO_HEADING/AUTO_MANEUVER 时挂起自动优化（R1：suspend 清测量状态、留基线）
        if (m == Mode::MANUAL || m == Mode::HOLD) tuner_.suspend();
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
        identTrial_ = IdentTrialState::NONE;  // P14：普通机动抢占进行中的辨识试验
        sequencer_.setLegs(legs);
        mode_ = Mode::AUTO_MANEUVER;
    }

    // P14：自动 Z 形辨识试验——执行 dz/phiz 的 Z 形机动，期间喂辨识器（自降频）；
    //       机动结束自动 极点配置 → 稳定性检查 → 应用新增益（AutoTuner::establishBaseline），
    //       结果经 identTrialState()/identifier()/pd() 读取
    void startIdentifyTrial(double dz = 20.0, double phiz = 20.0, int cycles = 4) {
        identifier_.reset();   // 重新辨识，不被历史数据污染
        sequencer_.setLegs(ManeuverSequencer::zigzag(0.0, dz, phiz, cycles, false));
        identTrial_ = IdentTrialState::RUNNING;
        mode_ = Mode::AUTO_MANEUVER;
    }
    IdentTrialState identTrialState() const { return identTrial_; }

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
                    // P14：Z 形辨识试验收官——辨识有效则 极点配置+应用，失败保持原增益
                    if (identTrial_ == IdentTrialState::RUNNING) finalizeIdentTrial();
                    // P11 整改：完成拍不再输出 0（那是绕过舵速限幅的瞬时跳变），
                    //           立即转入航向保持并闭环运行本拍
                    mode_ = Mode::AUTO_HEADING;
                    headingRef_ = s.headingDeg;  // 保持当前航向
                    rudderCmd = headingControlStep(s, dt);
                    break;
                }
                // P14：辨识试验期间喂辨识器（自降频，任意频率调用均正确；
                //       试验显式发起，不受调参器速度门控影响）
                if (identTrial_ == IdentTrialState::RUNNING) {
                    identifier_.update(s.rudderDeg, s.rateDegS, dt);
                }
                if (cmd.isHeading) {
                    headingRef_ = cmd.heading;
                    // P13 整改：机动 HEADING 段补齐增益调度查询（否则全程停留默认增益，
                    //           慢船直航段目标航向丢失）；辨识/ESC 在机动中仍保持挂起。
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

    const PdController& pd() const { return pd_; }
    const NomotoIdentifier& identifier() const { return identifier_; }
    const AutoTuner& tuner() const { return tuner_; }
    const ManeuverSequencer& sequencer() const { return sequencer_; }
    // 仿真/标定场景注入自定义调度表（开发文档 §4.4）
    GainSchedule& scheduleMut() { return schedule_; }
    // R1：调参器参数注入收窄为整组设置（替代暴露 mutable 引用的 tunerMut 后门）
    void setTunerParams(const AutoTuner::Params& p) { tuner_.setParams(p); }

private:
    // AUTO_HEADING 每拍闭环（P11 整改：从 step() 抽取，供机动完成拍复用）
    // R1 深化：自动优化编排全部收进 AutoTuner::tick()，本方法只剩
    //          调度查询 → 喂辨识器 → tick → 转发增益 → PD 闭环
    double headingControlStep(const SensorInput& s, double dt) {
        // P11 整改：切入 AUTO_HEADING 的首拍以实际舵角作为舵速限幅基准
        if (pendingBaseline_) {
            pendingBaseline_ = false;
            pd_.initBaseline(s.rudderDeg);
        }

        // §3.6 增益调度
        double Kp, Kd;
        schedule_.query(s.speedKn, s.seaState, Kp, Kd);

        // 辨识器自降频（R1）：任意频率喂，内部满 1 s 才做 RLS 拍
        identifier_.update(s.rudderDeg, s.rateDegS, dt);

        // §3.10 自动优化（R1：速度门控/辨识评估/ESC/稳定门/回退/扰动全部内收）
        const GainCommand g = tuner_.tick({headingRef_, s.headingDeg, s.rudderDeg,
                                          s.speedKn, dt, Kp, Kd, idSnapshot()});

        // P1 整改：用 setGains/setDt 替代 setParams，避免每周期 reset 清空状态
        pd_.setDt(dt);
        pd_.setGains(g.Kp, g.Kd);
        return pd_.update(headingRef_, s.headingDeg);
    }

    // 辨识器当前估计的快照（辨识器保持独立模块，经快照传入调参器）
    IdSnapshot idSnapshot() const {
        return {identifier_.excited(), identifier_.valid(), identifier_.K(), identifier_.T()};
    }

    // P14：Z 形辨识试验收官——辨识结果物理合理且极点配置/稳定检查通过，
    //       才把新增益设为基线并应用（ESC 从该基线继续微调）；否则保持原增益。
    //       R1：评估路径收进 AutoTuner::establishBaseline（机动刚结束振荡测量无效，
    //       osc 条件以 0 传入由 ζ/ωn 代数条件把关，与 P12 参考机动期间同一理由）。
    void finalizeIdentTrial() {
        const bool ok = tuner_.establishBaseline(idSnapshot());
        identTrial_ = ok ? IdentTrialState::OK : IdentTrialState::FAILED;
    }

    PdController       pd_;
    GainSchedule       schedule_;
    NomotoIdentifier   identifier_;
    AutoTuner          tuner_;
    ManeuverSequencer  sequencer_;
    Mode               mode_ = Mode::MANUAL;
    double             headingRef_ = 0.0;
    bool               pendingBaseline_ = false;      // P11: 首拍以实际舵角初始化舵速基准
    double             holdRudder_ = 0.0;     // P9: HOLD 模式冻结舵角
    double             lastRudderCmd_ = 0.0;   // P9: 上一拍舵角指令
    IdentTrialState    identTrial_ = IdentTrialState::NONE;  // P14: Z 形辨识试验状态
};

} // namespace ar
