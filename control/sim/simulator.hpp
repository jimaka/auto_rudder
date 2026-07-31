// simulator.hpp — 50 Hz 闭环 runner + 数据记录（对应开发文档 §10.2）
#pragma once
#include "nomoto_plant.hpp"
#include "disturbances.hpp"
#include <autopilot_core.hpp>  // include 路径由 ar_headers INTERFACE 目标提供
#include <vector>
#include <string>
#include <cstdio>
#include <chrono>

namespace ar::sim {

struct Sample {
    double t;
    double psi, psiRef, rudderCmd, rudderAct, rate, err;
    double Kp, Kd;
    int    mode;
};

struct SimResult {
    std::vector<Sample> samples;
    // 指标
    double overshootPct = 0.0;   // 超调百分比
    double settlingTime = 0.0;   // 5% 调节时间 (s)
    double ess1Sigma    = 0.0;   // 稳态误差 1σ (deg)
    double essBias      = 0.0;   // 稳态误差偏置 |mean| (deg)——PD 无积分，恒风下存在系统性偏置
    double rudderFreq   = 0.0;   // 操舵频次 (次/分钟)
    bool   pass         = true;
};

class Simulator {
public:
    NomotoPlant plant;
    Disturbances dist;
    AutopilotCore ap;
    double dt = 0.02;       // 50 Hz
    int    recordEvery = 1; // 每 N 拍记录一次
    bool   usePolePlacementGains = true;  // 用已知 K/T 极点配置增益（M1 验证 PD 本身）
    double zetaTarget = 0.85;
    double wnTarget   = 0.25;
    bool   enableDisturbance = false;     // M1 纯阶跃验证关闭干扰；带干扰场景另开

    // 最近一次运行的采样（所有 run* 模式均填充，供导出 CSV/可视化）
    std::vector<Sample> lastSamples;

    // 跑一次阶跃场景：先稳态 holdSec，再施加 stepDeg 阶跃，再观察 totalSec
    SimResult runStep(double speedKn, int seaState,
                      double stepDeg, double holdSec, double totalSec) {
        SimResult res;
        lastSamples.clear();
        ap.setMode(Mode::AUTO_HEADING);
        ap.setHeadingRef(0.0);
        plant.reset(0.0);

        // 用已知 K/T 极点配置增益安装调度表（覆盖全速段与全部海况行，
        // 避免按海况查询无匹配行时回退到默认增益 Kp=1/Kd=0.5）
        if (usePolePlacementGains) {
            const double Kp = wnTarget * wnTarget * plant.T / plant.K;
            const double Kd = (2.0 * zetaTarget * wnTarget * plant.T - 1.0) / plant.K;
            ap.scheduleMut().setTable({{0.0, 100.0, 0, Kp, Kd},
                                       {0.0, 100.0, 1, Kp, Kd},
                                       {0.0, 100.0, 2, Kp, Kd},
                                       {0.0, 100.0, 3, Kp, Kd}});
        }
        // 扰动幅值按场景海况缩放（未启用扰动时无副作用）
        dist.setSeaState(seaState);
        // M1 阶段禁用自动优化（P3/P6 未修，辨识器未收敛会给出坏增益覆盖调度表）
        {
            auto tp = ap.tunerMut().params();
            tp.highSpeedDeactivateKn = -1.0;  // 任意正船速都去激活
            ap.tunerMut().setParams(tp);
        }

        const int totalSteps = static_cast<int>(totalSec / dt);
        const int holdSteps  = static_cast<int>(holdSec / dt);
        const int stepStep   = holdSteps;  // 阶跃施加时刻

        double prevRudderCmd = 0.0;
        int dirChanges = 0;
        std::vector<double> errSteady;

        for (int k = 0; k < totalSteps; ++k) {
            double t = k * dt;
            if (k == stepStep) ap.setHeadingRef(stepDeg);

            SensorInput s;
            s.headingDeg = enableDisturbance ? dist.noisyHeading(plant.psi) : plant.psi;
            s.rateDegS   = enableDisturbance ? dist.noisyRate(plant.r) : plant.r;
            s.rudderDeg  = plant.deltaAct;
            s.speedKn    = speedKn;
            s.seaState  = seaState;

            double cmd = ap.step(s, dt);
            plant.step(cmd, dt, enableDisturbance ? dist.disturbAngle(t) : 0.0);

            if (k % recordEvery == 0) {
                Sample sm;
                sm.t = t;
                sm.psi = plant.psi;
                sm.psiRef = (k >= stepStep) ? stepDeg : 0.0;
                sm.rudderCmd = cmd;
                sm.rudderAct = plant.deltaAct;
                sm.rate = plant.r;
                sm.err = sm.psiRef - sm.psi;
                sm.Kp = ap.pd().params().Kp;
                sm.Kd = ap.pd().params().Kd;
                sm.mode = static_cast<int>(ap.mode());
                res.samples.push_back(sm);
                lastSamples.push_back(sm);
            }

            // 操舵频次：方向翻转计数（幅值 > 量化步长才计）
            if (k > 0) {
                double d = cmd - prevRudderCmd;
                if (std::abs(d) > 0.1) {
                    if ((prevRudderCmd > 0 && cmd < 0) || (prevRudderCmd < 0 && cmd > 0))
                        ++dirChanges;
                }
            }
            prevRudderCmd = cmd;

            // 阶跃后稳态段（最后 20 s）收集误差
            if (k > totalSteps - static_cast<int>(20.0 / dt) && k >= stepStep) {
                double e = (k >= stepStep ? stepDeg : 0.0) - plant.psi;
                errSteady.push_back(e);
            }
        }

        computeMetrics(res, stepDeg, holdSec, totalSec, dirChanges, errSteady);
        return res;
    }

    void exportCsv(const SimResult& r, const std::string& path) {
        exportCsv(r.samples, path);
    }

    void exportCsv(const std::vector<Sample>& samples, const std::string& path) {
        std::FILE* f = std::fopen(path.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "警告: 无法写入 CSV 文件: %s\n", path.c_str());
            return;
        }
        std::fprintf(f, "index,t,heading,headingRef,rudderCmd,rudderAct,rate,error,Kp,Kd,mode\n");
        for (size_t i = 0; i < samples.size(); ++i) {
            const auto& s = samples[i];
            std::fprintf(f, "%zu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
                         i, s.t, s.psi, s.psiRef, s.rudderCmd, s.rudderAct,
                         s.rate, s.err, s.Kp, s.Kd, s.mode);
        }
        std::fclose(f);
    }

    // 导出最近一次运行的采样（适用于 ident/esc/maneuver/validation 等不返回 SimResult 的模式）
    void exportLastCsv(const std::string& path) { exportCsv(lastSamples, path); }

    // RLS 闭环辨识场景：用 Z 形舵角激励辨识器，验证收敛到真值
    // 返回 (Khat, That, valid)
    struct IdentResult { double Khat, That; bool valid, excited; };
    IdentResult runIdent(double K, double T, double speedKn, double totalSec) {
        plant.K = K; plant.T = T; plant.reset(0.0);
        lastSamples.clear();
        ap.setMode(Mode::MANUAL);  // 手动注入舵角激励
        // 用极点配置增益保持闭环稳定的同时叠加 Z 形
        const double Kp = wnTarget * wnTarget * T / K;
        const double Kd = (2.0 * zetaTarget * wnTarget * T - 1.0) / K;
        ap.scheduleMut().setTable({{0.0, 100.0, 0, Kp, Kd}});
        // 启用自动优化路径让辨识器运行
        auto tp = ap.tunerMut().params();
        tp.highSpeedDeactivateKn = 100.0;  // 始终启用
        ap.tunerMut().setParams(tp);

        const int totalSteps = static_cast<int>(totalSec / dt);
        // Z 形激励：每 10 s 翻转一次 ±20°
        for (int k = 0; k < totalSteps; ++k) {
            double t = k * dt;
            double zDelta = (static_cast<int>(t / 10.0) % 2 == 0) ? 20.0 : -20.0;
            SensorInput s;
            s.headingDeg = plant.psi;
            s.rateDegS   = plant.r;
            s.rudderDeg  = plant.deltaAct;
            s.speedKn    = speedKn;
            s.seaState  = 0;
            // 手动模式：直接把 zDelta 作为舵指令注入（绕过 PD）
            // 但 autopilot MANUAL 返回 0，所以用 AUTO_HEADING + 大幅航向阶跃等效激励
            ap.setMode(Mode::AUTO_HEADING);
            ap.setHeadingRef(zDelta * 5);  // 大幅航向目标产生大舵角
            double cmd = ap.step(s, dt);
            plant.step(cmd, dt, 0.0);
            if (k % recordEvery == 0) {
                Sample sm; sm.t = t; sm.psi = plant.psi;
                sm.psiRef = zDelta * 5; sm.rudderCmd = cmd; sm.rudderAct = plant.deltaAct;
                sm.rate = plant.r; sm.err = sm.psiRef - plant.psi;
                sm.Kp = ap.pd().params().Kp; sm.Kd = ap.pd().params().Kd;
                sm.mode = static_cast<int>(ap.mode());
                lastSamples.push_back(sm);
            }
        }
        IdentResult r;
        r.Khat = ap.identifier().K();
        r.That = ap.identifier().T();
        r.valid = ap.identifier().valid();
        r.excited = ap.identifier().excited();
        return r;
    }

    // ESC 长期场景：Z 形航向参考激励 → RLS 辨识收敛 → 极点配置建立基线 → ESC 窗口调参
    // 激励设计：辨识器按 1 s 节拍采样，激励门为 |δ|>3° 且 (|r|>0.5°/s 或 |dr|>0.2°/s²)。
    //   旧 ±10°/30 s 阶跃 max|r|≈1.1°/s，1 s 差分后绝大多数样本过不了门，
    //   辨识只收到少量有偏样本 → 收敛到非物理值 → valid()=false → 基线永不建立
    //   → ESC 从不启动，而旧场景只查有界性仍打印 PASS（假阳性）。
    //   现改用海试辨识常用的 Z 形 ±40°/15 s 参考（物理合理），摆向期间 |r| 与
    //   |dr| 持续过门，辨识器在 ~1 min 内收敛到真实 K/T 附近。
    struct EscResult {
        double Kp0, Kd0;              // 调度表基线增益（起点）
        double KpFinal, KdFinal;      // 结束时实际增益
        double Khat, That;            // 辨识器最终 K/T 估计
        bool   identValid;            // 结束时辨识结果物理合理
        bool   baselineEstablished;   // 观测到极点配置/ESC 接管增益
        double baselineSec;           // 首次观测到接管的时刻（未接管为 -1）
        bool   gainsMoved;            // 增益相对起点移动超过阈值
        bool   stable;                // 增益与船态有限且有界
    };
    EscResult runEsc(double K, double T, double speedKn, double totalSec) {
        plant.K = K; plant.T = T; plant.reset(0.0);
        lastSamples.clear();
        ap.setMode(Mode::AUTO_HEADING);
        // 用极点配置增益作调度表基线
        const double Kp0 = wnTarget * wnTarget * T / K;
        const double Kd0 = (2.0 * zetaTarget * wnTarget * T - 1.0) / K;
        ap.scheduleMut().setTable({{0.0, 100.0, 0, Kp0, Kd0}});
        // 启用自动优化
        auto tp = ap.tunerMut().params();
        tp.highSpeedDeactivateKn = 100.0;
        ap.tunerMut().setParams(tp);

        // Z 形（zigzag）航向参考：每 legSec 在 ±refAmp 间翻转，全程确定性激励
        const double refAmp = 40.0;  // deg
        const double legSec = 15.0;  // s
        const int legSteps  = static_cast<int>(legSec / dt);
        const int totalSteps = static_cast<int>(totalSec / dt);
        double curRef = refAmp;
        ap.setHeadingRef(curRef);

        double KpFinal = Kp0, KdFinal = Kd0;
        bool   baselineSeen = false;
        double baselineSec  = -1.0;
        for (int k = 0; k < totalSteps; ++k) {
            double t = k * dt;
            if (k > 0 && k % legSteps == 0) {
                curRef = -curRef;
                ap.setHeadingRef(curRef);
            }
            SensorInput s;
            s.headingDeg = plant.psi;
            s.rateDegS   = plant.r;
            s.rudderDeg  = plant.deltaAct;
            s.speedKn    = speedKn;
            s.seaState  = 0;
            double cmd = ap.step(s, dt);
            plant.step(cmd, dt, 0.0);
            KpFinal = ap.pd().params().Kp;
            KdFinal = ap.pd().params().Kd;
            // 基线建立观测：AutopilotCore 的 baselineEstablished_ 无 getter，
            // 但其效果可观测——接管前每拍 PD 增益恒等于调度表值 Kp0/Kd0；
            // 接管后增益由极点配置（wn 目标不同）重写并叠加 ±5% ESC 载波，必然偏离。
            if (!baselineSeen &&
                (std::abs(KpFinal - Kp0) > 1e-9 * Kp0 ||
                 std::abs(KdFinal - Kd0) > 1e-9 * Kd0)) {
                baselineSeen = true;
                baselineSec  = t;
            }
            if (k % recordEvery == 0) {
                Sample sm; sm.t = t; sm.psi = plant.psi;
                sm.psiRef = curRef; sm.rudderCmd = cmd; sm.rudderAct = plant.deltaAct;
                sm.rate = plant.r; sm.err = curRef - plant.psi;
                sm.Kp = KpFinal; sm.Kd = KdFinal;
                sm.mode = static_cast<int>(ap.mode());
                lastSamples.push_back(sm);
            }
        }
        EscResult r;
        r.Kp0 = Kp0; r.Kd0 = Kd0;
        r.KpFinal = KpFinal; r.KdFinal = KdFinal;
        r.Khat = ap.identifier().K();
        r.That = ap.identifier().T();
        r.identValid = ap.identifier().valid();
        r.baselineEstablished = baselineSeen;
        r.baselineSec = baselineSec;
        // 增益移动判据：Kp 或 Kd 相对起点偏移 > 1%（基线接管+ESC 调参的共同效果）
        r.gainsMoved = (std::abs(KpFinal - Kp0) > 0.01 * Kp0)
                    || (std::abs(KdFinal - Kd0) > 0.01 * Kd0);
        // 稳定性：增益与船态有限、Kp 不发散（沿用原有界判据精神，Kd 同法）
        r.stable = std::isfinite(KpFinal) && std::isfinite(KdFinal)
                && std::isfinite(plant.psi) && std::isfinite(plant.r)
                && KpFinal > 0.0 && KpFinal < Kp0 * 5.0
                && KdFinal >= 0.0 && KdFinal < Kd0 * 5.0;
        return r;
    }

    // 六机动形态场景：跑预置机动，验证序列在仿真时长内真实执行完毕且数值有界
    struct ManeuverResult {
        bool   completed  = false;  // 序列真实执行完毕且全程数值有限
        bool   seqDone    = false;  // 序列器 done() 或已进入末段保持段
        double maxHeading = 0.0;
        double maxRudder  = 0.0;
    };
    ManeuverResult runManeuver(const std::vector<Leg>& legs, double K, double T,
                                double speedKn, int seaState, double totalSec) {
        plant.K = K; plant.T = T; plant.reset(0.0);
        lastSamples.clear();
        ap.setMode(Mode::MANUAL);
        ap.startManeuver(legs);
        dist.setSeaState(seaState);
        // 用极点配置增益（覆盖全部海况行，避免查询回退默认增益）
        const double Kp = wnTarget * wnTarget * T / K;
        const double Kd = (2.0 * zetaTarget * wnTarget * T - 1.0) / K;
        ap.scheduleMut().setTable({{0.0, 100.0, 0, Kp, Kd},
                                   {0.0, 100.0, 1, Kp, Kd},
                                   {0.0, 100.0, 2, Kp, Kd},
                                   {0.0, 100.0, 3, Kp, Kd}});
        auto tp = ap.tunerMut().params(); tp.highSpeedDeactivateKn = -1.0;
        ap.tunerMut().setParams(tp);

        const int totalSteps = static_cast<int>(totalSec / dt);
        double maxH = 0.0, maxR = 0.0;
        for (int k = 0; k < totalSteps; ++k) {
            double t = k * dt;
            SensorInput s;
            s.headingDeg = enableDisturbance ? dist.noisyHeading(plant.psi) : plant.psi;
            s.rateDegS   = enableDisturbance ? dist.noisyRate(plant.r) : plant.r;
            s.rudderDeg = plant.deltaAct; s.speedKn = speedKn; s.seaState = seaState;
            double cmd = ap.step(s, dt);
            plant.step(cmd, dt, enableDisturbance ? dist.disturbAngle(t) : 0.0);
            maxH = std::max(maxH, std::abs(plant.psi));
            maxR = std::max(maxR, std::abs(cmd));
            if (k % recordEvery == 0) {
                Sample sm; sm.t = t; sm.psi = plant.psi;
                sm.psiRef = 0.0; sm.rudderCmd = cmd; sm.rudderAct = plant.deltaAct;
                sm.rate = plant.r; sm.err = 0.0;
                sm.Kp = ap.pd().params().Kp; sm.Kd = ap.pd().params().Kd;
                sm.mode = static_cast<int>(ap.mode());
                lastSamples.push_back(sm);
            }
        }
        ManeuverResult r;
        r.maxHeading = maxH;
        r.maxRudder = maxR;
        // 完成判据：机动序列在仿真时长内真实执行完毕——
        //   (a) 序列器 done()：末段触发条件已满足（zigzag/circles/cloverleaf/search 等）；
        //   (b) 或已进入末段且末段为 trigger=NONE 的保持段（williamson/uTurn 的
        //       末段 HEADING 保持永不推进，到达该段即视为完成）。
        // 若自动驾驶仪无输出，航向不变，HEADING_REACHED 触发永不满足，序列停在
        // 首段，completed=false。另要求全程数值有限（发散/NaN 判 FAIL）。
        const int lastIdx = static_cast<int>(legs.size()) - 1;
        const bool reachedFinalHold = lastIdx >= 0
            && ap.sequencer().currentIndex() == lastIdx
            && legs.back().trigger == Leg::Trigger::NONE;
        r.seqDone = ap.sequencer().done() || reachedFinalHold;
        r.completed = r.seqDone && std::isfinite(maxH) && std::isfinite(maxR);
        return r;
    }

    // 全验证矩阵 + 实时性测量：跑带干扰的阶跃，测指标 + 单拍耗时
    struct ValidationResult {
        double overshootPct, settlingTime, ess1Sigma, essBias, rudderFreq;
        double maxStepMicros;  // 单拍最大耗时 (μs)
        bool pass;
    };
    ValidationResult runValidation(double K, double T, double speedKn, int seaState,
                                    double stepDeg, bool withDisturbance, double totalSec) {
        plant.K = K; plant.T = T; plant.reset(0.0);
        ap.setMode(Mode::AUTO_HEADING);
        ap.setHeadingRef(0.0);
        const double Kp = wnTarget * wnTarget * T / K;
        const double Kd = (2.0 * zetaTarget * wnTarget * T - 1.0) / K;
        ap.scheduleMut().setTable({{0.0, 100.0, 0, Kp, Kd},
                                   {0.0, 100.0, 1, Kp, Kd},
                                   {0.0, 100.0, 2, Kp, Kd},
                                   {0.0, 100.0, 3, Kp, Kd}});
        auto tp = ap.tunerMut().params(); tp.highSpeedDeactivateKn = -1.0;
        ap.tunerMut().setParams(tp);
        dist.setSeaState(seaState);
        // 注意：不写成员 enableDisturbance——本函数全程使用局部 withDisturbance，
        // 避免跨调用残留状态

        SimResult sr;
        sr.samples.clear();
        lastSamples.clear();
        const int totalSteps = static_cast<int>(totalSec / dt);
        const int holdSteps = static_cast<int>(10.0 / dt);
        double prevRudderCmd = 0.0;
        int dirChanges = 0;
        std::vector<double> errSteady;
        double maxStepMicros = 0.0;

        for (int k = 0; k < totalSteps; ++k) {
            double t = k * dt;
            if (k == holdSteps) ap.setHeadingRef(stepDeg);

            SensorInput s;
            s.headingDeg = withDisturbance ? dist.noisyHeading(plant.psi) : plant.psi;
            s.rateDegS   = withDisturbance ? dist.noisyRate(plant.r) : plant.r;
            s.rudderDeg  = plant.deltaAct;
            s.speedKn    = speedKn;
            s.seaState  = seaState;

            // 实时性测量
            auto t0 = std::chrono::steady_clock::now();
            double cmd = ap.step(s, dt);
            auto t1 = std::chrono::steady_clock::now();
            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            if (us > maxStepMicros) maxStepMicros = us;

            plant.step(cmd, dt, withDisturbance ? dist.disturbAngle(t) : 0.0);

            if (k % recordEvery == 0) {
                Sample sm; sm.t = t; sm.psi = plant.psi;
                sm.psiRef = (k >= holdSteps) ? stepDeg : 0.0;
                sm.rudderCmd = cmd; sm.rudderAct = plant.deltaAct; sm.rate = plant.r;
                sm.err = sm.psiRef - plant.psi; sm.Kp = ap.pd().params().Kp;
                sm.Kd = ap.pd().params().Kd; sm.mode = static_cast<int>(ap.mode());
                sr.samples.push_back(sm);
                lastSamples.push_back(sm);
            }
            if (k > 0) {
                double d = cmd - prevRudderCmd;
                if (std::abs(d) > 0.1 && ((prevRudderCmd > 0 && cmd < 0) || (prevRudderCmd < 0 && cmd > 0)))
                    ++dirChanges;
            }
            prevRudderCmd = cmd;
            if (k > totalSteps - static_cast<int>(20.0 / dt) && k >= holdSteps)
                errSteady.push_back((k >= holdSteps ? stepDeg : 0.0) - plant.psi);
        }
        // 复用 computeMetrics 逻辑
        computeMetrics(sr, stepDeg, 10.0, totalSec, dirChanges, errSteady);
        ValidationResult vr;
        vr.overshootPct = sr.overshootPct;
        vr.settlingTime = sr.settlingTime;
        vr.ess1Sigma = sr.ess1Sigma;
        vr.essBias = sr.essBias;
        vr.rudderFreq = sr.rudderFreq;
        vr.maxStepMicros = maxStepMicros;
        // PASS：指标达标 + 单拍耗时 < 5 ms（50 Hz 周期 20 ms，留 4 倍余量）
        vr.pass = sr.pass && maxStepMicros < 5000.0;
        return vr;
    }

private:
    // tStepSec：阶跃施加时刻 (s)。超调与调节时间只统计阶跃之后的数据。
    void computeMetrics(SimResult& res, double stepDeg, double tStepSec, double totalSec,
                        int dirChanges, const std::vector<double>& errSteady) {
        if (res.samples.empty()) return;
        // 超调：符号感知，且忽略阶跃前保持段。
        //   正阶跃取 max(ψ)-stepDeg；负阶跃取 stepDeg-min(ψ)
        double overshoot = 0.0;
        {
            bool any = false;
            double extremum = 0.0;
            for (const auto& s : res.samples) {
                if (s.t < tStepSec) continue;
                if (!any) { extremum = s.psi; any = true; }
                else if (stepDeg >= 0.0) extremum = std::max(extremum, s.psi);
                else                     extremum = std::min(extremum, s.psi);
            }
            if (any) overshoot = (stepDeg >= 0.0) ? (extremum - stepDeg)
                                                  : (stepDeg - extremum);
        }
        res.overshootPct = (std::abs(stepDeg) > 1e-9) ? (overshoot / std::abs(stepDeg)) * 100.0 : 0.0;

        // 5% 调节时间：从阶跃施加时刻起算，误差进入 ±5%·|stepDeg| 带后不再离开
        // 所需的时间（= 阶跃后最后一次越带时刻 - tStepSec；全程不越带为 0；
        // 迟迟不进带则接近总时长，自然判 FAIL）
        const double band = 0.05 * std::abs(stepDeg);
        double lastExit = tStepSec;
        for (const auto& s : res.samples) {
            if (s.t < tStepSec) continue;
            if (std::abs(s.err) > band) lastExit = s.t;
        }
        res.settlingTime = (lastExit > tStepSec) ? (lastExit - tStepSec) : 0.0;

        // 稳态 1σ 与偏置 |mean|（偏置暴露 PD 无积分在恒风下的系统性误差）
        if (!errSteady.empty()) {
            double mean = 0.0;
            for (double e : errSteady) mean += e;
            mean /= errSteady.size();
            double var = 0.0;
            for (double e : errSteady) var += (e - mean) * (e - mean);
            var /= errSteady.size();
            res.ess1Sigma = std::sqrt(var);
            res.essBias = std::abs(mean);
        }

        // 操舵频次
        res.rudderFreq = dirChanges * 60.0 / totalSec;

        // 判 PASS/FAIL（§1.5，原阈值不变；新增稳态偏置判据 |mean| ≤ 1°）
        res.pass = (res.overshootPct <= 10.0)
                && (res.settlingTime <= 30.0)
                && (res.ess1Sigma <= 1.0)
                && (res.essBias <= 1.0)
                && (res.rudderFreq <= 6.0);
    }
};

} // namespace ar::sim
