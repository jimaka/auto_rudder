// tuner_debug_cli.cpp
// 实船调试命令行：每行一条命令，每条命令恰好调用 TunerDebugApi 的一个功能。
// 用法：
//   交互：  ./ar_tuner_debug            （stdin/stdout，可经 ssh/串口终端使用）
//   脚本：  ./ar_tuner_debug < cmds.txt （输出为 key=value 行，便于上位机解析）
// 输出约定：成功 "OK ..."，失败/拒绝 "ERR <原因>"；观测类输出 key=value 序列。
#include "tuner_debug_api.hpp"
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <iostream>

using namespace ar;

namespace {

void printHelp() {
    std::cout <<
        "实船调试命令（每条命令=一个功能，可任意顺序单独调用）\n"
        "---- 观测 ----\n"
        "  status                              全量快照（增益/基线/ESC/窗口/振荡）\n"
        "  j                                   当前窗口 ITAE 代价\n"
        "  osc                                 实测振荡峰-峰幅值 (deg)\n"
        "  params                              当前调参参数表\n"
        "---- 层2 单功能 ----\n"
        "  pole <K> <T> [zeta wn]              极点配置解析解（默认 ζ=0.85 ωn=0.2）\n"
        "  check <K> <T> <Kp> <Kd> <osc>       稳定门四条件判定\n"
        "  baseline <K> <T>                    一次性建基线（试验收官语义）\n"
        "  eval <K> <T> <osc> <driftGate> <forceRebase>   辨识评估原语\n"
        "---- ESC 各环节 ----\n"
        "  dither <Kp> <Kd>                    对给定增益打一拍载波，返回叠加后值\n"
        "  accum <e_deg> <delta_deg> [oscOk]   喂一拍样本（成本/振荡/解调）\n"
        "  step <Kp> <Kd>                      做一次窗末梯度结算，返回新增益\n"
        "  clamp <new> <old>                   单次更新限幅\n"
        "---- 振荡检测 ----\n"
        "  oscfeed <e_deg>                     只喂振荡检测器\n"
        "  oscreset                            清零振荡检测器\n"
        "---- 回退/重锚/生命周期 ----\n"
        "  revert                              取回基线增益\n"
        "  rebase                              ESC 积分/测量状态重锚\n"
        "  suspend                             模式挂起（清测量、留基线）\n"
        "  reset                               全量复位\n"
        "---- 主流程（与在线行为一致） ----\n"
        "  tick <ref> <heading> <rudder> <speedKn> <dt> <sKp> <sKd> [exc valid K T]\n"
        "                                      跑一拍完整编排，返回本拍增益\n"
        "---- 强制注入（旁路稳定门，仅调试！） ----\n"
        "  forcegains <Kp> <Kd>                直接改写当前增益\n"
        "  forcebase <Kp> <Kd>                 直接改写基线并置基线标志\n"
        "  forcephase <rad>                    直接改写载波相位\n"
        "  forcegrad <gKp> <gKd>               直接改写梯度积分器\n"
        "  forceenabled <0|1>                  直接置速度门控标志（单环节调试前置）\n"
        "---- 其他 ----\n"
        "  help                                本说明\n"
        "  quit                                退出\n";
}

void printSnapshot(const TunerDebugApi::Snapshot& s) {
    std::printf("OK curKp=%.6f curKd=%.6f baseKp=%.6f baseKd=%.6f baseK=%.4f baseT=%.4f "
                "established=%d enabled=%d phase=%.4f gradKp=%.6f gradKd=%.6f lastJ=%.4f "
                "hpfY=%.6f demodKp=%.4f demodKd=%.4f demodN=%u win=%u samples=%zu "
                "refAge=%.1f osc=%.4f\n",
                s.curKp, s.curKd, s.baselineKp, s.baselineKd, s.baseK, s.baseT,
                s.baselineEstablished, s.enabled, s.escPhase, s.gradKp, s.gradKd,
                s.lastJ, s.hpfY, s.demodKpSum, s.demodKdSum, s.demodCount,
                s.winCounter, s.windowSamples, s.refAgeSec, s.oscAmplitude);
}

void printParams(const AutoTuner::Params& p) {
    std::printf("OK zeta=%.3f wn=%.3f escAmp=%.3f escFreq=%.3f escStep=%.3f "
                "hpfTau=%.2f lpfTau=%.2f wU=%.4f wDu=%.4f maxUpdateFrac=%.2f "
                "absMinUpdate=%.4f zetaMin=%.2f wnMax=%.2f oscMax=%.1f "
                "highSpeedKn=%.1f windowSec=%.1f dt=%.3f\n",
                p.zeta, p.wn, p.escAmp, p.escFreq, p.escStep, p.escHPFtau,
                p.escLPFtau, p.wU, p.wDu, p.maxUpdateFrac, p.absMinUpdate,
                p.zetaMin, p.wnMax, p.oscMax, p.highSpeedDeactivateKn,
                p.windowSec, p.dt);
}

// 从流读取 n 个 double，不足返回 false
bool readN(std::istringstream& ss, double* out, int n) {
    for (int i = 0; i < n; ++i) {
        if (!(ss >> out[i])) return false;
    }
    return true;
}

} // namespace

int main() {
    AutoTuner tuner;
    TunerDebugApi api(tuner);
    std::string line;
    std::printf("ar_tuner_debug ready. 'help' 查看命令表。\n");

    while (std::printf("> ") && std::getline(std::cin, line)) {
        std::istringstream ss(line);
        std::string cmd;
        if (!(ss >> cmd)) continue;

        if (cmd == "quit" || cmd == "exit") break;
        if (cmd == "help") { printHelp(); continue; }

        // ---- 观测 ----
        if (cmd == "status") { printSnapshot(api.snapshot()); continue; }
        if (cmd == "j")      { std::printf("OK J=%.6f\n", api.currentJ()); continue; }
        if (cmd == "osc")    { std::printf("OK osc=%.4f\n", api.oscAmplitude()); continue; }
        if (cmd == "params") { printParams(api.params()); continue; }

        // ---- 层2 单功能 ----
        if (cmd == "pole") {
            double v[4] = {0, 0, 0.85, 0.20};
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: pole <K> <T> [zeta wn]\n"); continue; }
            ss >> v[2] >> v[3];  // 可选参数（读不到保持默认）
            double Kp, Kd;
            if (api.polePlacement(v[0], v[1], v[2], v[3], Kp, Kd))
                std::printf("OK Kp=%.6f Kd=%.6f\n", Kp, Kd);
            else
                std::printf("ERR 极点配置失败（K/T 退化或非有限）\n");
            continue;
        }
        if (cmd == "check") {
            double v[5];
            if (!readN(ss, v, 5)) { std::printf("ERR 用法: check <K> <T> <Kp> <Kd> <osc>\n"); continue; }
            std::printf("OK pass=%d\n", api.stabilityCheck(v[0], v[1], v[2], v[3], v[4]) ? 1 : 0);
            continue;
        }
        if (cmd == "baseline") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: baseline <K> <T>\n"); continue; }
            std::printf("OK established=%d\n", api.establishBaseline(v[0], v[1]) ? 1 : 0);
            continue;
        }
        if (cmd == "eval") {
            double v[5];
            if (!readN(ss, v, 5)) { std::printf("ERR 用法: eval <K> <T> <osc> <driftGate> <forceRebase>\n"); continue; }
            std::printf("OK accepted=%d\n",
                        api.evaluateEstimate(v[0], v[1], v[2], v[3] != 0.0, v[4] != 0.0) ? 1 : 0);
            continue;
        }

        // ---- ESC 各环节 ----
        if (cmd == "dither") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: dither <Kp> <Kd>\n"); continue; }
            double Kp = v[0], Kd = v[1];
            api.ditherOnce(Kp, Kd);
            std::printf("OK Kp=%.6f Kd=%.6f\n", Kp, Kd);
            continue;
        }
        if (cmd == "accum") {
            double v[3] = {0, 0, 1};
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: accum <e_deg> <delta_deg> [oscOk]\n"); continue; }
            ss >> v[2];
            api.accumulateOnce(v[0], v[1], v[2] != 0.0);
            std::printf("OK\n");
            continue;
        }
        if (cmd == "step") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: step <Kp> <Kd>\n"); continue; }
            double Kp = v[0], Kd = v[1];
            api.escStepOnce(Kp, Kd);
            std::printf("OK Kp=%.6f Kd=%.6f\n", Kp, Kd);
            continue;
        }
        if (cmd == "clamp") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: clamp <new> <old>\n"); continue; }
            std::printf("OK value=%.6f\n", api.clampUpdate(v[0], v[1]));
            continue;
        }

        // ---- 振荡检测 ----
        if (cmd == "oscfeed") {
            double v[1];
            if (!readN(ss, v, 1)) { std::printf("ERR 用法: oscfeed <e_deg>\n"); continue; }
            api.oscFeed(v[0]);
            std::printf("OK osc=%.4f\n", api.oscAmplitude());
            continue;
        }
        if (cmd == "oscreset") { api.oscReset(); std::printf("OK\n"); continue; }

        // ---- 回退/重锚/生命周期 ----
        if (cmd == "revert") {
            double Kp = 0, Kd = 0;
            api.revertToBaseline(Kp, Kd);
            std::printf("OK Kp=%.6f Kd=%.6f\n", Kp, Kd);
            continue;
        }
        if (cmd == "rebase")  { api.rebaseEsc(); std::printf("OK\n"); continue; }
        if (cmd == "suspend") { api.suspend();   std::printf("OK\n"); continue; }
        if (cmd == "reset")   { api.reset();     std::printf("OK\n"); continue; }

        // ---- 主流程 ----
        if (cmd == "tick") {
            double v[11] = {};
            if (!readN(ss, v, 7)) {
                std::printf("ERR 用法: tick <ref> <heading> <rudder> <speedKn> <dt> <sKp> <sKd> [exc valid K T]\n");
                continue;
            }
            readN(ss, v + 7, 4);  // 可选辨识快照（缺省 excited=0 valid=0）
            TunerInput in;
            in.headingRefDeg = v[0]; in.headingDeg = v[1]; in.rudderDeg = v[2];
            in.speedKn = v[3]; in.dt = v[4]; in.schedKp = v[5]; in.schedKd = v[6];
            in.id.excited = (v[7] != 0.0); in.id.valid = (v[8] != 0.0);
            in.id.K = v[9]; in.id.T = v[10];
            const GainCommand g = api.tick(in);
            std::printf("OK Kp=%.6f Kd=%.6f\n", g.Kp, g.Kd);
            continue;
        }

        // ---- 强制注入（旁路稳定门，仅调试） ----
        if (cmd == "forcegains") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: forcegains <Kp> <Kd>\n"); continue; }
            api.forceGains(v[0], v[1]);
            std::printf("OK 警告：已旁路稳定门直接写增益\n");
            continue;
        }
        if (cmd == "forcebase") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: forcebase <Kp> <Kd>\n"); continue; }
            api.forceBaseline(v[0], v[1]);
            std::printf("OK 警告：已旁路极点配置与稳定门直接写基线\n");
            continue;
        }
        if (cmd == "forcephase") {
            double v[1];
            if (!readN(ss, v, 1)) { std::printf("ERR 用法: forcephase <rad>\n"); continue; }
            api.forceEscPhase(v[0]);
            std::printf("OK\n");
            continue;
        }
        if (cmd == "forcegrad") {
            double v[2];
            if (!readN(ss, v, 2)) { std::printf("ERR 用法: forcegrad <gKp> <gKd>\n"); continue; }
            api.forceGradient(v[0], v[1]);
            std::printf("OK\n");
            continue;
        }
        if (cmd == "forceenabled") {
            double v[1];
            if (!readN(ss, v, 1)) { std::printf("ERR 用法: forceenabled <0|1>\n"); continue; }
            api.forceEnabled(v[0] != 0.0);
            std::printf("OK\n");
            continue;
        }

        std::printf("ERR 未知命令 '%s'，'help' 查看命令表\n", cmd.c_str());
    }
    return 0;
}
