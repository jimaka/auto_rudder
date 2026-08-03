// main.cpp — 闭环仿真执行器（对应开发文档 §10.2）
//   用法：ar_sim --scenario step --K 0.2 --T 8 --seconds 120 --out step.csv
#include "simulator.hpp"
#include "scenarios.hpp"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace ar;
using namespace ar::sim;

static void printHelp() {
    std::printf("Usage: ar_sim [options]\n");
    std::printf("  --list              列出内置场景\n");
    std::printf("  --all               跑全部默认矩阵\n");
    std::printf("  --scenario <name>   指定场景名\n");
    std::printf("  --ident             跑 RLS 闭环辨识场景\n");
    std::printf("  --trial             跑自动 Z 形辨识试验（试验后自动出辨识 K/T 并应用极点配置增益）\n");
    std::printf("  --esc               跑 ESC 长期场景\n");
    std::printf("  --maneuver <name>   跑六机动形态场景 (williamson/uturn/zigzag/circles/cloverleaf/search)\n");
    std::printf("  --validate          跑全验证矩阵 + 实时性测量\n");
    std::printf("  --K <val>           覆盖船参数 K\n");
    std::printf("  --T <val>           覆盖船参数 T\n");
    std::printf("  --speed <val>       覆盖船速 (kn)\n");
    std::printf("  --sea <val>         覆盖海况 (0-3)，缩放风/浪/噪声扰动强度（1=基准）\n");
    std::printf("  --dist              启用风/浪/噪声干扰\n");
    std::printf("  --seconds <val>     覆盖总时长 (s)\n");
    std::printf("  --out <path>        导出 CSV 路径\n");
}

// 安全数值解析：非法输入返回 false 而不是让 std::stod/stoi 抛异常终止进程
static bool parseDouble(const std::string& s, double& out) {
    try {
        size_t pos = 0;
        out = std::stod(s, &pos);
        return pos == s.size();
    } catch (...) { return false; }
}
static bool parseInt(const std::string& s, int& out) {
    try {
        size_t pos = 0;
        out = std::stoi(s, &pos);
        return pos == s.size();
    } catch (...) { return false; }
}

// 各机动的默认仿真时长（s）：序列必须能在该时长内真实执行完毕。
// cloverleaf 为 4×(270° 回转 + 20 s 直航)，典型船 (K=0.2) 约需 380 s；
// search 为边长递增方搜，典型船约需 250 s；其余 300 s 足够。
static double defaultManeuverSec(const std::string& m) {
    if (m == "cloverleaf") return 480.0;
    if (m == "search")     return 360.0;
    return 300.0;
}

int main(int argc, char** argv) {
    std::string scenarioName = "step_typical";
    std::string outPath;
    std::string maneuverName;
    bool runAll = false;
    bool listOnly = false;
    bool runIdent = false;
    bool runTrial = false;
    bool runEscMode = false;
    bool runValidateMode = false;
    bool runManeuverMode = false;
    double Koverride = -1.0, Toverride = -1.0, secOverride = -1.0;
    double speedOverride = -1.0;
    int    seaOverride = -1;
    bool   distOverride = false;
    bool   hasDistOverride = false;

    // 统一的位置无关参数解析：所有模式标志与选项顺序任意；
    // 未知参数或缺/错参数值 → 报错 + 用法 + 非零退出
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        // 取下一参数值；缺失时报错
        auto takeVal = [&](const char* flag, std::string& val) -> bool {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "错误: %s 缺少参数值\n", flag);
                return false;
            }
            val = argv[++i];
            return true;
        };
        std::string v;
        if (a == "--list") listOnly = true;
        else if (a == "--all") runAll = true;
        else if (a == "--ident") runIdent = true;
        else if (a == "--trial") runTrial = true;
        else if (a == "--esc") runEscMode = true;
        else if (a == "--validate") runValidateMode = true;
        else if (a == "--dist") { distOverride = true; hasDistOverride = true; }
        else if (a == "--maneuver") {
            if (!takeVal("--maneuver", v)) { printHelp(); return 1; }
            runManeuverMode = true; maneuverName = v;
        }
        else if (a == "--scenario") {
            if (!takeVal("--scenario", v)) { printHelp(); return 1; }
            scenarioName = v;
        }
        else if (a == "--K" || a == "--T" || a == "--seconds" || a == "--speed") {
            if (!takeVal(a.c_str(), v)) { printHelp(); return 1; }
            double d = 0.0;
            if (!parseDouble(v, d)) {
                std::fprintf(stderr, "错误: %s 的参数 '%s' 不是有效数值\n", a.c_str(), v.c_str());
                return 1;
            }
            if (a == "--K") Koverride = d;
            else if (a == "--T") Toverride = d;
            else if (a == "--seconds") secOverride = d;
            else speedOverride = d;
        }
        else if (a == "--sea") {
            if (!takeVal("--sea", v)) { printHelp(); return 1; }
            if (!parseInt(v, seaOverride)) {
                std::fprintf(stderr, "错误: --sea 的参数 '%s' 不是有效整数\n", v.c_str());
                return 1;
            }
        }
        else if (a == "--out") {
            if (!takeVal("--out", v)) { printHelp(); return 1; }
            outPath = v;
        }
        else if (a == "--help" || a == "-h") { printHelp(); return 0; }
        else {
            std::fprintf(stderr, "错误: 未知参数 '%s'\n", a.c_str());
            printHelp();
            return 1;
        }
    }

    auto matrix = defaultMatrix();

    if (listOnly) {
        for (const auto& s : matrix) std::printf("%s\n", s.name.c_str());
        return 0;
    }

    if (runIdent) {
        // RLS 闭环辨识场景
        Simulator sim;
        sim.dt = 0.02;
        double K = (Koverride > 0) ? Koverride : 0.2;
        double T = (Toverride > 0) ? Toverride : 8.0;
        double sec = (secOverride > 0) ? secOverride : 120.0;
        auto r = sim.runIdent(K, T, 10.0, sec);
        std::printf("RLS 辨识结果: K=%.4f (真值 %.2f), T=%.4f (真值 %.2f), valid=%d excited=%d\n",
                    r.Khat, K, r.That, T, r.valid, r.excited);
        bool ok = r.valid && std::abs(r.Khat - K) < 0.05 && std::abs(r.That - T) < 2.0;
        std::printf("%s\n", ok ? "PASS" : "FAIL");
        if (!outPath.empty()) { sim.exportLastCsv(outPath); std::printf("CSV 导出: %s\n", outPath.c_str()); }
        return ok ? 0 : 1;
    }

    // --trial 跑自动 Z 形辨识试验（P14）
    if (runTrial) {
        Simulator sim;
        sim.dt = 0.02;
        const double K = (Koverride > 0) ? Koverride : 0.2;
        const double T = (Toverride > 0) ? Toverride : 8.0;
        const double sec = (secOverride > 0) ? secOverride : 300.0;
        const double speedKn = (speedOverride > 0) ? speedOverride : 10.0;
        auto r = sim.runIdentTrial(K, T, speedKn, sec);
        const double errK = (r.Khat - K) / K * 100.0;
        const double errT = (r.That - T) / T * 100.0;
        std::printf("Z形辨识试验: K=%.3f→K̂=%.4f (%+.1f%%)  T=%.1f→T̂=%.2f (%+.1f%%)  valid=%d  用时 %.0fs\n",
                    K, r.Khat, errK, T, r.That, errT, (int)r.identValid, r.trialSec);
        std::printf("增益: 试验前 Kp=%.2f Kd=%.2f → 试验后 Kp=%.4f Kd=%.4f  applied=%d\n",
                    r.KpOld, r.KdOld, r.KpNew, r.KdNew, (int)r.applied);
        if (!outPath.empty()) { sim.exportLastCsv(outPath); std::printf("CSV 导出: %s\n", outPath.c_str()); }
        // PASS：收官应用成功 + 辨识误差 ≤25%（与 --ident 容差一致）
        const bool ok = r.applied && std::abs(errK) <= 25.0 && std::abs(errT) <= 25.0;
        std::printf("%s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    }

    // --esc 跑 ESC 长期场景
    if (runEscMode) {
        Simulator sim;
        sim.dt = 0.02;
        double K = (Koverride > 0) ? Koverride : 0.2;
        double T = (Toverride > 0) ? Toverride : 8.0;
        // 默认 480 s：辨识收敛（约 1~2 min）+ 基线建立后再留数个 60 s ESC 评估窗口
        double sec = (secOverride > 0) ? secOverride : 480.0;
        auto r = sim.runEsc(K, T, 10.0, sec);
        std::printf("ESC 长期: Kp %.4f→%.4f Kd %.4f→%.4f | 辨识 K=%.4f (真值 %.2f) T=%.4f (真值 %.2f) valid=%d | baseline=%d(t=%.0fs) moved=%d stable=%d\n",
                    r.Kp0, r.KpFinal, r.Kd0, r.KdFinal,
                    r.Khat, K, r.That, T, r.identValid ? 1 : 0,
                    r.baselineEstablished ? 1 : 0, r.baselineSec,
                    r.gainsMoved ? 1 : 0, r.stable ? 1 : 0);
        // PASS：辨识收敛且物理合理 + 基线建立（ESC 真正启动过）+ 增益确实移动 + 数值稳定有界
        // 任一不满足即 FAIL——ESC 从未启动时增益与初值逐位相同，moved=0，必然 FAIL
        bool ok = r.identValid && r.baselineEstablished && r.gainsMoved && r.stable;
        std::printf("%s\n", ok ? "PASS" : "FAIL");
        if (!outPath.empty()) { sim.exportLastCsv(outPath); std::printf("CSV 导出: %s\n", outPath.c_str()); }
        return ok ? 0 : 1;
    }

    // --validate 跑全验证矩阵 + 实时性测量
    if (runValidateMode) {
        double sec = (secOverride > 0) ? secOverride : 120.0;
        std::printf("%-22s %8s %8s %8s %8s %8s %10s %s\n",
                    "case","overshoot","settle","ess1σ","bias","freq","maxStepμs","PASS");
        bool allPass = true;
        // 典型 + 边界船参，无干扰/有干扰（带干扰工况用海况 1 基准扰动幅值）
        struct V { const char* name; double K, T; bool dist; };
        std::vector<V> vs = {
            {"typical_nodist", 0.2, 8.0, false},
            {"typical_dist",   0.2, 8.0, true},
            {"fast_nodist",    0.5, 3.0, false},
            {"slow_nodist",    0.1, 20.0, false},
        };
        for (auto& v : vs) {
            // 每次新建仿真器以隔离状态
            Simulator s; s.dt = 0.02;
            auto r = s.runValidation(v.K, v.T, 10.0, v.dist ? 1 : 0, 30.0, v.dist, sec);
            std::printf("%-22s %7.2f%% %7.2fs %7.3f %7.3f %7.2f %9.1f %s\n",
                        v.name, r.overshootPct, r.settlingTime, r.ess1Sigma, r.essBias,
                        r.rudderFreq, r.maxStepMicros, r.pass?"PASS":"FAIL");
            if (!outPath.empty()) {
                std::string p = outPath;
                auto dot = p.find_last_of('.');
                if (dot != std::string::npos)
                    p = p.substr(0, dot) + "_" + v.name + p.substr(dot);
                else
                    p = p + "_" + v.name;
                s.exportLastCsv(p);
                std::printf("CSV 导出: %s\n", p.c_str());
            }
            if (!r.pass) allPass = false;
        }
        std::printf("%s\n", allPass ? "ALL PASS" : "SOME FAIL");
        return allPass ? 0 : 1;
    }

    // --maneuver <name> 跑六机动形态场景
    if (runManeuverMode) {
        Simulator sim; sim.dt = 0.02;
        double K = (Koverride > 0) ? Koverride : 0.2;
        double T = (Toverride > 0) ? Toverride : 8.0;
        double sec = (secOverride > 0) ? secOverride : defaultManeuverSec(maneuverName);
        double speedKn = (speedOverride > 0) ? speedOverride : 10.0;
        bool   dist = hasDistOverride ? distOverride : false;
        // 未显式指定海况时：带干扰默认海况 1（基准扰动幅值），无干扰为 0
        int    seaState = (seaOverride >= 0) ? seaOverride : (dist ? 1 : 0);
        sim.enableDisturbance = dist;
        std::vector<ar::Leg> legs;
        if (maneuverName == "williamson") legs = ManeuverSequencer::williamson(0.0);
        else if (maneuverName == "uturn")  legs = ManeuverSequencer::uTurn(0.0, true);
        else if (maneuverName == "zigzag") legs = ManeuverSequencer::zigzag(0.0, 10.0, 15.0, 3);
        else if (maneuverName == "circles") legs = ManeuverSequencer::circles(15.0, 360.0);
        // 直航段时长随 T 缩放：慢船出转超调大，20 s 直航拉不回目标航向（四叶畸变），tLeg≈2.5T 才够消超调
        // 注意签名为 cloverleaf(psi0, tLeg, dMax)
        else if (maneuverName == "cloverleaf") legs = ManeuverSequencer::cloverleaf(0.0, std::max(20.0, 2.5 * T), 20.0);
        else if (maneuverName == "search")   legs = ManeuverSequencer::search(0.0, 20.0, 10.0, 4);
        else { std::fprintf(stderr, "未知机动: %s\n", maneuverName.c_str()); return 1; }
        auto r = sim.runManeuver(legs, K, T, speedKn, seaState, sec);
        bool ok = r.completed && r.maxRudder <= 36.0;  // 序列真实执行完毕 + 限幅生效（含余量）
        std::printf("机动 %-12s K=%.2f T=%.2f v=%.1fkn sea=%d dist=%d: completed=%d seqDone=%d maxH=%.1f° maxR=%.1f° %s\n",
                    maneuverName.c_str(), K, T, speedKn, seaState, (int)dist,
                    r.completed, r.seqDone, r.maxHeading, r.maxRudder, ok?"PASS":"FAIL");
        if (!outPath.empty()) { sim.exportLastCsv(outPath); std::printf("CSV 导出: %s\n", outPath.c_str()); }
        return ok ? 0 : 1;
    }

    std::vector<Scenario> targets;
    if (runAll) targets = matrix;
    else {
        bool found = false;
        for (const auto& s : matrix) if (s.name == scenarioName) { targets.push_back(s); found = true; break; }
        if (!found) { std::printf("场景未找到: %s\n", scenarioName.c_str()); return 1; }
    }

    std::printf("%-16s %6s %6s %8s %8s %8s %8s %8s %s\n",
                "scenario", "K", "T", "overshoot", "settle", "ess1σ", "bias", "freq", "PASS");
    bool allPass = true;
    for (auto sc : targets) {
        if (Koverride > 0) sc.K = Koverride;
        if (Toverride > 0) sc.T = Toverride;
        if (speedOverride > 0) sc.speedKn = speedOverride;   // 进入增益调度查询
        if (seaOverride >= 0) sc.seaState = seaOverride;     // 进入增益调度查询 + 扰动强度
        if (secOverride > 0) sc.totalSec = secOverride;

        Simulator sim;
        sim.plant.K = sc.K;
        sim.plant.T = sc.T;
        sim.dt = 0.02;
        if (hasDistOverride) sim.enableDisturbance = distOverride;
        SimResult r = sim.runStep(sc.speedKn, sc.seaState, sc.stepDeg, sc.holdSec, sc.totalSec);

        std::printf("%-16s %6.2f %6.2f %7.2f%% %7.2fs %7.3f %7.3f %7.2f %s\n",
                    sc.name.c_str(), sc.K, sc.T,
                    r.overshootPct, r.settlingTime, r.ess1Sigma, r.essBias, r.rudderFreq,
                    r.pass ? "PASS" : "FAIL");
        if (!r.pass) allPass = false;

        if (!outPath.empty()) {
            std::string p = outPath;
            if (targets.size() > 1) {
                auto dot = p.find_last_of('.');
                if (dot != std::string::npos)
                    p = p.substr(0, dot) + "_" + sc.name + p.substr(dot);
                else
                    p = p + "_" + sc.name;
            }
            sim.exportCsv(r, p);
            std::printf("CSV 导出: %s\n", p.c_str());
        }
    }
    std::printf("%s\n", allPass ? "ALL PASS" : "SOME FAIL");
    return allPass ? 0 : 1;
}
