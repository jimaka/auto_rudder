// scenarios.hpp — 验证矩阵场景定义（对应开发文档 §10.2）
#pragma once
#include "simulator.hpp"
#include <string>
#include <vector>

namespace ar::sim {

struct Scenario {
    std::string name;
    double K, T;          // 船参数
    double speedKn;
    int    seaState;
    double stepDeg;
    double holdSec;
    double totalSec;
};

// §10.2 验证矩阵：每项跑典型参数与边界参数
// 海况取 1（基准扰动幅值）：默认不启用扰动时海况不影响结果；
// 一旦加 --dist 即得到基准强度扰动，--sea 0 则为平静近零扰动
inline std::vector<Scenario> defaultMatrix() {
    return {
        // 30° 阶跃场景 × 三组船参
        {"step_typical", 0.2, 8.0, 10.0, 1, 30.0, 10.0, 120.0},
        {"step_fast",    0.5, 3.0, 10.0, 1, 30.0, 10.0, 120.0},
        {"step_slow",    0.1, 20.0, 10.0, 1, 30.0, 10.0, 180.0},
    };
}

// RLS 辨识场景：Z 形激励 + 阶跃，验证在线辨识收敛到真值
struct IdentScenario {
    std::string name;
    double K, T;
    double speedKn;
    double totalSec;
};

// ESC 长期场景：跑足够长让 ESC 窗口触发，验证增益不发散且性能指标下降
struct EscScenario {
    std::string name;
    double K, T;
    double speedKn;
    double totalSec;  // 应 ≥ 数个评估窗口
};

// 六机动形态场景：验证各预置机动序列能完整跑完且不发散
struct ManeuverSimScenario {
    std::string name;
    double K, T;
    double speedKn;
    double totalSec;
};

// 全验证矩阵 + 实时性测量
struct ValidationScenario {
    std::string name;
    double K, T;
    double speedKn;
    int    seaState;
    double stepDeg;
    bool   withDisturbance;
    double totalSec;
};

} // namespace ar::sim
