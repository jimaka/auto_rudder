// gain_schedule.hpp
// 增益调度表（对应文档 §3.6 / §3.10.4 按船速分段）
#pragma once
#include <vector>
#include <algorithm>

namespace ar {

struct GainPoint {
    double speedMin;   // kn
    double speedMax;   // kn
    int    seaState;   // 0..3
    double Kp;
    double Kd;
};

class GainSchedule {
public:
    GainSchedule() {
        // §3.10.4 默认分段（与文档表一致）
        // < 4 kn, sea 0~1
        table_.push_back({0.0,   4.0, 0, 2.0, 0.8});
        table_.push_back({0.0,   4.0, 1, 2.0, 0.8});
        // 4~10 kn
        table_.push_back({4.0,  10.0, 0, 1.5, 0.6});
        table_.push_back({4.0,  10.0, 1, 1.5, 0.6});
        table_.push_back({4.0,  10.0, 2, 1.0, 0.4});
        table_.push_back({4.0,  10.0, 3, 1.0, 0.4});
        // 10~18 kn
        table_.push_back({10.0, 18.0, 0, 1.2, 0.5});
        table_.push_back({10.0, 18.0, 1, 1.2, 0.5});
        table_.push_back({10.0, 18.0, 2, 0.9, 0.4});
        table_.push_back({10.0, 18.0, 3, 0.9, 0.4});
        // 18~25 kn
        table_.push_back({18.0, 25.0, 0, 1.0, 0.4});
        table_.push_back({18.0, 25.0, 1, 1.0, 0.4});
        table_.push_back({18.0, 25.0, 2, 0.7, 0.3});
        table_.push_back({18.0, 25.0, 3, 0.7, 0.3});
    }

    // 按船速与海况查询，速度维中点锚定线性插值（P5 整改：消除段切换跳变）
    // 名义增益落在每段中点 m_i，v 在 [m_i, m_{i+1}] 间线性插值；边界外取首/末段名义值。
    void query(double speedKn, int seaState, double& Kp, double& Kd) const {
        seaState = std::max(0, std::min(3, seaState));

        const GainPoint* prev = nullptr;
        double prevMid = 0.0;
        for (const auto& pt : table_) {
            if (pt.seaState != seaState) continue;
            const double mid = 0.5 * (pt.speedMin + pt.speedMax);

            // 低于最低段中点 → 取首段名义值
            if (prev == nullptr) {
                if (speedKn <= mid) { Kp = pt.Kp; Kd = pt.Kd; return; }
                prev = &pt;
                prevMid = mid;
                continue;
            }

            // 在相邻中点间插值；单次遍历避免控制周期内额外分配内存。
            if (speedKn <= mid) {
                const double span = mid - prevMid;
                const double f = (span > 1e-9) ? (speedKn - prevMid) / span : 0.0;
                Kp = prev->Kp + f * (pt.Kp - prev->Kp);
                Kd = prev->Kd + f * (pt.Kd - prev->Kd);
                return;
            }
            prev = &pt;
            prevMid = mid;
        }

        if (prev == nullptr) { Kp = 1.0; Kd = 0.5; return; }
        // 高于最高段中点（NaN 也落入兜底）→ 取末段名义值
        Kp = prev->Kp; Kd = prev->Kd;
    }

    const std::vector<GainPoint>& table() const { return table_; }
    // P5 配套：允许标定结果或仿真场景替换整表（开发文档 §4.4）
    void setTable(const std::vector<GainPoint>& t) { table_ = t; }

private:
    std::vector<GainPoint> table_;
};

} // namespace ar
