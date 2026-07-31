// nomoto_plant.hpp — 闭环仿真被控对象（对应开发文档 §10.2）
//   Nomoto 一阶：T·dr + r = K·δ_act
//   舵机一阶惯性 + 舵速物理限幅
//   状态：r(转向率 deg/s)、psi(航向 deg)、deltaAct(实际舵角 deg)
#pragma once
#include <cmath>

namespace ar::sim {

struct NomotoPlant {
    double K = 0.2;        // 旋回性指数 (1/s)
    double T = 8.0;         // 追随性指数 (s)
    double tauS = 0.3;      // 舵机一阶时间常数 (s)
    double ddotMax = 10.0;  // 舵速物理限幅 (deg/s)
    double dMax = 35.0;     // 舵角物理限幅 (deg)

    double r = 0.0;         // 转向率 (deg/s)
    double psi = 0.0;       // 航向 (deg)
    double deltaAct = 0.0;  // 实际舵角 (deg)

    void reset(double psi0 = 0.0) {
        r = 0.0; psi = psi0; deltaAct = 0.0;
    }

    // 一步推进：deltaCmd 指令舵角(deg)、dt 步长(s)、disturb 等效扰动舵角(deg)
    void step(double deltaCmd, double dt, double disturb = 0.0) {
        // 舵机一阶惯性 + 限幅
        double ddot = (deltaCmd - deltaAct) / tauS;
        if (ddot >  ddotMax) ddot =  ddotMax;
        if (ddot < -ddotMax) ddot = -ddotMax;
        deltaAct += ddot * dt;
        if (deltaAct >  dMax) deltaAct =  dMax;
        if (deltaAct < -dMax) deltaAct = -dMax;

        // Nomoto: T·dr + r = K·(deltaAct + disturb)
        const double deltaEff = deltaAct + disturb;
        const double dr = (K * deltaEff - r) / T;
        r += dr * dt;
        psi += r * dt;
        // 航向归一化到 (-180, 180]
        while (psi >  180.0) psi -= 360.0;
        while (psi <= -180.0) psi += 360.0;
    }
};

} // namespace ar::sim
