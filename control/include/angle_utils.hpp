// angle_utils.hpp
// 角度归一化与差值计算（对应文档 §3.2 角度归一化）
#pragma once
#include <cmath>

namespace ar {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

// 将角度归一化到 (-π, π]，单位：弧度
inline double normalizeAngleRad(double a) {
    while (a > PI)  a -= 2.0 * PI;
    while (a <= -PI) a += 2.0 * PI;
    return a;
}

// 将角度归一化到 (-180, 180]，单位：度
inline double normalizeAngleDeg(double a) {
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

// 最小带符号角度差 (ref - act)，结果落在 (-π, π]
inline double angleDiffRad(double ref, double act) {
    return normalizeAngleRad(ref - act);
}

// 度版本
inline double angleDiffDeg(double ref, double act) {
    return normalizeAngleDeg(ref - act);
}

} // namespace ar
