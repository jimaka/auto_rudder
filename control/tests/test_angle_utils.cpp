// test_angle_utils.cpp — 角度归一化与差值（对应开发文档 §3.6 前置用例）
#include "angle_utils.hpp"
#include <gtest/gtest.h>
using namespace ar;

TEST(AngleUtils, NormalizePositiveBoundary) {
    // 左开右闭：(−180, 180]，+180 必须保留为 +180
    EXPECT_DOUBLE_EQ(normalizeAngleDeg(180.0), 180.0);
}

TEST(AngleUtils, NormalizeNegativeBoundary) {
    // -180 应折叠为 +180（左开）
    EXPECT_DOUBLE_EQ(normalizeAngleDeg(-180.0), 180.0);
}

TEST(AngleUtils, NormalizePositiveFold) {
    EXPECT_DOUBLE_EQ(normalizeAngleDeg(540.0), 180.0);   // 540 - 360 = 180
}

TEST(AngleUtils, NormalizeNegativeFold) {
    EXPECT_NEAR(normalizeAngleDeg(-541.0), 179.0, 1e-12);
}

TEST(AngleUtils, DiffAcrossZeroPositive) {
    EXPECT_NEAR(angleDiffDeg(1.0, 359.0), 2.0, 1e-12);
}

TEST(AngleUtils, DiffAcrossZeroNegative) {
    EXPECT_NEAR(angleDiffDeg(359.0, 1.0), -2.0, 1e-12);
}

TEST(AngleUtils, RadDegConsistency) {
    EXPECT_NEAR(normalizeAngleRad(normalizeAngleDeg(170.0) * DEG2RAD),
                170.0 * DEG2RAD, 1e-12);
}
