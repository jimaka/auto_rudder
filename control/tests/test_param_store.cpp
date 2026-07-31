// test_param_store.cpp — 参数存储序列化/CRC/范围检查（对应开发文档 §3.9.8 + P8 整改）
#include "param_store.hpp"
#include <gtest/gtest.h>
#include <vector>
using namespace ar;

static ControlParams makeSample() {
    ControlParams p;
    p.rudderZero = -1.5; p.rudderLeft = -28.0; p.rudderRight = 28.0;
    p.ddotMax = 12.0;
    p.K = 0.18; p.T = 9.0;
    p.Kp = 1.6; p.Kd = 8.6;
    p.schedTable = {
        {0.0, 4.0, 0, 2.0, 0.8},
        {4.0, 10.0, 0, 1.5, 0.6},
        {10.0, 18.0, 0, 1.2, 0.5},
        {18.0, 25.0, 0, 1.0, 0.4},
    };
    p.version = 5;
    p.timestamp = 1234567;
    return p;
}

TEST(ParamStore, SerializeRoundtrip) {
    auto p = makeSample();
    p.updateCrc();
    auto buf = p.serialize();
    ASSERT_FALSE(buf.empty());

    ControlParams q;
    ASSERT_TRUE(q.deserialize(buf.data(), buf.size()));
    EXPECT_DOUBLE_EQ(q.rudderZero, p.rudderZero);
    EXPECT_DOUBLE_EQ(q.rudderLeft, p.rudderLeft);
    EXPECT_DOUBLE_EQ(q.rudderRight, p.rudderRight);
    EXPECT_DOUBLE_EQ(q.ddotMax, p.ddotMax);
    EXPECT_DOUBLE_EQ(q.K, p.K);
    EXPECT_DOUBLE_EQ(q.T, p.T);
    EXPECT_DOUBLE_EQ(q.Kp, p.Kp);
    EXPECT_DOUBLE_EQ(q.Kd, p.Kd);
    EXPECT_EQ(q.version, p.version);
    EXPECT_EQ(q.timestamp, p.timestamp);
    ASSERT_EQ(q.schedTable.size(), p.schedTable.size());
    for (size_t i = 0; i < q.schedTable.size(); ++i) {
        EXPECT_DOUBLE_EQ(q.schedTable[i].speedMin, p.schedTable[i].speedMin);
        EXPECT_DOUBLE_EQ(q.schedTable[i].speedMax, p.schedTable[i].speedMax);
        EXPECT_EQ(q.schedTable[i].seaState, p.schedTable[i].seaState);
        EXPECT_DOUBLE_EQ(q.schedTable[i].Kp, p.schedTable[i].Kp);
        EXPECT_DOUBLE_EQ(q.schedTable[i].Kd, p.schedTable[i].Kd);
    }
}

TEST(ParamStore, CrcDetectsCorruption) {
    auto p = makeSample();
    p.updateCrc();
    auto buf = p.serialize();
    // 篡改一个字节（不碰 crc，crc 在 p.crc 字段，buf 不含 crc）
    buf[5] ^= 0xFF;
    ControlParams q;
    ASSERT_TRUE(q.deserialize(buf.data(), buf.size()));
    q.crc = p.crc;  // 用原 crc 校验
    EXPECT_FALSE(q.verifyCrc());
}

TEST(ParamStore, CrcSelfExclusion) {
    // P8 整改：CRC 范围不应包含 crc 字段自身
    // serialize() 不含 crc，updateCrc() 后 verifyCrc() 应通过
    auto p = makeSample();
    p.updateCrc();
    EXPECT_TRUE(p.verifyCrc());
    // 改 crc 字段本身不应影响校验通过性（crc 不在 CRC 计算范围内）
    uint32_t savedCrc = p.crc;
    p.crc = 0xDEADBEEF;
    EXPECT_FALSE(p.verifyCrc());  // 错的 crc 当然不通过
    p.crc = savedCrc;
    EXPECT_TRUE(p.verifyCrc());
}

TEST(ParamStore, ValidateRangeRejectsBad) {
    auto p = makeSample();
    EXPECT_TRUE(p.validateRange());
    p.K = -0.1;  // K 必须 > 0
    EXPECT_FALSE(p.validateRange());
    p = makeSample();
    p.T = 0.5;   // T 必须 >= 1
    EXPECT_FALSE(p.validateRange());
    p = makeSample();
    p.ddotMax = 100.0;  // ddotMax 必须 <= 30
    EXPECT_FALSE(p.validateRange());
    p = makeSample();
    p.Kd = -1.0;  // Kd 必须 >= 0
    EXPECT_FALSE(p.validateRange());
}

TEST(ParamStore, SaveLoadRoundtrip) {
    // 用内存缓冲模拟 EEPROM
    std::vector<uint8_t> storage(4096, 0);
    auto readFn = [&](uint8_t* buf, size_t len) -> bool {
        std::memcpy(buf, storage.data(), std::min(len, storage.size()));
        return true;
    };
    auto writeFn = [&](const uint8_t* buf, size_t len) -> bool {
        if (len > storage.size()) storage.resize(len);
        std::memcpy(storage.data(), buf, len);
        return true;
    };
    uint32_t fakeClock = 1000;
    auto clockFn = [&]() -> uint32_t { return fakeClock; };

    ParamStore store(readFn, writeFn, clockFn);
    auto p = makeSample();
    ASSERT_TRUE(store.save(p));
    EXPECT_EQ(p.timestamp, 1000);  // 自动填时间戳
    EXPECT_EQ(p.version, 6u);     // 版本号 +1

    ControlParams q;
    ASSERT_TRUE(store.load(q));
    EXPECT_DOUBLE_EQ(q.K, p.K);
    EXPECT_DOUBLE_EQ(q.Kd, p.Kd);
    EXPECT_EQ(q.version, 6u);
    EXPECT_EQ(q.timestamp, 1000u);
    EXPECT_TRUE(q.verifyCrc());
    EXPECT_TRUE(q.validateRange());
}

TEST(ParamStore, LoadDetectsCorruption) {
    std::vector<uint8_t> storage(4096, 0);
    auto readFn = [&](uint8_t* buf, size_t len) -> bool {
        std::memcpy(buf, storage.data(), std::min(len, storage.size()));
        return true;
    };
    auto writeFn = [&](const uint8_t* buf, size_t len) -> bool {
        if (len > storage.size()) storage.resize(len);
        std::memcpy(storage.data(), buf, len);
        return true;
    };
    ParamStore store(readFn, writeFn, {});
    auto p = makeSample();
    ASSERT_TRUE(store.save(p));
    // 篡改 payload 中间一字节
    storage[10] ^= 0xFF;
    ControlParams q;
    EXPECT_FALSE(store.load(q));  // CRC 不匹配
}

TEST(ParamStore, LittleEndianPortable) {
    // P8 整改：显式小端，跨平台一致
    auto p = makeSample();
    p.rudderZero = 1.0;
    auto buf = p.serialize();
    // rudderZero 是第一个 double，小端 8 字节：3F F0 00 00 00 00 00 00
    EXPECT_EQ(buf[0], 0x00);
    EXPECT_EQ(buf[1], 0x00);
    EXPECT_EQ(buf[2], 0x00);
    EXPECT_EQ(buf[3], 0x00);
    EXPECT_EQ(buf[4], 0x00);
    EXPECT_EQ(buf[5], 0x00);
    EXPECT_EQ(buf[6], 0xF0);
    EXPECT_EQ(buf[7], 0x3F);
}
