// param_store.hpp
// 参数存储与 CRC 校验（对应文档 §3.9.8 / §3.10.9）
//   P8 整改：
//   - 逐字段显式小端序列化，CRC 范围排除 crc 字段自身，消除填充字节依赖
//   - 新增区块 D（增益调度表）
//   - save 自动填 timestamp（注入时钟函数，便于测试）
//   - load 后 validateRange() 物理范围检查
//   纯算法层：实际 EEPROM/Flash 读写由 BSP 层注入（IO 接口）
#pragma once
#include "gain_schedule.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <functional>

namespace ar {

// CRC32 (IEEE 802.3 多项式 0xEDB88320)
inline uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// 小端写入工具
inline void writeLEu32(std::vector<uint8_t>& buf, uint32_t v) {
    for (int i = 0; i < 4; ++i) buf.push_back(static_cast<uint8_t>((v >> (8*i)) & 0xFF));
}
inline void writeLEf64(std::vector<uint8_t>& buf, double v) {
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>((u >> (8*i)) & 0xFF));
}
inline bool readLEu32(const uint8_t* buf, size_t len, size_t& off, uint32_t& out) {
    if (off + 4 > len) return false;
    out = 0;
    for (int i = 0; i < 4; ++i) out |= static_cast<uint32_t>(buf[off+i]) << (8*i);
    off += 4;
    return true;
}
inline bool readLEf64(const uint8_t* buf, size_t len, size_t& off, double& out) {
    if (off + 8 > len) return false;
    uint64_t u = 0;
    for (int i = 0; i < 8; ++i) u |= static_cast<uint64_t>(buf[off+i]) << (8*i);
    std::memcpy(&out, &u, sizeof(out));
    off += 8;
    return true;
}

// 控制参数集（与 EEPROM 区块 A~D 对应）
struct ControlParams {
    // 区块 A：舵角标定
    double rudderZero = 0.0;     // N0
    double rudderLeft = 0.0;     // NL
    double rudderRight = 0.0;    // NR
    double ddotMax = 10.0;       // 舵速限幅 (deg/s)
    // 区块 B：模型参数
    double K = 0.0;
    double T = 0.0;
    // 区块 C：PD 增益
    double Kp = 1.0;
    double Kd = 0.5;
    // 区块 D：增益调度表（P8 新增）
    std::vector<GainPoint> schedTable;
    // 元信息
    uint32_t version = 0;
    uint32_t timestamp = 0;
    uint32_t crc = 0;

    // P8 整改：逐字段显式序列化（不含 crc 字段）
    // 格式：[8×double 区块A/B/C][version u32][timestamp u32][schedCount u32][sched 表]
    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> buf;
        buf.reserve(8*8 + 4*3 + schedTable.size()*(8*4+4));
        writeLEf64(buf, rudderZero);
        writeLEf64(buf, rudderLeft);
        writeLEf64(buf, rudderRight);
        writeLEf64(buf, ddotMax);
        writeLEf64(buf, K);
        writeLEf64(buf, T);
        writeLEf64(buf, Kp);
        writeLEf64(buf, Kd);
        writeLEu32(buf, version);
        writeLEu32(buf, timestamp);
        writeLEu32(buf, static_cast<uint32_t>(schedTable.size()));
        for (const auto& pt : schedTable) {
            writeLEf64(buf, pt.speedMin);
            writeLEf64(buf, pt.speedMax);
            writeLEu32(buf, static_cast<uint32_t>(pt.seaState));
            writeLEf64(buf, pt.Kp);
            writeLEf64(buf, pt.Kd);
        }
        return buf;
    }

    // 计算并写入 CRC（CRC 范围为 serialize() 的全部字节，不含 crc 字段自身）
    void updateCrc() {
        auto buf = serialize();
        crc = crc32(buf.data(), buf.size());
    }

    // 校验 CRC（当前 crc 字段与重算值比对）
    bool verifyCrc() const {
        auto buf = serialize();
        return crc32(buf.data(), buf.size()) == crc;
    }

    // P8 整改：物理范围检查
    bool validateRange() const {
        if (!std::isfinite(rudderZero) || !std::isfinite(rudderLeft) || !std::isfinite(rudderRight)) return false;
        if (!std::isfinite(ddotMax) || ddotMax < 1.0 || ddotMax > 30.0) return false;
        if (!std::isfinite(K) || K <= 0.0 || K > 2.0) return false;
        if (!std::isfinite(T) || T < 1.0 || T > 60.0) return false;
        if (!std::isfinite(Kp) || Kp <= 0.0) return false;
        if (!std::isfinite(Kd) || Kd < 0.0) return false;
        return true;
    }

    // P8 整改：从字节流反序列化（不含 crc），off 输出消耗字节数，返回是否格式完整
    bool deserializeFrom(const uint8_t* buf, size_t len, size_t& off) {
        if (!readLEf64(buf, len, off, rudderZero)) return false;
        if (!readLEf64(buf, len, off, rudderLeft)) return false;
        if (!readLEf64(buf, len, off, rudderRight)) return false;
        if (!readLEf64(buf, len, off, ddotMax)) return false;
        if (!readLEf64(buf, len, off, K)) return false;
        if (!readLEf64(buf, len, off, T)) return false;
        if (!readLEf64(buf, len, off, Kp)) return false;
        if (!readLEf64(buf, len, off, Kd)) return false;
        if (!readLEu32(buf, len, off, version)) return false;
        if (!readLEu32(buf, len, off, timestamp)) return false;
        uint32_t n = 0;
        if (!readLEu32(buf, len, off, n)) return false;
        schedTable.clear();
        schedTable.reserve(n);
        for (uint32_t i = 0; i < n; ++i) {
            GainPoint pt{};
            if (!readLEf64(buf, len, off, pt.speedMin)) return false;
            if (!readLEf64(buf, len, off, pt.speedMax)) return false;
            uint32_t ss = 0;
            if (!readLEu32(buf, len, off, ss)) return false;
            pt.seaState = static_cast<int>(ss);
            if (!readLEf64(buf, len, off, pt.Kp)) return false;
            if (!readLEf64(buf, len, off, pt.Kd)) return false;
            schedTable.push_back(pt);
        }
        return true;
    }
    // 便捷重载：从完整 payload 反序列化（off 从 0 起）
    bool deserialize(const uint8_t* buf, size_t len) {
        size_t off = 0;
        return deserializeFrom(buf, len, off);
    }
};

// 参数存储抽象：注入读写接口 + 时钟
class ParamStore {
public:
    using ReadFn  = std::function<bool(uint8_t* buf, size_t len)>;
    using WriteFn = std::function<bool(const uint8_t* buf, size_t len)>;
    using ClockFn = std::function<uint32_t()>;

    ParamStore(ReadFn r, WriteFn w, ClockFn c = {})
        : read_(r), write_(w), clock_(c) {}

    // 保存：填时间戳 → 版本号 +1 → 计算 CRC → 写入 [payload][crc u32]
    bool save(ControlParams& p) {
        if (clock_) p.timestamp = clock_();
        p.version += 1;
        p.updateCrc();  // 计算 crc 并存入 p.crc
        auto payload = p.serialize();
        std::vector<uint8_t> full;
        full.reserve(payload.size() + 4);
        full.insert(full.end(), payload.begin(), payload.end());
        writeLEu32(full, p.crc);  // 追加 crc 到末尾
        return write_(full.data(), full.size());
    }

    // 加载：读取 → 反序列化 payload → 读 crc → 校验
    bool load(ControlParams& p) {
        // 先读一个足够大的缓冲（最大支持 4096 字节）
        std::vector<uint8_t> full(4096, 0);
        if (!read_(full.data(), full.size())) return false;
        // 反序列化 payload（不含末尾 4 字节 crc），off 输出消耗字节数
        size_t off = 0;
        if (!p.deserializeFrom(full.data(), full.size(), off)) return false;
        // 末尾 4 字节为 crc
        if (off + 4 > full.size()) return false;
        uint32_t storedCrc = 0;
        for (int i = 0; i < 4; ++i) storedCrc |= static_cast<uint32_t>(full[off+i]) << (8*i);
        p.crc = storedCrc;
        return p.verifyCrc();
    }

private:
    ReadFn  read_;
    WriteFn write_;
    ClockFn clock_;
};

} // namespace ar
