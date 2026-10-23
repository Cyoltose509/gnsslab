#pragma once
#include "GnssStruct.h"
#include "MathUtils.h" // Math::rotationZ（负角约定，详见 rotEcefZ 注释）
#include <vector>
#include <map>
#include <string>

#include "Ephemeris.h"

/// 精密轨道 SP3 读取器（支持 SP3c 的 P/V 记录）。
class Sp3OrbitReader {
public:
    static constexpr int kSp3FieldW = 14; // 每个数值字段宽度
    static constexpr int kSp3IdLen = 4; // 'P'/'V' + 3 字符星号

    static SatID parseSatId(const std::string &s) {
        if (s.size() < 3) return {};
        return {s[0], std::stoi(s.substr(1, 2))};
    }

    // SP3 用 999999.999999 表示缺值
    static bool isValid(const double v) { return std::fabs(v) < 999999.0; }

    // 按定宽列读取第 i 个数值字段(0..3)，字段缺失/不可解析时返回 false。
    static bool getField(const std::string &line, const int i, double &out) {
        const size_t pos = kSp3IdLen + kSp3FieldW * static_cast<size_t>(i);
        if (pos >= line.size()) return false;
        const std::string f = line.substr(pos, kSp3FieldW);
        const size_t a = f.find_first_not_of(' ');
        if (a == std::string::npos) return false;
        try { out = std::stod(f.substr(a)); } catch (...) { return false; }
        return !std::isnan(out);
    }
    struct Record {
        CommonTime t;
        Eigen::Vector3d pos; // 单位 m
        Eigen::Vector3d vel; // 单位 m/s
        double clock = 0.0; // 钟差，单位 s
        double clockRate = 0.0; // 钟速，单位 s/s
        bool hasVel = false;
        bool hasClock = false; // 该文件本历元 P 行给出钟差列
        bool hasClockRate = false; // 同上，V 行给出钟速列
    };

    /// 读取 SP3 文件。成功返回 true。
    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    /// SP3 是否自带钟差
    [[nodiscard]] bool hasClock() const { return anyClock; }

    /// 卫星是否可用
    [[nodiscard]] bool contains(const SatID &sat) const {
        const auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }

    /// 卫星在时刻 t 是否落在 SP3 时间跨度内
    [[nodiscard]] bool covers(const SatID &sat, const CommonTime &t) const {
        const auto it = data.find(sat);
        if (it == data.end() || it->second.empty()) return false;
        // 端点容差取常见 SP3 最大采样间隔(15 min)，避免边界历元被误判覆盖不足
        constexpr double tol = 900.0;
        return t - it->second.front().t >= -tol && it->second.back().t - t >= -tol;
    }

    /// 在某时刻 t 插值出卫星 PVT。
    [[nodiscard]] PVT getPVT(const SatID &sat, const CommonTime &t) const;

    /// 数据时间跨度（用于诊断）
    [[nodiscard]] std::pair<CommonTime, CommonTime> timeSpan(const SatID &sat) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }

    /// 只读访问内部卫星→记录表（自动探测参考卫星等场景用）。
    [[nodiscard]] const std::map<SatID, std::vector<Record> > &getData() const { return data; }

private:
    std::map<SatID, std::vector<Record> > data;
    std::map<SatID, std::vector<double> > relTimes;
    bool anyClock = false; // 该文件是否含钟差列（read() 时置位）
};
