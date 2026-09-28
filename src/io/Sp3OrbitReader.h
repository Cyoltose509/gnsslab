#pragma once
#include "GnssStruct.h"
#include <vector>
#include <map>
#include <string>

/// 精密轨道 SP3 读取器（支持 SP3c 的 P/V 记录）。
class Sp3OrbitReader {
public:
    struct Record {
        CommonTime t;
        Eigen::Vector3d pos; // 单位 m
        Eigen::Vector3d vel; // 单位 m/s
        double clock = 0.0;      // 钟差，单位 s
        double clockRate = 0.0;  // 钟速，单位 s/s
        bool hasVel = false;
        bool hasClock = false; // 该文件本历元 P 行给出钟差列
        bool hasClockRate = false; // 同上，V 行给出钟速列
    };

    /// 读取 SP3 文件。成功返回 true。
    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    /// SP3 是否自带钟差（超快/最终产品含钟差列，此时无需单独的 CLK 文件）。
    /// 轨道-only 产品钟差列为 0，调用方仍需另给 CLK。
    [[nodiscard]] bool hasClock() const { return anyClock; }

    /// 卫星是否可用
    [[nodiscard]] bool contains(const SatID &sat) const {
        const auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }

    /// 在某时刻 t 插值出卫星 PVT。
    [[nodiscard]] PVT getPVT(const SatID &sat, const CommonTime &t) const;

    /// 数据时间跨度（用于诊断）
    [[nodiscard]] std::pair<CommonTime, CommonTime> timeSpan(const SatID &sat) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }

    /// 只读访问内部卫星→记录表（自动探测参考卫星等场景用）。
    [[nodiscard]] const std::map<SatID, std::vector<Record>> &getData() const { return data; }

private:
    std::map<SatID, std::vector<Record> > data;
    // 每卫星相对首历元的秒数向量(读入时一次性预计算)，避免 getPVT 每次调用都 O(n) 拷贝全部轨道历元时间。
    std::map<SatID, std::vector<double> > relTimes;
    bool anyClock = false; // 该文件是否含钟差列（read() 时置位）
};
