#pragma once

#include <map>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <memory>
#include "GnssStruct.h"
#include "Ephemeris.h"
#include "TimeConvert.h"

/// 星历表：拥有所有星历数据的所有权（深拷贝），支持多线程安全的浅拷贝
/// 每颗卫星保存多条星历（按 toe 不同），查询时按历元时间选取最接近的一条。
struct EphemerisTable {
    std::map<int, std::vector<std::shared_ptr<GPSEphem>>> gps;
    std::map<int, std::vector<std::shared_ptr<BDSEphem>>> bds;

    /// 按卫星 ID + 历元时间查星历：返回 toe 距该历元最近的一条。
    /// epoch 允许与星历使用不同时间系统（内部会对齐后比较）。
    [[nodiscard]] Ephemeris *find(const SatID &sat, const CommonTime &epoch) {
        if (sat.system == 'G') return pickClosest(gps, sat.id, epoch);
        if (sat.system == 'C') return pickClosest(bds, sat.id, epoch);
        return nullptr;
    }

    /// 兼容旧接口：无历元时返回该卫星最后加入的一条星历。
    [[nodiscard]] Ephemeris *find(const SatID &sat) {
        if (sat.system == 'G') {
            const auto it = gps.find(sat.id);
            return (it != gps.end() && !it->second.empty()) ? it->second.back().get() : nullptr;
        }
        if (sat.system == 'C') {
            const auto it = bds.find(sat.id);
            return (it != bds.end() && !it->second.empty()) ? it->second.back().get() : nullptr;
        }
        return nullptr;
    }

    [[nodiscard]] bool empty() const {
        return gps.empty() && bds.empty();
    }

private:
    template<typename MapT>
    static Ephemeris *pickClosest(MapT &m, const int id, const CommonTime &epoch) {
        const auto it = m.find(id);
        if (it == m.end() || it->second.empty()) return nullptr;

        Ephemeris *best = nullptr;
        double bestDt = (std::numeric_limits<double>::max)();
        for (auto &e: it->second) {
            CommonTime te = e->getCommonTime(); // 星历参考时刻(toe)，星历自身时间系统
            CommonTime ep = epoch; // 观测历元，可能是 GPS 系统
            convertTimeSystem(ep, te.m_timeSystem); // 对齐到同一时间系统再比较
            const double dt = std::fabs(ep - te);
            if (dt < bestDt) {
                bestDt = dt;
                best = e.get();
            }
        }
        return best;
    }
};
