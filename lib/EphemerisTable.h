#pragma once

#include <map>
#include <fstream>
#include <memory>
#include "GnssStruct.h"
#include "Ephemeris.h"

/// 星历表：拥有所有星历数据的所有权（深拷贝），支持多线程安全的浅拷贝
struct EphemerisTable {
    std::map<int, std::shared_ptr<GPSEphem>> gps;
    std::map<int, std::shared_ptr<BDSEphem>> bds;
    // 未来: std::map<int, GlonassEphem> glo;
    // 未来: std::map<int, GalileoEphem> gal;

    /// 按卫星 ID 查星历（返回非 const 指针，svPVT 会修改内部缓存）
    [[nodiscard]] Ephemeris* find(const SatID &sat) {
        if (sat.system == 'G') {
            const auto it = gps.find(sat.id);
            return it != gps.end() ? it->second.get() : nullptr;
        }
        if (sat.system == 'C') {
            const auto it = bds.find(sat.id);
            return it != bds.end() ? it->second.get() : nullptr;
        }
        return nullptr;
    }

    [[nodiscard]] bool empty() const { return gps.empty() && bds.empty(); }
};
