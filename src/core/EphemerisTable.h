#pragma once

#include <map>
#include <vector>
#include <limits>
#include <memory>
#include "GnssStruct.h"
#include "Ephemeris.h"
#include "TimeConvert.h"
#include "Const.h"

struct EphemerisTable {
    std::map<int, std::vector<std::shared_ptr<GPSEphem>>> gps;
    std::map<int, std::vector<std::shared_ptr<BDSEphem>>> bds;
    std::map<int, std::vector<std::shared_ptr<GalileoEphem>>> gal;
    std::map<int, std::vector<std::shared_ptr<GlonassEphem>>> glo;

    static constexpr double EPH_MAX_AGE_SEC = 6.0 * SEC_PER_HOUR;

    static double ageSeconds(Ephemeris *e, const CommonTime &epoch) {
        if (!e) return (std::numeric_limits<double>::max)();
        const CommonTime te = e->getCommonTime();
        CommonTime ep = epoch;
        convertTimeSystem(ep, te.m_timeSystem);
        return std::fabs(ep - te);
    }

    [[nodiscard]] Ephemeris *find(const SatID &sat, const CommonTime &epoch) {
        if (sat.system == 'G') return pickClosest(gps, sat.id, epoch);
        if (sat.system == 'C') return pickClosest(bds, sat.id, epoch);
        if (sat.system == 'E') return pickClosest(gal, sat.id, epoch);
        if (sat.system == 'R') return pickClosest(glo, sat.id, epoch);
        return nullptr;
    }

    [[nodiscard]] bool empty() const {
        return gps.empty() && bds.empty() && gal.empty() && glo.empty();
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
            if (const double dt = std::fabs(ep - te); dt < bestDt) {
                bestDt = dt;
                best = e.get();
            }
        }
        // 最近的星历仍超出有效期，视为不可用。
        if (best && ageSeconds(best, epoch) > EPH_MAX_AGE_SEC) return nullptr;
        return best;
    }
};
