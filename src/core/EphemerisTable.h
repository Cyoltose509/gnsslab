#pragma once

#include <map>
#include <vector>
#include <limits>
#include <memory>
#include <algorithm>
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
        Time::convertSystem(ep, te.m_timeSystem);
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

    /// 把观测源自带的星历（other 可为空）按历元补进来。
    /// 「观测源自带星历」指 NovAtel 日志里嵌的 GPSEPHEM/GLOEPHEM 等记录：reader 逐历元
    /// 累积出快照，经 Observer::Item::eph 带出。外部导航文件（BRDC/nav）之外，这是第二
    /// 个星历来源；缺了它，只给日志不给星历文件的实时单站解算就没有可用星历。
    ///
    /// 合并规则（三个约束缺一不可）：
    /// 1. 不能整体替换：日志内星历每颗星只保留最新一条（LogReader::storeEph 是覆盖而非追加），
    ///    那条可能已过期数小时，整体替换会把 BRDC 里可用的换成过期的 → 该星全程无 PVT。
    /// 2. 判缺按「该历元附近（EPH_MAX_AGE_SEC 内）是否已有**可用**星历」，而非「PRN 是否已存在」：
    ///    只按 PRN 判时，日期对不上的 BRDC 会被误认为已有而不补，日志内当天正确的星历进不来
    ///    → 整段解算全部无 PVT。
    /// 3. 目标表已有可用条目时保留不动（不覆盖），源表条目按参考时刻去重后并入，
    ///    最终由 pickClosest 按历元挑最近的一条。
    void mergeEmbedded(const std::shared_ptr<EphemerisTable> &other, const CommonTime &epoch) {
        if (!other) return;
        mergeMapForEpoch(gps, other->gps, epoch);
        mergeMapForEpoch(bds, other->bds, epoch);
        mergeMapForEpoch(gal, other->gal, epoch);
        mergeMapForEpoch(glo, other->glo, epoch);
    }

private:
    template<typename MapT>
    static void mergeMapForEpoch(MapT &dst, const MapT &src, const CommonTime &epoch) {
        for (const auto &[prn, v]: src) {
            if (v.empty()) continue;
            auto it = dst.find(prn);
            if (it == dst.end()) {
                dst[prn] = v;
                continue;
            }
            // 目标表已有该 PRN 的可用星历 → 保留（不覆盖，避免用日志内过期条目顶掉 BRDC 的）
            if (hasUsableNear(it->second, epoch)) continue;
            for (const auto &e: v) {
                const auto t = e->getCommonTime();
                const bool dup = std::any_of(it->second.begin(), it->second.end(),
                                             [&](const auto &x) {
                                                 return std::fabs(x->getCommonTime() - t) < 1e-6;
                                             });
                if (!dup) it->second.push_back(e);
            }
        }
    }

    template<typename VecT>
    static bool hasUsableNear(const VecT &v, const CommonTime &epoch) {
        for (const auto &e: v) {
            if (!e) continue;
            CommonTime te = e->getCommonTime();
            CommonTime ep = epoch;
            Time::convertSystem(ep, te.m_timeSystem);
            if (std::fabs(ep - te) <= EPH_MAX_AGE_SEC) return true;
        }
        return false;
    }

    template<typename MapT>
    static Ephemeris *pickClosest(MapT &m, const int id, const CommonTime &epoch) {
        const auto it = m.find(id);
        if (it == m.end() || it->second.empty()) return nullptr;

        Ephemeris *best = nullptr;
        double bestDt = (std::numeric_limits<double>::max)();
        for (auto &e: it->second) {
            CommonTime te = e->getCommonTime(); // 星历参考时刻(toe)，星历自身时间系统
            CommonTime ep = epoch; // 观测历元，可能是 GPS 系统
            Time::convertSystem(ep, te.m_timeSystem); // 对齐到同一时间系统再比较
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
