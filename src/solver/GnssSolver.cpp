#include "GnssSolver.h"

#include "Products.h"

#include "Velocity.h"
#include "core/Const.h"
#include "Log.h"
#include "i18n/I18n.h"


GnssSolver::GnssSolver() : prod(std::make_shared<Products>()) {}


bool GnssSolver::ephUnhealthy(const SatID &sat, const Ephemeris *eph, const bool enabled) {
    if (!enabled || !eph || eph->health == 0u) return false;
    if (mHealthWarned.insert(sat).second)
        LOG_WARN << "[HEALTH] " << I18n::tr("log.unhealthy", sat.toString(), eph->health);
    return true;
}


Velocity::DopplerResult GnssSolver::estimateVelocityByDoppler(const ObsData &obs,
                                                              const std::map<SatID, PVT> &pvtMap,
                                                              const SatValueMap &elevMap,
                                                              const Eigen::Vector3d &pos,
                                                              const Weight &w) {
    std::vector<Velocity::DopplerSat> list;
    list.reserve(pvtMap.size());
    for (const auto &[sat, pvt]: pvtMap) {
        const auto it = obs.satTypeValueData.find(sat);
        if (it == obs.satTypeValueData.end() || !wantDopplerSat(sat, it->second)) continue;

        // 第一频点的多普勒；拿不到就不参与（不拿别的频点凑数，波长对不上）。
        double doppler = 0.0;
        bool found = false;
        for (const auto &[k, v]: it->second) {
            if (k.size() >= 2 && k[0] == 'D' && k[1] == '1') {
                doppler = v;
                found = true;
                break;
            }
        }
        if (!found) continue;

        Velocity::DopplerSat ds;
        ds.sat = sat;
        ds.pos = pvt.p;
        ds.vel = pvt.v;
        ds.freqHz = sat.system == 'R' ? getGlonassFreq('1', sat.chan) : getFreq(sat.system, "L1");
        ds.dopplerHz = doppler;
        ds.posWeight = w.weight(elevMap.at(sat));
        list.push_back(ds);
    }
    const Velocity::DopplerResult v = Velocity::byDoppler(pos, list);
    // 少于 4 颗（!ok）：速度留 0，交给导出侧的空值逻辑
    if (v.ok) {
        result.vel = v.vel;
        result.sigmaVel = v.sigma;
        result.sigmaV = v.sigma.norm();
    }
    return v;
}
