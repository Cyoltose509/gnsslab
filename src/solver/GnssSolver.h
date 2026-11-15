#pragma once

#include "core/equ/Result.h"
#include "core/obs/ObsStruct.h"
#include "core/obs/Weight.h"
#include "core/geom/Ephemeris.h"
#include "core/SatID.h"
#include "Velocity.h"
#include <map>
#include <memory>
#include <set>

struct Products;

/// 单站/双站解算器的公共底座
class GnssSolver {
public:
    GnssSolver();

    virtual ~GnssSolver() = default;

    /// 解算输入（星历/格网/精密产品/接收机天线）；正序与倒序解算器共用同一份，谁都不会被搬空。
    /// RTK 只装星历那一栏，其余留空。
    std::shared_ptr<Products> prod;

    Result result{};

    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};

protected:
    /// 广播电文自报不健康的卫星直接剔除
    bool ephUnhealthy(const SatID &sat, const Ephemeris *eph, bool enabled);

    std::set<SatID> mHealthWarned;

    /// 多普勒测速
    Velocity::DopplerResult estimateVelocityByDoppler(const ObsData &obs, const std::map<SatID, PVT> &pvtMap,
                                                      const SatValueMap &elevMap,
                                                      const Eigen::Vector3d &pos, const Weight &w);

    /// 本星是否参与多普勒测速
    [[nodiscard]] virtual bool wantDopplerSat(const SatID &, const TypeValueMap &) const {
        return true;
    }
};
