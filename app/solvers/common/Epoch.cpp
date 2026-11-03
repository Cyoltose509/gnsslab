#include "Epoch.h"

#include <limits>

#include "CoordConvert.h"

namespace Solvers {
    void PlotData::insert(const int index, const Epoch &ep, const XYZ &refECEF) {
        times.push_back(index);
        const auto &result = ep.sppResult;
        const bool solved = ep.solved;
        constexpr double dopNaN = std::numeric_limits<double>::quiet_NaN();
        sigmaPs.push_back(solved ? result.sigmaP : 0.0);
        sigmaPFloatVals.push_back(solved && result.sigmaPFloat > 0 ? result.sigmaPFloat : dopNaN);
        sigmaPFixedVals.push_back(solved && result.sigmaPFixed > 0 ? result.sigmaPFixed : dopNaN);
        sigmaVs.push_back(solved ? result.sigmaV : 0.0);
        pdops.push_back(solved ? result.pdop : dopNaN);
        gdops.push_back(solved ? result.gdop : dopNaN);
        hdops.push_back(solved ? result.hdop : dopNaN);
        vdops.push_back(solved ? result.vdop : dopNaN);
        tdops.push_back(solved ? result.tdop : dopNaN);


        if (ep.rtkStat != 0) ratioVals.push_back(ep.rtkRatio);
        else if (solved) ratioVals.push_back(result.ratio);
        else ratioVals.push_back(std::numeric_limits<double>::quiet_NaN());


        if (solved) {
            auto enu = Coord::XYZToENU(result.xyz, refECEF);
            enu_e.push_back(enu[0]);
            enu_n.push_back(enu[1]);
            enu_u.push_back(enu[2]);
        } else {
            enu_e.push_back(0);
            enu_n.push_back(0);
            enu_u.push_back(0);
        }


        if (solved) {
            Eigen::Vector3d fxyz;
            if (ep.rtkStat != 0) fxyz = ep.rtkFloatXyz;
            else if (result.xyzFloat.squaredNorm() > 1.0) fxyz = result.xyzFloat;
            else fxyz = result.xyz;
            auto fenu = Coord::XYZToENU(fxyz, refECEF);
            float_e.push_back(fenu[0]);
            float_n.push_back(fenu[1]);
            float_u.push_back(fenu[2]);
        } else {
            constexpr double nan = std::numeric_limits<double>::quiet_NaN();
            float_e.push_back(nan);
            float_n.push_back(nan);
            float_u.push_back(nan);
        }

        if (solved) {
            for (auto &[satID, res]: result.postRes) {
                satResTimes[satID].push_back(index);
                satResVals[satID].push_back(res);
            }
            for (auto &[satID, res]: result.postResPhase) {
                satResPhaseTimes[satID].push_back(index);
                satResPhaseVals[satID].push_back(res);
            }
        }
        newed = true;
    }

    void PlotData::refreshENU(const std::vector<Epoch> &ep, const XYZ &refECEF) {
        for (int i = 0; i < static_cast<int>(enu_e.size()); i++) {
            const auto &r = ep[i].sppResult;
            if (ep[i].solved) {
                auto enu = Coord::XYZToENU(r.xyz, refECEF);
                enu_e[i] = enu[0];
                enu_n[i] = enu[1];
                enu_u[i] = enu[2];
            } else {
                enu_e[i] = 0;
                enu_n[i] = 0;
                enu_u[i] = 0;
            }

            if (ep[i].solved) {
                Eigen::Vector3d fxyz;
                if (ep[i].rtkStat != 0) fxyz = ep[i].rtkFloatXyz;
                else if (r.xyzFloat.squaredNorm() > 1.0) fxyz = r.xyzFloat;
                else fxyz = r.xyz;
                auto fenu = Coord::XYZToENU(fxyz, refECEF);
                float_e[i] = fenu[0];
                float_n[i] = fenu[1];
                float_u[i] = fenu[2];
            } else {
                constexpr double nan = std::numeric_limits<double>::quiet_NaN();
                float_e[i] = nan;
                float_n[i] = nan;
                float_u[i] = nan;
            }
        }
    }

    void PlotData::clear() {
        times.clear();
        sigmaPs.clear();
        sigmaPFloatVals.clear();
        sigmaPFixedVals.clear();
        sigmaVs.clear();
        pdops.clear();
        gdops.clear();
        hdops.clear();
        vdops.clear();
        tdops.clear();
        enu_e.clear();
        enu_n.clear();
        enu_u.clear();
        ratioVals.clear();
        float_e.clear();
        float_n.clear();
        float_u.clear();
        satResTimes.clear();
        satResVals.clear();
        satResPhaseTimes.clear();
        satResPhaseVals.clear();
        resRangeReady = false;
        resYlo = -8.0;
        resYhi = 8.0;
        resPhaseRangeReady = false;
        resPhaseYlo = -0.5;
        resPhaseYhi = 0.5;
    }
}
