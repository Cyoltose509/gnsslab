#pragma once

#include <map>
#include <string>
#include <vector>

#include "GnssStruct.h"
#include "SPP.h"
#include "RTK.h"

namespace Solvers {
    struct SkyPoint {
        int epIdx = 0;
        float azDeg = 0, elDeg = 0;
        bool used = true;
    };

    struct SatVis {
        SatID sat;
        char sys = 0;
        Eigen::Vector3d pos{0, 0, 0};
        bool used = true;
    };

    struct TrajPoint {
        int epIdx = 0;
        Eigen::Vector3d pos{0, 0, 0};
    };

    struct Epoch {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        int numObs = 0;
        std::vector<SatID> satIds;
        std::vector<PVT> satPVTs;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<bool> rejected;
        std::vector<TypeValueMap> allObs;
        Result sppResult;
        int numSatsResult = 0;

        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        Eigen::Vector3d rtkBaseFloat{0, 0, 0};
        Eigen::Vector3d rtkFixedBaseline{0, 0, 0};
        Eigen::Vector3d rtkFloatXyz = Eigen::Vector3d::Zero();
        double rtkRatio = 0.0;
        int rtkStat = 0;

        void getFromObs(const ObsData &obs) {
            week = obs.weekSecond.week;
            sow = obs.weekSecond.sow;
            satIds.clear();
            elevations.clear();
            azimuths.clear();
            satPVTs.clear();
            rejected.clear();
            allObs.clear();
            for (auto &[sat, typeMap]: obs.satTypeValueData) {
                satIds.push_back(sat);
                elevations.push_back(0.0);
                azimuths.push_back(0.0);
                satPVTs.emplace_back();
                rejected.push_back(false);
                allObs.push_back(typeMap);
            }
            numObs = static_cast<int>(satIds.size());
        }

        void getFromSPP(const ::SPP &spp) {
            const auto &result = spp.result;
            const double r = result.xyz.norm();
            if (const bool posValid = r > 6.35e6 && r < 8.0e6; result.numSats > 0 && posValid) {
                solved = true;
                sppResult = result;
                fillSatFields(spp.satElevData, spp.satAzimData, spp.satPVTTransTime, spp.satRejected);
            } else {
                solved = false;
                sppResult.reset();
            }
        }

        void getFromRTK(const ::RTK &rtk) {
            const Result &r = rtk.result;
            if (r.numSats <= 0) {
                solved = false;
                return;
            }
            solved = true;
            sppResult = r;
            rtkStat = r.stat;
            rtkRatio = r.ratio;
            rtkBaseFloat = rtk.mBaselineFloat;
            if (r.stat == 1) rtkFixedBaseline = rtk.mBaselineFixed;
            rtkFloatXyz = rtk.mBaseXyz + rtk.mBaselineFloat;
            fillSatFields(rtk.satElevData, rtk.satAzimData, rtk.satPVTRecTime, rtk.mUsedSats, true);
        }

    private:
        /// 按 satIds 的顺序回填各星的仰角/方位角/PVT，并据此定出 rejected 与 numSatsResult。
        /// SPP 与 RTK 的差别只在 PVT 取发射时刻还是接收时刻、以及「被剔除」的判定依据，
        /// 故用 invertRejected 区分：SPP 给的是被拒集合，RTK 给的是使用集合。
        template<typename ElevMap, typename AzimMap, typename PVTMap, typename SatSet>
        void fillSatFields(const ElevMap &elev, const AzimMap &azim, const PVTMap &pvts,
                           const SatSet &satFlag, const bool invertRejected = false) {
            const int n = static_cast<int>(satIds.size());
            for (int i = 0; i < n; i++) {
                if (auto it = elev.find(satIds[i]); it != elev.end()) elevations[i] = it->second;
                if (auto it2 = azim.find(satIds[i]); it2 != azim.end()) azimuths[i] = it2->second;
                if (auto it3 = pvts.find(satIds[i]); it3 != pvts.end()) satPVTs[i] = it3->second;
            }
            numSatsResult = 0;
            for (int i = 0; i < n; i++) {
                const bool flagged = satFlag.count(satIds[i]) > 0;
                const bool ok = (invertRejected ? flagged : !flagged) && satPVTs[i].p.squaredNorm() > 1.0;
                rejected[i] = !ok;
                if (ok) ++numSatsResult;
            }
        }
    };

    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaPFloatVals;
        std::vector<double> sigmaPFixedVals;
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> gdops, hdops, vdops, tdops;
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;
        bool newed = false;

        std::vector<double> ratioVals;
        std::vector<double> float_e, float_n, float_u;

        bool resRangeReady = false;
        double resYlo = -8.0, resYhi = 8.0;

        bool resPhaseRangeReady = false;
        double resPhaseYlo = -0.5, resPhaseYhi = 0.5;

        std::map<SatID, std::vector<double> > satResTimes;
        std::map<SatID, std::vector<double> > satResVals;
        std::map<SatID, std::vector<double> > satResPhaseTimes;
        std::map<SatID, std::vector<double> > satResPhaseVals;

        void insert(int index, const Epoch &ep, const XYZ &refECEF);

        void refreshENU(const std::vector<Epoch> &ep, const XYZ &refECEF);

        void clear();
    };
}
