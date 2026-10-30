#pragma once

#include <string>
#include <vector>
#include <map>
#include <utility>
#include <functional>

#include "Const.h"
#include "SolverTask.h"
#include "imgui.h"

namespace Solvers::Charts {

    struct SatRow {
        SatID sat;
        bool used = true;
        double elevDeg = 0;
        double azimDeg = 0;
        bool hasAzim = false;
        double satX = 0, satY = 0, satZ = 0;
        bool hasSatXYZ = false;
        std::vector<std::pair<std::string, std::string>> extra;
    };

    void renderSatelliteTable(const std::vector<SatRow> &rows, int &selectedSatIdx,
                              const char *id = "##satTable");
    void renderENUPlot(const std::vector<double> &times,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed, bool fixedY = false,
                       bool hasFloat = false,
                       const std::vector<double> &float_e = {},
                       const std::vector<double> &float_n = {},
                       const std::vector<double> &float_u = {},
                       int epochCount = -1,
                       float plotH = 380.0f);

    void renderRatioPlot(const std::vector<double> &times,
                         const std::vector<double> &ratioVals,
                         int &selectedEpoch, bool &newed, int epochCount = -1);

    void renderSatResidualPlot(const std::map<SatID, std::vector<double>> &satResT,
                               const std::map<SatID, std::vector<double>> &satResV,
                               const std::map<SatID, std::vector<double>> &phaseT,
                               const std::map<SatID, std::vector<double>> &phaseV,
                               int *resMode,
                               int &selectedEpoch, int epochCount, bool &newed,
                               const double *codeLo, const double *codeHi,
                               const double *phaseLo, const double *phaseHi,
                               float plotH = 0.0f);

    struct SatRowView {
        SatID sat;
        bool used = true;
        double elevRad = 0, azimRad = 0;
        bool hasXYZ = false;
        double x = 0, y = 0, z = 0;
    };
    struct SkyView {
        SatID sat;
        double azimRad = 0, elevRad = 0;
        bool used = true;
    };

  template<typename Proj, typename Fill>
    std::vector<SatRow> buildSatRows(const size_t n, Proj &&proj, Fill &&fill) {
        std::vector<SatRow> rows;
        rows.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            SatRow sr;
            const SatRowView v = proj(i);
            sr.sat = v.sat;
            sr.used = v.used;
            sr.hasAzim = true;
            sr.elevDeg = v.elevRad * RAD_TO_DEG;
            sr.azimDeg = v.azimRad * RAD_TO_DEG;
            if (v.hasXYZ) {
                sr.hasSatXYZ = true;
                sr.satX = v.x; sr.satY = v.y; sr.satZ = v.z;
            }
            fill(sr, i);
            rows.push_back(std::move(sr));
        }
        return rows;
    }

  template<typename Proj>
    std::vector<std::pair<SatID, GnssTask::SkyPoint> > buildCurSkyPoints(const int epIdx, const size_t n, Proj &&proj, const double minElevDeg = -1.0) {
        std::vector<std::pair<SatID, GnssTask::SkyPoint> > pts;
        for (size_t i = 0; i < n; ++i) {
            SkyView v = proj(i);
            const double elDeg = v.elevRad * RAD_TO_DEG;
            if (elDeg <= minElevDeg) continue;
            pts.emplace_back(v.sat,
                GnssTask::SkyPoint{epIdx, static_cast<float>(v.azimRad * RAD_TO_DEG), static_cast<float>(elDeg), v.used});
        }
        return pts;
    }

    void RenderSkyplot(const std::map<SatID, std::vector<GnssTask::SkyPoint>> &tracks,
                       const std::vector<std::pair<SatID, GnssTask::SkyPoint>> &curPts,
                       float sizePx = 360.0f);

    void RenderLeo3D(const Eigen::Vector3d &leoPos,
                     const std::vector<GnssTask::TrajPoint> &leoTraj,
                     const std::vector<GnssTask::TrajPoint> &refTraj,
                     float sizePx = 460.0f);

    struct ChartVisibility {
        bool satTable  = true;
        bool posPanel  = true;
        bool plot3d    = true;
        bool residual  = true;
        bool rtn       = true;
        bool rms       = true;
    };

    struct PosTabView {
        int epochCount = 0;
        bool busy = false;
        int selectedIdx = -1;
        int *selectedEpoch = nullptr;
        int *selectedSatIdx = nullptr;
        Eigen::Vector3d *refECEF = nullptr;
        std::function<void()> onRefChanged;
        bool showRefEnu = true;
        bool showRefInput = true;

        std::string statusText;
        ImVec4 statusColor = ImVec4(0.3f, 1, 0.3f, 1);

        std::string epochInfo;

        std::vector<SatRow> satRows;
        bool hasSatRows = false;

        bool showSatTable = true;
        bool showPosPanel = true;

        bool showResidual = true;

        bool showObsDetail = false;
        TypeValueMap detailObs;
        char detailSystem = 0;


        bool solved = false;
        Eigen::Vector3d xyz{0, 0, 0}, enu{0, 0, 0}, blh{0, 0, 0};
        bool showBaseXyz = false;
        Eigen::Vector3d *baseXyz = nullptr;
        double sigmaP = 0;
        bool showZtd = false; double ztd = 0;
        bool showVelDop = false;
        bool showVel = true;
        Eigen::Vector3d vel{0, 0, 0};
        double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
        int numSatsResult = 0, numObs = 0;
        std::string noSolveMsg;


        bool showSolveStatus = false;
        std::string solveStatusText;
        ImVec4 solveStatusColor = ImVec4(1, 1, 1, 1);

        bool fixedY = false;
        std::vector<double> times, enu_e, enu_n, enu_u;
        std::vector<double> enu_solved;
        bool newed = false;
        bool showEnPlan = true;
        std::string refCenterLabel;
        bool showRatio = false;
        std::vector<double> ratioVals;
        bool hasFloatSeries = false;
        std::vector<double> float_e, float_n, float_u;

        std::map<SatID, std::vector<GnssTask::SkyPoint>> skyTracks;
        std::vector<std::pair<SatID, GnssTask::SkyPoint>> curSkyPts;
        std::map<SatID, std::vector<double>> resT, resV;


        std::map<SatID, std::vector<double>> resPhaseT, resPhaseV;


        bool hasResidualData = false;
        bool robustRes = false; double resYlo = -8, resYhi = 8;
        bool robustResPhase = false; double resPhaseYlo = -0.5, resPhaseYhi = 0.5;
        int *resMode = nullptr;

        bool showSigmaDop = false;
        bool isRtk = false;
        bool showSigmaV = true;
        bool showPdop = true;
        std::vector<double> sigmaPs, sigmaVs, pdops;
        std::vector<double> sigmaPFloatVals;
        std::vector<double> gdops, hdops, vdops, tdops;


        bool showFixRate = false;
        std::string fixRateText;

        bool showRtn = false;
        bool hasRef = false;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        bool showRms = false;
        std::vector<double> rmsCode, rmsPhase;

        bool show3d = false;
        ChartVisibility *renderToggles = nullptr;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        Eigen::Vector3d leoRef3d{0, 0, 0};
        std::vector<GnssTask::SatVis> gnssVis;
        std::vector<GnssTask::TrajPoint> leoTraj;
        std::vector<GnssTask::TrajPoint> leoRefTraj;
        std::map<SatID, std::vector<GnssTask::TrajPoint>> gnssTraj;


        bool hasFixedDistinction = false;
        bool hasFixed = false;
        Eigen::Vector3d xyzFixedDisp{0, 0, 0};
        Eigen::Vector3d xyzFloatDisp{0, 0, 0};
        double sigmaFixedDisp = 0, sigmaFloatDisp = 0;
        bool hasBaseline = false;
        Eigen::Vector3d baseFloatDisp{0, 0, 0};
        Eigen::Vector3d baseFixedDisp{0, 0, 0};
        bool hasVelDisp = false;
        Eigen::Vector3d velDisp{0, 0, 0};
        double sigmaVDisp = 0;

        bool showRefTable = false;
        bool isLeoRtn = false;
        Eigen::Vector3d refDisp{0, 0, 0};
        double abs3dDisp = 0;
        Eigen::Vector3d enuFixedDisp{0, 0, 0};
        Eigen::Vector3d enuFloatDisp{0, 0, 0};
        double enuFixed3d = 0, enuFloat3d = 0;
        Eigen::Vector3d rtnDisp{0, 0, 0};
        double rtn3dDisp = 0;

        std::string solStatShort;
        double solRatio = 0;

        bool isDone = false;
        std::function<void()> onExportCsv;
    };

    void RenderPositioningTab(const PosTabView &v);

}
