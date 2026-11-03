#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>

#include "imgui.h"
#include "GuiCharts.h"

namespace Solvers::Charts {

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
        ImVec4 statusColor = UI::kColPass;

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

        std::map<SatID, std::vector<Solvers::SkyPoint>> skyTracks;
        std::vector<std::pair<SatID, Solvers::SkyPoint>> curSkyPts;
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
        std::vector<Solvers::SatVis> gnssVis;
        std::vector<Solvers::TrajPoint> leoTraj;
        std::vector<Solvers::TrajPoint> leoRefTraj;
        std::map<SatID, std::vector<Solvers::TrajPoint>> gnssTraj;


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

    void renderPositioningTab(const PosTabView &v);

}
