#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>

#include "imgui.h"
#include "GuiCharts.h"
#include "ui/GuiColors.h"

namespace Solvers::Charts {

    /// 定位页的视图模型。图表序列与天空图轨迹只持有引用，指向 View 里跨帧复用的副本；
    /// 逐历元的派生量（当前历元的坐标、DOP、表格行）每帧现算，体积只与卫星数有关。
    /// `s` 是序列本体（派生量/断点轴），`p` 是它逐历元的数值。
    struct PosTabView {
        PosTabView(const PlotSeries &series, const std::map<SatID, std::vector<SkyPoint> > &sky)
            : s(series), p(series.data), skyTracks(sky) {
        }

        const PlotSeries &s;
        const PlotData &p;
        const std::map<SatID, std::vector<SkyPoint> > &skyTracks;

        int epochCount = 0;
        bool busy = false;
        int *selectedEpoch = nullptr;
        int *selectedSatIdx = nullptr;
        int *resMode = nullptr;
        Eigen::Vector3d *refECEF = nullptr;
        std::function<void()> onRefChanged;
        bool showRefEnu = true;

        std::string statusText;
        ImVec4 statusColor = UI::kColPass;

        std::string epochInfo;

        std::vector<SatRow> satRows;
        bool hasSatRows = false;

        bool showObsDetail = false;
        TypeValueMap detailObs;
        char detailSystem = 0;


        bool solved = false;
        double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
        int numSatsResult = 0, numObs = 0;
        std::string noSolveMsg;


        bool showSolveStatus = false;
        ImVec4 solveStatusColor = ImVec4(1, 1, 1, 1);

        bool showEnPlan = true;
        bool hasFloatSeries = false;

        std::vector<std::pair<SatID, SkyPoint> > curSkyPts;

        bool showSigmaDop = false;
        bool isRtk = false;
        bool showSigmaV = true;

        bool showFixRate = false;
        std::string fixRateText;

        bool showRtn = false;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        bool show3d = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;


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

        std::function<void()> onExportCSV;
    };

    void renderPositioningTab(const PosTabView &v);

}
