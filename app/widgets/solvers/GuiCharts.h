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
        bool used = true;                 // 参与定位 / 被排除
        double elevDeg = 0;               // 仰角(°)
        double azimDeg = 0;               // 方位角(°)
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
                       float plotH = 380.0f);   // 绘图区高度；由调用方按容器给，不再写死


    // E-N 平面图（含中心点拖拽）是渲染内部函数，声明在 GuiCharts.cpp 的匿名命名空间里，
    // 因为它依赖 PosTabView 的完整定义（参考坐标指针 / 回调 / ENU 序列）。


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
                               float plotH = 0.0f);   // 0 = 填满容器剩余高度（默认）

    struct SatRowView {
        SatID sat;
        bool used = true;
        double elevRad = 0, azimRad = 0;   // 弧度
        bool hasXYZ = false;
        double x = 0, y = 0, z = 0;        // 卫星 ECEF 位置 (m)
    };
    struct SkyView {
        SatID sat;
        double azimRad = 0, elevRad = 0;   // 弧度
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
        bool satTable  = true;   // 卫星概览表
        bool posPanel  = true;   // 位置面板
        bool plot3d    = true;   // LEO 平面投影(E-N)大图（关掉则不画，直接减负）
        bool residual  = true;   // 逐星残差图
        bool rtn       = true;   // R/T/N/3D 偏差图
        bool rms       = true;   // 伪距/相位 RMS 图
    };

    struct PosTabView {
        int epochCount = 0;
        bool busy = false; // 仍在读观测/加载产
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

        // 位置面板
        bool solved = false;
        Eigen::Vector3d xyz{0, 0, 0}, enu{0, 0, 0}, blh{0, 0, 0};
        bool showBaseXyz = false;              // RTK：在参考坐标下显示基站坐标（只读）
        Eigen::Vector3d *baseXyz = nullptr;
        double sigmaP = 0;
        bool showZtd = false; double ztd = 0;
        bool showVelDop = false;
        bool showVel = true;
        Eigen::Vector3d vel{0, 0, 0};
        double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
        int numSatsResult = 0, numObs = 0;
        std::string noSolveMsg;

        // 右侧面板「解状态」区：固定/浮点 + ratio，位于导出 CSV 上方，带分割线
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
        // 相位残差（PlotData::satResPhaseTimes/Vals）。残差图有"码/相位"切换，两份都要给；
        // 只有当前选中的那一份会被画，但快照是全量（不许抽稀，见 GuiSolverPanel）。
        std::map<SatID, std::vector<double>> resPhaseT, resPhaseV;
        // 是否存在可画残差（与当前码/相位模式无关）。判定"要不要画残差图"必须用它：
        // 切模式只深拷对应那一份，用 resV.empty() 判断会在切相位那一帧误判成"无数据"。
        bool hasResidualData = false;
        bool robustRes = false; double resYlo = -8, resYhi = 8;
        bool robustResPhase = false; double resPhaseYlo = -0.5, resPhaseYhi = 0.5;
        int *resMode = nullptr;          // 0=码(伪距)残差 1=载波相位残差，指向 SolveTask::resMode

        bool showSigmaDop = false;
        bool isRtk = false;              // RTK 模式：DOP 不含 TDOP（双差消钟差），图表/文本均不显示 TDOP
        bool showSigmaV = true;
        bool showPdop = true;
        std::vector<double> sigmaPs, sigmaVs, pdops;
        std::vector<double> sigmaPFloatVals;  // 浮点解 σP 时序（RTK 有；SPP/PPP/LEO 为空则只画单线）
        std::vector<double> gdops, hdops, vdops, tdops;  // 完整 DOP 时序（P·G·H·V·T），供 DOP 叠图

        // 固定率（PPP/RTK/LEO）：页眉"Wk SOW | 定位"右侧实时统计固定解占比
        bool showFixRate = false;
        std::string fixRateText;

        bool showRtn = false;
        bool hasRef = false;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        bool showRms = false;
        std::vector<double> rmsCode, rmsPhase;

        bool show3d = false;
        ChartVisibility *renderToggles = nullptr;
        Eigen::Vector3d leoPos3d{0, 0, 0};     // LEO 解算位置 ECEF (m)
        Eigen::Vector3d leoRef3d{0, 0, 0};     // LEO 参考轨道位置 ECEF (m)
        std::vector<GnssTask::SatVis> gnssVis;           // 当前历元各 GNSS 卫星 ECEF (m)
        std::vector<GnssTask::TrajPoint> leoTraj;        // LEO 解算历史轨迹（米）
        std::vector<GnssTask::TrajPoint> leoRefTraj;     // 参考轨道历史轨迹（米）
        std::map<SatID, std::vector<GnssTask::TrajPoint>> gnssTraj; // GNSS 卫星历史轨迹（米）

        // 三表（坐标表 / 参考+ENU(RTN) 表 / DOP+状态表）所需数据，由 GuiSolverPanel 填充
        bool hasFixedDistinction = false;  // 模式是否区分固定/浮点（RTK/PPP/LEO=true, SPP=false）
        bool hasFixed = false;             // 当前历元是否固定解
        Eigen::Vector3d xyzFixedDisp{0, 0, 0};   // 固定解 ECEF（PPP/LEO 用 xyzFixed，RTK 用 r.xyz）
        Eigen::Vector3d xyzFloatDisp{0, 0, 0};   // 浮点解 ECEF
        double sigmaFixedDisp = 0, sigmaFloatDisp = 0;
        bool hasBaseline = false;          // RTK：显示基线行
        Eigen::Vector3d baseFloatDisp{0, 0, 0};  // 浮点基线向量（相对基站）
        Eigen::Vector3d baseFixedDisp{0, 0, 0};  // 固定基线向量（= 绝对 ECEF - 基站）
        bool hasVelDisp = false;
        Eigen::Vector3d velDisp{0, 0, 0};
        double sigmaVDisp = 0;
        // 参考 + ENU/RTN 表（仅在有固定算法时；LEO 无参考轨道不显示）
        bool showRefTable = false;
        bool isLeoRtn = false;             // LEO：ENU 改 RTN
        Eigen::Vector3d refDisp{0, 0, 0};
        double abs3dDisp = 0;              // 参考行 3D-RMS（主解相对参考的 ENU 幅值）
        Eigen::Vector3d enuFixedDisp{0, 0, 0};
        Eigen::Vector3d enuFloatDisp{0, 0, 0};
        double enuFixed3d = 0, enuFloat3d = 0;
        Eigen::Vector3d rtnDisp{0, 0, 0};  // LEO RTN（R/T/N）
        double rtn3dDisp = 0;
        // 解状态 / ratio（用于 DOP 表追加列与解状态文本块）
        std::string solStatShort;          // 如 "固定(Fixed)" / "浮点(Float)"
        double solRatio = 0;

        bool isDone = false;
        std::function<void()> onExportCsv;
    };

    void RenderPositioningTab(const PosTabView &v);

}
