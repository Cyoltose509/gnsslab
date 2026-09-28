#pragma once

#include <string>
#include <vector>
#include <map>
#include <utility>
#include <functional>

#include "Const.h"
#include "SolverTask.h"
#include "imgui.h"

/// 跨页面复用的 GNSS 可视化 widget（SPP / PPP 共用，避免各处理器重复实现）。
namespace GuiCharts {

    struct SatRow {
        SatID sat;
        bool used = true;                 // 参与定位 / 被排除
        double elevDeg = 0;               // 仰角(°)
        double azimDeg = 0;               // 方位角(°)
        bool hasAzim = false;
        double satX = 0, satY = 0, satZ = 0;  // 卫星 ECEF 位置(m)
        bool hasSatXYZ = false;
        std::vector<std::pair<std::string, std::string>> extra;
    };

    void renderSatelliteTable(const std::vector<SatRow> &rows, int &selectedSatIdx,
                              const char *id = "##satTable");
    void renderENUPlot(const std::vector<double> &times,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed, bool fixedY = false);

    void renderSatResidualPlot(const std::map<SatID, std::vector<double>> &satResT,
                               const std::map<SatID, std::vector<double>> &satResV,
                               int &selectedEpoch, int epochCount, bool &newed,
                               const double *yLo = nullptr, const double *yHi = nullptr,
                               float plotH = 320.0f);

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
    void buildSkyTracks(std::map<SatID, std::vector<SkyPoint>> &tracks, const int epIdx, const size_t n, Proj &&proj) {
        for (size_t i = 0; i < n; ++i) {
            SkyView v = proj(i);
            tracks[v.sat].push_back(
                SkyPoint{epIdx, static_cast<float>(v.azimRad * RAD_TO_DEG), static_cast<float>(v.elevRad * RAD_TO_DEG), v.used});
        }
    }

  template<typename Proj>
    std::vector<std::pair<SatID, SkyPoint> > buildCurSkyPoints(const int epIdx, const size_t n, Proj &&proj, const double minElevDeg = -1.0) {
        std::vector<std::pair<SatID, SkyPoint> > pts;
        for (size_t i = 0; i < n; ++i) {
            SkyView v = proj(i);
            const double elDeg = v.elevRad * RAD_TO_DEG;
            if (elDeg <= minElevDeg) continue;
            pts.emplace_back(v.sat,
                SkyPoint{epIdx, static_cast<float>(v.azimRad * RAD_TO_DEG), static_cast<float>(elDeg), v.used});
        }
        return pts;
    }

    void RenderSkyplot(const std::map<SatID, std::vector<SkyPoint>> &tracks,
                       const std::vector<std::pair<SatID, SkyPoint>> &curPts,
                       float sizePx = 360.0f);

    void RenderLeo3D(const Vector3d &leoPos,
                     const std::vector<TrajPoint> &leoTraj,
                     const std::vector<TrajPoint> &refTraj,
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
        bool busy = false; // 仍在读观测/加载产品；epochCount==0 时用于区分"尚未产出历元"与"真无解算历元"
        int selectedIdx = -1;
        int *selectedEpoch = nullptr;
        int *selectedSatIdx = nullptr;
        Vector3d *refECEF = nullptr;
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
        Vector3d xyz{0, 0, 0}, enu{0, 0, 0}, blh{0, 0, 0};
        double sigmaP = 0;
        bool showZtd = false; double ztd = 0;
        bool showVelDop = false;
        bool showVel = true;
        Vector3d vel{0, 0, 0};
        double pdop = 0, gdop = 0, hdop = 0, vdop = 0, tdop = 0;
        int numSatsResult = 0, numObs = 0;
        std::string noSolveMsg;

        bool fixedY = false;
        std::vector<double> times, enu_e, enu_n, enu_u;
        bool newed = false;

        std::map<SatID, std::vector<SkyPoint>> skyTracks;
        std::vector<std::pair<SatID, SkyPoint>> curSkyPts;
        std::map<SatID, std::vector<double>> resT, resV;
        bool robustRes = false; double resYlo = -8, resYhi = 8;

        bool showSigmaDop = false;
        bool showSigmaV = true;
        bool showPdop = true;
        std::vector<double> sigmaPs, sigmaVs, pdops;

        bool showRtn = false;
        bool hasRef = false;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        bool showRms = false;
        std::vector<double> rmsCode, rmsPhase;

        bool show3d = false;
        ChartVisibility *renderToggles = nullptr;
        Vector3d leoPos3d{0, 0, 0};     // LEO 解算位置 ECEF (m)
        Vector3d leoRef3d{0, 0, 0};     // LEO 参考轨道位置 ECEF (m)
        std::vector<SatVis> gnssVis;           // 当前历元各 GNSS 卫星 ECEF (m)
        std::vector<TrajPoint> leoTraj;        // LEO 解算历史轨迹（米）
        std::vector<TrajPoint> leoRefTraj;     // 参考轨道历史轨迹（米）
        std::map<SatID, std::vector<TrajPoint>> gnssTraj; // GNSS 卫星历史轨迹（米）

        bool isDone = false;
        std::function<void()> onExportCsv;
    };

    void RenderPositioningTab(const PosTabView &v);

} // namespace GuiCharts
