#pragma once

#include <vector>
#include <map>
#include <limits>
#include <utility>

#include "Const.h"
#include "solvers/common/Core.h"
#include "implot.h"

namespace Solvers::Charts {

    struct SatRow {
        SatID sat;
        bool used = true;
        double elevDeg = 0;
        double azimDeg = 0;
        bool hasAzim = false;
        double satX = 0, satY = 0, satZ = 0;
        bool hasSatXYZ = false;
        double codeRes = std::numeric_limits<double>::quiet_NaN();
        double phaseRes = std::numeric_limits<double>::quiet_NaN();
    };

    // 红色可拖拽时间游标：拖动结果经 lround+clamp 同步回 *selectedEpoch（空指针只画线不同步）。
    void epochCursor(int id, int *selectedEpoch, int epochCount);

    // follow 为真时把 X 轴右端推到最新时刻，新历元到达时滚动跟随
    inline void xAxisFollow(const std::vector<double> &t, const bool follow,
                            const ImPlotCond cond = ImPlotCond_Always) {
        if (follow && !t.empty() && std::isfinite(t.back()))
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, t.back(), cond);
    }

    void renderSatelliteTable(const std::vector<SatRow> &rows, int &selectedSatIdx);
    void renderENUPlot(const BrokenAxis &axis,
                       const std::vector<double> &enu_e,
                       const std::vector<double> &enu_n,
                       const std::vector<double> &enu_u,
                       int &selectedEpoch, bool &newed,
                       bool hasFloat = false,
                       const std::vector<double> &float_e = {},
                       const std::vector<double> &float_n = {},
                       const std::vector<double> &float_u = {},
                       int epochCount = -1,
                       float plotH = 380.0f);

    void renderSatResidualPlot(const std::map<SatID, std::vector<double>> &satResV,
                               const std::map<SatID, std::vector<double>> &phaseV,
                               const std::map<SatID, BrokenAxis> &codeAxis,
                               const std::map<SatID, BrokenAxis> &phaseAxis,
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
        double codeRes = std::numeric_limits<double>::quiet_NaN();
        double phaseRes = std::numeric_limits<double>::quiet_NaN();
    };
    struct SkyView {
        SatID sat;
        double azimRad = 0, elevRad = 0;
        bool used = true;
    };

  template<typename Proj>
    std::vector<SatRow> buildSatRows(const std::size_t n, Proj &&proj) {
        std::vector<SatRow> rows;
        rows.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
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
            sr.codeRes = v.codeRes;
            sr.phaseRes = v.phaseRes;
            rows.push_back(std::move(sr));
        }
        return rows;
    }

  template<typename Proj>
    std::vector<std::pair<SatID, SkyPoint> > buildCurSkyPoints(const int epIdx, const std::size_t n, Proj &&proj, const double minElevDeg = -1.0) {
        std::vector<std::pair<SatID, SkyPoint> > pts;
        for (std::size_t i = 0; i < n; ++i) {
            SkyView v = proj(i);
            const double elDeg = v.elevRad * RAD_TO_DEG;
            if (elDeg <= minElevDeg) continue;
            pts.emplace_back(v.sat,
                SkyPoint{epIdx, static_cast<float>(v.azimRad * RAD_TO_DEG), static_cast<float>(elDeg), v.used});
        }
        return pts;
    }

    void renderSkyplot(const std::map<SatID, std::vector<SkyPoint>> &tracks,
                       const std::vector<std::pair<SatID, SkyPoint>> &curPts,
                       float sizePx = 360.0f);

    void renderLeo3D(const Eigen::Vector3d &leoPos,
                     const std::vector<TrajPoint> &leoTraj,
                     const std::vector<TrajPoint> &refTraj,
                     float sizePx = 460.0f);

}
