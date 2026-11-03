#pragma once

#include "CoordStruct.h"
#include "TropoGrid.h"
#include "Troposphere.h"

#include <Eigen/Eigen>

class CommonTime;
class METReader;

/// 测站对流层快照
class TropoSite {
public:
    bool valid = false;               // 位置不可用时 false → 该历元不改正
    double latRad = 0.0, lonRad = 0.0;
    double hgt = 0.0;                 // 大地高(m)
    double doy = 0.0, mjd = 0.0;      // 年积日（1 起）、修正儒略日
    double zhd = 0.0;                 // 天顶干延迟(m)
    double zwd = 0.0;                 // 天顶湿延迟(m)
    double gradN = 0.0, gradE = 0.0;  // TRO 水平梯度(m)
    TropoGrid::Site grid;             // VMF3/GPT3 取用结果

    void reset();

    void update(const TropoGrid &table, Troposphere::Map map, const Eigen::Vector3d &xyzEcef,
                const CommonTime &epoch, const METReader *met = nullptr,
                const FrameInfo &frame = Frame::WGS84);

    [[nodiscard]] double latDeg() const;
    [[nodiscard]] double lonDeg() const;

    /// 湿延迟先验(m)
    [[nodiscard]] double zwdPrior(Troposphere::Map map) const;

    /// 解析后的天顶湿延迟(m)
    [[nodiscard]] double zwdResolved() const { return zwd; }

    /// 干/湿映射函数
    [[nodiscard]] double mapping(Troposphere::Map map, double elevRad, double *mW = nullptr) const;

    /// 干项(m)
    [[nodiscard]] double dryDelay(Troposphere::Map map, double elevRad, double *mW = nullptr,
                                  double azimRad = 0.0, bool withGradient = false) const;

    /// 湿项(m)
    [[nodiscard]] static double wetDelay(const double zwd, const double mapW) { return zwd * mapW; }
};
