#pragma once

#include "core/geom/CoordStruct.h"
#include "core/geom/Ephemeris.h"
#include <Eigen/Eigen>
#include "core/TimeStruct.h"
#include <cmath>


namespace Coord {
    BLH XYZToBLH(const XYZ &xyz, const GeodeticFrame &frame);

    XYZ BLHToXYZ(const BLH &blh, const GeodeticFrame &frame);

    ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const GeodeticFrame &frame = Frame::WGS84);

    XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const GeodeticFrame &frame = Frame::WGS84);

    Eigen::Vector3d XYZToRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                             const Eigen::Vector3d &velRef);

    inline NED ENUToNED(const ENU &enu) { return {enu.N(), enu.E(), -enu.U()}; }
    inline ENU NEDToENU(const NED &ned) { return {ned.E(), ned.N(), -ned.D()}; }

    // 指导书式(36)-(39)：以 (lat0, lon0) 为原点的站心 N/E (m)，角度为弧度
    inline void LLHToLocalNE(const double lat, const double lon, const double lat0, const double lon0,
                             double &n, double &e) {
        constexpr double a = 6378137.0;
        constexpr double f = 1.0 / 298.257222101;
        constexpr double e2 = f * (2.0 - f);
        const double s = std::sin(lat0), w = std::sqrt(1.0 - e2 * s * s);
        n = (lat - lat0) * a * (1.0 - e2) / (w * w * w);
        e = (lon - lon0) * a / w * std::cos(lat0);
    }

    NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const GeodeticFrame &frame = Frame::WGS84);

    XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const GeodeticFrame &frame = Frame::WGS84);

    double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const GeodeticFrame &frame = Frame::WGS84);

    double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const GeodeticFrame &frame = Frame::WGS84);

    void geometricRangeAndLos(const Eigen::Vector3d &satPos, const Eigen::Vector3d &recvPos,
                              double &rho, Eigen::Vector3d &losUnit);
}
