#pragma once

#include "Const.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

class Ionosphere {
public:
    // 电离层穿刺点(IPP)几何
    struct IppGeometry {
        double latDeg = 0.0;     // IPP 纬度 [deg]
        double lonDeg = 0.0;     // IPP 经度 [deg]
        double vtecToStec = 1.0; // VTEC[TECU] -> STEC[el/m²]（含斜路径映射与 1e16 换算）
        double bMag = 0.0;       // IPP 处地磁场强度 [T]
        double cosThetaB = 0.0;  // LOS 与地磁场夹角余弦
    };

    static IppGeometry ippGeometry(const Vector3d &xyzRecv, const Vector3d &xyzSat,
                                   const double elev, const double azim, const double ionoHeightKm) {
        IppGeometry g;
        const double hIon = ionoHeightKm * 1000.0;                          // 单层高度 (m)
        const double z = PI / 2.0 - elev;                                    // 接收机处天顶距 (rad)
        const double alpha = std::asin((Rearth / (Rearth + hIon)) * std::sin(z)); // IPP 天顶距
        const double psi = z - alpha;                                        // 地心角 (rad)
        const double mfac = 1.0 / std::cos(alpha);                           // 斜路径映射因子

        // IPP 经纬度：在接收机大地坐标上沿方位角推进地心角 psi
        const auto blh = XYZtoBLH(xyzRecv, Frame::WGS84);
        const double latR = blh[0], lonR = blh[1]; // rad
        const double sPsi = std::sin(psi), cPsi = std::cos(psi);
        const double sAz = std::sin(azim), cAz = std::cos(azim);
        const double latI = std::asin(std::sin(latR) * cPsi + std::cos(latR) * sPsi * cAz);
        const double lonI = lonR + std::atan2(sPsi * sAz,
                                              std::cos(latR) * cPsi - std::sin(latR) * sPsi * cAz);
        g.latDeg = latI * 180.0 / PI;
        g.lonDeg = lonI * 180.0 / PI;
        g.vtecToStec = mfac * 1.0e16;

        // 偶极子地磁场 @ IPP（地磁北极近似 lat 80°N, lon 290°E）
        const double latM = 80.0 * PI / 180.0, lonM = 290.0 * PI / 180.0;
        const Vector3d mHat(std::cos(latM) * std::cos(lonM),
                            std::cos(latM) * std::sin(lonM),
                            std::sin(latM));
        const Vector3d rHat = xyzRecv.normalized(); // 近似 IPP 方向(地心单位向量)
        const double cTm = rHat.dot(mHat);
        const double rIpp = Rearth + hIon;
        const double k = B0 * std::pow(Rearth, 3.0) / (rIpp * rIpp * rIpp);
        const Vector3d bHat = (3.0 * cTm * rHat - mHat).normalized();
        g.bMag = k * std::sqrt(1.0 + 3.0 * cTm * cTm);

        const Vector3d los = (xyzSat - xyzRecv).normalized();
        g.cosThetaB = bHat.dot(los);
        return g;
    }

    // 二阶(高阶)电离层改正系数
    static double secondOrderK(const double vtec, const IppGeometry &ipp,
                               const double f1, const double f2) {
        if (f1 <= 0.0 || f2 <= 0.0) return 0.0;
        const double stec = vtec * ipp.vtecToStec; // 斜 TEC (el/m²)
        const double s = 7527.0 * C_MPS * ipp.bMag * ipp.cosThetaB * stec;
        return s / (f1 * f2 * (f1 + f2));
    }

private:
    static constexpr double Rearth = 6371000.0; // 平均地球半径 (m)
    static constexpr double B0 = 3.12e-5;       // 赤道表面场 (T)
};
