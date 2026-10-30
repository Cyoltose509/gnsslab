#pragma once

#include "Const.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

class Ionosphere {
public:

    struct IppGeometry {
        double latDeg = 0.0;
        double lonDeg = 0.0;
        double vtecToStec = 1.0;
        double bMag = 0.0;
        double cosThetaB = 0.0;
    };

    static IppGeometry ippGeometry(const Eigen::Vector3d &xyzRecv, const Eigen::Vector3d &xyzSat,
                                   const double elev, const double azim, const double ionoHeightKm) {
        IppGeometry g;
        const double hIon = ionoHeightKm * 1000.0;
        const double z = PI / 2.0 - elev;
        const double alpha = std::asin((Rearth / (Rearth + hIon)) * std::sin(z));
        const double psi = z - alpha;
        const double mfac = 1.0 / std::cos(alpha);


        const auto blh = XYZToBLH(xyzRecv, Frame::WGS84);
        const double latR = blh[0], lonR = blh[1];
        const double sPsi = std::sin(psi), cPsi = std::cos(psi);
        const double sAz = std::sin(azim), cAz = std::cos(azim);
        const double latI = std::asin(std::sin(latR) * cPsi + std::cos(latR) * sPsi * cAz);
        const double lonI = lonR + std::atan2(sPsi * sAz,
                                              std::cos(latR) * cPsi - std::sin(latR) * sPsi * cAz);
        g.latDeg = latI * RAD_TO_DEG;
        g.lonDeg = lonI * RAD_TO_DEG;
        g.vtecToStec = mfac * 1.0e16;


        const double latM = 80.0 * DEG_TO_RAD, lonM = 290.0 * DEG_TO_RAD;
        const Eigen::Vector3d mHat(std::cos(latM) * std::cos(lonM),
                            std::cos(latM) * std::sin(lonM),
                            std::sin(latM));
        const Eigen::Vector3d rHat = xyzRecv.normalized();
        const double cTm = rHat.dot(mHat);
        const double rIpp = Rearth + hIon;
        const double k = B0 * std::pow(Rearth, 3.0) / (rIpp * rIpp * rIpp);
        const Eigen::Vector3d bHat = (3.0 * cTm * rHat - mHat).normalized();
        g.bMag = k * std::sqrt(1.0 + 3.0 * cTm * cTm);

        const Eigen::Vector3d los = (xyzSat - xyzRecv).normalized();
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
