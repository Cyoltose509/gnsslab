#pragma once
#include <Eigen/Dense>

#include "core/ins/InsStruct.h"
#include "core/Const.h"
#include "Exception.h"


struct SiteParams {
    double latDeg = 0.0;
    double g = 9.7936174;
};

inline Eigen::Vector3d navGravityDir() { return {0.0, 0.0, 1.0}; }

inline Eigen::Vector3d navEarthRate(const double latDeg) {
    const double lat = latDeg * DEG_TO_RAD;
    return {OMEGA_EARTH * std::cos(lat), 0.0, -OMEGA_EARTH * std::sin(lat)};
}

inline DCM triad(const Eigen::Vector3d &vb1, const Eigen::Vector3d &vb2,
                 const Eigen::Vector3d &vn1, const Eigen::Vector3d &vn2) {
    auto basis = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        if (a.norm() < 1e-12 || b.norm() < 1e-12)
            throw GeometryException("triad: degenerate vector pair (site params unfilled or measured vector is zero)");
        const Eigen::Vector3d r1 = a.normalized();
        Eigen::Vector3d r2 = b - b.dot(r1) * r1;
        r2.normalize();
        const Eigen::Vector3d r3 = r1.cross(r2);
        Eigen::Matrix3d M;
        M.col(0) = r1;
        M.col(1) = r2;
        M.col(2) = r3;
        return M;
    };
    return DCM(basis(vn1, vn2) * basis(vb1, vb2).transpose());
}

inline DCM alignDCM(const Eigen::Vector3d &gB, const Eigen::Vector3d &wB, const SiteParams &sp) {
    if (!std::isfinite(sp.latDeg) || std::fabs(sp.latDeg) > 90.0)
        throw GeometryException("alignDCM: SiteParams.latDeg must lie in [-90, 90] (local latitude)");
    return triad(gB, wB, navGravityDir(), navEarthRate(sp.latDeg));
}

struct ImuCalibration {
    Eigen::Matrix3d accelM = Eigen::Matrix3d::Identity();
    Eigen::Vector3d accelB = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroScale = Eigen::Vector3d::Ones();
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();

    [[nodiscard]] Eigen::Vector3d specificForce(const ImuSample &s) const {
        return accelM.inverse() * (Eigen::Vector3d(s.ax, s.ay, s.az) - accelB);
    }

    [[nodiscard]] Eigen::Vector3d angularRate(const ImuSample &s) const {
        return (Eigen::Vector3d(s.gx, s.gy, s.gz) - gyroBias).cwiseQuotient(gyroScale);
    }
};
