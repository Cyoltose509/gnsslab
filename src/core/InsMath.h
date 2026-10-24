#pragma once
#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

#include "core/InsStruct.h"
#include "core/Const.h"


struct SiteParams {
    double g = 0.0;         // 当地重力加速度 (m/s^2)
    double latDeg = 0.0;    // 当地纬度 (°N)
    double earthRate = 0.0; // 地球自转角速率 (rad/s)
};

inline Eigen::Vector3d navGravity(const double g) { return {0.0, 0.0, g}; }

inline Eigen::Vector3d navEarthRate(const double latDeg, const double earthRate) {
    const double lat = latDeg * DEG_TO_RAD;
    return {earthRate * std::cos(lat), 0.0, -earthRate * std::sin(lat)};
}

inline Eigen::Matrix3d triad(const Eigen::Vector3d &vb1, const Eigen::Vector3d &vb2,
                             const Eigen::Vector3d &vn1, const Eigen::Vector3d &vn2) {
    auto basis = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        if (a.norm() < 1e-12 || b.norm() < 1e-12)
            throw std::invalid_argument("triad: degenerate vector (站点参数未填或实测矢量为零)");
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
    return basis(vn1, vn2) * basis(vb1, vb2).transpose();
}

// 静态解析粗对准（TRIAD）：给定机体实测重力方向 g_b 与角速度 w_b
// （g_b 已取「比力取负」，w_b 已扣陀螺零偏），返回 C_b^n。
inline Eigen::Matrix3d alignDCM(const Eigen::Vector3d &gB, const Eigen::Vector3d &wB, const SiteParams &sp) {
    if (!std::isfinite(sp.g) || sp.g <= 0.0)
        throw std::invalid_argument("alignDCM: SiteParams.g 必须为正数（当地重力加速度，见参数页）");
    if (!std::isfinite(sp.latDeg) || std::fabs(sp.latDeg) > 90.0)
        throw std::invalid_argument("alignDCM: SiteParams.latDeg 应在 [-90, 90]（当地纬度）");
    if (!std::isfinite(sp.earthRate) || sp.earthRate <= 0.0)
        throw std::invalid_argument("alignDCM: SiteParams.earthRate 必须为正数（地球自转角速率）");
    return triad(gB, wB, navGravity(sp.g), navEarthRate(sp.latDeg, sp.earthRate));
}

struct ImuCalibration {
    Eigen::Matrix3d accelM = Eigen::Matrix3d::Identity(); // counts per (m/s^2)
    Eigen::Vector3d accelB = Eigen::Vector3d::Zero(); // counts
    Eigen::Vector3d gyroScale = Eigen::Vector3d::Ones(); // counts per (rad/s)
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero(); // counts / 历元

    [[nodiscard]] Eigen::Vector3d specificForce(const ImuSample &s) const {
        return accelM.inverse() * (Eigen::Vector3d(s.ax, s.ay, s.az) - accelB);
    }

    [[nodiscard]] Eigen::Vector3d angularRate(const ImuSample &s) const {
        return (Eigen::Vector3d(s.gx, s.gy, s.gz) - gyroBias).cwiseQuotient(gyroScale);
    }
};
