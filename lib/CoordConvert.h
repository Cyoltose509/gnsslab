#pragma once

#include "CoordStruct.h"
#include <Eigen/Eigen>

// 坐标转换函数
inline BLH XYZtoBLH(const XYZ &xyz, const ReferenceFrame &frame) {
    // 获取椭球参数
    const double a = frame.a;
    const double e2 = frame.e2;
    // 计算水平距离 rho（即 sqrt(x^2 + y^2)）
    const double rho = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);

    // 定义阈值，用于判断是否在极点
    constexpr double eps = 1.0e-13;

    // 判断是否在极点
    if (rho < eps) {
        // 在极点，根据 z 的符号判断是南极还是北极
        double B = xyz.Z() > 0 ? PI / 2 : -PI / 2; // 北极为 +90°，南极为 -90°
        double L = 0.0; // 经度在极点无定义，通常设为 0
        double H = fabs(xyz.Z()) - a * sqrt(1 - e2); // 高度计算

        return {B, L, H};
    }

    // 不在极点，正常计算
    double B0 = atan2(xyz.Z(), rho);

    // 迭代计算大地纬度 B
    int iterationCount = 0;
    double B1, N;
    do {
        N = a / sqrt(1 - e2 * sin(B0) * sin(B0));
        B1 = atan2(xyz.Z() + e2 * N * sin(B0), rho);

        if (fabs(B1 - B0) < eps) break;

        B0 = B1;
        iterationCount++;

        if (constexpr int maxIterations = 100; iterationCount > maxIterations) {
            throw std::runtime_error("Iteration did not converge.");
        }
    } while (true);

    // 计算大地经度 L
    double L = atan2(xyz[1], xyz[0]);

    // 计算高度 H
    double H = rho / cos(B1) - N;

    // 返回大地坐标
    return {B1, L, H};
}

inline XYZ BLHtoXYZ(const BLH &blh, const ReferenceFrame &frame) {
    const double B = blh[0];  // 纬度 (rad)
    const double L = blh[1];  // 经度 (rad)
    const double H = blh[2];  // 高程 (m)

    const double a = frame.a;
    const double e2 = frame.e2;

    const double sinB = sin(B);
    const double cosB = cos(B);
    const double sinL = sin(L);
    const double cosL = cos(L);

    const double N = a / sqrt(1 - e2 * sinB * sinB);

    double x = (N + H) * cosB * cosL;
    double y = (N + H) * cosB * sinL;
    double z = (N * (1 - e2) + H) * sinB;

    return {x, y, z};
}

inline Eigen::Matrix3d computeRotationMatrix(const double B, const double L) {
    const double sinB = std::sin(B);
    const double cosB = std::cos(B);
    const double sinL = std::sin(L);
    const double cosL = std::cos(L);

    Eigen::Matrix3d R;
    R << -sinL, cosL, 0.0,
            -sinB * cosL, -sinB * sinL, cosB,
            cosB * cosL, cosB * sinL, sinB;

    return R;
}

inline ENU XYZtoENU(const XYZ &xyz, const XYZ &refXYZ, const ReferenceFrame &frame = Frame::WGS84) {
    const auto diffXYZ = xyz - refXYZ;
    const auto refBLH = XYZtoBLH(refXYZ, frame);
    const Eigen::Matrix3d R = computeRotationMatrix(refBLH.B(), refBLH.L());
    return {R * diffXYZ};
}

inline XYZ ENUtoXYZ(const ENU &enu, const XYZ &refXYZ, const ReferenceFrame &frame = Frame::WGS84) {
    const auto refBLH = XYZtoBLH(refXYZ, frame);
    auto R = computeRotationMatrix(refBLH.B(), refBLH.L());
    const auto diffXYZ = R.transpose() * enu;
    XYZ resXYZ;
    resXYZ << refXYZ[0] + diffXYZ[0], refXYZ[1] + diffXYZ[1], refXYZ[2] + diffXYZ[2];
    return resXYZ;
}

inline double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const ReferenceFrame &frame = Frame::WGS84) {
    // 先转到ENU
    const ENU enu = XYZtoENU(targetXYZ, refXYZ, frame);
    const double elev = atan2(enu(2), sqrt(enu(0) * enu(0) + enu(1) * enu(1)));
    return elev;
}

inline double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const ReferenceFrame &frame = Frame::WGS84) {
    // 转到ENU
    const ENU enu = XYZtoENU(targetXYZ, refXYZ, frame);
    double az = atan2(enu(0), enu(1));
    if (az < 0.0) az += PI * 2.0;
    return az;
}
