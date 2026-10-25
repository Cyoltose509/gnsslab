#pragma once

#include "CoordStruct.h"
#include <Eigen/Eigen>
#include "MathUtils.h"
#include "TimeConvert.h"
#include <Eigen/Geometry>
#include <utility>


using DCM = Eigen::Matrix3d;

inline DCM EulerToDCM(const double roll, const double pitch, const double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    DCM C;
    C << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy,
            cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy,
            -sp, sr * cp, cr * cp;
    return C;
}

inline void DCMToEuler(const DCM &C, double &roll, double &pitch, double &yaw) {
    pitch = std::atan2(-C(2, 0), std::sqrt(C(2, 1) * C(2, 1) + C(2, 2) * C(2, 2)));
    roll = std::atan2(C(2, 1), C(2, 2));
    yaw = std::atan2(C(1, 0), C(0, 0));
}

inline DCM ECEFToENUDCM(const double B, const double L) {
    const double sinB = std::sin(B);
    const double cosB = std::cos(B);
    const double sinL = std::sin(L);
    const double cosL = std::cos(L);

    DCM R;
    R << -sinL, cosL, 0.0,
            -sinB * cosL, -sinB * sinL, cosB,
            cosB * cosL, cosB * sinL, sinB;

    return R;
}


inline DCM ECEFToLVLHDCM(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
    const Eigen::Vector3d ur = pos.normalized(); // 径向(向外) Zb
    Eigen::Vector3d ut;
    if (vel.squaredNorm() > 1.0) ut = vel.normalized(); // 沿迹 Xb
    else ut = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d uc = ur.cross(ut); // 轨道法向(角动量方向) Yb
    if (uc.squaredNorm() < 1e-6) uc = Eigen::Vector3d(1, 0, 0);
    uc.normalize();
    ut = uc.cross(ur).normalized(); // 重新正交化：沿迹 = 法向 × 径向
    DCM M;
    M.row(0) = -ut; // Xb 沿迹
    M.row(1) = uc; // Yb 轨道法向
    M.row(2) = ur; // Zb 径向
    return M;
}

inline DCM ECEFToSVBodyDCM(const Eigen::Vector3d &pTx, const Eigen::Vector3d &rsunEcef) {
    const Eigen::Vector3d ez = -pTx.normalized();
    const Eigen::Vector3d es = (rsunEcef - pTx).normalized();
    const Eigen::Vector3d ey = ez.cross(es).normalized();
    const Eigen::Vector3d ex = ey.cross(ez);
    DCM M;
    M.row(0) = ex;
    M.row(1) = ey;
    M.row(2) = ez;
    return M;
}

inline DCM ECEFToNEDDCM(const double B, const double L) {
    const double sinB = std::sin(B), cosB = std::cos(B);
    const double sinL = std::sin(L), cosL = std::cos(L);
    DCM C;
    C << -sinB * cosL, -sinB * sinL, cosB,
            -sinL, cosL, 0.0,
            -cosB * cosL, -cosB * sinL, -sinB;
    return C;
}

inline DCM NEDToECEFDCM(const double B, const double L) {
    return ECEFToNEDDCM(B, L).transpose();
}

inline DCM ECIToECEFDCM(const CommonTime &t, double &gmstOut, const double xp = 0.0, const double yp = 0.0) {
    const double jdFull = static_cast<double>(CommonTime2JulianDate(t).jd);
    const double T = (jdFull - 2451545.0) / 36525.0; // 儒略世纪 (TT≈UTC，可忽略)
    const double t2 = T * T, t3 = t2 * T;
    double f[5];
    fundamentalArguments(T, f);
    // IAU 1976 岁差
    const double ze = (2306.2181 * T + 0.30188 * t2 + 0.017998 * t3) * ARCSEC_TO_RAD;
    const double th = (2004.3109 * T - 0.42665 * t2 - 0.041833 * t3) * ARCSEC_TO_RAD;
    const double z = (2306.2181 * T + 1.09468 * t2 + 0.018203 * t3) * ARCSEC_TO_RAD;
    const double eps = (84381.448 - 46.8150 * T - 0.00059 * t2 + 0.001813 * t3) * ARCSEC_TO_RAD;
    DCM R1 = Math::rotationZ(-z);
    DCM R2 = Math::rotationY(th);
    DCM R3 = Math::rotationZ(-ze);
    DCM R = R1 * R2;
    const DCM P = R * R3; // P = Rz(-z)*Ry(th)*Rz(-ze)
    // IAU 1980 章动
    double dpsi, deps;
    iau1980Nutation(T, f, dpsi, deps);
    R1 = Math::rotationX(-eps - deps);
    R2 = Math::rotationZ(-dpsi);
    R3 = Math::rotationX(eps);
    R = R1 * R2;
    const DCM N = R * R3;
    const double gmst = gmstFromTime(t);
    double gast = gmst + dpsi * std::cos(eps);
    gast += (0.00264 * std::sin(f[4]) + 0.000063 * std::sin(2.0 * f[4])) * ARCSEC_TO_RAD;
    R1 = Math::rotationY(-xp);
    R2 = Math::rotationX(-yp);
    R3 = Math::rotationZ(gast);
    const DCM W = R1 * R2;
    R = W * R3;
    const DCM NP = N * P;
    gmstOut = gmst;
    return R * NP;
}


inline BLH XYZToBLH(const XYZ &xyz, const FrameInfo &frame) {
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

inline XYZ BLHToXYZ(const BLH &blh, const FrameInfo &frame) {
    const double B = blh[0]; // 纬度 (rad)
    const double L = blh[1]; // 经度 (rad)
    const double H = blh[2]; // 高程 (m)

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

inline ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto diffXYZ = xyz - refXYZ;
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
    return {R * diffXYZ};
}

inline XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
    const auto diffXYZ = R.transpose() * enu;
    XYZ resXYZ;
    resXYZ << refXYZ[0] + diffXYZ[0], refXYZ[1] + diffXYZ[1], refXYZ[2] + diffXYZ[2];
    return resXYZ;
}

inline Eigen::Vector3d XYZToRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                                const Eigen::Vector3d &velRef) {
    const Eigen::Vector3d b = ECEFToLVLHDCM(posRef, velRef) * (posEst - posRef);
    return {b.z(), -b.x(), b.y()};
}

inline NED ENUToNED(const ENU &enu) { return {enu.N(), enu.E(), -enu.U()}; }
inline ENU NEDToENU(const NED &ned) { return {ned.E(), ned.N(), -ned.D()}; }

inline NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const Eigen::Vector3d diff = xyz - refXYZ;
    const auto refBLH = XYZToBLH(refXYZ, frame);
    return {ECEFToNEDDCM(refBLH.B(), refBLH.L()) * diff};
}

inline XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Vector3d d = ECEFToNEDDCM(refBLH.B(), refBLH.L()).transpose() * Eigen::Vector3d(ned);
    return {refXYZ.X() + d.x(), refXYZ.Y() + d.y(), refXYZ.Z() + d.z()};
}


inline double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84) {
    // 先转到ENU
    const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
    const double elev = atan2(enu(2), sqrt(enu(0) * enu(0) + enu(1) * enu(1)));
    return elev;
}

inline double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84) {
    // 转到ENU
    const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
    double az = atan2(enu(0), enu(1));
    if (az < 0.0) az += PI * 2.0;
    return az;
}

inline void geometricRangeAndLos(const Eigen::Vector3d &satPos, const Eigen::Vector3d &recvPos,
                                 double &rho, Eigen::Vector3d &losUnit) {
    const Eigen::Vector3d d = recvPos - satPos;
    rho = d.norm();
    losUnit = d.normalized();
}


// 姿态四元数
class AttQuat {
public:
    AttQuat() = default; // 单位四元数
    explicit AttQuat(Eigen::Quaterniond q) : mQ(std::move(q)) {
    }

    // ---- 静态工厂 ----
    static AttQuat identity() { return AttQuat(Eigen::Quaterniond::Identity()); }

    static AttQuat fromDCM(const DCM &C) { return AttQuat(Eigen::Quaterniond(C)); }

    static AttQuat fromEuler(const double roll, const double pitch, const double yaw) {
        return AttQuat(Eigen::Quaterniond(EulerToDCM(roll, pitch, yaw)));
    }

    // 旋转矢量 → 四元数（绕 rv 方向精确旋转 |rv|，指数映射）
    static AttQuat fromRotationVector(const Eigen::Vector3d &rv) {
        const double ang = rv.norm();
        if (ang < 1e-12) return identity();
        return AttQuat(Eigen::Quaterniond(Eigen::AngleAxisd(ang, rv / ang)));
    }

    // ---- 导出 ----
    [[nodiscard]] DCM toDCM() const { return mQ.toRotationMatrix(); }

    void toEuler(double &roll, double &pitch, double &yaw) const { DCMToEuler(toDCM(), roll, pitch, yaw); }

    [[nodiscard]] Eigen::Vector3d toRotationVector() const {
        const Eigen::AngleAxisd aa(normalized().toDCM()); // 先归一化再取轴角（Eigen 假定单位四元数）
        return aa.axis() * aa.angle();
    }

    [[nodiscard]] double norm() const { return mQ.coeffs().norm(); }

    // ---- 运算 ----
    [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d &v) const { return mQ * v; }

    [[nodiscard]] AttQuat operator*(const AttQuat &rhs) const { return AttQuat(mQ * rhs.mQ); } // Hamilton 积

    [[nodiscard]] AttQuat conjugated() const { return AttQuat(mQ.conjugate()); }

    [[nodiscard]] AttQuat normalized() const {
        const double n = mQ.coeffs().norm();
        return n > 1e-12 ? AttQuat(Eigen::Quaterniond(mQ.coeffs() / n)) : identity(); // 防除零兜底
    }

    [[nodiscard]] AttQuat update(const Eigen::Vector3d &omegaBody, const double dt) const {
        return (normalized() * fromRotationVector(omegaBody * dt)).normalized();
    }

    [[nodiscard]] const Eigen::Quaterniond &raw() const { return mQ; }

private:
    Eigen::Quaterniond mQ{1.0, 0.0, 0.0, 0.0}; // (w, x, y, z)，默认单位
};
