#pragma once

#include "CoordStruct.h"
#include <Eigen/Eigen>
#include "TimeConvert.h"
#include <Eigen/Geometry>


using DCM = Eigen::Matrix3d;

DCM EulerToDCM(double roll, double pitch, double yaw);

void DCMToEuler(const DCM &C, double &roll, double &pitch, double &yaw);

DCM ECEFToENUDCM(double B, double L);

DCM ECEFToLVLHDCM(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);

DCM ECEFToSVBodyDCM(const Eigen::Vector3d &pTx, const Eigen::Vector3d &rsunEcef);

DCM ECEFToNEDDCM(double B, double L);

DCM NEDToECEFDCM(double B, double L);

DCM ECIToECEFDCM(const CommonTime &t, double &gmstOut, double xp = 0.0, double yp = 0.0);

BLH XYZToBLH(const XYZ &xyz, const FrameInfo &frame);

XYZ BLHToXYZ(const BLH &blh, const FrameInfo &frame);

ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84);

XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84);

Eigen::Vector3d XYZToRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                         const Eigen::Vector3d &velRef);

inline NED ENUToNED(const ENU &enu) { return {enu.N(), enu.E(), -enu.U()}; }
inline ENU NEDToENU(const NED &ned) { return {ned.E(), ned.N(), -ned.D()}; }

NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84);

XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84);

double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84);

double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84);

void geometricRangeAndLos(const Eigen::Vector3d &satPos, const Eigen::Vector3d &recvPos,
                          double &rho, Eigen::Vector3d &losUnit);


// 姿态四元数
class AttQuat {
public:
    AttQuat() = default; // 单位四元数
    explicit AttQuat(Eigen::Quaterniond q);

    // ---- 静态工厂 ----
    static AttQuat identity();

    static AttQuat fromDCM(const DCM &C);

    static AttQuat fromEuler(double roll, double pitch, double yaw);

    static AttQuat fromRotationVector(const Eigen::Vector3d &rv);

    // ---- 导出 ----
    [[nodiscard]] DCM toDCM() const;

    void toEuler(double &roll, double &pitch, double &yaw) const;

    [[nodiscard]] Eigen::Vector3d toRotationVector() const;

    [[nodiscard]] double norm() const;

    // ---- 运算 ----
    [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d &v) const;

    [[nodiscard]] AttQuat operator*(const AttQuat &rhs) const; // Hamilton 积
    [[nodiscard]] AttQuat conjugated() const;

    [[nodiscard]] AttQuat normalized() const;

    [[nodiscard]] AttQuat update(const Eigen::Vector3d &omegaBody, double dt) const;

    [[nodiscard]] const Eigen::Quaterniond &raw() const;

private:
    Eigen::Quaterniond mQ{1.0, 0.0, 0.0, 0.0}; // (w, x, y, z)，默认单位
};
