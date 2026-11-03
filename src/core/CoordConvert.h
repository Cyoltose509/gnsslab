#pragma once

#include "CoordStruct.h"
#include <Eigen/Eigen>
#include "TimeConvert.h"
#include <Eigen/Geometry>


using DCM = Eigen::Matrix3d;

namespace Coord {
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
}


// 姿态四元数
class ATTQuat {
public:
    ATTQuat() = default; // 单位四元数
    explicit ATTQuat(Eigen::Quaterniond q);


    static ATTQuat identity();

    static ATTQuat fromDCM(const DCM &C);

    static ATTQuat fromEuler(double roll, double pitch, double yaw);

    static ATTQuat fromRotationVector(const Eigen::Vector3d &rv);


    static ATTQuat slerp(const ATTQuat &a, const ATTQuat &b, double t);


    [[nodiscard]] DCM toDCM() const;

    void toEuler(double &roll, double &pitch, double &yaw) const;

    [[nodiscard]] Eigen::Vector3d toRotationVector() const;

    [[nodiscard]] double norm() const;


    [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d &v) const;

    [[nodiscard]] ATTQuat operator*(const ATTQuat &rhs) const;

    [[nodiscard]] ATTQuat conjugated() const;

    [[nodiscard]] ATTQuat normalized() const;

    [[nodiscard]] ATTQuat update(const Eigen::Vector3d &omegaBody, double dt) const;

    [[nodiscard]] const Eigen::Quaterniond &raw() const;

private:
    Eigen::Quaterniond mQ{1.0, 0.0, 0.0, 0.0}; // (w, x, y, z)
};
