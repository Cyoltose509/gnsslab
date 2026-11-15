#pragma once

#include "core/TimeStruct.h"

#include <Eigen/Geometry>


struct Euler {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};


class DCM {
public:
    DCM() = default; ///< 单位旋转

    explicit DCM(Eigen::Matrix3d m);

    static DCM fromEuler(const Euler &e);

    static DCM fromECEFToENU(double B, double L);

    static DCM fromECEFToNED(double B, double L);

    static DCM fromNEDToECEF(double B, double L);

    /// 局部水平局部竖直（LVLH）：x 沿迹、y 轨道面负法向、z 指向地心。
    static DCM fromECEFToLVLH(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);

    /// 卫星本体：z 指向地心，y 为轨道面法向，x 补右手系。
    static DCM fromECEFToSVBody(const Eigen::Vector3d &pTx, const Eigen::Vector3d &rsunECEF);

    /// 地心惯性 → 地心地固（岁差 + 章动 + 极移 + 自转）。
    static DCM fromECIToECEF(const CommonTime &t, double xp = 0.0, double yp = 0.0);

    [[nodiscard]] Euler toEuler() const;

    /// 脱壳给 Eigen 做矩阵–矩阵运算（协方差旋转这类），类型检查到此为止。
    [[nodiscard]] const Eigen::Matrix3d &matrix() const { return mMat; }

    [[nodiscard]] Eigen::Vector3d row(const int i) const { return mMat.row(i).transpose(); }

    [[nodiscard]] Eigen::Vector3d col(const int i) const { return mMat.col(i); }

    /// 逆（＝转置）：把靶系的量转回源系。
    [[nodiscard]] DCM transpose() const { return DCM(mMat.transpose()); }

    [[nodiscard]] double det() const { return mMat.determinant(); }

    /// ‖CᵀC−I‖∞，
    [[nodiscard]] double orthonormalityError() const {
        return (mMat.transpose() * mMat - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    }

    [[nodiscard]] Eigen::Vector3d operator*(const Eigen::Vector3d &v) const { return mMat * v; }

    [[nodiscard]] DCM operator*(const DCM &rhs) const { return DCM(mMat * rhs.mMat); }

private:
    Eigen::Matrix3d mMat = Eigen::Matrix3d::Identity();
};


class ATTQuat {
public:
    ATTQuat() = default; // 单位四元数
    explicit ATTQuat(Eigen::Quaterniond q);


    static ATTQuat identity();

    static ATTQuat fromDCM(const DCM &C);

    static ATTQuat fromEuler(const Euler &e);

    static ATTQuat fromRotationVector(const Eigen::Vector3d &rv);


    static ATTQuat slerp(const ATTQuat &a, const ATTQuat &b, double t);


    [[nodiscard]] DCM toDCM() const;

    [[nodiscard]] Euler toEuler() const;

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
