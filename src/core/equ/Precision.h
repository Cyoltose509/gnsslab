#pragma once

#include <algorithm>

#include <Eigen/Core>

#include "core/geom/Attitude.h"
#include "core/geom/CoordConvert.h"
#include "core/equ/EquStruct.h"
#include "core/equ/Result.h"


inline bool geometryDOPs(Result &result, const EquSys &eq, const Eigen::Vector3d &blh) {
    Eigen::Matrix4d N = Eigen::Matrix4d::Zero();
    bool hasClock = false;
    int n = 0;
    for (const auto &[eid, ed]: eq.obsEquData) {
        if (eid.obsType.empty() || eid.obsType[0] == 'L') continue;
        Eigen::Vector4d h = Eigen::Vector4d::Zero();
        for (const auto &[var, coeff]: ed.varCoeffData) {
            if (const auto pt = var.par; pt == Parameter::dX) h[0] = coeff;
            else if (pt == Parameter::dY) h[1] = coeff;
            else if (pt == Parameter::dZ) h[2] = coeff;
            else if (pt.isClock()) {
                h[3] = coeff;
                hasClock = true;
            }
        }
        if (h.head<3>().isZero()) continue;
        N += h * h.transpose();
        ++n;
    }
    const int dim = hasClock ? 4 : 3;
    if (n < dim) return false;
    const Eigen::MatrixXd Ns = N.topLeftCorner(dim, dim);
    const Eigen::FullPivLU<Eigen::MatrixXd> lu(Ns);
    if (!lu.isInvertible()) return false;
    const Eigen::MatrixXd inv = lu.inverse();
    const Eigen::Matrix3d Q = inv.topLeftCorner(3, 3);
    const double clockVar = hasClock ? inv(3, 3) : 0.0;
    result.pdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    result.gdop = std::sqrt(result.pdop * result.pdop + clockVar);
    result.tdop = clockVar > 0.0 ? std::sqrt(clockVar) : 0.0;
    const DCM R = DCM::fromECEFToENU(blh[0], blh[1]);
    const Eigen::Matrix3d C = R.matrix() * Q * R.matrix().transpose();
    result.hdop = std::sqrt(C(0, 0) + C(1, 1));
    result.vdop = std::sqrt(C(2, 2));
    return true;
}

/// 验后方差因子
inline double varianceFactor(const double chi2, const int dof) {
    if (dof <= 0) return 1.0;
    return std::sqrt((std::max)(chi2 / dof, 0.0));
}

inline double posteriorSigma0(const Eigen::VectorXd &v, const Eigen::VectorXd &w, const int numUnk) {
    const Eigen::Index n = (std::min)(v.size(), w.size());
    double chi2 = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) chi2 += w(i) * v(i) * v(i);
    return varianceFactor(chi2, static_cast<int>(n) - numUnk);
}

/// 位置 1σ
inline double positionSigma(const Eigen::Matrix3d &Q, const double sigma0) {
    return std::sqrt((std::max)(Q.trace(), 0.0)) * sigma0;
}
