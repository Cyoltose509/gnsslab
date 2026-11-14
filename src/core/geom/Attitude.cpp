#include <utility>

#include "core/geom/Attitude.h"

#include "MathUtils.h"
#include "core/Const.h"


DCM::DCM(Eigen::Matrix3d m) : mMat(std::move(m)) {
}

DCM DCM::fromEuler(const Euler &e) {
    const double cr = std::cos(e.roll), sr = std::sin(e.roll);
    const double cp = std::cos(e.pitch), sp = std::sin(e.pitch);
    const double cy = std::cos(e.yaw), sy = std::sin(e.yaw);
    Eigen::Matrix3d m;
    m << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy,
            cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy,
            -sp, sr * cp, cr * cp;
    return DCM(m);
}

Euler DCM::toEuler() const {
    Euler e;
    e.pitch = std::atan2(-mMat(2, 0), std::sqrt(mMat(2, 1) * mMat(2, 1) + mMat(2, 2) * mMat(2, 2)));
    e.roll = std::atan2(mMat(2, 1), mMat(2, 2));
    e.yaw = std::atan2(mMat(1, 0), mMat(0, 0));
    return e;
}

DCM DCM::fromECEFToENU(const double B, const double L) {
    const double sinB = std::sin(B);
    const double cosB = std::cos(B);
    const double sinL = std::sin(L);
    const double cosL = std::cos(L);

    Eigen::Matrix3d m;
    m << -sinL, cosL, 0.0,
            -sinB * cosL, -sinB * sinL, cosB,
            cosB * cosL, cosB * sinL, sinB;
    return DCM(m);
}

DCM DCM::fromECEFToNED(const double B, const double L) {
    const double sinB = std::sin(B), cosB = std::cos(B);
    const double sinL = std::sin(L), cosL = std::cos(L);
    Eigen::Matrix3d m;
    m << -sinB * cosL, -sinB * sinL, cosB,
            -sinL, cosL, 0.0,
            -cosB * cosL, -cosB * sinL, -sinB;
    return DCM(m);
}

DCM DCM::fromNEDToECEF(const double B, const double L) {
    return fromECEFToNED(B, L).transpose();
}

DCM DCM::fromECEFToLVLH(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
    const Eigen::Vector3d ur = pos.normalized();
    Eigen::Vector3d ut;
    if (vel.squaredNorm() > 1.0) ut = vel.normalized();
    else ut = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d uc = ur.cross(ut);
    if (uc.squaredNorm() < 1e-6) uc = Eigen::Vector3d(1, 0, 0);
    uc.normalize();
    ut = uc.cross(ur).normalized();
    Eigen::Matrix3d m;
    m.row(0) = ut;
    m.row(1) = -uc;
    m.row(2) = -ur;
    return DCM(m);
}

DCM DCM::fromECEFToSVBody(const Eigen::Vector3d &pTx, const Eigen::Vector3d &rsunECEF) {
    const Eigen::Vector3d ez = -pTx.normalized();
    const Eigen::Vector3d es = (rsunECEF - pTx).normalized();
    const Eigen::Vector3d ey = ez.cross(es).normalized();
    const Eigen::Vector3d ex = ey.cross(ez);
    Eigen::Matrix3d m;
    m.row(0) = ex;
    m.row(1) = ey;
    m.row(2) = ez;
    return DCM(m);
}

DCM DCM::fromECIToECEF(const CommonTime &t, const double xp, const double yp) {
    const double jdFull = static_cast<double>(t.toJulianDate().jd);
    const double T = (jdFull - 2451545.0) / 36525.0;
    const double t2 = T * T, t3 = t2 * T;
    double f[5];
    fundamentalArguments(T, f);

    const double ze = (2306.2181 * T + 0.30188 * t2 + 0.017998 * t3) * ARCSEC_TO_RAD;
    const double th = (2004.3109 * T - 0.42665 * t2 - 0.041833 * t3) * ARCSEC_TO_RAD;
    const double z = (2306.2181 * T + 1.09468 * t2 + 0.018203 * t3) * ARCSEC_TO_RAD;
    const double eps = (84381.448 - 46.8150 * T - 0.00059 * t2 + 0.001813 * t3) * ARCSEC_TO_RAD;
    DCM R1(Math::rotationZ(-z));
    DCM R2(Math::rotationY(th));
    DCM R3(Math::rotationZ(-ze));
    DCM R = R1 * R2;
    const DCM P = R * R3;

    double dpsi, deps;
    iau1980Nutation(T, f, dpsi, deps);
    R1 = DCM(Math::rotationX(-eps - deps));
    R2 = DCM(Math::rotationZ(-dpsi));
    R3 = DCM(Math::rotationX(eps));
    R = R1 * R2;
    const DCM N = R * R3;
    const double gmst = t.toGMST();
    double gast = gmst + dpsi * std::cos(eps);
    gast += (0.00264 * std::sin(f[4]) + 0.000063 * std::sin(2.0 * f[4])) * ARCSEC_TO_RAD;
    R1 = DCM(Math::rotationY(-xp));
    R2 = DCM(Math::rotationX(-yp));
    R3 = DCM(Math::rotationZ(gast));
    const DCM W = R1 * R2;
    R = W * R3;
    return R * (N * P);
}


ATTQuat::ATTQuat(Eigen::Quaterniond q) : mQ(std::move(q)) {
}

ATTQuat ATTQuat::identity() { return ATTQuat(Eigen::Quaterniond::Identity()); }

ATTQuat ATTQuat::fromDCM(const DCM &C) { return ATTQuat(Eigen::Quaterniond(C.matrix())); }

ATTQuat ATTQuat::fromEuler(const Euler &e) { return ATTQuat(Eigen::Quaterniond(DCM::fromEuler(e).matrix())); }

ATTQuat ATTQuat::fromRotationVector(const Eigen::Vector3d &rv) {
    const double ang = rv.norm();
    if (ang < 1e-12) return identity();
    return ATTQuat(Eigen::Quaterniond(Eigen::AngleAxisd(ang, rv / ang)));
}

ATTQuat ATTQuat::slerp(const ATTQuat &a, const ATTQuat &b, const double t) {
    return ATTQuat(a.mQ.slerp(t, b.mQ)).normalized();
}

DCM ATTQuat::toDCM() const { return DCM(mQ.toRotationMatrix()); }

Euler ATTQuat::toEuler() const { return toDCM().toEuler(); }

Eigen::Vector3d ATTQuat::toRotationVector() const {
    const Eigen::AngleAxisd aa(normalized().toDCM().matrix());
    return aa.axis() * aa.angle();
}

double ATTQuat::norm() const { return mQ.coeffs().norm(); }

Eigen::Vector3d ATTQuat::rotate(const Eigen::Vector3d &v) const { return mQ * v; }

ATTQuat ATTQuat::operator*(const ATTQuat &rhs) const { return ATTQuat(mQ * rhs.mQ); }

ATTQuat ATTQuat::conjugated() const { return ATTQuat(mQ.conjugate()); }

ATTQuat ATTQuat::normalized() const {
    const double n = mQ.coeffs().norm();
    return n > 1e-12 ? ATTQuat(Eigen::Quaterniond(mQ.coeffs() / n)) : identity();
}

ATTQuat ATTQuat::update(const Eigen::Vector3d &omegaBody, const double dt) const {
    return (normalized() * fromRotationVector(omegaBody * dt)).normalized();
}

const Eigen::Quaterniond &ATTQuat::raw() const { return mQ; }
