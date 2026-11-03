#include "CoordConvert.h"

#include "MathUtils.h"

namespace Coord {
    DCM EulerToDCM(const double roll, const double pitch, const double yaw) {
        const double cr = std::cos(roll), sr = std::sin(roll);
        const double cp = std::cos(pitch), sp = std::sin(pitch);
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        DCM C;
        C << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy,
                cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy,
                -sp, sr * cp, cr * cp;
        return C;
    }

    void DCMToEuler(const DCM &C, double &roll, double &pitch, double &yaw) {
        pitch = std::atan2(-C(2, 0), std::sqrt(C(2, 1) * C(2, 1) + C(2, 2) * C(2, 2)));
        roll = std::atan2(C(2, 1), C(2, 2));
        yaw = std::atan2(C(1, 0), C(0, 0));
    }

    DCM ECEFToENUDCM(const double B, const double L) {
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


    DCM ECEFToLVLHDCM(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) {
        const Eigen::Vector3d ur = pos.normalized();
        Eigen::Vector3d ut;
        if (vel.squaredNorm() > 1.0) ut = vel.normalized();
        else ut = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d uc = ur.cross(ut);
        if (uc.squaredNorm() < 1e-6) uc = Eigen::Vector3d(1, 0, 0);
        uc.normalize();
        ut = uc.cross(ur).normalized();
        DCM M;
        M.row(0) = ut;
        M.row(1) = -uc;
        M.row(2) = -ur;
        return M;
    }

    DCM ECEFToSVBodyDCM(const Eigen::Vector3d &pTx, const Eigen::Vector3d &rsunEcef) {
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

    DCM ECEFToNEDDCM(const double B, const double L) {
        const double sinB = std::sin(B), cosB = std::cos(B);
        const double sinL = std::sin(L), cosL = std::cos(L);
        DCM C;
        C << -sinB * cosL, -sinB * sinL, cosB,
                -sinL, cosL, 0.0,
                -cosB * cosL, -cosB * sinL, -sinB;
        return C;
    }

    DCM NEDToECEFDCM(const double B, const double L) {
        return ECEFToNEDDCM(B, L).transpose();
    }

    DCM ECIToECEFDCM(const CommonTime &t, double &gmstOut, const double xp, const double yp) {
        const double jdFull = static_cast<double>(Time::CommonToJulian(t).jd);
        const double T = (jdFull - 2451545.0) / 36525.0;
        const double t2 = T * T, t3 = t2 * T;
        double f[5];
        fundamentalArguments(T, f);

        const double ze = (2306.2181 * T + 0.30188 * t2 + 0.017998 * t3) * ARCSEC_TO_RAD;
        const double th = (2004.3109 * T - 0.42665 * t2 - 0.041833 * t3) * ARCSEC_TO_RAD;
        const double z = (2306.2181 * T + 1.09468 * t2 + 0.018203 * t3) * ARCSEC_TO_RAD;
        const double eps = (84381.448 - 46.8150 * T - 0.00059 * t2 + 0.001813 * t3) * ARCSEC_TO_RAD;
        DCM R1 = Math::rotationZ(-z);
        DCM R2 = Math::rotationY(th);
        DCM R3 = Math::rotationZ(-ze);
        DCM R = R1 * R2;
        const DCM P = R * R3;

        double dpsi, deps;
        iau1980Nutation(T, f, dpsi, deps);
        R1 = Math::rotationX(-eps - deps);
        R2 = Math::rotationZ(-dpsi);
        R3 = Math::rotationX(eps);
        R = R1 * R2;
        const DCM N = R * R3;
        const double gmst = Time::CommonToGMST(t);
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


    BLH XYZToBLH(const XYZ &xyz, const FrameInfo &frame) {
        const double a = frame.a;
        const double e2 = frame.e2;

        const double rho = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);


        constexpr double eps = 1.0e-13;


        if (rho < eps) {
            double B = xyz.Z() > 0 ? PI / 2 : -PI / 2;
            double L = 0.0;
            double H = std::fabs(xyz.Z()) - a * std::sqrt(1 - e2);

            return {B, L, H};
        }


        double B0 = std::atan2(xyz.Z(), rho);


        int iterationCount = 0;
        double B1, N;
        do {
            N = a / std::sqrt(1 - e2 * std::sin(B0) * std::sin(B0));
            B1 = std::atan2(xyz.Z() + e2 * N * std::sin(B0), rho);

            if (std::fabs(B1 - B0) < eps) break;

            B0 = B1;
            iterationCount++;

            if (constexpr int maxIterations = 100; iterationCount > maxIterations)
                throw GeometryException("ECEF→大地坐标：纬度迭代 100 次未收敛（输入 X/Y/Z 可能不是有限值或接近退化）");
        } while (true);


        double L = std::atan2(xyz[1], xyz[0]);


        double H = rho / std::cos(B1) - N;


        return {B1, L, H};
    }

    XYZ BLHToXYZ(const BLH &blh, const FrameInfo &frame) {
        const double B = blh[0];
        const double L = blh[1];
        const double H = blh[2];

        const double a = frame.a;
        const double e2 = frame.e2;

        const double sinB = std::sin(B);
        const double cosB = std::cos(B);
        const double sinL = std::sin(L);
        const double cosL = std::cos(L);

        const double N = a / std::sqrt(1 - e2 * sinB * sinB);

        double x = (N + H) * cosB * cosL;
        double y = (N + H) * cosB * sinL;
        double z = (N * (1 - e2) + H) * sinB;

        return {x, y, z};
    }

    ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame) {
        const auto diffXYZ = xyz - refXYZ;
        const auto refBLH = XYZToBLH(refXYZ, frame);
        const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
        return {R * diffXYZ};
    }

    XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const FrameInfo &frame) {
        const auto refBLH = XYZToBLH(refXYZ, frame);
        const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
        const auto diffXYZ = R.transpose() * enu;
        XYZ resXYZ;
        resXYZ << refXYZ[0] + diffXYZ[0], refXYZ[1] + diffXYZ[1], refXYZ[2] + diffXYZ[2];
        return resXYZ;
    }

    Eigen::Vector3d XYZToRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                             const Eigen::Vector3d &velRef) {
        const Eigen::Vector3d b = ECEFToLVLHDCM(posRef, velRef) * (posEst - posRef);
        return {-b.z(), b.x(), -b.y()};
    }

    NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame) {
        const Eigen::Vector3d diff = xyz - refXYZ;
        const auto refBLH = XYZToBLH(refXYZ, frame);
        return {ECEFToNEDDCM(refBLH.B(), refBLH.L()) * diff};
    }

    XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const FrameInfo &frame) {
        const auto refBLH = XYZToBLH(refXYZ, frame);
        const Eigen::Vector3d d = ECEFToNEDDCM(refBLH.B(), refBLH.L()).transpose() * Eigen::Vector3d(ned);
        return {refXYZ.X() + d.x(), refXYZ.Y() + d.y(), refXYZ.Z() + d.z()};
    }


    double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame) {
        const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
        const double elev = std::atan2(enu(2), std::sqrt(enu(0) * enu(0) + enu(1) * enu(1)));
        return elev;
    }

    double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame) {
        const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
        double az = std::atan2(enu(0), enu(1));
        if (az < 0.0) az += PI * 2.0;
        return az;
    }

    void geometricRangeAndLos(const Eigen::Vector3d &satPos, const Eigen::Vector3d &recvPos,
                              double &rho, Eigen::Vector3d &losUnit) {
        const Eigen::Vector3d d = recvPos - satPos;
        rho = d.norm();
        losUnit = d.normalized();
    }
}


ATTQuat::ATTQuat(Eigen::Quaterniond q) : mQ(std::move(q)) {
}

ATTQuat ATTQuat::identity() { return ATTQuat(Eigen::Quaterniond::Identity()); }

ATTQuat ATTQuat::fromDCM(const DCM &C) { return ATTQuat(Eigen::Quaterniond(C)); }

ATTQuat ATTQuat::fromEuler(const double roll, const double pitch, const double yaw) {
    return ATTQuat(Eigen::Quaterniond(Coord::EulerToDCM(roll, pitch, yaw)));
}

ATTQuat ATTQuat::fromRotationVector(const Eigen::Vector3d &rv) {
    const double ang = rv.norm();
    if (ang < 1e-12) return identity();
    return ATTQuat(Eigen::Quaterniond(Eigen::AngleAxisd(ang, rv / ang)));
}

ATTQuat ATTQuat::slerp(const ATTQuat &a, const ATTQuat &b, const double t) {
    return ATTQuat(a.mQ.slerp(t, b.mQ)).normalized();
}

DCM ATTQuat::toDCM() const { return mQ.toRotationMatrix(); }

void ATTQuat::toEuler(double &roll, double &pitch, double &yaw) const { Coord::DCMToEuler(toDCM(), roll, pitch, yaw); }

Eigen::Vector3d ATTQuat::toRotationVector() const {
    const Eigen::AngleAxisd aa(normalized().toDCM());
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
