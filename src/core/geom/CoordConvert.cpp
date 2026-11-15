#include "core/geom/CoordConvert.h"

#include "core/geom/Attitude.h"

namespace Coord {
    BLH XYZToBLH(const XYZ &xyz, const GeodeticFrame &frame) {
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
                throw GeometryException("ECEF->geodetic: latitude iteration did not converge in 100 steps "
                                        "(X/Y/Z not finite or nearly degenerate)");
        } while (true);


        double L = std::atan2(xyz[1], xyz[0]);


        double H = rho / std::cos(B1) - N;


        return {B1, L, H};
    }

    XYZ BLHToXYZ(const BLH &blh, const GeodeticFrame &frame) {
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

    ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const GeodeticFrame &frame) {
        const auto diffXYZ = xyz - refXYZ;
        const auto refBLH = XYZToBLH(refXYZ, frame);
        return {DCM::fromECEFToENU(refBLH.B(), refBLH.L()) * diffXYZ};
    }

    XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const GeodeticFrame &frame) {
        const auto refBLH = XYZToBLH(refXYZ, frame);
        const auto diffXYZ = DCM::fromECEFToENU(refBLH.B(), refBLH.L()).transpose() * Eigen::Vector3d(enu);
        XYZ resXYZ;
        resXYZ << refXYZ[0] + diffXYZ[0], refXYZ[1] + diffXYZ[1], refXYZ[2] + diffXYZ[2];
        return resXYZ;
    }

    Eigen::Vector3d XYZToRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                             const Eigen::Vector3d &velRef) {
        const Eigen::Vector3d b = DCM::fromECEFToLVLH(posRef, velRef) * (posEst - posRef);
        return {-b.z(), b.x(), -b.y()};
    }

    NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const GeodeticFrame &frame) {
        const Eigen::Vector3d diff = xyz - refXYZ;
        const auto refBLH = XYZToBLH(refXYZ, frame);
        return {DCM::fromECEFToNED(refBLH.B(), refBLH.L()) * diff};
    }

    XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const GeodeticFrame &frame) {
        const auto refBLH = XYZToBLH(refXYZ, frame);
        const Eigen::Vector3d d = DCM::fromECEFToNED(refBLH.B(), refBLH.L()).transpose() * Eigen::Vector3d(ned);
        return {refXYZ.X() + d.x(), refXYZ.Y() + d.y(), refXYZ.Z() + d.z()};
    }


    double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const GeodeticFrame &frame) {
        const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
        const double elev = std::atan2(enu(2), std::sqrt(enu(0) * enu(0) + enu(1) * enu(1)));
        return elev;
    }

    double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const GeodeticFrame &frame) {
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

