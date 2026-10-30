#pragma once

#include "Const.h"
#include "CoordConvert.h"
#include "TimeConvert.h"
#include <Eigen/Eigen>


class Geodesy {
public:

    static void sunMoonECEF(const CommonTime &t, Eigen::Vector3d &rsunEcef, Eigen::Vector3d &rmoonEcef, double &gmstOut) {
        const JulianDate jd = CommonTime2JulianDate(t);
        const double D = static_cast<double>(jd.jd) - 2451545.0;
        const double T = D / 36525.0;
        double f[5];
        fundamentalArguments(T, f);


        const double eps = (23.439291 - 0.0130042 * T) * DEG_TO_RAD;
        const double sine = std::sin(eps), cose = std::cos(eps);


        const double Ms = 357.5277233 + 35999.05034 * T;
        const double ls = 280.460 + 36000.770 * T
                          + 1.914666471 * std::sin(Ms * DEG_TO_RAD) + 0.019994643 * std::sin(2.0 * Ms * DEG_TO_RAD);
        const double rs = ASTRONOMICAL_UNIT * (1.000140612 - 0.016708617 * std::cos(Ms * DEG_TO_RAD) - 0.000139589 * std::cos(
                                                   2.0 * Ms * DEG_TO_RAD));
        const double sl = std::sin(ls * DEG_TO_RAD), cl = std::cos(ls * DEG_TO_RAD);
        const Eigen::Vector3d sunEci(rs * cl, rs * cose * sl, rs * sine * sl);


        const double lm = 218.32 + 481267.883 * T + 6.29 * std::sin(f[0]) - 1.27 * std::sin(f[0] - 2.0 * f[3])
                          + 0.66 * std::sin(2.0 * f[3]) + 0.21 * std::sin(2.0 * f[0]) - 0.19 * std::sin(f[1]) - 0.11 * std::sin(2.0 * f[2]);
        const double pm = 5.13 * std::sin(f[2]) + 0.28 * std::sin(f[0] + f[2]) - 0.28 * std::sin(f[2] - f[0]) - 0.17 * std::sin(
                              f[2] - 2.0 * f[3]);
        const double rm = RADIUS_EARTH / std::sin((0.9508 + 0.0518 * std::cos(f[0]) + 0.0095 * std::cos(f[0] - 2.0 * f[3])
                                                   + 0.0078 * std::cos(2.0 * f[3]) + 0.0028 * std::cos(2.0 * f[0])) * DEG_TO_RAD);
        const double slm = std::sin(lm * DEG_TO_RAD), clm = std::cos(lm * DEG_TO_RAD);
        const double sp = std::sin(pm * DEG_TO_RAD), cp = std::cos(pm * DEG_TO_RAD);
        const Eigen::Vector3d moonEci(rm * cp * clm, rm * (cose * cp * slm - sine * sp), rm * (sine * cp * slm + cose * sp));


        const Eigen::Matrix3d U = ECIToECEFDCM(t, gmstOut);
        rsunEcef = U * sunEci;
        rmoonEcef = U * moonEci;
    }


    static Eigen::Vector3d solidTideDisplacement(const Eigen::Vector3d &recvEcef, const CommonTime &t) {
        Eigen::Vector3d rsun, rmoon;
        double gmst;
        sunMoonECEF(t, rsun, rmoon, gmst);
        auto blh = XYZToBLH(recvEcef, Frame::WGS84);
        const double pos[2] = {blh[0], blh[1]};
        Eigen::Matrix3d E = ECEFToENUDCM(blh[0], blh[1]);
        const Eigen::Vector3d eu = E.row(2).transpose();
        double dr1[3], dr2[3];
        tidePotential(eu.data(), rsun.data(), SUN_GM, pos, dr1);
        tidePotential(eu.data(), rmoon.data(), MOON_GM, pos, dr2);

        const double sin2l = std::sin(2.0 * pos[0]);
        const double du = -0.012 * sin2l * std::sin(gmst + pos[1]);
        double dr[3];
        dr[0] = dr1[0] + dr2[0] + du * eu[0];
        dr[1] = dr1[1] + dr2[1] + du * eu[1];
        dr[2] = dr1[2] + dr2[2] + du * eu[2];
        return {dr[0], dr[1], dr[2]};
    }



    static double meridianRadius(const double B, const FrameInfo &frame = Frame::WGS84) {
        const double s = std::sin(B);
        const double w2 = 1.0 - frame.e2 * s * s;
        return frame.a * (1.0 - frame.e2) / (w2 * std::sqrt(w2));
    }

    static double primeVerticalRadius(const double B, const FrameInfo &frame = Frame::WGS84) {
        const double s = std::sin(B);
        return frame.a / std::sqrt(1.0 - frame.e2 * s * s);
    }


    static double normalGravity(const double latDeg, const FrameInfo &frame = Frame::WGS84, const double h = 0.0) {
        const double s = std::sin(latDeg * DEG_TO_RAD);
        const double s2 = s * s;
        const double g0 = 9.7803267715 * (1.0 + 0.00193185138639 * s2) / std::sqrt(1.0 - frame.e2 * s2);
        return g0 - 3.0877e-6 * h;
    }

    static Eigen::Vector3d earthRateInECEF(const double omega = OMEGA_EARTH) {
        return {0.0, 0.0, omega};
    }

    static Eigen::Vector3d transportRateInNav(const double latDeg, const double vNorth, const double vEast,
                                              const double height = 0.0,
                                              const FrameInfo &frame = Frame::WGS84) {
        const double B = latDeg * DEG_TO_RAD;
        const double Rn = primeVerticalRadius(B, frame) + height;
        const double Rm = meridianRadius(B, frame) + height;
        return {vEast / Rn, -vNorth / Rm, -vEast * std::tan(B) / Rn};
    }

    static double phaseWindupCorrection(const Eigen::Vector3d &pTx,
                                        const Eigen::Vector3d &recv, const CommonTime &epoch,
                                        const Eigen::Vector3d &exr, const Eigen::Vector3d &eyr) {
        Eigen::Vector3d rsun, rmoon;
        double gmst;
        sunMoonECEF(epoch, rsun, rmoon, gmst);
        const Eigen::Matrix3d bodyM = ECEFToSVBodyDCM(pTx, rsun);
        const Eigen::Vector3d exs = bodyM.row(0).transpose();
        const Eigen::Vector3d eys = bodyM.row(1).transpose();

        const Eigen::Vector3d ek = (recv - pTx).normalized();
        const Eigen::Vector3d eks = ek.cross(eys);
        const Eigen::Vector3d ekr = ek.cross(eyr);
        const Eigen::Vector3d ds = exs - ek * ek.dot(exs) - eks;
        const Eigen::Vector3d dr = exr - ek * ek.dot(exr) + ekr;

        double cosp = ds.dot(dr) / (ds.norm() * dr.norm());
        if (cosp < -1.0) cosp = -1.0;
        else if (cosp > 1.0) cosp = 1.0;
        double ph = std::acos(cosp) / (2.0 * PI);
        if (const Eigen::Vector3d drs = ds.cross(dr); ek.dot(drs) < 0.0) ph = -ph;
        return ph;
    }

    static double phaseWindupCorrection(const Eigen::Vector3d &pTx,
                                        const Eigen::Vector3d &recv, const CommonTime &epoch) {
        auto blh = XYZToBLH(recv, Frame::WGS84);
        Eigen::Matrix3d E = ECEFToENUDCM(blh[0], blh[1]);
        const Eigen::Vector3d exr = E.row(1).transpose();
        const Eigen::Vector3d eyr = -E.row(0).transpose();
        return phaseWindupCorrection(pTx, recv, epoch, exr, eyr);
    }

    static Eigen::Vector3d poleTideDisplacement(const Eigen::Vector3d &recvEcef, const CommonTime &t,
                                         const double xp, const double yp) {
        const auto blh = XYZToBLH(recvEcef, Frame::WGS84);
        const double phi = blh[0], lam = blh[1];
        const double th = PI / 2.0 - phi;


        const auto yds = CommonTime2YDSTime(t);
        const double year = static_cast<double>(yds.year) + static_cast<double>(yds.doy) / 365.25;
        const double xbar = 0.054 + 0.00083 * (year - 2000.0);
        const double ybar = 0.357 + 0.00395 * (year - 2000.0);


        const double m1 = xp / ARCSEC_TO_RAD - xbar;
        const double m2 = -(yp / ARCSEC_TO_RAD - ybar);


        const double A = m1 * std::cos(lam) + m2 * std::sin(lam);
        const double B = m1 * std::sin(lam) - m2 * std::cos(lam);
        constexpr double MM = 1.0e-3;
        const double Sr = -33.0 * std::sin(2.0 * th) * A * MM;
        const double Sth = -9.0 * std::cos(2.0 * th) * A * MM;
        const double Slam = 9.0 * std::cos(th) * B * MM;


        const double st = std::sin(th), ct = std::cos(th);
        const double sl = std::sin(lam), cl = std::cos(lam);
        const Eigen::Vector3d south(ct * cl, ct * sl, -st);
        const Eigen::Vector3d east(-sl, cl, 0.0);
        const Eigen::Vector3d up(st * cl, st * sl, ct);
        return Sth * south + Slam * east + Sr * up;
    }

    // 单天体固体潮位移(度2+度3)
    static void tidePotential(const double *eu, const double *rp, const double gm_p, const double *pos, double *dr) {
        constexpr double H3 = 0.292, L3 = 0.015;
        const double r = std::sqrt(rp[0] * rp[0] + rp[1] * rp[1] + rp[2] * rp[2]);
        dr[0] = dr[1] = dr[2] = 0.0;
        if (r <= 0.0) return;
        const double ep[3] = {rp[0] / r, rp[1] / r, rp[2] / r};
        const double K2 = gm_p / EARTH_GM * RADIUS_EARTH * RADIUS_EARTH * RADIUS_EARTH * RADIUS_EARTH / (r * r * r);
        const double K3 = K2 * RADIUS_EARTH / r;
        const double latp = std::asin(ep[2]);
        const double lonp = std::atan2(ep[1], ep[0]);
        const double cosp = std::cos(latp);
        const double sinl = std::sin(pos[0]);
        const double cosl = std::cos(pos[0]);
        const double p = (3.0 * sinl * sinl - 1.0) / 2.0;
        const double H2 = 0.6078 - 0.0006 * p;
        const double L2 = 0.0847 + 0.0002 * p;
        const double a = ep[0] * eu[0] + ep[1] * eu[1] + ep[2] * eu[2];
        double dp = K2 * 3.0 * L2 * a;
        double du = K2 * (H2 * (1.5 * a * a - 0.5) - 3.0 * L2 * a * a);
        // 度 3
        dp += K3 * L3 * (7.5 * a * a - 1.5);
        du += K3 * (H3 * (2.5 * a * a * a - 1.5 * a) - L3 * (7.5 * a * a - 1.5) * a);
        // 异相(仅径向)
        du += 3.0 / 4.0 * 0.0025 * K2 * std::sin(2.0 * latp) * std::sin(2.0 * pos[0]) * std::sin(pos[1] - lonp);
        du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * std::sin(2.0 * (pos[1] - lonp));
        dr[0] = dp * ep[0] + du * eu[0];
        dr[1] = dp * ep[1] + du * eu[1];
        dr[2] = dp * ep[2] + du * eu[2];
    }

};
