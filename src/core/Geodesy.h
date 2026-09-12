#pragma once

#include "Const.h"
#include "CoordConvert.h"
#include "TimeConvert.h"
#include "MathUtils.h"

using namespace  Eigen;

class Geodesy {
public:

    // 太阳/月球地心 ECEF 位置
    static void sunMoonECEF(const CommonTime &t, Vector3d &rsunEcef, Vector3d &rmoonEcef, double &gmstOut) {
        const JulianDate jd = CommonTime2JulianDate(t);
        const double D = static_cast<double>(jd.jd) - 2451545.0; // 自 J2000 日数
        const double T = D / 36525.0; // 儒略世纪
        double f[5];
        fundamentalArguments(T, f);

        // 黄赤交角(日期)
        const double eps = (23.439291 - 0.0130042 * T) * DEG_TO_RAD;
        const double sine = std::sin(eps), cose = std::cos(eps);

        // 太阳
        const double Ms = 357.5277233 + 35999.05034 * T;
        const double ls = 280.460 + 36000.770 * T
                          + 1.914666471 * std::sin(Ms * DEG_TO_RAD) + 0.019994643 * std::sin(2.0 * Ms * DEG_TO_RAD);
        const double rs = ASTRONOMICAL_UNIT * (1.000140612 - 0.016708617 * std::cos(Ms * DEG_TO_RAD) - 0.000139589 * std::cos(
                                                   2.0 * Ms * DEG_TO_RAD));
        const double sl = std::sin(ls * DEG_TO_RAD), cl = std::cos(ls * DEG_TO_RAD);
        const Vector3d sunEci(rs * cl, rs * cose * sl, rs * sine * sl);

        // 月球
        const double lm = 218.32 + 481267.883 * T + 6.29 * std::sin(f[0]) - 1.27 * std::sin(f[0] - 2.0 * f[3])
                          + 0.66 * std::sin(2.0 * f[3]) + 0.21 * std::sin(2.0 * f[0]) - 0.19 * std::sin(f[1]) - 0.11 * std::sin(2.0 * f[2]);
        const double pm = 5.13 * std::sin(f[2]) + 0.28 * std::sin(f[0] + f[2]) - 0.28 * std::sin(f[2] - f[0]) - 0.17 * std::sin(
                              f[2] - 2.0 * f[3]);
        const double rm = RADIUS_EARTH / std::sin((0.9508 + 0.0518 * std::cos(f[0]) + 0.0095 * std::cos(f[0] - 2.0 * f[3])
                                                   + 0.0078 * std::cos(2.0 * f[3]) + 0.0028 * std::cos(2.0 * f[0])) * DEG_TO_RAD);
        const double slm = std::sin(lm * DEG_TO_RAD), clm = std::cos(lm * DEG_TO_RAD);
        const double sp = std::sin(pm * DEG_TO_RAD), cp = std::cos(pm * DEG_TO_RAD);
        const Vector3d moonEci(rm * cp * clm, rm * (cose * cp * slm - sine * sp), rm * (sine * cp * slm + cose * sp));

        // ECI→ECEF
        const Matrix3d U = ECItoECEF(t, gmstOut);
        rsunEcef = U * sunEci;
        rmoonEcef = U * moonEci;
    }

    // 固体潮位移(ECEF, 米)
    static Vector3d solidTideDisplacement(const Vector3d &recvEcef, const CommonTime &t) {
        Vector3d rsun, rmoon;
        double gmst;
        sunMoonECEF(t, rsun, rmoon, gmst);
        auto blh = XYZtoBLH(recvEcef, Frame::WGS84);
        const double pos[2] = {blh[0], blh[1]};
        Matrix3d E = getBLMatrix(blh[0], blh[1]);
        const Vector3d eu = E.row(2).transpose();
        double dr1[3], dr2[3];
        tidePotential(eu.data(), rsun.data(), SUN_GM, pos, dr1);
        tidePotential(eu.data(), rmoon.data(), MOON_GM, pos, dr2);
        // 频率域 K1 径向项
        const double sin2l = std::sin(2.0 * pos[0]);
        const double du = -0.012 * sin2l * std::sin(gmst + pos[1]);
        double dr[3];
        dr[0] = dr1[0] + dr2[0] + du * eu[0];
        dr[1] = dr1[1] + dr2[1] + du * eu[1];
        dr[2] = dr1[2] + dr2[2] + du * eu[2];
        return {dr[0], dr[1], dr[2]};
    }

    // 卫星相位中心机体框架
    static void satelliteBodyFrame(const Vector3d &pTx, const Vector3d &rsunEcef,
                                   Vector3d &ex, Vector3d &ey, Vector3d &ez) {
        ez = -pTx.normalized();
        const Vector3d es = (rsunEcef - pTx).normalized();
        ey = ez.cross(es).normalized();
        ex = ey.cross(ez);
    }

    // 相位缠绕改正的未 unwrap 原始量（单位 cycle）。
    // 卫星端用 satelliteYaw 偏航模型；接收机天线基 (exr, eyr) 由调用方注入，满足 exr×eyr = 天线法向：
    //   地面站 => ENU(North, West)；LEO => 机体 LVLH(沿迹 Xb, -跨迹 Yb)，使法向朝空间。
    static double phaseWindupCorrection(const Vector3d &pTx, const Vector3d &vTx,
                                        const Vector3d &recv, const CommonTime &epoch,
                                        const Vector3d &exr, const Vector3d &eyr) {
        Vector3d rsun, rmoon;
        double gmst;
        sunMoonECEF(epoch, rsun, rmoon, gmst);
        Vector3d exs, eys;
        satelliteYaw(pTx, vTx, rsun, exs, eys); // 卫星星体 x/y(ECEF)

        const Vector3d ek = (recv - pTx).normalized(); // 卫星->接收机 单位向量
        const Vector3d eks = ek.cross(eys);
        const Vector3d ekr = ek.cross(eyr);
        const Vector3d ds = exs - ek * ek.dot(exs) - eks;
        const Vector3d dr = exr - ek * ek.dot(exr) + ekr;

        double cosp = ds.dot(dr) / (ds.norm() * dr.norm());
        if (cosp < -1.0) cosp = -1.0;
        else if (cosp > 1.0) cosp = 1.0;
        double ph = std::acos(cosp) / (2.0 * PI); // cycle
        if (const Vector3d drs = ds.cross(dr); ek.dot(drs) < 0.0) ph = -ph;
        return ph;
    }

    // 地面站版：接收机天线基 = ENU(North, West)，法向朝天(Up)。
    static double phaseWindupCorrection(const Vector3d &pTx, const Vector3d &vTx,
                                        const Vector3d &recv, const CommonTime &epoch) {
        auto blh = XYZtoBLH(recv, Frame::WGS84);
        Matrix3d E = getBLMatrix(blh[0], blh[1]); // 行 = East, North, Up
        const Vector3d exr = E.row(1).transpose(); // 接收机 x = North
        const Vector3d eyr = -E.row(0).transpose(); // 接收机 y = West
        return phaseWindupCorrection(pTx, vTx, recv, epoch, exr, eyr);
    }

private:
    // IAU 1980 章动系数表：列 = l, l', F, D, Ω, 周期, Δψ(0.1mas), Δψ率, Δε(0.1mas), Δε率
    static constexpr double NUTATION_1980[106][10] = {
        {0, 0, 0, 0, 1, -6798.4, -171996, -174.2, 92025, 8.9},
        {0, 0, 2, -2, 2, 182.6, -13187, -1.6, 5736, -3.1},
        {0, 0, 2, 0, 2, 13.7, -2274, -0.2, 977, -0.5},
        {0, 0, 0, 0, 2, -3399.2, 2062, 0.2, -895, 0.5},
        {0, -1, 0, 0, 0, -365.3, -1426, 3.4, 54, -0.1},
        {1, 0, 0, 0, 0, 27.6, 712, 0.1, -7, 0.0},
        {0, 1, 2, -2, 2, 121.7, -517, 1.2, 224, -0.6},
        {0, 0, 2, 0, 1, 13.6, -386, -0.4, 200, 0.0},
        {1, 0, 2, 0, 2, 9.1, -301, 0.0, 129, -0.1},
        {0, -1, 2, -2, 2, 365.2, 217, -0.5, -95, 0.3},
        {-1, 0, 0, 2, 0, 31.8, 158, 0.0, -1, 0.0},
        {0, 0, 2, -2, 1, 177.8, 129, 0.1, -70, 0.0},
        {-1, 0, 2, 0, 2, 27.1, 123, 0.0, -53, 0.0},
        {1, 0, 0, 0, 1, 27.7, 63, 0.1, -33, 0.0},
        {0, 0, 0, 2, 0, 14.8, 63, 0.0, -2, 0.0},
        {-1, 0, 2, 2, 2, 9.6, -59, 0.0, 26, 0.0},
        {-1, 0, 0, 0, 1, -27.4, -58, -0.1, 32, 0.0},
        {1, 0, 2, 0, 1, 9.1, -51, 0.0, 27, 0.0},
        {-2, 0, 0, 2, 0, -205.9, -48, 0.0, 1, 0.0},
        {-2, 0, 2, 0, 1, 1305.5, 46, 0.0, -24, 0.0},
        {0, 0, 2, 2, 2, 7.1, -38, 0.0, 16, 0.0},
        {2, 0, 2, 0, 2, 6.9, -31, 0.0, 13, 0.0},
        {2, 0, 0, 0, 0, 13.8, 29, 0.0, -1, 0.0},
        {1, 0, 2, -2, 2, 23.9, 29, 0.0, -12, 0.0},
        {0, 0, 2, 0, 0, 13.6, 26, 0.0, -1, 0.0},
        {0, 0, 2, -2, 0, 173.3, -22, 0.0, 0, 0.0},
        {-1, 0, 2, 0, 1, 27.0, 21, 0.0, -10, 0.0},
        {0, 2, 0, 0, 0, 182.6, 17, -0.1, 0, 0.0},
        {0, 2, 2, -2, 2, 91.3, -16, 0.1, 7, 0.0},
        {-1, 0, 0, 2, 1, 32.0, 16, 0.0, -8, 0.0},
        {0, 1, 0, 0, 1, 386.0, -15, 0.0, 9, 0.0},
        {1, 0, 0, -2, 1, -31.7, -13, 0.0, 7, 0.0},
        {0, -1, 0, 0, 1, -346.6, -12, 0.0, 6, 0.0},
        {2, 0, -2, 0, 0, -1095.2, 11, 0.0, 0, 0.0},
        {-1, 0, 2, 2, 1, 9.5, -10, 0.0, 5, 0.0},
        {1, 0, 2, 2, 2, 5.6, -8, 0.0, 3, 0.0},
        {0, -1, 2, 0, 2, 14.2, -7, 0.0, 3, 0.0},
        {0, 0, 2, 2, 1, 7.1, -7, 0.0, 3, 0.0},
        {1, 1, 0, -2, 0, -34.8, -7, 0.0, 0, 0.0},
        {0, 1, 2, 0, 2, 13.2, 7, 0.0, -3, 0.0},
        {-2, 0, 0, 2, 1, -199.8, -6, 0.0, 3, 0.0},
        {0, 0, 0, 2, 1, 14.8, -6, 0.0, 3, 0.0},
        {2, 0, 2, -2, 2, 12.8, 6, 0.0, -3, 0.0},
        {1, 0, 0, 2, 0, 9.6, 6, 0.0, 0, 0.0},
        {1, 0, 2, -2, 1, 23.9, 6, 0.0, -3, 0.0},
        {0, 0, 0, -2, 1, -14.7, -5, 0.0, 3, 0.0},
        {0, -1, 2, -2, 1, 346.6, -5, 0.0, 3, 0.0},
        {2, 0, 2, 0, 1, 6.9, -5, 0.0, 3, 0.0},
        {1, -1, 0, 0, 0, 29.8, 5, 0.0, 0, 0.0},
        {1, 0, 0, -1, 0, 411.8, -4, 0.0, 0, 0.0},
        {0, 0, 0, 1, 0, 29.5, -4, 0.0, 0, 0.0},
        {0, 1, 0, -2, 0, -15.4, -4, 0.0, 0, 0.0},
        {1, 0, -2, 0, 0, -26.9, 4, 0.0, 0, 0.0},
        {2, 0, 0, -2, 1, 212.3, 4, 0.0, -2, 0.0},
        {0, 1, 2, -2, 1, 119.6, 4, 0.0, -2, 0.0},
        {1, 1, 0, 0, 0, 25.6, -3, 0.0, 0, 0.0},
        {1, -1, 0, -1, 0, -3232.9, -3, 0.0, 0, 0.0},
        {-1, -1, 2, 2, 2, 9.8, -3, 0.0, 1, 0.0},
        {0, -1, 2, 2, 2, 7.2, -3, 0.0, 1, 0.0},
        {1, -1, 2, 0, 2, 9.4, -3, 0.0, 1, 0.0},
        {3, 0, 2, 0, 2, 5.5, -3, 0.0, 1, 0.0},
        {-2, 0, 2, 0, 2, 1615.7, -3, 0.0, 1, 0.0},
        {1, 0, 2, 0, 0, 9.1, 3, 0.0, 0, 0.0},
        {-1, 0, 2, 4, 2, 5.8, -2, 0.0, 1, 0.0},
        {1, 0, 0, 0, 2, 27.8, -2, 0.0, 1, 0.0},
        {-1, 0, 2, -2, 1, -32.6, -2, 0.0, 1, 0.0},
        {0, -2, 2, -2, 1, 6786.3, -2, 0.0, 1, 0.0},
        {-2, 0, 0, 0, 1, -13.7, -2, 0.0, 1, 0.0},
        {2, 0, 0, 0, 1, 13.8, 2, 0.0, -1, 0.0},
        {3, 0, 0, 0, 0, 9.2, 2, 0.0, 0, 0.0},
        {1, 1, 2, 0, 2, 8.9, 2, 0.0, -1, 0.0},
        {0, 0, 2, 1, 2, 9.3, 2, 0.0, -1, 0.0},
        {1, 0, 0, 2, 1, 9.6, -1, 0.0, 0, 0.0},
        {1, 0, 2, 2, 1, 5.6, -1, 0.0, 1, 0.0},
        {1, 1, 0, -2, 1, -34.7, -1, 0.0, 0, 0.0},
        {0, 1, 0, 2, 0, 14.2, -1, 0.0, 0, 0.0},
        {0, 1, 2, -2, 0, 117.5, -1, 0.0, 0, 0.0},
        {0, 1, -2, 2, 0, -329.8, -1, 0.0, 0, 0.0},
        {1, 0, -2, 2, 0, 23.8, -1, 0.0, 0, 0.0},
        {1, 0, -2, -2, 0, -9.5, -1, 0.0, 0, 0.0},
        {1, 0, 2, -2, 0, 32.8, -1, 0.0, 0, 0.0},
        {1, 0, 0, -4, 0, -10.1, -1, 0.0, 0, 0.0},
        {2, 0, 0, -4, 0, -15.9, -1, 0.0, 0, 0.0},
        {0, 0, 2, 4, 2, 4.8, -1, 0.0, 0, 0.0},
        {0, 0, 2, -1, 2, 25.4, -1, 0.0, 0, 0.0},
        {-2, 0, 2, 4, 2, 7.3, -1, 0.0, 1, 0.0},
        {2, 0, 2, 2, 2, 4.7, -1, 0.0, 0, 0.0},
        {0, -1, 2, 0, 1, 14.2, -1, 0.0, 0, 0.0},
        {0, 0, -2, 0, 1, -13.6, -1, 0.0, 0, 0.0},
        {0, 0, 4, -2, 2, 12.7, 1, 0.0, 0, 0.0},
        {0, 1, 0, 0, 2, 409.2, 1, 0.0, 0, 0.0},
        {1, 1, 2, -2, 2, 22.5, 1, 0.0, -1, 0.0},
        {3, 0, 2, -2, 2, 8.7, 1, 0.0, 0, 0.0},
        {-2, 0, 2, 2, 2, 14.6, 1, 0.0, -1, 0.0},
        {-1, 0, 0, 0, 2, -27.3, 1, 0.0, -1, 0.0},
        {0, 0, -2, 2, 1, -169.0, 1, 0.0, 0, 0.0},
        {0, 1, 2, 0, 1, 13.1, 1, 0.0, 0, 0.0},
        {-1, 0, 4, 0, 2, 9.1, 1, 0.0, 0, 0.0},
        {2, 1, 0, -2, 0, 131.7, 1, 0.0, 0, 0.0},
        {2, 0, 0, 2, 0, 7.1, 1, 0.0, 0, 0.0},
        {2, 0, 2, -2, 1, 12.8, 1, 0.0, -1, 0.0},
        {2, 0, -2, 0, 1, -943.2, 1, 0.0, 0, 0.0},
        {1, -1, 0, -2, 0, -29.3, 1, 0.0, 0, 0.0},
        {-1, 0, 0, 1, 1, -388.3, 1, 0.0, 0, 0.0},
        {-1, -1, 0, 2, 1, 35.0, 1, 0.0, 0, 0.0},
        {0, 1, 0, 1, 0, 27.3, 1, 0.0, 0, 0.0},
    };

    // 五的根本天文参数(IAU 1980 章动 / 月球幅角)
    static void fundamentalArguments(const double t, double f[5]) {
        static const double fc[5][5] = {
            {134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470},
            {357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149},
            {93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417},
            {297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169},
            {125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939}
        };
        double tt[4] = {t, 0, 0, 0};
        for (int i = 1; i < 4; ++i) tt[i] = tt[i - 1] * t;
        for (int i = 0; i < 5; ++i) {
            f[i] = fc[i][0] * 3600.0;
            for (int j = 0; j < 4; ++j) f[i] += fc[i][j + 1] * tt[j];
            f[i] = std::fmod(f[i] * ARCSEC_TO_RAD, 2.0 * PI);
        }
    }

    // IAU 1980 章动 (rtkcmn.c nut_iau1980)
    static void iau1980Nutation(const double t, const double f[5], double &dpsi, double &deps) {
        dpsi = deps = 0.0;
        for (const auto &i: NUTATION_1980) {
            double ang = 0.0;
            for (int j = 0; j < 5; ++j) ang += i[j] * f[j];
            dpsi += (i[6] + i[7] * t) * std::sin(ang);
            deps += (i[8] + i[9] * t) * std::cos(ang);
        }
        dpsi *= 1e-4 * ARCSEC_TO_RAD; // 0.1 mas -> rad
        deps *= 1e-4 * ARCSEC_TO_RAD;
    }

    // ECI→ECEF 变换矩阵 U(列主序)。含 IAU1976 岁差 P、IAU1980 章动 N、视恒星时 GAST、零极移(单位阵)。
    static Matrix3d ECItoECEF(const CommonTime &t, double &gmstOut) {
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
        Matrix3d R1 = Math::rotationZ(-z);
        Matrix3d R2 = Math::rotationY(th);
        Matrix3d R3 = Math::rotationZ(-ze);
        Matrix3d R = R1 * R2;
        const Matrix3d P = R * R3; // P = Rz(-z)*Ry(th)*Rz(-ze)
        // IAU 1980 章动
        double dpsi, deps;
        iau1980Nutation(T, f, dpsi, deps);
        R1 = Math::rotationX(-eps - deps);
        R2 = Math::rotationZ(-dpsi);
        R3 = Math::rotationX(eps);
        R = R1 * R2;
        const Matrix3d N = R * R3;
        const double gmst = gmstFromTime(t);
        double gast = gmst + dpsi * std::cos(eps);
        gast += (0.00264 * std::sin(f[4]) + 0.000063 * std::sin(2.0 * f[4])) * ARCSEC_TO_RAD;
        // 极移(erpv=0 → W=单位阵)；U = W*Rz(gast)*N*P
        R1 = Math::rotationY(0.0);
        R2 = Math::rotationX(0.0);
        R3 = Math::rotationZ(gast);
        const Matrix3d W = R1 * R2;
        R = W * R3;
        const Matrix3d NP = N * P;
        gmstOut = gmst;
        return R * NP;
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

    // 卫星 yaw 姿态（nominal-yaw）：用于相位缠绕的卫星星体 x/y
    static void satelliteYaw(const Vector3d &pTx, const Vector3d &vTx,
                             const Vector3d &rsun, Vector3d &exs, Vector3d &eys) {
        Vector3d ri_pos = pTx;
        Vector3d ri_vel = vTx;
        ri_vel[0] -= OMEGA_EARTH * ri_pos[1]; // 地球自转修正后的轨道速度
        ri_vel[1] += OMEGA_EARTH * ri_pos[0];
        const Vector3d en = ri_pos.cross(ri_vel).normalized(); // 轨道法向
        const Vector3d ep = rsun.cross(en).normalized(); // 太阳-轨道面交线
        const Vector3d es = pTx.normalized();
        const Vector3d esun = rsun.normalized();
        const double beta = PI / 2.0 - std::acos(en.dot(esun)); // 太阳赤纬角
        const double E = std::acos(es.dot(ep)); // 轨道角
        double mu = PI / 2.0 + (es.dot(esun) <= 0.0 ? -E : E);
        if (mu < -PI / 2.0) mu += 2.0 * PI;
        else if (mu >= PI / 2.0) mu -= 2.0 * PI;
        const double yaw = std::fabs(beta) < 1e-12 && std::fabs(mu) < 1e-12
                               ? PI
                               : std::atan2(-std::tan(beta), std::sin(mu)) + PI;
        const Vector3d ex = en.cross(es).normalized();
        const double cosy = std::cos(yaw), siny = std::sin(yaw);
        exs = -siny * en + cosy * ex;
        eys = -cosy * en - siny * ex;
    }
};
