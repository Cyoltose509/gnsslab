#include "Ephemeris.h"

inline double c_Ek(const double M, const double e) {
    double E = M;
    double E_old;
    int iter = 0;
    do {
        E_old = E;
        E = M + e * sin(E_old);
        if (iter++ > 100) break;
    } while (std::abs(E - E_old) > 1e-13);
    return E;
}

constexpr double lonPRN[6] = {0.0, 1.027, 1.396, 1.931, 2.443, 2.793};

PVT Ephemeris::svPVT(CommonTime t) {
    convertTimeSystem(t, timeSystem);
    PVT sv;
    const double tk = t - getCommonTime();
    const double A = rootA * rootA; //计算轨道长半径
    const double n0 = sqrt(refFrame.gm / (A * A * A)); //平均角速度0.000145859rad/s
    const double nA = n0 + dn; //改正平均角速度
    const double Mk = m0 + nA * tk;
    const double Ek = c_Ek(Mk, e);
    const double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2)); //计算真近点角
    const double phi = vk + omega; //卫星与升交点间地心夹角
    const double s2 = sin(2 * phi), c2 = cos(2 * phi);
    const double Qu = cus * s2 + cuc * c2, Qr = crs * s2 + crc * c2, Qi = cis * s2 + cic * c2; //短周期摄动项
    const double uk = phi + Qu, rk = A * (1 - e * cos(Ek)) + Qr; //短周期摄动改正，ik的值不确定
    const double ik = i0 + Qi + idot * tk;
    const double x0 = rk * cos(uk), y0 = rk * sin(uk);

    // 处理北斗 GEO 卫星 (PRN 1-5, 59-62)
    double OMEk, sOMEk, cOMEk;
    if (type == 'C' && ((prn >= 1 && prn <= 5) || (prn >= 59 && prn <= 62))) {
        OMEk = omega0 + omegaDot * tk - refFrame.omega * toe;
        sOMEk = sin(OMEk), cOMEk = cos(OMEk);
        const double cosik = cos(ik), sinik = sin(ik);
        const double xgk = x0 * cOMEk - y0 * cosik * sOMEk;
        const double ygk = x0 * sOMEk + y0 * cosik * cOMEk;
        const double zgk = y0 * sinik;

        constexpr double I_5 = -5.0 * PI / 180.0;
        const double I_1 = refFrame.omega * tk;
        const double cosI_1 = cos(I_1), sinI_1 = sin(I_1);
        const double cosI_5 = cos(I_5), sinI_5 = sin(I_5);

        sv.p[0] = cosI_1 * xgk + sinI_1 * cosI_5 * ygk + sinI_1 * sinI_5 * zgk;
        sv.p[1] = -sinI_1 * xgk + cosI_1 * cosI_5 * ygk + cosI_1 * sinI_5 * zgk;
        sv.p[2] = -sinI_5 * ygk + cosI_5 * zgk;
    } else {
        OMEk = omega0 - refFrame.omega * toe + (omegaDot - refFrame.omega) * tk; //计算升交点经度
        sOMEk = sin(OMEk), cOMEk = cos(OMEk);
        sv.p[0] = x0 * cOMEk - y0 * cos(ik) * sOMEk;
        sv.p[1] = x0 * sOMEk + y0 * cos(ik) * cOMEk;
        sv.p[2] = y0 * sin(ik);
    }

    const double Edot = nA / (1 - e * cos(Ek)); //偏近点角速率
    const double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek)); //升交角距速率
    const double rdot = A * e * sin(Ek) * Edot + 2 * (crs * c2 - crc * s2) * phidot; //轨道半径速率
    const double ukdot = phidot + 2 * phidot * (cus * c2 - cuc * s2);
    const double OMEkdot = omegaDot - refFrame.omega; //升交点速率
    const double ikdot = 2 * phidot * (cis * c2 - cic * s2) + idot;
    const double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
    const double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk); //轨道平面内速度分量
    const double cosik = cos(ik), sinik = sin(ik);
    const double xkdot = x0dot * cOMEk - x0 * sOMEk * OMEkdot
                         - (y0dot * cosik * sOMEk + y0 * (-sinik * ikdot) * sOMEk + y0 * cosik * cOMEk * OMEkdot);
    const double ykdot = x0dot * sOMEk + x0 * cOMEk * OMEkdot
                         + (y0dot * cosik * cOMEk + y0 * (-sinik * ikdot) * cOMEk - y0 * cosik * sOMEk * OMEkdot);
    const double zkdot = y0dot * sinik + y0 * cosik * ikdot;
    sv.v[0] = xkdot;
    sv.v[1] = ykdot;
    sv.v[2] = zkdot;
    double dt = tk + toe - toc;
    if (dt > HALF_WEEK) dt -= FULL_WEEK;
    if (dt < -HALF_WEEK) dt += FULL_WEEK;
    sv.clockBias = a0 + a1 * dt + a2 * dt * dt;
    sv.clockDrift = a1 + 2 * a2 * dt;
    sv.relativityCorrection = -2.0 * sqrt(refFrame.gm) / (C_MPS * C_MPS) * e * rootA * sin(Ek);
    fixTGD(sv);
    return sv;
}
