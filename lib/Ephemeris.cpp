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
    PVT sv{};

    const double tk = t - getCommonTime();

    // 轨道基本参数
    const double A = rootA * rootA;
    const double n0 = sqrt(refFrame->gm / (A*A*A));
    const double nA = n0 + dn;
    const double Mk = m0 + nA*tk;
    const double Ek = c_Ek(Mk, e);
    const double vk = 2 * atan(sqrt((1+e)/(1-e)) * tan(Ek/2));
    const double phi = vk + omega;

    // 短周期摄动
    const double s2 = sin(2*phi), c2 = cos(2*phi);
    const double Qu = cus*s2 + cuc*c2;
    const double Qr = crs*s2 + crc*c2;
    const double Qi = cis*s2 + cic*c2;

    const double uk = phi + Qu;
    const double rk = A * (1 - e*cos(Ek)) + Qr;
    const double ik = i0 + Qi + idot*tk;

    const double x0 = rk * cos(uk);
    const double y0 = rk * sin(uk);

    bool isGEO = (type == 'C' && prn >= 1 && prn <= 5);

    if (isGEO) {
        // --- GEO卫星，直接用同步轨道公式 ---
        constexpr double r_geo = 42164e3; // 地心距
        double lon = lonPRN[prn];
        double i_geo = 0.0;
        const double coslon = cos(lon), sinlon = sin(lon);
        const double cosi = cos(i_geo), sini = sin(i_geo);

        sv.p[0] = r_geo * coslon;
        sv.p[1] = r_geo * sinlon;
        sv.p[2] = r_geo * sini;
    } else {
        // --- MEO/IGSO卫星 ---
        double OMEk = omega0 + (omegaDot - refFrame->omega)*tk - refFrame->omega*toe;
        double cosOMEk = cos(OMEk), sinOMEk = sin(OMEk);
        double cosik = cos(ik), sinik = sin(ik);

        sv.p[0] = x0 * cosOMEk - y0 * cosik * sinOMEk;
        sv.p[1] = x0 * sinOMEk + y0 * cosik * cosOMEk;
        sv.p[2] = y0 * sinik;
    }

    // --- 保留你原来的速度公式 ---
    double cosik = cos(ik), sinik = sin(ik);
    double OMEk = omega0 + (omegaDot - refFrame->omega)*tk - refFrame->omega*toe;
    double sOMEk = sin(OMEk), cOMEk = cos(OMEk);
    const double Edot = nA / (1 - e*cos(Ek));
    const double phidot = sqrt(1 - e*e) * Edot / (1 - e*cos(Ek));
    const double rdot = A * e * sin(Ek) * Edot + 2 * (crs*c2 - crc*s2) * phidot;
    const double ukdot = phidot + 2 * phidot * (cus*c2 - cuc*s2);
    const double OMEkdot = omegaDot - refFrame->omega;
    const double ikdot = 2 * phidot * (cis*c2 - cic*s2) + idot;
    const double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
    const double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk);

    sv.v[0] = x0dot * cOMEk - x0 * sOMEk * OMEkdot
             - (y0dot * cosik * sOMEk + y0 * (-sinik * ikdot) * sOMEk + y0 * cosik * cOMEk * OMEkdot);
    sv.v[1] = x0dot * sOMEk + x0 * cOMEk * OMEkdot
             + (y0dot * cosik * cOMEk + y0 * (-sinik * ikdot) * cOMEk - y0 * cosik * sOMEk * OMEkdot);
    sv.v[2] = y0dot * sinik + y0 * cosik * ikdot;

    // --- 卫星时钟 ---
    double dt = tk + toe - toc;
    if (dt > HALF_WEEK) dt -= FULL_WEEK;
    if (dt < -HALF_WEEK) dt += FULL_WEEK;
    sv.clockBias = a0 + a1*dt + a2*dt*dt;
    sv.clockDrift = a1 + 2*a2*dt;

    // --- 相对论修正 ---
    sv.relativityCorrection = -2.0 * sqrt(refFrame->gm) / (C_MPS*C_MPS) * e * rootA * sin(Ek);

    return sv;
}