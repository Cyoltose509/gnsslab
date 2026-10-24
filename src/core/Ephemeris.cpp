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


PVT Ephemeris::getPVT(CommonTime t) {
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

    double OMEk, sOMEk, cOMEk;
    if (getSatType(type, prn) == SatType::GEO) {
        OMEk = omega0 + omegaDot * tk - refFrame.omega * toe;
        sOMEk = sin(OMEk), cOMEk = cos(OMEk);
        const double cosik = cos(ik), sinik = sin(ik);
        const double xgk = x0 * cOMEk - y0 * cosik * sOMEk;
        const double ygk = x0 * sOMEk + y0 * cosik * cOMEk;
        const double zgk = y0 * sinik;

        constexpr double I_5 = -5.0 * DEG_TO_RAD;
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

PVT GlonassEphem::getPVT(CommonTime t) {
    convertTimeSystem(t, timeSystem); // 观测时刻对齐到星历时间系统（GPS→UTC，减跳秒）
    PVT sv;
    const double dt = t - toeCommon; // 相对参考历元的物理秒数

    constexpr double MU = Frame::PZ90.gm * 1e-9;
    constexpr double OMGE = Frame::PZ90.omega;
    constexpr double J2_GLO = Frame::PZ90.j2;
    constexpr double RE_GLO = Frame::PZ90.a * 1e-3;
    double x[6] = {X, Y, Z, vX, vY, vZ};
    const double acc[3] = {aX, aY, aZ};

    auto deq = [&](const double s[6], double f[6]) {
        const double r2 = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
        const double r = std::sqrt(r2);
        const double aJ = 1.5 * J2_GLO * MU * RE_GLO * RE_GLO / (r2 * r2 * r); // 1.5·J2·μ·Re²/r⁵
        const double b = 5.0 * s[2] * s[2] / r2;
        const double c = -MU / (r2 * r) - aJ * (1.0 - b);
        const double omg2 = OMGE * OMGE;
        f[0] = s[3];
        f[1] = s[4];
        f[2] = s[5];
        f[3] = (c + omg2) * s[0] + 2.0 * OMGE * s[4] + acc[0];
        f[4] = (c + omg2) * s[1] - 2.0 * OMGE * s[3] + acc[1];
        f[5] = (c - 2.0 * aJ) * s[2] + acc[2];
    };

    double tt = dt;
    while (std::fabs(tt) > 1e-9) {
        const double h = (tt > 0.0 ? 1.0 : -1.0);
        const double hh = (std::fabs(tt) < 1.0) ? tt : h;
        double k1[6], k2[6], k3[6], k4[6], tmp[6];
        deq(x, k1);
        for (int i = 0; i < 6; ++i) tmp[i] = x[i] + 0.5 * hh * k1[i];
        deq(tmp, k2);
        for (int i = 0; i < 6; ++i) tmp[i] = x[i] + 0.5 * hh * k2[i];
        deq(tmp, k3);
        for (int i = 0; i < 6; ++i) tmp[i] = x[i] + hh * k3[i];
        deq(tmp, k4);
        for (int i = 0; i < 6; ++i)
            x[i] += hh / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        tt -= hh;
    }

    sv.p = Eigen::Vector3d(x[0], x[1], x[2]) * 1000.0; // km → m
    sv.v = Eigen::Vector3d(x[3], x[4], x[5]) * 1000.0; // km/s → m/s

    sv.clockBias = -tauN + tauD * dt;
    sv.clockDrift = tauD;
    sv.relativityCorrection = 0.0;
    return sv;
}
