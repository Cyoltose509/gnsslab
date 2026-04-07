#include <string>
#include "NavEphGPS.h"

using namespace std;

void NavEphGPS::printData() const {
    cout << "****************************************************************"
            << "************" << endl
            << "GPS Broadcast Ephemeris Data: " << endl;
    cout << "Toc: " << this->CivilToc.year << " " << this->CivilToc.month << " "
            << this->CivilToc.day << " " << this->CivilToc.hour << " "
            << this->CivilToc.minute << " " << this->CivilToc.second << endl;
    cout << scientific << setprecision(8)
            << "af0: " << setw(16) << af0 << endl
            << "af1: " << setw(16) << af1 << endl
            << "af2: " << setw(16) << af2 << endl;

    cout << "IODE: " << setw(16) << IODE << endl
            << "Crs:  " << setw(16) << Crs << endl
            << "Delta_n: " << setw(16) << Delta_n << endl
            << "M0: " << setw(16) << M0 << endl;

    cout << "Cuc: " << setw(16) << Cuc << endl
            << "ecc: " << setw(16) << ecc << endl
            << "Cus: " << setw(16) << Cus << endl
            << "sqrt_A: " << setw(16) << sqrt_A << endl;

    cout << "Toe: " << setw(16) << Toe << endl;
    cout << "Cic: " << setw(16) << Cic << endl;
    cout << "OMEGA_0: " << setw(16) << OMEGA_0 << endl;
    cout << "Cis: " << setw(16) << Cis << endl;

    cout << "i0: " << setw(16) << i0 << endl;
    cout << "Crc: " << setw(16) << Crc << endl;
    cout << "omega: " << setw(16) << omega << endl;
    cout << "OMEGA_DOT: " << setw(16) << OMEGA_DOT << endl;

    cout << "IDOT: " << setw(16) << IDOT << endl;
    cout << "Codes_On_L2_Channel: " << setw(16) << L2Codes << endl;
    cout << "GPSWeek: " << setw(16) << GPSWeek << endl;
    cout << "L2P_data_flag: " << setw(16) << L2Pflag << endl;

    cout << "URA: " << setw(16) << URA << endl;
    cout << "SV_health: " << setw(16) << SV_health << endl;
    cout << "TGD: " << setw(16) << TGD << endl;
    cout << "IODC: " << setw(16) << IODC << endl;

    cout << "HOWtime: " << setw(16) << HOWtime << endl;
    cout << "fitInterval: " << setw(16) << fitInterval << endl;

    cout << "ctToc: " << ctToc.toString() << endl;
    cout << "ctToe: " << ctToe.toString() << endl;
}


double NavEphGPS::svClockBias(const CommonTime &t) const {
    double dtc, elaptc;
    elaptc = t - ctToc;
    cout << "elaptc:" << elaptc << endl;
    dtc = af0 + elaptc * (af1 + elaptc * af2);
    cout << "af0:" << af0 << "af1:" << af1 << "af2:" << af2 << endl;
    cout << "dtc:" << dtc << endl;
    return dtc;
}

double NavEphGPS::svClockDrift(const CommonTime &t) const {
    double drift, elaptc;
    elaptc = t - ctToc;
    drift = af1 + elaptc * af2;
    return drift;
}

// Compute satellite relativity correction (sec) at the given time
// throw Invalid Request if the required data has not been stored.
double NavEphGPS::svRelativity(const CommonTime &t) const {
    auto ell = Frame::GPS;
    ///Semi-major axis
    double A = sqrt_A * sqrt_A;

    ///Computed mean motion (rad/sec)
    double n0 = std::sqrt(ell.GM / (A * A * A));

    ///Time from ephemeris reference epoch
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;

    ///Corrected mean motion
    double n = n0 + Delta_n;

    ///Mean anomaly
    double Mk = M0 + n * tk;

    ///Kepler's Equation for Eccentric Anomaly
    ///solved by iteration
    double twoPI = 2.0e0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * ::sin(Mk);
    int loop_cnt = 1;
    double F, G, delea;
    do {
        F = Mk - (Ek - ecc * ::sin(Ek));
        G = 1.0 - ecc * ::cos(Ek);
        delea = F / G;
        Ek = Ek + delea;
        loop_cnt++;
    } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

    return (REL_CONST * ecc * std::sqrt(A) * ::sin(Ek));
}

double NavEphGPS::svURA(const CommonTime &t) const {
    double ephURA = URA;
    return ephURA;
}

PVT NavEphGPS::svPVT(const CommonTime &t) const {
    PVT sv;
    auto ell = Frame::GPS;

    ///Semi-major axis
    double A = sqrt_A * sqrt_A;

    ///Computed mean motion (rad/sec)
    double n0 = std::sqrt(ell.GM / (A * A * A));

    ///Time from ephemeris reference epoch
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;

    ///Corrected mean motion
    double n = n0 + Delta_n;

    ///Mean anomaly
    double Mk = M0 + n * tk;

    ///Kepler's Equation for Eccentric Anomaly
    ///solved by iteration
    double twoPI = 2.0e0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * ::sin(Mk);
    int loop_cnt = 1;
    double F, G, delea;
    do {
        F = Mk - (Ek - ecc * ::sin(Ek));
        G = 1.0 - ecc * ::cos(Ek);
        delea = F / G;
        Ek = Ek + delea;
        loop_cnt++;
    } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

    ///compute clock corrections
    sv.relcorr = svRelativity(t);
    sv.clkbias = svClockBias(t);
    sv.clkdrift = svClockDrift(t);

    ///True Anomaly
    double q = std::sqrt(1.0 - ecc * ecc);
    double sinEk = ::sin(Ek);
    double cosEk = ::cos(Ek);

    double GSTA = q * sinEk;
    double GCTA = cosEk - ecc;
    double vk = atan2(GSTA, GCTA);

    ///Eccentric Anomaly
    //Ek = std::acos((ecc+ ::cos(vk))/(1+ecc* ::cos(vk)));

    ///Argument of Latitude
    double phi_k = vk + omega;
    double cos2phi_k = ::cos(2.0 * phi_k);
    double sin2phi_k = ::sin(2.0 * phi_k);

    double duk = cos2phi_k * Cuc + sin2phi_k * Cus;
    double drk = cos2phi_k * Crc + sin2phi_k * Crs;
    double dik = cos2phi_k * Cic + sin2phi_k * Cis;

    double uk = phi_k + duk;
    double rk = A * (1.0 - ecc * cosEk) + drk;
    double ik = i0 + dik + IDOT * tk;

    ///Positions in orbital plane.
    double xip = rk * ::cos(uk);
    double yip = rk * ::sin(uk);

    ///Corrected longitude of ascending node.
    double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.Omega) * tk - ell.Omega * Toe;

    ///Earth-fixed coordinates.
    double sinOMG_k = ::sin(OMEGA_k);
    double cosOMG_k = ::cos(OMEGA_k);
    double cosik = ::cos(ik);
    double sinik = ::sin(ik);

    double xef = xip * cosOMG_k - yip * cosik * sinOMG_k;
    double yef = xip * sinOMG_k + yip * cosik * cosOMG_k;
    double zef = yip * sinik;
    sv.p[0] = xef;
    sv.p[1] = yef;
    sv.p[2] = zef;

    /// Compute velocity of rotation coordinates
    double dek, dlk, div, domk, duv, drv, dxp, dyp;
    dek = n * A / rk;

    //=====
    double dek2 = n / (1.0 - ecc * cosEk);

    cout << "dek:" << dek << endl;
    cout << "dek2:" << dek2 << endl;

    //======

    dlk = sqrt_A * q * std::sqrt(ell.GM) / (rk * rk);

    //====
    double dlk2;
    dlk2 = q * dek2 / (1.0 - ecc * cosEk);
    cout << "dlk:" << dlk << endl;
    cout << "dlk2:" << dlk2 << endl;

    //=====

    div = IDOT - 2.0e0 * dlk * (Cic * sin2phi_k - Cis * cos2phi_k);
    domk = OMEGA_DOT - ell.Omega;
    duv = dlk * (1.e0 + 2.e0 * (Cus * cos2phi_k - Cuc * sin2phi_k));
    drv = A * ecc * dek * sinEk - 2.e0 * dlk * (Crc * sin2phi_k - Crs * cos2phi_k);
    dxp = drv * ::cos(uk) - rk * ::sin(uk) * duv;
    dyp = drv * ::sin(uk) + rk * ::cos(uk) * duv;

    /// Calculate velocities
    double vxef = dxp * cosOMG_k - xip * sinOMG_k * domk - dyp * cosik * sinOMG_k
                  + yip * (sinik * sinOMG_k * div - cosik * cosOMG_k * domk);
    double vyef = dxp * sinOMG_k + xip * cosOMG_k * domk + dyp * cosik * cosOMG_k
                  - yip * (sinik * cosOMG_k * div + cosik * sinOMG_k * domk);
    double vzef = dyp * sinik + yip * cosik * div;

    sv.v[0] = vxef;
    sv.v[1] = vyef;
    sv.v[2] = vzef;

    return sv;
}

bool NavEphGPS::isValid(const CommonTime &ct) const {
    if (ct < beginValid || ct > endValid) return false;
    return true;
}

inline double c_Ek(const double E0, const double e) {
    double E = E0;
    double E_1;
    do {
        E_1 = E;
        E = E0 + e * sin(E_1);
    } while (E_1 != E);
    return E;
}

PVT Ephemeris::svPVT(const WeekSecond &ws) const {
    PVT sv;
    double tk = ws-getWeekSecond();
    double A = RootA * RootA; //计算轨道长半径
    double n0 = sqrt(refFrame.GM / pow(A, 3)); //平均角速度0.000145859rad/s
    double nA = n0 + dn; //改正平均角速度
    double Mk = M0 + nA * tk;
    double Ek = c_Ek(Mk, e);
    double vk = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(Ek / 2)); //计算真近点角
    double phi = vk + omega; //卫星与升交点间地心夹角
    double s2 = sin(2 * phi), c2 = cos(2 * phi);
    double Qu = cus * s2 + cuc * c2, Qr = crs * s2 + crc * c2, Qi = cis * s2 + cic * c2; //短周期摄动项
    double uk = phi + Qu, rk = A * (1 - e * cos(Ek)) + Qr; //短周期摄动改正，ik的值不确定
    double ik = i0 + Qi + idot * tk;
    double x0 = rk * cos(uk), y0 = rk * sin(uk);
    double OMEk = Omega0 - refFrame.Omega * toe + (Omegadot - refFrame.Omega) * tk; //计算升交点经度
    double xk = x0 * cos(OMEk) - y0 * cos(ik) * sin(OMEk);
    double yk = x0 * sin(OMEk) + y0 * cos(ik) * cos(OMEk);
    double zk = y0 * sin(ik);
    sv.p[0] = xk;
    sv.p[1] = yk;
    sv.p[2] = zk;
    double Edot = nA / (1 - e * cos(Ek)); //偏近点角速率
    double phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(Ek)); //升交角距速率
    double rdot = A * e * sin(Ek) * Edot + 2 * (crs * c2 - crc * s2) * phidot; //轨道半径速率
    double ukdot = phidot + 2 * phidot * (cus * c2 - cuc * s2);
    double OMEkdot = Omegadot - refFrame.Omega; //升交点速率
    double ikdot = 2 * phidot * (cis * c2 - cic * s2) + idot;
    double x0dot = rdot * cos(uk) - rk * ukdot * sin(uk);
    double y0dot = rdot * sin(uk) + rk * ukdot * cos(uk); //轨道平面内速度分量
    double cosik = cos(ik), sinik = sin(ik);
    double xkdot = x0dot * cos(OMEk) - x0 * sin(OMEk) * OMEkdot
                   - (y0dot * cosik * sin(OMEk) + y0 * (-sinik * ikdot) * sin(OMEk) + y0 * cosik * cos(OMEk) * OMEkdot);
    double ykdot = x0dot * sin(OMEk) + x0 * cos(OMEk) * OMEkdot
                   + (y0dot * cosik * cos(OMEk) + y0 * (-sinik * ikdot) * cos(OMEk) - y0 * cosik * sin(OMEk) * OMEkdot);
    double zkdot = y0dot * sinik + y0 * cosik * ikdot;
    sv.v[0] = xkdot;
    sv.v[1] = ykdot;
    sv.v[2] = zkdot;
    double dt = tk + toe - toc;
    if (dt > 302400) dt -= 604800;
    if (dt < -302400) dt += 604800;
    sv.clkbias = a0 + a1 * dt + a2 * dt * dt;
    sv.clkdrift = a1 + 2 * a2 * dt;
    sv.relcorr = -2.0 * sqrt(refFrame.GM) / (C_MPS * C_MPS) * e * sqrt(A) * sin(Ek);
    return sv;
}

