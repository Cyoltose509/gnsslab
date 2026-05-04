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
    double n0 = std::sqrt(ell.gm / (A * A * A));

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
    return URA;
}

PVT NavEphGPS::svPVT(const CommonTime &t) const {
    PVT sv;
    auto ell = Frame::GPS;

    ///Semi-major axis
    const double A = sqrt_A * sqrt_A;

    ///Computed mean motion (rad/sec)
    const double n0 = std::sqrt(ell.gm / (A * A * A));

    ///Time from ephemeris reference epoch
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;

    ///Corrected mean motion
    const double n = n0 + Delta_n;

    ///Mean anomaly
    double Mk = M0 + n * tk;

    ///Kepler's Equation for Eccentric Anomaly
    ///solved by iteration
    constexpr double twoPI = 2.0e0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * ::sin(Mk);
    int loop_cnt = 1;
    double delea;
    do {
        const double F = Mk - (Ek - ecc * ::sin(Ek));
        const double G = 1.0 - ecc * ::cos(Ek);
        delea = F / G;
        Ek = Ek + delea;
        loop_cnt++;
    } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

    ///compute clock corrections
    sv.relcorr = svRelativity(t);
    sv.clkbias = svClockBias(t);
    sv.clkdrift = svClockDrift(t);

    ///True Anomaly
    const double q = std::sqrt(1.0 - ecc * ecc);
    const double sinEk = ::sin(Ek);
    const double cosEk = ::cos(Ek);

    const double GSTA = q * sinEk;
    const double GCTA = cosEk - ecc;
    const double vk = atan2(GSTA, GCTA);

    ///Eccentric Anomaly
    //Ek = std::acos((ecc+ ::cos(vk))/(1+ecc* ::cos(vk)));

    ///Argument of Latitude
    const double phi_k = vk + omega;
    const double cos2phi_k = ::cos(2.0 * phi_k);
    const double sin2phi_k = ::sin(2.0 * phi_k);
    const double duk = cos2phi_k * Cuc + sin2phi_k * Cus;
    const double drk = cos2phi_k * Crc + sin2phi_k * Crs;
    const double dik = cos2phi_k * Cic + sin2phi_k * Cis;

    const double uk = phi_k + duk;
    const double rk = A * (1.0 - ecc * cosEk) + drk;
    const double ik = i0 + dik + IDOT * tk;

    ///Positions in orbital plane.
    const double xip = rk * ::cos(uk);
    const double yip = rk * ::sin(uk);

    ///Corrected longitude of ascending node.
    const double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.omega) * tk - ell.omega * Toe;

    ///Earth-fixed coordinates.
    const double sinOMG_k = ::sin(OMEGA_k);
    const double cosOMG_k = ::cos(OMEGA_k);
    const double cosik = ::cos(ik);
    const double sinik = ::sin(ik);

    const double xef = xip * cosOMG_k - yip * cosik * sinOMG_k;
    const double yef = xip * sinOMG_k + yip * cosik * cosOMG_k;
    const double zef = yip * sinik;
    sv.p[0] = xef;
    sv.p[1] = yef;
    sv.p[2] = zef;

    /// Compute velocity of rotation coordinates
    const double dek = n * A / rk;

    //=====
    const double dek2 = n / (1.0 - ecc * cosEk);

    cout << "dek:" << dek << endl;
    cout << "dek2:" << dek2 << endl;

    //======

    const double dlk = sqrt_A * q * std::sqrt(ell.gm) / (rk * rk);

    //====
    const double dlk2 = q * dek2 / (1.0 - ecc * cosEk);
    cout << "dlk:" << dlk << endl;
    cout << "dlk2:" << dlk2 << endl;

    //=====

    const double div = IDOT - 2.0e0 * dlk * (Cic * sin2phi_k - Cis * cos2phi_k);
    const double domk = OMEGA_DOT - ell.omega;
    const double duv = dlk * (1.e0 + 2.e0 * (Cus * cos2phi_k - Cuc * sin2phi_k));
    const double drv = A * ecc * dek * sinEk - 2.e0 * dlk * (Crc * sin2phi_k - Crs * cos2phi_k);
    const double dxp = drv * ::cos(uk) - rk * ::sin(uk) * duv;
    const double dyp = drv * ::sin(uk) + rk * ::cos(uk) * duv;

    /// Calculate velocities
    const double vxef = dxp * cosOMG_k - xip * sinOMG_k * domk - dyp * cosik * sinOMG_k
                        + yip * (sinik * sinOMG_k * div - cosik * cosOMG_k * domk);
    const double vyef = dxp * sinOMG_k + xip * cosOMG_k * domk + dyp * cosik * cosOMG_k
                        - yip * (sinik * cosOMG_k * div + cosik * sinOMG_k * domk);
    const double vzef = dyp * sinik + yip * cosik * div;

    sv.v[0] = vxef;
    sv.v[1] = vyef;
    sv.v[2] = vzef;

    return sv;
}

bool NavEphGPS::isValid(const CommonTime &ct) const {
    if (ct < beginValid || ct > endValid) return false;
    return true;
}



