#pragma once

#include <string>
#include <iostream>
#include <string_view>


using namespace std;

//  PI
constexpr double PI = 3.141592653589793238462643383280;
// m/s, speed of light; this value defined by GPS but applies to GAL and GLO.
constexpr double C_MPS = 2.99792458e8;
// Conversion Factor from degrees to radians (unit: degrees^-1)
static constexpr double DEG_TO_RAD = 1.745329251994329576923691e-2;
// Conversion Factor from radians to degrees (unit: degrees)
static constexpr double RAD_TO_DEG = 57.29577951308232087679815;
/// relativity constant (sec/sqrt(m))
constexpr double REL_CONST = -4.442807633e-10;
/// relativity constant for BDS (sec/sqrt(m))
constexpr double REL_CONST_BDS = -4.442807309e-10;


/// Add this offset to convert Modified Julian Date to Julian Date.
constexpr double MJD_TO_JD = 2400000.5;
constexpr double MJD_TO_JD2020 = -58849.5;
/// 'Julian day' offset from MJD
constexpr long MJD_JDAY = 2400001L;
/// Modified Julian Date of UNIX epoch (Jan. 1, 1970).
constexpr long UNIX_MJD = 40587L;

/// Seconds per half week.
constexpr long HALF_WEEK = 302400L;
/// Seconds per whole week.
constexpr long FULL_WEEK = 604800L;

/// Seconds per day.
constexpr long SEC_PER_DAY = 86400L;
/// Days per second.
constexpr double DAY_PER_SEC = 1.0 / SEC_PER_DAY;


/// Milliseconds in a second.
constexpr long MS_PER_SEC = 1000L;
/// Seconds per millisecond.
constexpr double SEC_PER_MS = 1.0 / MS_PER_SEC;

/// Milliseconds in a day.
constexpr long MS_PER_DAY = MS_PER_SEC * SEC_PER_DAY;
/// Days per milliseconds.
constexpr double DAY_PER_MS = 1.0 / MS_PER_DAY;

// Nominal mean angular velocity of the Earth (rad/s)
constexpr double OMEGA_EARTH = 7.292115e-5;

constexpr double RADIUS_EARTH = 6378137.0;
// system-specific constants

// GPS -------------------------------------------
/// 'Julian day' of GPS epoch (Jan. 6, 1980).
constexpr double GPS_EPOCH_JD = 2444244.5;
/// Modified Julian Date of GPS epoch (Jan. 6, 1980).
constexpr long GPS_EPOCH_MJD = 44244L;
/// Modified Julian Date of BDT epoch (Jan. 6, 1980).
constexpr long BDT_EPOCH_MJD = 53736;
/// Weeks per GPS Epoch
constexpr long GPS_WEEK_PER_EPOCH = 1024L;

/// Zcounts in a  day.
constexpr long ZCOUNT_PER_DAY = 57600L;
/// Days in a Zcount
constexpr double DAY_PER_ZCOUNT = 1.0 / ZCOUNT_PER_DAY;
/// Zcounts in a week.
constexpr long ZCOUNT_PER_WEEK = 403200L;
/// Weeks in a Zcount.
constexpr double WEEK_PER_ZCOUNT = 1.0 / ZCOUNT_PER_WEEK;

// BDS -------------------------------------------
/// 'Julian day' of BDS epoch (Jan. 1, 2006).
constexpr double BDS_EPOCH_JD = 2453736.5;
/// Modified Julian Date of BDS epoch (Jan. 1, 2006).
constexpr long BDS_EPOCH_MJD = 53736L;
/// Weeks per BDS Epoch
constexpr long BDS_WEEK_PER_EPOCH = 8192L;


// GPS L1 carrier frequency in Hz
constexpr double L1_FREQ_GPS = 1575.42e6;
// GPS L2 carrier frequency in Hz
constexpr double L2_FREQ_GPS = 1227.60e6;
// GPS L5 carrier frequency in Hz
constexpr double L5_FREQ_GPS = 1176.45e6;

// GPS L1 carrier wavelength in meters
constexpr double L1_WAVELENGTH_GPS = 0.190293672798;
// GPS L2 carrier wavelength in meters
constexpr double L2_WAVELENGTH_GPS = 0.244210213425;
// GPS L5 carrier wavelength in meters
constexpr double L5_WAVELENGTH_GPS = 0.254828048791;

constexpr double L5_FREQ_BDS = 1176.450e6; // B2a (BDS-3)
constexpr double L8_FREQ_BDS = 1191.795e6; // B2=B21+B2b/2
constexpr double L7_FREQ_BDS = 1207.140e6; // B2b (BDS-3/BDS-2)
constexpr double L6_FREQ_BDS = 1268.520e6; // B3  (BDS-3/BDS-2)
constexpr double L2_FREQ_BDS = 1561.098e6; // B1I (BDS-3/BDS-2)
constexpr double L1_FREQ_BDS = 1575.420e6; // B1C (BDS-3)

constexpr double L1_WAVELENGTH_BDS = C_MPS / L1_FREQ_BDS;
constexpr double L2_WAVELENGTH_BDS = C_MPS / L2_FREQ_BDS;
constexpr double L6_WAVELENGTH_BDS = C_MPS / L6_FREQ_BDS;
constexpr double L7_WAVELENGTH_BDS = C_MPS / L7_FREQ_BDS;
constexpr double L8_WAVELENGTH_BDS = C_MPS / L8_FREQ_BDS;
constexpr double L5_WAVELENGTH_BDS = C_MPS / L5_FREQ_BDS;

constexpr double getWavelength(const char sys, const int &n) {
    if (n == 0) {
        std::cerr << "getWavelength():frequency no must be positive integer!" << endl;
        exit(-1);
    }

    if (sys == 'G') {
        if (n == 1) return L1_WAVELENGTH_GPS;
        if (n == 2) return L2_WAVELENGTH_GPS;
        if (n == 5) return L5_WAVELENGTH_GPS;
    } else if (sys == 'C') {
        if (n == 1) return L1_WAVELENGTH_BDS;
        if (n == 2) return L2_WAVELENGTH_BDS;
        if (n == 5) return L5_WAVELENGTH_BDS;
        if (n == 7) return L7_WAVELENGTH_BDS;
        if (n == 8) return L8_WAVELENGTH_BDS;
        if (n == 6) return L6_WAVELENGTH_BDS;
    } else {
        std::cerr << "don't support system except GPS and Beidou" << endl;
    }

    return 0.0;
}

constexpr double getFreq(const char sys, const int &n) {
    if (sys == 'G') {
        if (n == 1) return L1_FREQ_GPS;
        if (n == 2) return L2_FREQ_GPS;
        if (n == 5) return L5_FREQ_GPS;
    } else if (sys == 'C') {
        if (n == 1) return L1_FREQ_BDS;
        if (n == 2) return L2_FREQ_BDS;
        if (n == 5) return L5_FREQ_BDS;
        if (n == 7) return L7_FREQ_BDS;
        if (n == 8) return L8_FREQ_BDS;
        if (n == 6) return L6_FREQ_BDS;
    } else {
        std::cerr << "don't support system except GPS and Beidou" << endl;
    }
    return 0.0;
}

constexpr double getFreq(const char sys, const std::string_view type) noexcept {
    if (type.size() < 2) return 0.0;

    const char band = type[1];

    switch (sys) {
        case 'G':
            switch (band) {
                case '1': return L1_FREQ_GPS;
                case '2': return L2_FREQ_GPS;
                case '5': return L5_FREQ_GPS;
                default: ;
            }
            break;

        case 'C':
            switch (band) {
                case '1': return L1_FREQ_BDS;
                case '2': return L2_FREQ_BDS;
                case '5': return L5_FREQ_BDS;
                case '6': return L6_FREQ_BDS;
                case '7': return L7_FREQ_BDS;
                case '8': return L8_FREQ_BDS;
                default: ;
            }
            break;
        default: ;
    }

    return 0.0;
}

inline double getGamma(const char sys, const std::string_view type1, const std::string_view type2) {
    const double f1 = getFreq(sys, type1);
    const double f2 = getFreq(sys, type2);
    return f1 * f1 / (f2 * f2);
}

struct DualCode {
    const char *code1;
    const char *code2;
    bool valid;
};


constexpr DualCode getDualCode(const char sys) {
    switch (sys) {
        case 'G': return {"C1", "C2", true};
        case 'C': return {"C2", "C6", true};
        default: return {"", "", false};
    }
}

enum class SatType {
    MEO,
    GEO,
    IGSO,
};
constexpr SatType getSatType(const char sys, const int prn, const bool old = false) {
    if (old) {
        switch (sys) {
            case 'G': return SatType::MEO;
            case 'C': {
                if (prn >= 1 && prn <= 4) return SatType::GEO;
                if (prn >= 59 && prn <= 62) return SatType::GEO;
                if ((prn >= 38 && prn <= 40) || (prn >= 13 && prn <= 16))return SatType::IGSO;
                return SatType::MEO;
            }
            default: return SatType::MEO;
        }
    }
    switch (sys) {
        case 'G': return SatType::MEO;
        case 'C': {
            if (prn >= 1 && prn <= 4) return SatType::GEO;
            if (prn >= 6 && prn <= 10) return SatType::IGSO;
            return SatType::MEO;
        }
        default: return SatType::MEO;
    }
}
