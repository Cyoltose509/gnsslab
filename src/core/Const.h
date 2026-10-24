#pragma once

#include <string>
#include <iostream>
#include <string_view>


using namespace std;
/// Baarda w 检验阈值 (α=0.001)
static constexpr double W_THRESHOLD = 3.29;
///  PI
constexpr double PI = 3.141592653589793238462643383280;
/// m/s, speed of light; this value defined by GPS but applies to GAL and GLO.
constexpr double C_MPS = 2.99792458e8;
/// Conversion Factor from degrees to radians (unit: degrees^-1)
static constexpr double DEG_TO_RAD = 1.745329251994329576923691e-2;
/// Conversion Factor from radians to degrees (unit: degrees)
static constexpr double RAD_TO_DEG = 57.29577951308232087679815;
/// relativity constant (sec/sqrt(m))
constexpr double REL_CONST = -4.442807633e-10;
/// relativity constant for BDS (sec/sqrt(m))
constexpr double REL_CONST_BDS = -4.442807309e-10;


/// Add this offset to convert Modified Julian Date to Julian Date.
constexpr double MJD_TO_JD = 2400000.5;
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

/// Seconds per hour.
constexpr long SEC_PER_HOUR = 3600L;
/// Seconds per minute.
constexpr long SEC_PER_MIN = 60L;


/// Milliseconds in a second.
constexpr long MS_PER_SEC = 1000L;
/// Seconds per millisecond.
constexpr double SEC_PER_MS = 1.0 / MS_PER_SEC;

/// Milliseconds in a day.
constexpr long MS_PER_DAY = MS_PER_SEC * SEC_PER_DAY;
/// Days per milliseconds.
constexpr double DAY_PER_MS = 1.0 / MS_PER_DAY;

/// Nominal mean angular velocity of the Earth (rad/s)
constexpr double OMEGA_EARTH = 7.292115e-5;

constexpr double RADIUS_EARTH = 6378137.0;
// 天体/地球物理常数
constexpr double EARTH_GM          = 3.986004415e14; // 地球引力常数 GM (m^3 s^-2)
constexpr double SUN_GM            = 1.327124e20;    // 太阳引力常数 GM (m^3 s^-2)
constexpr double MOON_GM           = 4.902801e12;    // 月球引力常数 GM (m^3 s^-2)
constexpr double ASTRONOMICAL_UNIT = 149597870691.0; // 1 AU (m)
constexpr double ARCSEC_TO_RAD     = DEG_TO_RAD / 3600.0; // 角秒 → 弧度
// system-specific constants

// GPS -------------------------------------------
/// 'Julian day' of GPS epoch (Jan. 6, 1980).
constexpr double GPS_EPOCH_JD = 2444244.5;
/// Modified Julian Date of GPS epoch (Jan. 6, 1980).
constexpr long GPS_EPOCH_MJD = 44244L;
/// Modified Julian Date of BDT epoch (Jan. 6, 1980).
constexpr long BDT_EPOCH_MJD = 53736;
/// Modified Julian Date of GAL epoch (Galileo System Time, Aug. 22, 1999).
constexpr long GAL_EPOCH_MJD = 51412L;
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


/// GPS L1 carrier frequency in Hz
constexpr double L1_FREQ_GPS = 1575.42e6;
/// GPS L2 carrier frequency in Hz
constexpr double L2_FREQ_GPS = 1227.60e6;
/// GPS L5 carrier frequency in Hz
constexpr double L5_FREQ_GPS = 1176.45e6;

/// GPS L1 carrier wavelength in meters
constexpr double L1_WAVELENGTH_GPS = 0.190293672798;
/// GPS L2 carrier wavelength in meters
constexpr double L2_WAVELENGTH_GPS = 0.244210213425;
/// GPS L5 carrier wavelength in meters
constexpr double L5_WAVELENGTH_GPS = 0.254828048791;

constexpr double L5_FREQ_BDS = 1176.450e6; /// B2a (BDS-3)
constexpr double L8_FREQ_BDS = 1191.795e6; /// B2=B21+B2b/2
constexpr double L7_FREQ_BDS = 1207.140e6; /// B2b (BDS-3/BDS-2)
constexpr double L6_FREQ_BDS = 1268.520e6; /// B3  (BDS-3/BDS-2)
constexpr double L2_FREQ_BDS = 1561.098e6; /// B1I (BDS-3/BDS-2)
constexpr double L1_FREQ_BDS = 1575.420e6; /// B1C (BDS-3)

constexpr double L1_WAVELENGTH_BDS = C_MPS / L1_FREQ_BDS;
constexpr double L2_WAVELENGTH_BDS = C_MPS / L2_FREQ_BDS;
constexpr double L6_WAVELENGTH_BDS = C_MPS / L6_FREQ_BDS;
constexpr double L7_WAVELENGTH_BDS = C_MPS / L7_FREQ_BDS;
constexpr double L8_WAVELENGTH_BDS = C_MPS / L8_FREQ_BDS;
constexpr double L5_WAVELENGTH_BDS = C_MPS / L5_FREQ_BDS;

// ===== Galileo (CDMA，频率固定，与 GPS/BDS 同构) =====
constexpr double L1_FREQ_GAL = 1575.420e6; /// E1
constexpr double L5_FREQ_GAL = 1176.450e6; /// E5a
constexpr double L7_FREQ_GAL = 1207.140e6; /// E5b
constexpr double L6_FREQ_GAL = 1278.750e6; /// E6
constexpr double L8_FREQ_GAL = 1191.795e6; /// E5 = (E5a+E5b)/2

constexpr double L1_WAVELENGTH_GAL = C_MPS / L1_FREQ_GAL;
constexpr double L5_WAVELENGTH_GAL = C_MPS / L5_FREQ_GAL;
constexpr double L7_WAVELENGTH_GAL = C_MPS / L7_FREQ_GAL;
constexpr double L6_WAVELENGTH_GAL = C_MPS / L6_FREQ_GAL;
constexpr double L8_WAVELENGTH_GAL = C_MPS / L8_FREQ_GAL;

// ===== GLONASS (FDMA，频率随通道号 k 偏移) =====
// G1 = BASE1 + k*DFREQ1, G2 = BASE2 + k*DFREQ2 （k 即 RINEX "GLONASS SLOT / FRQ #" 给出的频率通道号）
constexpr double GLO_BASE_FREQ1 = 1.60200e9;
constexpr double GLO_DFREQ1     = 0.56250e6;
constexpr double GLO_BASE_FREQ2 = 1.24600e9;
constexpr double GLO_DFREQ2     = 0.43750e6;
constexpr double GLO_BASE_FREQ3 = 1.202025e9; /// G3 (CDMA，无 FDMA 步进)
constexpr double GLO_DFREQ3     = 0.0;

/// GLONASS FDMA 频率：band='1'→G1, '2'→G2, '3'→G3；channel=k（频率通道号）
inline double getGlonassFreq(const char band, const int channel) {
    switch (band) {
        case '1': return GLO_BASE_FREQ1 + channel * GLO_DFREQ1;
        case '2': return GLO_BASE_FREQ2 + channel * GLO_DFREQ2;
        case '3': return GLO_BASE_FREQ3 + channel * GLO_DFREQ3;
        default: return 0.0;
    }
}

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
    } else if (sys == 'E') {
        if (n == 1) return L1_WAVELENGTH_GAL;
        if (n == 5) return L5_WAVELENGTH_GAL;
        if (n == 7) return L7_WAVELENGTH_GAL;
        if (n == 6) return L6_WAVELENGTH_GAL;
        if (n == 8) return L8_WAVELENGTH_GAL;
    } else {
        std::cerr << "don't support system except GPS, Beidou and Galileo" << endl;
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
    } else if (sys == 'E') {
        if (n == 1) return L1_FREQ_GAL;
        if (n == 5) return L5_FREQ_GAL;
        if (n == 7) return L7_FREQ_GAL;
        if (n == 6) return L6_FREQ_GAL;
        if (n == 8) return L8_FREQ_GAL;
    } else {
        std::cerr << "don't support system except GPS, Beidou and Galileo" << endl;
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
                // BDS B1 频段同时含 B1C(1575.420) 与 B1I(1561.098)，由第3字符(I/C)区分。
                case '1': return type.size() >= 3 && type[2] == 'I' ? L2_FREQ_BDS : L1_FREQ_BDS;
                case '2': return L2_FREQ_BDS;
                case '5': return L5_FREQ_BDS;
                case '6': return L6_FREQ_BDS;
                case '7': return L7_FREQ_BDS;
                case '8': return L8_FREQ_BDS;
                default: ;
            }
            break;

        case 'E':
            switch (band) {
                case '1': return L1_FREQ_GAL;   // E1
                case '5': return L5_FREQ_GAL;   // E5a
                case '7': return L7_FREQ_GAL;   // E5b
                case '6': return L6_FREQ_GAL;   // E6
                case '8': return L8_FREQ_GAL;   // E5 (E5a+E5b)/2
                default: ;
            }
            break;

        case 'R':
            // GLONASS FDMA
            switch (band) {
                case '1': return getGlonassFreq('1', 0);
                case '2': return getGlonassFreq('2', 0);
                case '3': return getGlonassFreq('3', 0);
                default: ;
            }
            break;
        default: ;
    }

    return 0.0;
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
            if (prn >= 59 && prn <= 62) return SatType::GEO;
            if (prn >= 6 && prn <= 10) return SatType::IGSO;
            return SatType::MEO;
        }
        default: return SatType::MEO;
    }
}

