#include <cmath>
#include <map>
#include "TimeConvert.h"
#include "TimeStruct.h"
#include "Const.h"

#define debug 0
#define debugYDS 0

using namespace std;

// 在给定的历史记录中查找跳秒
double getLeapSeconds(const CommonTime &ct) {
    std::map<double, double> leapData;

    // mjd;
    //
    leapData[41317.0] = 10;
    leapData[41499.0] = 11;
    leapData[41683.0] = 12;
    leapData[42048.0] = 13;
    leapData[42413.0] = 14;
    leapData[42778.0] = 15;
    leapData[43144.0] = 16;
    leapData[43509.0] = 17;
    leapData[43874.0] = 18;
    leapData[44239.0] = 19;
    leapData[44786.0] = 20;
    leapData[45151.0] = 21;
    leapData[45516.0] = 22;
    leapData[46247.0] = 23;
    leapData[47161.0] = 24;
    leapData[47892.0] = 25;
    leapData[48257.0] = 26;
    leapData[48804.0] = 27;
    leapData[49169.0] = 28;
    leapData[49534.0] = 29;
    leapData[50083.0] = 30;
    leapData[50630.0] = 31;
    leapData[51179.0] = 32;
    leapData[53736.0] = 33;
    leapData[54832.0] = 34;
    leapData[56109.0] = 35;
    leapData[57204.0] = 36;
    leapData[57754.0] = 37;

    const double mjd_ct = ct.m_day;

    // 1972.1.1
    if (mjd_ct < 41317.0) {
        throw InvalidRequest("Time MUST be greater than 1972 for leapSec!");
    }

    double leapSec = 0;

    //  循环所有的跳秒记录
    for (const auto [time, leap]: leapData) {
        if (time <= mjd_ct)
            leapSec = leap;
    }

    return leapSec;
}

// 时间系统转换
void convertTimeSystem(CommonTime &in_time, const TimeSystem &targetSys) {
    const TimeSystem inTS = in_time.m_timeSystem;
    double dt = 0;
    in_time.m_timeSystem = targetSys;
    if (inTS == targetSys)
        return;
    if (inTS == TimeSystem::GPS) // GAL -> TAI
        dt = 19;
    else if (inTS == TimeSystem::UTC) // GLO -> TAI
        dt = getLeapSeconds(in_time + dt - getLeapSeconds(in_time));
    else if (inTS == TimeSystem::BDT) // BDT -> TAI
        dt = 33;
    else if (inTS == TimeSystem::TAI) // TAI -> TAI
        dt = 0;
    else {
        throw InvalidRequest("Invalid input TimeSystem " + inTS.toString());
    }
    if (targetSys == TimeSystem::GPS) // TAI -> GAL
        dt -= 19;
    else if (targetSys == TimeSystem::UTC) {
        dt -= getLeapSeconds(in_time + dt - getLeapSeconds(in_time));
    } else if (targetSys == TimeSystem::BDT) // TAI -> BDT
        dt -= 33;
    else if (targetSys == TimeSystem::TAI) // TAI
        dt -= 0;
    else {
        throw InvalidRequest("Invalid output TimeSystem " + targetSys.toString());
    }
    in_time += dt;
}

//
void convertJD2YMD(const double jd, int &iyear, int &imonth, int &iday) {
    const double a = std::floor(jd + 0.5);
    const double b = a + 1537;
    const double c = std::floor((b - 122.1) / 365.25);
    const double d = std::floor(365.25 * c);
    const double e = std::floor((b - d) / 30.6001);
    iday = static_cast<int>(b) - d - std::floor(30.6001 * e) + (jd + 0.5) - std::floor(jd + 0.5); //NOLINT
    imonth = static_cast<int>(e) - 1 - 12. * std::floor(e / 14); //NOLINT
    iyear = static_cast<int>(c) - 4715 - std::floor((7 + imonth) / 10); //NOLINT
}

double convertYMD2JD(int iyear, int imonth, const int iday) {
    if (imonth <= 2) {
        imonth += 12;
        iyear -= 1;
    }

    const double B = 2 - floor(iyear / 100) + floor(iyear / 400);
    const double jd_double = floor(365.25 * (iyear + 4716)) + floor(30.6001 * (imonth + 1)) + B + iday - 1524.5;
    return jd_double;
}

void convertSOD2HMS(double sod, int &hh, int &mm, double &sec) {
    // Get us to within one day.
    if (sod < 0) {
        sod += (1 + static_cast<unsigned long>(sod / SEC_PER_DAY)) * SEC_PER_DAY;
    } else if (sod >= SEC_PER_DAY) {
        sod -= static_cast<unsigned long>(sod / SEC_PER_DAY) * SEC_PER_DAY;
    }

    double temp; // variable to hold the integer part of sod
    sod = modf(sod, &temp); // sod holds the fraction, temp the integer
    const long seconds = static_cast<long>(temp); // get temp into a real integer

    hh = seconds / 3600;
    mm = seconds % 3600 / 60;
    sec = static_cast<double>(seconds % 60) + sod;
}

double convertHMS2SOD(const int hh, const int mm, const double sec) {
    return sec + 60. * (mm + 60. * hh);
}

CommonTime CivilTime2CommonTime(const CivilTime &civil_t) {
    CommonTime ct;
    // get the julian day
    const double jday = convertYMD2JD(civil_t.year, civil_t.month, civil_t.day);

    // convert jday to mjd day.
    int mjd_day = round(jday) - MJD_TO_JD; //NOLINT

    // get the second of day
    const double sod = convertHMS2SOD(civil_t.hour, civil_t.minute, civil_t.second);

    // mjd_day + sod
    ct.set(mjd_day, sod, civil_t.timeSys);

    return ct;
}

CivilTime CommonTime2CivilTime(const CommonTime &ct) {
    CivilTime civilt;
    long mjd_day;

    double sod;
    TimeSystem sys;
    // get the julian day, second of day, and fractional second of day
    ct.get(mjd_day, sod, sys);
    civilt.timeSys = sys;

    const double jday = mjd_day + MJD_TO_JD;

    // convert the julian day to calendar "year/month/day of month"
    convertJD2YMD(jday, civilt.year, civilt.month, civilt.day);

    // convert the (whole) second of day to "hour/minute/second"
    convertSOD2HMS(sod, civilt.hour, civilt.minute, civilt.second);

    return civilt;
}

CommonTime JulianDate2CommonTime(const JulianDate &jd) {
    CommonTime ct;
    const long double temp_jd(jd.jd);

    const long mjd_day = static_cast<long>(temp_jd - MJD_TO_JD);

    const long double sod = (temp_jd - std::floor(temp_jd)) * SEC_PER_DAY;

    ct.set(mjd_day, static_cast<double>(sod), jd.timeSystem);
    return ct;
}

JulianDate CommonTime2JulianDate(const CommonTime &ct) {
    JulianDate jd;
    long mjd_day;
    double sod;
    ct.get(mjd_day, sod, jd.timeSystem);

    const double jday = mjd_day + MJD_TO_JD;
    jd.jd = static_cast<long double>(jday) + static_cast<long double>(sod) * DAY_PER_SEC;

    return jd;
}


CommonTime YDSTime2CommonTime(const YDSTime &ydst) {
    CommonTime ct;
    const auto jday = static_cast<long>(convertYMD2JD(ydst.year, 1, 1) + ydst.doy - 1);
    ct.set(jday, ydst.sod, ydst.timeSystem);
    return ct;
}

YDSTime CommonTime2YDSTime(const CommonTime &ct) {
    YDSTime ydst;
    long mjday;
    double secDay;
    ct.get(mjday, secDay, ydst.timeSystem);
    ydst.sod = secDay;

    const double jday = mjday + MJD_TO_JD;

    int month, day;
    convertJD2YMD(jday, ydst.year, month, day);

    ydst.doy = jday - convertYMD2JD(ydst.year, 1, 1) + 1;//NOLINT
    return ydst;
}

void MJD2CommonTime(const MJD &mjd, CommonTime &ct) {
    const auto mday = static_cast<double>(mjd.mjd);
    // tmp now holds the partial days
    double sod = mday - static_cast<long>(mday);
    // convert tmp to seconds of day
    sod *= SEC_PER_DAY;

    ct.set(static_cast<long>(mday), sod, mjd.timeSystem);
}

void CommonTime2MJD(const CommonTime &ct, MJD &mjd) {
    long mday;
    double sod;
    ct.get(mday, sod, mjd.timeSystem);
    mjd.mjd = static_cast<long double>(mday) + static_cast<long double>(sod) * DAY_PER_SEC;
}

void CommonTime2JD2020(const CommonTime &ct, JD2020 &jd) {
    long mday;
    double sod;
    ct.get(mday, sod, jd.timeSystem);
    jd.jd = static_cast<long double>(mday) + static_cast<long double>(sod) * DAY_PER_SEC + MJD_TO_JD2020;
}

void JD20202CommonTime(const JD2020 &jd, CommonTime &ct) {
    const auto mday = static_cast<double>(jd.jd);
    // tmp now holds the partial days
    double sod = mday - static_cast<long>(mday);
    // convert tmp to seconds of day
    sod *= SEC_PER_DAY - MJD_TO_JD2020;
    ct.set(static_cast<long>(mday), sod, jd.timeSystem);
}

void MJD2JD2020(const MJD &mjd, JD2020 &jd) {
    jd.jd = mjd.mjd + MJD_TO_JD2020;
}

void JD20202MJD(const JD2020 &jd, MJD &mjd) {
    mjd.mjd = jd.jd - MJD_TO_JD2020;
}


void CommonTime2WeekSecond(const CommonTime &ct, WeekSecond &wk) {
    MJD mjd;
    CommonTime2MJD(ct, mjd);
    if (mjd.mjd < wk.MJDEpoch()) {
        throw InvalidRequest("Unable to convert to Week/Second - before Epoch.");
    }

    long mday;
    double sod;
    ct.get(mday, sod, wk.timeSystem);

    // find the number of days since the beginning of the Epoch
    mday -= wk.MJDEpoch();
    // find out how many weeks that is
    wk.week = static_cast<int>(mday / 7);
    // find out what the day of week is
    mday %= 7;

    wk.sow = mday * SEC_PER_DAY + sod;
}

void WeekSecond2CommonTime(const WeekSecond &wk, CommonTime &ct) {
    //int dow = static_cast<int>( sow * DAY_PER_SEC );
    // Appears to have rounding issues on 32-bit platforms

    const int dow = static_cast<int>(wk.sow / SEC_PER_DAY);
    // NB this assumes MJDEpoch is an integer - what if epoch H:M:S != 0:0:0 ?
    const long mday = wk.MJDEpoch() + 7 * wk.week + dow; //NOLINT
    const double sod(wk.sow - SEC_PER_DAY * dow);
    ct.set(mday, sod, wk.timeSystem);
}
