#include <cmath>
#include <vector>
#include <algorithm>
#include "TimeConvert.h"
#include "TimeStruct.h"
#include "Const.h"

// 在给定的历史记录中查找跳秒
double getLeapSeconds(const CommonTime &ct) {
    static const std::vector<std::pair<double, double>> leapData = {
        {41317.0, 10}, {41499.0, 11}, {41683.0, 12}, {42048.0, 13}, {42413.0, 14},
        {42778.0, 15}, {43144.0, 16}, {43509.0, 17}, {43874.0, 18}, {44239.0, 19},
        {44786.0, 20}, {45151.0, 21}, {45516.0, 22}, {46247.0, 23}, {47161.0, 24},
        {47892.0, 25}, {48257.0, 26}, {48804.0, 27}, {49169.0, 28}, {49534.0, 29},
        {50083.0, 30}, {50630.0, 31}, {51179.0, 32}, {53736.0, 33}, {54832.0, 34},
        {56109.0, 35}, {57204.0, 36}, {57754.0, 37},
    };

    const auto mjd_ct = static_cast<double>(ct.m_day);

    // 1972.1.1
    if (mjd_ct < leapData.front().first) {
        throw InvalidRequest("Time MUST be greater than 1972 for leapSec!");
    }

    auto it = std::upper_bound(leapData.begin(), leapData.end(), mjd_ct,
                               [](const double v, const std::pair<double, double> &p) { return v < p.first; });
    --it;
    return it->second;
}

// 时间系统转换
void convertTimeSystem(CommonTime &in_time, const TimeSystem targetSys) {
    const TimeSystem inTS = in_time.m_timeSystem;
    if (inTS == targetSys) {
        in_time.m_timeSystem = targetSys; // 同系统：仅确保标记一致
        return;
    }
    auto cOf = [&](const TimeSystem s) -> double {
        switch (s) {
            case TimeSystem::GPS: return 19.0;//NOLINT
            case TimeSystem::GAL: return 19.0;
            case TimeSystem::BDT: return 33.0;
            case TimeSystem::TAI: return 0.0;
            case TimeSystem::UTC: return getLeapSeconds(in_time); // 仅依赖 m_day，与 ts 无关
            default: throw InvalidRequest("Invalid TimeSystem");
        }
    };
    const double dt = cOf(inTS) - cOf(targetSys);
    in_time.m_timeSystem = targetSys;
    in_time += dt;
}

//
void convertJD2YMD(const double jd, int &iyear, int &imonth, int &iday) {
    const double a = std::floor(jd + 0.5);
    const double b = a + 1537;
    const double c = std::floor((b - 122.1) / 365.25);
    const double d = std::floor(365.25 * c);
    const double e = std::floor((b - d) / 30.6001);
    // 调用方传入的 jday 恒为「整数 + 0.5」，故末尾 (jd+0.5)-floor(jd+0.5) 恒为 0，直接略去。
    iday = static_cast<int>(b) - d - std::floor(30.6001 * e); //NOLINT
    imonth = static_cast<int>(e) - 1 - 12. * std::floor(e / 14); //NOLINT
    iyear = static_cast<int>(c) - 4715 - std::floor((7 + imonth) / 10); //NOLINT
}

double convertYMD2JD(int iyear, int imonth, const int iday) {
    if (imonth <= 2) {
        imonth += 12;
        iyear -= 1;
    }

    const double B = 2 - std::floor(iyear / 100) + std::floor(iyear / 400);
    const double jd_double = std::floor(365.25 * (iyear + 4716)) + std::floor(30.6001 * (imonth + 1)) + B + iday - 1524.5;
    return jd_double;
}

void convertSOD2HMS(double sod, int &hh, int &mm, double &sec) {
    // 归化到 [0, SEC_PER_DAY)：fmod 对任意范围（含多日负秒）都稳健，
    // 旧实现用 unsigned 强转只在 |sod|<1 日时正确。
    sod = std::fmod(sod, SEC_PER_DAY);
    if (sod < 0.0)
        sod += SEC_PER_DAY;

    double temp; // variable to hold the integer part of sod
    sod = std::modf(sod, &temp); // sod holds the fraction, temp the integer
    const long seconds = static_cast<long>(temp); // get temp into a real integer

    hh = seconds / SEC_PER_HOUR;
    mm = seconds % SEC_PER_HOUR / SEC_PER_MIN;
    sec = static_cast<double>(seconds % SEC_PER_MIN) + sod;
}

double convertHMS2SOD(const int hh, const int mm, const double sec) {
    return sec + 60. * (mm + 60. * hh);
}

CommonTime CivilTime2CommonTime(const CivilTime &civil_t) {
    CommonTime ct;
    // get the julian day
    const double jday = convertYMD2JD(civil_t.year, civil_t.month, civil_t.day);

    // convert jday to mjd day.
    int mjd_day = std::round(jday) - MJD_TO_JD; //NOLINT

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

double gmstFromTime(const CommonTime &t) {
    const double jdFull = static_cast<double>(CommonTime2JulianDate(t).jd);
    const double jdStart = std::floor(jdFull + 0.5) - 0.5;   // 当日 00:00 UTC 的 JD
    const double ut = (jdFull - jdStart) * SEC_PER_DAY;       // 当日秒数
    const double t1 = (jdStart - 2451545.0) / 36525.0;
    const double t2 = t1 * t1, t3 = t2 * t1;
    const double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2e-6 * t3;
    const double gmst = gmst0 + 1.002737909350795 * ut;
    return std::fmod(gmst, SEC_PER_DAY) * PI / 43200.0;
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

// 日历 ↔ "距 1970-01-01 的天数"（Howard Hinnant 的 days_from_civil / civil_from_days），
// 精确且无依赖。原先内联在 app/download/ProductMeta.cpp 的匿名 namespace 里，
// 搬到这里是因为它与"产品元数据"无关，是通用日历算法。口径说明见 TimeConvert.h。
long daysFromCivil(long y, const unsigned m, const unsigned d) {
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const auto yoe = static_cast<unsigned>(y - era * 400);
    const auto mp = m > 2u ? m - 3u : m + 9u;
    const auto doy = (153u * mp + 2u) / 5u + d - 1u;
    const auto doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097L + static_cast<long>(doe) - 719468L;
}

void civilFromDays(long z, int &y, unsigned &m, unsigned &d) {
    z += 719468L;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const auto doe = static_cast<unsigned>(z - era * 146097L);
    const auto yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    y = static_cast<int>(yoe) + static_cast<int>(era) * 400;
    const auto doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    const auto mp = (5u * doy + 2u) / 153u;
    d = doy - (153u * mp + 2u) / 5u + 1u;
    m = mp < 10u ? mp + 3u : mp - 9u;
    y += m <= 2;
}
