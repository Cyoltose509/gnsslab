#include <cmath>
#include <vector>
#include <algorithm>
#include "TimeConvert.h"
#include "TimeStruct.h"
#include "Const.h"

namespace Time {
    double getLeapSeconds(const CommonTime &ct) {
        static const std::vector<std::pair<double, double> > leapData = {
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

    void convertSystem(CommonTime &in_time, const TimeSystem targetSys) {
        const TimeSystem inTS = in_time.m_timeSystem;
        if (inTS == targetSys) {
            in_time.m_timeSystem = targetSys; // 同系统：仅确保标记一致
            return;
        }
        auto cOf = [&](const TimeSystem s) -> double {
            switch (s) {
                case TimeSystem::GPS: return 19.0; //NOLINT
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


    long civilToYMD(const YearMonthDay &ymd) {
        long y = ymd.year;
        y -= ymd.month <= 2 ? 1 : 0;
        const long era = (y >= 0 ? y : y - 399) / 400;
        const auto yoe = static_cast<unsigned>(y - era * 400);
        const auto m = static_cast<unsigned>(ymd.month);
        const auto mp = m > 2u ? m - 3u : m + 9u;
        const auto doy = (153u * mp + 2u) / 5u + static_cast<unsigned>(ymd.day) - 1u;
        const auto doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
        return era * 146097L + static_cast<long>(doe) - 719468L;
    }

    YearMonthDay YMDToCivil(long days) {
        days += 719468L;
        const long era = (days >= 0 ? days : days - 146096) / 146097;
        const auto doe = static_cast<unsigned>(days - era * 146097L);
        const auto yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
        const auto doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
        const auto mp = (5u * doy + 2u) / 153u;

        YearMonthDay ymd;
        ymd.day = static_cast<int>(doy - (153u * mp + 2u) / 5u + 1u);
        ymd.month = static_cast<int>(mp < 10u ? mp + 3u : mp - 9u);
        ymd.year = static_cast<int>(yoe) + static_cast<int>(era) * 400 + (ymd.month <= 2 ? 1 : 0);
        return ymd;
    }

    YearMonthDay JDToYMD(const double jd) {
        return YMDToCivil(static_cast<long>(std::floor(jd + 0.5)) - 2440588L);
    }

    int daysInMonth(const int year, const int month) {
        const int m = std::clamp(month, 1, 12);
        // 差值法：闰年规则（含百年/四百年）本来就只在 civilToDays 里写了一份，不再另抄一张月长表
        const YearMonthDay next = m == 12 ? YearMonthDay{year + 1, 1, 1} : YearMonthDay{year, m + 1, 1};
        return static_cast<int>(civilToYMD(next) - civilToYMD({year, m, 1}));
    }

    std::optional<YearMonthDay> DOYToYMD(const int year, const int doy) {
        // 年长同样从 civilToDays 差值取，闰年规则不另写
        const long jan1 = civilToYMD({year, 1, 1});
        if (doy < 1 || doy > civilToYMD({year + 1, 1, 1}) - jan1) return std::nullopt;
        return YMDToCivil(jan1 + doy - 1);
    }

    int YMDToDOY(const YearMonthDay &ymd) {
        if (ymd.month < 1 || ymd.month > 12 || ymd.day < 1 || ymd.day > 31) return 0;
        const long doy = civilToYMD(ymd) - civilToYMD({ymd.year, 1, 1}) + 1;
        return doy >= 1 && doy <= 366 ? static_cast<int>(doy) : 0;
    }

    double YMDToJD(const YearMonthDay &ymd) {
        return static_cast<double>(civilToYMD(ymd)) + 2440587.5;
    }

    double MJDToDOY(const double mjd) {
        const YearMonthDay ymd = JDToYMD(mjd + MJD_TO_JD);
        return mjd - (YMDToJD({ymd.year, 1, 1}) - MJD_TO_JD) + 1.0;
    }

    HourMinSec SODToHMS(double sod) {
        sod = std::fmod(sod, SEC_PER_DAY);
        if (sod < 0.0)
            sod += SEC_PER_DAY;

        double integerPart; // 整数秒，小数秒留在 sod 里
        sod = std::modf(sod, &integerPart);
        const long seconds = static_cast<long>(integerPart);

        return {
            static_cast<int>(seconds / SEC_PER_HOUR),
            static_cast<int>(seconds % SEC_PER_HOUR / SEC_PER_MIN),
            static_cast<double>(seconds % SEC_PER_MIN) + sod
        };
    }

    double HMSToSOD(const HourMinSec &hms) {
        return hms.second + 60. * (hms.minute + 60. * hms.hour);
    }

    CommonTime CivilToCommon(const CivilTime &civil_t) {
        // get the julian day
        const double jday = YMDToJD({civil_t.year, civil_t.month, civil_t.day});

        // convert jday to mjd day.
        const int mjd_day = std::round(jday) - MJD_TO_JD; //NOLINT

        // get the second of day
        const double sod = HMSToSOD({civil_t.hour, civil_t.minute, civil_t.second});

        // mjd_day + sod
        return CommonTime(mjd_day, sod, civil_t.timeSys);
    }

    CivilTime CommonToCivil(const CommonTime &ct) {
        CivilTime civilt;
        civilt.timeSys = ct.m_timeSystem;

        const auto [year, month, day] = JDToYMD(static_cast<double>(ct.m_day) + MJD_TO_JD);
        civilt.year = year;
        civilt.month = month;
        civilt.day = day;

        const auto [hour, minute, second] = SODToHMS(ct.m_sod);
        civilt.hour = hour;
        civilt.minute = minute;
        civilt.second = second;

        return civilt;
    }

    CommonTime JulianToCommon(const JulianDate &jd) {
        const long double temp_jd(jd.jd);

        const auto mjd_day = static_cast<long>(temp_jd - MJD_TO_JD);

        const long double sod =
                (temp_jd - (static_cast<long double>(mjd_day) + MJD_TO_JD)) * SEC_PER_DAY;

        return CommonTime(mjd_day, static_cast<double>(sod), jd.timeSystem);
    }

    JulianDate CommonToJulian(const CommonTime &ct) {
        const double jday = static_cast<double>(ct.m_day) + MJD_TO_JD;
        return JulianDate(static_cast<long double>(jday) +
                          static_cast<long double>(ct.m_sod) * DAY_PER_SEC, ct.m_timeSystem);
    }

    CommonTime MJDToCommon(const MJD &mjd) {
        const auto mday = static_cast<double>(mjd.mjd);
        // 整数日留下，小数日折成日内秒
        double sod = mday - static_cast<long>(mday);
        sod *= SEC_PER_DAY;

        return CommonTime(static_cast<long>(mday), sod, mjd.timeSystem);
    }

    MJD CommonToMJD(const CommonTime &ct) {
        return MJD(static_cast<long double>(ct.m_day) +
                   static_cast<long double>(ct.m_sod) * DAY_PER_SEC, ct.m_timeSystem);
    }

    CommonTime YDSToCommon(const YDSTime &ydst) {
        const double mjd = YMDToJD({ydst.year, 1, 1}) + static_cast<double>(ydst.doy - 1) - MJD_TO_JD;
        return CommonTime(static_cast<long>(std::llround(mjd)), ydst.sod, ydst.timeSystem);
    }

    YDSTime CommonToYDS(const CommonTime &ct) {
        const double jday = static_cast<double>(ct.m_day) + MJD_TO_JD;
        const YearMonthDay ymd = JDToYMD(jday);

        return YDSTime(ymd.year,
                       static_cast<int>(jday - YMDToJD({ymd.year, 1, 1}) + 1), //NOLINT
                       ct.m_sod, ct.m_timeSystem);
    }

    CommonTime WeekSecondToCommon(const WeekSecond &wk) {
        //int dow = static_cast<int>( sow * DAY_PER_SEC );
        // Appears to have rounding issues on 32-bit platforms

        const int dow = static_cast<int>(wk.sow / SEC_PER_DAY);
        // NB this assumes MJDEpoch is an integer - what if epoch H:M:S != 0:0:0 ?
        const long mday = wk.MJDEpoch() + 7 * wk.week + dow; //NOLINT
        const double sod(wk.sow - SEC_PER_DAY * dow);
        return CommonTime(mday, sod, wk.timeSystem);
    }

    WeekSecond CommonToWeekSecond(const CommonTime &ct, const TimeSystem sys) {
        // 周基准按调用方给的时标取（原实现读入参 wk 的 timeSystem），事后结果跟随 ct
        const WeekSecond basis(0, 0.0, sys);
        if (const MJD mjd = CommonToMJD(ct); mjd.mjd < basis.MJDEpoch()) {
            throw InvalidRequest("Unable to convert to Week/Second - before Epoch.");
        }

        long mday = ct.m_day;
        const double sod = ct.m_sod;

        // find the number of days since the beginning of the Epoch
        mday -= basis.MJDEpoch();
        // find out how many weeks that is
        const auto week = static_cast<unsigned int>(mday / 7);
        // find out what the day of week is
        mday %= 7;

        WeekSecond wk(week, static_cast<double>(mday) * SEC_PER_DAY + sod, ct.m_timeSystem);
        return wk;
    }

    double CommonToGMST(const CommonTime &t) {
        const double jdFull = static_cast<double>(CommonToJulian(t).jd);
        const double jdStart = std::floor(jdFull + 0.5) - 0.5; // 当日 00:00 UTC 的 JD
        const double ut = (jdFull - jdStart) * SEC_PER_DAY; // 当日秒数
        const double t1 = (jdStart - 2451545.0) / 36525.0;
        const double t2 = t1 * t1, t3 = t2 * t1;
        const double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2e-6 * t3;
        const double gmst = gmst0 + 1.002737909350795 * ut;
        return std::fmod(gmst, SEC_PER_DAY) * PI / 43200.0;
    }
}
