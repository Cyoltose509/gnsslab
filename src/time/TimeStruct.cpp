#include "TimeStruct.h"

#include <algorithm>
#include <vector>


double CommonTime::leapSeconds() const {
    static const std::vector<std::pair<double, double> > leapData = {
        {41317.0, 10}, {41499.0, 11}, {41683.0, 12}, {42048.0, 13}, {42413.0, 14},
        {42778.0, 15}, {43144.0, 16}, {43509.0, 17}, {43874.0, 18}, {44239.0, 19},
        {44786.0, 20}, {45151.0, 21}, {45516.0, 22}, {46247.0, 23}, {47161.0, 24},
        {47892.0, 25}, {48257.0, 26}, {48804.0, 27}, {49169.0, 28}, {49534.0, 29},
        {50083.0, 30}, {50630.0, 31}, {51179.0, 32}, {53736.0, 33}, {54832.0, 34},
        {56109.0, 35}, {57204.0, 36}, {57754.0, 37},
    };

    const auto mjdCt = static_cast<double>(m_day);

    // 1972.1.1
    if (mjdCt < leapData.front().first) {
        throw InvalidRequest("Time MUST be greater than 1972 for leapSec!");
    }

    auto it = std::upper_bound(leapData.begin(), leapData.end(), mjdCt,
                               [](const double v, const std::pair<double, double> &p) { return v < p.first; });
    --it;
    return it->second;
}

void CommonTime::convertSystem(const TimeSystem target) {
    const TimeSystem inTS = m_timeSystem;
    if (inTS == target) {
        m_timeSystem = target; // 同系统：仅确保标记一致
        return;
    }
    const auto cOf = [&](const TimeSystem s) -> double {
        switch (s) {
            case TimeSystem::GPS: return 19.0; //NOLINT
            case TimeSystem::GAL: return 19.0;
            case TimeSystem::BDT: return 33.0;
            case TimeSystem::TAI: return 0.0;
            case TimeSystem::UTC: return leapSeconds(); // 仅依赖 m_day，与 ts 无关
            default: throw InvalidRequest("Invalid TimeSystem");
        }
    };
    const double dt = cOf(inTS) - cOf(target);
    m_timeSystem = target;
    *this += dt;
}

CivilTime CommonTime::toCivilTime() const {
    CivilTime civilt;
    civilt.timeSys = m_timeSystem;

    const auto [year, month, day] = YearMonthDay::fromJD(static_cast<double>(m_day) + MJD_TO_JD);
    civilt.year = year;
    civilt.month = month;
    civilt.day = day;

    const auto [hour, minute, second] = HourMinSec::fromSOD(m_sod);
    civilt.hour = hour;
    civilt.minute = minute;
    civilt.second = second;

    return civilt;
}

YDSTime CommonTime::toYDS() const {
    const double jday = static_cast<double>(m_day) + MJD_TO_JD;
    const YearMonthDay ymd = YearMonthDay::fromJD(jday);

    return YDSTime(ymd.year,
                   static_cast<int>(jday - YearMonthDay{ymd.year, 1, 1}.toJD() + 1), //NOLINT
                   m_sod, m_timeSystem);
}

WeekSecond CommonTime::toWeekSecond() const {
    // 周基准跟随本对象自己的时标：GPS/GAL/BDT 各有纪元，UTC/TAI 没有周纪元
    const WeekSecond basis(0, 0.0, m_timeSystem);
    if (basis.MJDEpoch() <= 0 || toMJD().mjd < basis.MJDEpoch()) {
        throw InvalidRequest("Unable to convert to Week/Second - no week epoch in this time system or before Epoch.");
    }

    long mday = m_day;
    // find the number of days since the beginning of the Epoch
    mday -= basis.MJDEpoch();
    // find out how many weeks that is
    const auto week = static_cast<unsigned int>(mday / 7);
    // find out what the day of week is
    mday %= 7;

    return {week, static_cast<double>(mday) * SEC_PER_DAY + m_sod, m_timeSystem};
}

MJD CommonTime::toMJD() const {
    return MJD(static_cast<long double>(m_day) + static_cast<long double>(m_sod) * DAY_PER_SEC, m_timeSystem);
}

JulianDate CommonTime::toJulianDate() const {
    const double jday = static_cast<double>(m_day) + MJD_TO_JD;
    return JulianDate(static_cast<long double>(jday) +
                      static_cast<long double>(m_sod) * DAY_PER_SEC, m_timeSystem);
}

double CommonTime::toGMST() const {
    const double jdFull = static_cast<double>(toJulianDate().jd);
    const double jdStart = std::floor(jdFull + 0.5) - 0.5; // 当日 00:00 UTC 的 JD
    const double ut = (jdFull - jdStart) * SEC_PER_DAY; // 当日秒数
    const double t1 = (jdStart - 2451545.0) / 36525.0;
    const double t2 = t1 * t1, t3 = t2 * t1;
    const double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2e-6 * t3;
    const double gmst = gmst0 + 1.002737909350795 * ut;
    return std::fmod(gmst, SEC_PER_DAY) * PI / 43200.0;
}

CommonTime CivilTime::toCommonTime() const {
    // get the julian day
    const double jday = YearMonthDay{year, month, day}.toJD();

    // convert jday to mjd day.
    const int mjd_day = std::round(jday) - MJD_TO_JD; //NOLINT

    // get the second of day
    const double sod = HourMinSec{hour, minute, second}.toSOD();

    // mjd_day + sod
    return CommonTime(mjd_day, sod, timeSys);
}

CivilTime CivilTime::fromCommonTime(const CommonTime &ct) {
    return ct.toCivilTime();
}

long YearMonthDay::toDays() const {
    long y = year;
    y -= month <= 2 ? 1 : 0;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const auto yoe = static_cast<unsigned>(y - era * 400);
    const auto m = static_cast<unsigned>(month);
    const auto mp = m > 2u ? m - 3u : m + 9u;
    const auto doy = (153u * mp + 2u) / 5u + static_cast<unsigned>(day) - 1u;
    const auto doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097L + static_cast<long>(doe) - 719468L;
}

YearMonthDay YearMonthDay::fromDays(long days) {
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

double YearMonthDay::toJD() const {
    return static_cast<double>(toDays()) + 2440587.5;
}

YearMonthDay YearMonthDay::fromJD(const double jd) {
    return fromDays(static_cast<long>(std::floor(jd + 0.5)) - 2440588L);
}

int YearMonthDay::toDOY() const {
    if (month < 1 || month > 12 || day < 1 || day > 31) return 0;
    const long doy = toDays() - YearMonthDay{year, 1, 1}.toDays() + 1;
    return doy >= 1 && doy <= 366 ? static_cast<int>(doy) : 0;
}

std::optional<YearMonthDay> YearMonthDay::fromDOY(const int year, const int doy) {
    // 年长从 toDays 差值取，闰年规则不另写
    const long jan1 = YearMonthDay{year, 1, 1}.toDays();
    if (doy < 1 || doy > YearMonthDay{year + 1, 1, 1}.toDays() - jan1) return std::nullopt;
    return fromDays(jan1 + doy - 1);
}

int YearMonthDay::daysInMonth(const int year, const int month) {
    const int m = std::clamp(month, 1, 12);
    // 差值法：闰年规则（含百年/四百年）本来就只在 toDays 里写了一份，不再另抄一张月长表
    const YearMonthDay next = m == 12 ? YearMonthDay{year + 1, 1, 1} : YearMonthDay{year, m + 1, 1};
    return static_cast<int>(next.toDays() - YearMonthDay{year, m, 1}.toDays());
}

double HourMinSec::toSOD() const {
    return second + 60. * (minute + 60. * hour);
}

HourMinSec HourMinSec::fromSOD(double sod) {
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

CommonTime JulianDate::toCommonTime() const {
    const auto mjd_day = static_cast<long>(jd - MJD_TO_JD);

    const long double sod =
            (jd - (static_cast<long double>(mjd_day) + MJD_TO_JD)) * SEC_PER_DAY;

    return CommonTime(mjd_day, static_cast<double>(sod), timeSystem);
}

JulianDate JulianDate::fromCommonTime(const CommonTime &ct) {
    return ct.toJulianDate();
}

CommonTime YDSTime::toCommonTime() const {
    const double mjd = YearMonthDay{year, 1, 1}.toJD() + static_cast<double>(doy - 1) - MJD_TO_JD;
    return CommonTime(static_cast<long>(std::llround(mjd)), sod, timeSystem);
}

YDSTime YDSTime::fromCommonTime(const CommonTime &ct) {
    return ct.toYDS();
}

CommonTime MJD::toCommonTime() const {
    const auto mday = static_cast<double>(mjd);
    // 整数日留下，小数日折成日内秒
    double sod = mday - static_cast<long>(mday);
    sod *= SEC_PER_DAY;

    return CommonTime(static_cast<long>(mday), sod, timeSystem);
}

MJD MJD::fromCommonTime(const CommonTime &ct) {
    return ct.toMJD();
}

double MJD::toDOY() const {
    const YearMonthDay ymd = YearMonthDay::fromJD(mjd + MJD_TO_JD);
    return mjd - (YearMonthDay{ymd.year, 1, 1}.toJD() - MJD_TO_JD) + 1.0;
}

CommonTime WeekSecond::toCommonTime() const {
    const int dow = static_cast<int>(sow / SEC_PER_DAY);
    // NB this assumes MJDEpoch is an integer - what if epoch H:M:S != 0:0:0 ?
    const long mday = MJDEpoch() + 7 * week + dow; //NOLINT
    const double sod(sow - SEC_PER_DAY * dow);
    return CommonTime(mday, sod, timeSystem);
}

WeekSecond WeekSecond::fromCommonTime(const CommonTime &ct) {
    // 周基准与结果标签都跟随 ct 的时标，成为 toCommonTime() 的严格逆
    return ct.toWeekSecond();
}
