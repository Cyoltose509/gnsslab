#pragma once

#include "TimeStruct.h"

#include <optional>

namespace Time {
    double getLeapSeconds(const CommonTime &ct);

    void convertSystem(CommonTime &in_time, TimeSystem targetSys);


    long civilToYMD(const YearMonthDay &ymd);

    YearMonthDay YMDToCivil(long days);

    int daysInMonth(int year, int month);

    std::optional<YearMonthDay> DOYToYMD(int year, int doy);

    int YMDToDOY(const YearMonthDay &ymd);

    YearMonthDay JDToYMD(double jd);

    double YMDToJD(const YearMonthDay &ymd);

    double MJDToDOY(double mjd);

    HourMinSec SODToHMS(double sod);

    double HMSToSOD(const HourMinSec &hms);

    CommonTime CivilToCommon(const CivilTime &civil_t);

    CivilTime CommonToCivil(const CommonTime &ct);

    CommonTime JulianToCommon(const JulianDate &jd);

    JulianDate CommonToJulian(const CommonTime &ct);

    CommonTime MJDToCommon(const MJD &mjd);

    MJD CommonToMJD(const CommonTime &ct);

    CommonTime YDSToCommon(const YDSTime &ydst);

    YDSTime CommonToYDS(const CommonTime &ct);


    CommonTime WeekSecondToCommon(const WeekSecond &wk);

    WeekSecond CommonToWeekSecond(const CommonTime &ct, TimeSystem sys = TimeSystem::GPS);

    double CommonToGMST(const CommonTime &t);
}
