#pragma once

#include "TimeStruct.h"

// 读取跳秒
double getLeapSeconds(const CommonTime &ct);

// 时间系统转换
void convertTimeSystem(CommonTime &in_time, TimeSystem targetSys);

void convertJD2YMD(double jd, int &iyear, int &imonth, int &iday);

double convertYMD2JD(int iyear, int imonth, int iday);

void convertSOD2HMS(double sod, int &hh, int &mm, double &sec);

double convertHMS2SOD(int hh, int mm, double sec);

CommonTime CivilTime2CommonTime(const CivilTime &civil_t);

CivilTime CommonTime2CivilTime(const CommonTime &ct);

CommonTime JulianDate2CommonTime(const JulianDate &jd);

JulianDate CommonTime2JulianDate(const CommonTime &ct);

// 平格林尼治恒星时 GMST(rad)，IAU 1980 公式（UT1=UTC、忽略极移）。
// 供 ECI↔ECEF 与固体潮/相位缠绕使用（见 core/Geodesy.h）。
double gmstFromTime(const CommonTime &t);

void CommonTime2MJD(const CommonTime &ct, MJD &mjd);

void MJD2CommonTime(const MJD &mjd, CommonTime &ct);

void CommonTime2JD2020(const CommonTime &ct, JD2020 &jd);

void JD20202CommonTime(JD2020 &jd, CommonTime &ct);

void MJD2JD2020(MJD &mjd, JD2020 &jd);

void JD20202MJD(JD2020 &jd, MJD &mjd);

CommonTime YDSTime2CommonTime(const YDSTime &ydst);

YDSTime CommonTime2YDSTime(const CommonTime &ct);

void CommonTime2WeekSecond(const CommonTime &ct, WeekSecond &wk);

void WeekSecond2CommonTime(const WeekSecond &wk, CommonTime &ct);
