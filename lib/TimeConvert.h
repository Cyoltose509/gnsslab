#pragma once

#include "TimeStruct.h"

// 读取跳秒
double getLeapSeconds(const CommonTime &ct);

// 时间系统转换
void convertTimeSystem(CommonTime &in_time, const TimeSystem &targetSys);

void convertJD2YMD(double jd, int &iyear, int &imonth, int &iday);

double convertYMD2JD(int iyear, int imonth, int iday);

void convertSOD2HMS(double sod, int &hh, int &mm, double &sec);

double convertHMS2SOD(int hh, int mm, double sec);

CommonTime CivilTime2CommonTime(const CivilTime &civil_t);

CivilTime CommonTime2CivilTime(const CommonTime &ct);

CommonTime JulianDate2CommonTime(JulianDate &jd);

JulianDate CommonTime2JulianDate(CommonTime &ct);

void CommonTime2MJD(const CommonTime &ct, MJD &mjd);

void MJD2CommonTime(MJD &mjd, CommonTime &ct);

void CommonTime2JD2020(const CommonTime &ct, JD2020 &jd);

void JD20202CommonTime(JD2020 &jd, CommonTime &ct);

void MJD2JD2020(MJD &mjd, JD2020 &jd);

void JD20202MJD(JD2020 &jd, MJD &mjd);

CommonTime YDSTime2CommonTime(YDSTime &ydst);

YDSTime CommonTime2YDSTime(const CommonTime &ct);

void CommonTime2WeekSecond(const CommonTime &ct, WeekSecond &wk);

void WeekSecond2CommonTime(const WeekSecond &wk, CommonTime &ct);
