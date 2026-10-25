#pragma once

#include "TimeStruct.h"

// 读取跳秒
double getLeapSeconds(const CommonTime &ct);

// 时间系统转换
void convertTimeSystem(CommonTime &in_time, TimeSystem targetSys);

// JD ↔ 日历（儒略日 JD 为「整数+0.5」口径）。2026-10-24 起委派给 Hinnant 整数
// 内核 daysFromCivil/civilFromDays（JD = days + 2440587.5），全文件唯一一套
// 格里历闰逻辑；行为与旧 Fliegel 双精度式完全一致（含小数部分的舍入口径）。
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

CommonTime YDSTime2CommonTime(const YDSTime &ydst);

YDSTime CommonTime2YDSTime(const CommonTime &ct);

void CommonTime2WeekSecond(const CommonTime &ct, WeekSecond &wk);

void WeekSecond2CommonTime(const WeekSecond &wk, CommonTime &ct);

// 日历 ↔ "距 1970-01-01 的天数"（Howard Hinnant 的 days_from_civil / civil_from_days），
// 精确且无依赖。**全文件唯一的日历内核**：convertJD2YMD/convertYMD2JD 也委派到
// 这两个函数（JD = days + 2440587.5）。原先内联在 app/download/ProductMeta.cpp 的
// 匿名 namespace 里，搬到这里是因为它与"产品元数据"无关，是通用日历算法。
long daysFromCivil(long y, unsigned m, unsigned d);

void civilFromDays(long z, int &y, unsigned &m, unsigned &d);

