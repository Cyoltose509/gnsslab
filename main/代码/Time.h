#pragma once
#include<iostream>
#include<cmath>
using namespace std;
struct COMMONTIME
{
	unsigned short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
};
struct MJDTIME
{
	int Days;
	double FracDay;
	MJDTIME()
	{
		Days = 0;
		FracDay = 0;
	}
};
struct GPSTIME
{
	unsigned short Week;
	double SecOfWeek;
	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0;
	}
};
int CommonTimeToMJDTime(COMMONTIME* UTC, MJDTIME* MJD)
{
	double u;
	if (UTC->Day < 0 || UTC->Hour < 0 || UTC->Minute < 0 || UTC->Month < 0 || UTC->Second < 0 || UTC->Year < 0)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	else//调整月份
	{
		if (UTC->Month <= 2)
		{
			UTC->Year -= 1;
			UTC->Month += 12;
		}
		MJD->Days = floor(365.25 * UTC->Year) + floor(30.6001 * (UTC->Month + 1)) + UTC->Day - 679019;
		u = UTC->Hour + UTC->Minute / 60 + UTC->Second / 3600;
		MJD->Days = MJD->Days + floor(u / 24);
		MJD->FracDay = u / 24 - floor(u / 24);
		return 1;
	}
}
int MJDTimeToCommonTime(MJDTIME* MJD, COMMONTIME* UTC)
{
	double JD;
	if (MJD->Days < 0 || MJD->FracDay < 0 || MJD->FracDay>1)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	JD = MJD->Days + MJD->FracDay + 2400000.5;
	int JD_int = floor(JD + 0.5);
	double JD_frac = JD + 0.5 - JD_int;
	long long z = static_cast<long long>(JD_int);
	double A, B, C, D, E, F;
	if (z >= 2299161)
	{
		A = static_cast<long long>(floor((z - 1867216.25) / 36524.25));
		B = z + 1 + A - static_cast<long long>(floor(A / 4));
	}
	else
	{
		B = z;
	}
	C = B + 1524;
	D = static_cast<long long>(floor((C - 122.1) / 365.25));
	E = static_cast<long long>(floor(365.25 * D));
	F = static_cast<long long>(floor((C - E) / 30.6001));
	UTC->Day = static_cast<unsigned short>(C - E - static_cast<long long>(floor(30.6001 * F)));
	if (F < 14)
	{
		UTC->Month = static_cast<unsigned short>(F - 1);
	}
	else
	{
		UTC->Month = static_cast<unsigned short>(F - 13);
	}
	if (UTC->Month > 2)
	{
		UTC->Year = static_cast<unsigned short>(D - 4716);
	}
	else
	{
		UTC->Year = static_cast<unsigned short>(D - 4715);
	}
	double Sec = JD_frac * 86400.0;
	int hour = static_cast<int>(floor(Sec / 3600.0));
	Sec -= hour * 3600;
	int minute = static_cast<int>(floor(Sec / 60));
	UTC->Second = Sec - minute * 60.0;
	UTC->Hour = static_cast<unsigned short>(hour);
	UTC->Minute = static_cast<unsigned short>(minute);
	// 简单合理性检查（确保月份在 1~12 之间）
	if (UTC->Month < 1 || UTC->Month > 12 || UTC->Day < 1 || UTC->Day > 31 ||
		UTC->Hour > 23 || UTC->Minute > 59 || UTC->Second >= 60.0)
	{
		return 0;
	}

	return 1;
}
int MJDTimeToGPSTime(MJDTIME* MJD, GPSTIME* GPS)
{
	if (MJD->Days < 0 || MJD->FracDay < 0 || MJD->FracDay>1)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	double mjd = MJD->Days + MJD->FracDay;
	GPS->Week = floor((mjd - 44244) / 7);
	GPS->SecOfWeek = (mjd - 44244 - GPS->Week * 7) * 86400;
	return 1;
}
int GPSTimeToMJDTime(GPSTIME* GPS, MJDTIME* MJD)
{
	if (GPS->Week < 0 || GPS->SecOfWeek < 0)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	double mjd;
	mjd = 44244 + GPS->Week * 7 + GPS->SecOfWeek / 86400;
	MJD->Days = floor(mjd);
	MJD->FracDay = mjd - floor(mjd);
	return 1;
}
int COMMONTimeToGPSTime(COMMONTIME* UTC, GPSTIME* GPS)
{
	if (UTC->Day < 0 || UTC->Hour < 0 || UTC->Minute < 0 || UTC->Month < 0 || UTC->Second < 0 || UTC->Year < 0)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	MJDTIME MJD;
	if (!CommonTimeToMJDTime(UTC, &MJD))
	{
		return 0;
	}
	MJDTimeToGPSTime(&MJD, GPS);
	return 1;
}
int GPSTimeToCOMMONTime(GPSTIME* GPS, COMMONTIME* UTC)
{
	if (GPS->Week < 0 || GPS->SecOfWeek < 0)
	{
		cout << "无法转化" << endl;
		return 0;
	}
	MJDTIME MJD;
	if (!GPSTimeToMJDTime(GPS, &MJD))
	{
		return 0;
	}
	MJDTimeToCommonTime(&MJD, UTC);
	return 1;
}
void gpsTimeToBdsTime(int gpsWeek, double gpsTow,
	int& bdsWeek, double& bdsTow)
{
	// BDT 与 GPS 时间差
	bdsWeek = gpsWeek - 1356;
	bdsTow = gpsTow - 14.0;
	// 处理秒跨周
	if (bdsTow < 0.0) {
		bdsTow += 604800.0;
		bdsWeek--;
	}
	else if (bdsTow >= 604800.0) {
		bdsTow -= 604800.0;
		bdsWeek++;
	}
}
