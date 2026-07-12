#pragma once

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <fstream>

#include "EphemerisTable.h"
#include "GnssStruct.h"

using namespace std;

/// RINEX 3.x 导航文件读取器
/// 用法：loadFile(gnssNavFile, ephTable) 直接将 GPS/BDS 星历写入 EphemerisTable
class RinexNavStore {
public:
    RinexNavStore() = default;
    virtual ~RinexNavStore() = default;

    /// 读一个 RINEX nav 文件（.xxN / .xxG / .xxC），星历写入 ephTable
    void loadFile(const string &file, EphemerisTable &ephTable);

    /// RINEX header 信息（ionosphere / time system / leap seconds — 质量分析用）
    struct TimeSysCorr {
        double A0, A1;
        int refSOW, refWeek;
    };

    double version = 0;
    string fileType, fileSys, fileProgram, fileAgency, date;
    vector<string> commentList;
    map<string, vector<double>> ionoCorrData;
    map<string, TimeSysCorr> timeSysCorrData;
    long leapSeconds = 0, leapDelta = 0, leapWeek = 0, leapDay = 0;

private:
    static void loadGPSEph(GPSEphem &eph, string &line, fstream &navFile);

    static void loadBDSEph(BDSEphem &eph, string &line, fstream &navFile);
};
