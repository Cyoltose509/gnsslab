#pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>

#include "EphemerisTable.h"
#include "GnssStruct.h"
#include "RinexFileBase.h"

using namespace std;

class RinexNavStore : public RinexFileBase {
public:
    RinexNavStore() = default;

    ~RinexNavStore() override = default;

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
    void loadGPSEph(GPSEphem &eph, string &line) const;

    void loadBDSEph(BDSEphem &eph, string &line) const;

    void loadGalEph(GalileoEphem &eph, string &line) const;

    void loadGloEph(GlonassEphem &eph, const string &line) const;

};
