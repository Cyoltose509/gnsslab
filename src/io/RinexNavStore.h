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


    void loadFile(const string &file, EphemerisTable &ephTable);


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
