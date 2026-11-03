#pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>

#include "EphemerisTable.h"
#include "GnssStruct.h"
#include "rinex/RINEXFileBase.h"
class RINEXNavStore : public RINEXFileBase {
public:
    RINEXNavStore() = default;

    ~RINEXNavStore() override = default;


    void loadFile(const std::string &file, EphemerisTable &ephTable);


    struct TimeSysCorr {
        double A0, A1;
        int refSOW, refWeek;
    };

    double version = 0;
    std::string fileType, fileSys, fileProgram, fileAgency, date;
    std::vector<std::string> commentList;
    std::map<std::string, std::vector<double>> ionoCorrData;
    std::map<std::string, TimeSysCorr> timeSysCorrData;
    long leapSeconds = 0, leapDelta = 0, leapWeek = 0, leapDay = 0;

private:
    void loadGPSEph(GPSEphem &eph, std::string &line) const;

    void loadBDSEph(BDSEphem &eph, std::string &line) const;

    void loadGalEph(GalileoEphem &eph, std::string &line) const;

    void loadGloEph(GlonassEphem &eph, const std::string &line) const;

};
