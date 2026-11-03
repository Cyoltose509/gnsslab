#pragma once

#include "GnssStruct.h"
#include "rinex/RINEXFileBase.h"
#include "TimeConvert.h"

#include <string>
#include <vector>

class METReader : public RINEXFileBase {
public:
    struct Record {
        double mjd = 0.0;
        double presHpa = 0.0; // PR
        double tempC = 0.0;   // TD（干温）
        double humi = 0.0;    // HR / 100（相对湿度 0~1）
    };

    bool read(const std::string &path);

    void clear() { recs.clear(); }

    [[nodiscard]] bool empty() const { return recs.empty(); }
    [[nodiscard]] int count() const { return static_cast<int>(recs.size()); }

    [[nodiscard]] bool get(double mjd, double &presHpa, double &tempC, double &humi) const;

private:
    std::vector<Record> recs; // 按 mjd 升序
};
