#pragma once
#include "GnssStruct.h"
#include "MathUtils.h"
#include "RinexFileBase.h"
#include <vector>
#include <map>
#include <set>
#include <string>


class ClkReader : public RinexFileBase {
public:
    struct Record {
        CommonTime t;
        double bias;
        double drift;
    };

    bool read(const std::string &path);

    bool read(const std::string &path, const std::set<char> &allowedSys);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] bool contains(const SatID &sat) const {
        const auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }

    [[nodiscard]] double getClockBias(const SatID &sat, const CommonTime &t) const;

    [[nodiscard]] bool covers(const SatID &sat, const CommonTime &t, const double maxGap = 120.0) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }


    [[nodiscard]] const std::map<SatID, std::vector<Record>> &records() const { return data; }

private:
    std::map<SatID, std::vector<Record> > data;

    std::map<SatID, std::vector<double> > relTimes;
};
