#pragma once

#include "CoordConvert.h"
#include "GnssStruct.h"

#include <Eigen/Eigen>

#include <map>
#include <string>
#include <vector>


class ATTReader {
public:
    bool read(const std::string &path);

    void clear() { data.clear(); }

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] bool quaternion(const SatID &sat, double mjd, ATTQuat &q) const;

    [[nodiscard]] bool bodyDCM(const SatID &sat, double mjd, Eigen::Matrix3d &M) const;

    static constexpr double kMaxGapSec = 3600.0;

private:
    struct Sample {
        double mjd = 0.0;
        ATTQuat q;
    };

    std::map<SatID, std::vector<Sample> > data;
};
