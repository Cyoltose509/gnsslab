#pragma once

#include "GnssStruct.h"
#include <string>
#include <vector>


class ErpReader {
public:
    struct Record {
        double mjd = 0.0;
        double xp = 0.0;
        double yp = 0.0;
        double ut1Utc = 0.0;
        double lod = 0.0;
    };

    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] int size() const { return static_cast<int>(data.size()); }

    [[nodiscard]] bool getErp(const CommonTime &t, double &xp, double &yp, double &ut1Utc) const;

private:
    std::vector<Record> data;
};
