#pragma once

#include "GnssStruct.h"
#include <string>
#include <vector>


class IonexReader {
public:
    struct TecMap {
        CommonTime t;
        double mjd = 0.0;
        std::vector<double> lats;
        double lon1 = -180.0, lon2 = 180.0, dlon = 5.0;
        double hgt = 450.0;
        std::vector<std::vector<double> > tec;
    };

    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return maps.empty(); }

    [[nodiscard]] int mapCount() const { return static_cast<int>(maps.size()); }

    [[nodiscard]] double heightKm() const { return hgtKm; }

    [[nodiscard]] bool getTec(const CommonTime &t, double latDeg, double lonDeg,
                              double &tec) const;

private:
    [[nodiscard]] static double interpolateMap(const TecMap &mp, double latDeg, double lonDeg) ;

    std::vector<TecMap> maps;
    std::vector<double> mjds;
    double hgtKm = 450.0;
    double exponent = -1.0;
};
