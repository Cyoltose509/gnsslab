#pragma once

#include "GnssStruct.h"
#include <string>
#include <vector>

/// IONEX 全球电离层图 GIM（如 `COD..._01H_GIM.INX`）读取器
class IonexReader {
public:
    struct TecMap {
        CommonTime t;
        double mjd = 0.0;                       // 内插用
        std::vector<double> lats;               // 各纬度行(度)，通常 87.5 → -87.5
        double lon1 = -180.0, lon2 = 180.0, dlon = 5.0; // 经度范围(度)
        double hgt = 450.0;                     // 单层高度(km)
        std::vector<std::vector<double> > tec;  // tec[iLat][iLon]，TECU
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
    std::vector<double> mjds;  // 与 maps 同步的历元 MJD 序列(升序)，供 getTec 时间括号查找
    double hgtKm = 450.0;
    double exponent = -1.0; // 文件内数值 × 10^exponent = TECU
};
