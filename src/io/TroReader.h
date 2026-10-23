#pragma once

#include "GnssStruct.h"
#include "TimeConvert.h"
#include <map>
#include <string>
#include <vector>

class TroReader {
public:
    struct Record {
        CommonTime t;
        double mjd = 0.0;    // 由 t 换算的修约儒略日
        double ztd = 0.0;    // 天顶总延迟 TROTOT (m)
        double ztdStd = 0.0; // TROTOT 形式误差 (m)
        double gradN = 0.0;  // 北向水平梯度 TGNTOT (m)
        double gradE = 0.0;  // 东向水平梯度 TGETOT (m)
        double gradNStd = 0.0;
        double gradEStd = 0.0;
    };

    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] int stationCount() const { return static_cast<int>(data.size()); }

    [[nodiscard]] bool hasStation(const std::string &station) const;

    [[nodiscard]] bool getZtd(const std::string &station, const CommonTime &t,
                              double &ztd, double &sigma) const;

    [[nodiscard]] bool getTropoGrad(const std::string &station, const CommonTime &t,
                                    double &gradN, double &gradE,
                                    double &gradNStd, double &gradEStd) const;

private:
    [[nodiscard]] const std::vector<Record> *findStation(const std::string &station) const;

    std::map<std::string, std::vector<Record> > data;
};
