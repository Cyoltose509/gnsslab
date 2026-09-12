#pragma once
#include "GnssStruct.h"
#include "TimeConvert.h"
#include <vector>
#include <map>
#include <set>
#include <string>

/// 精密钟差 CLK (RINEX CLK) 读取器。
class ClkReader {
public:
    struct Record {
        CommonTime t;
        double bias; // 钟差，秒
        double drift; // 钟漂，秒/秒
    };

    bool read(const std::string &path);

    bool read(const std::string &path, const std::set<char> &allowedSys);

    [[nodiscard]] bool empty() const { return data.empty(); }

    [[nodiscard]] bool contains(const SatID &sat) const {
        auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }

    [[nodiscard]] double getClockBias(const SatID &sat, const CommonTime &t) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }

private:
    std::map<SatID, std::vector<Record> > data;
    // 每卫星相对首历元的秒数向量(读入时一次性预计算)，避免 getClockBias 每次调用都 O(n) 拷贝全部钟差时间。
    std::map<SatID, std::vector<double> > relTimes;
};
