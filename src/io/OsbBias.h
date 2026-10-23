#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"
#include <map>
#include <set>
#include <string>
#include <utility>

class OsbBias {
public:
    bool read(const std::string &path);

   [[nodiscard]] bool empty() const { return data.empty(); }

    // 返回 OSB 文件中出现过的全部卫星（用于按系统估计码 IF 偏置 datum 均值）。
    [[nodiscard]] std::set<SatID> satelliteSet() const;

    [[nodiscard]] double getBias(const SatID &sat, const std::string &obsCode) const;

    [[nodiscard]] double codeBias(const FreqCombo &def, const SatID &sat) const;
    [[nodiscard]] double codeBiasAligned(const FreqCombo &def, const SatID &sat) const;
    static std::string refCodeFor(const std::string &code);
    [[nodiscard]] double phaseBias(const FreqCombo &def, const SatID &sat) const;
    void biases(const FreqCombo &def, const SatID &sat,
                double &bP1, double &bP2, double &bL1, double &bL2) const;

private:
    std::map<std::pair<SatID, std::string>, double> data;
};
