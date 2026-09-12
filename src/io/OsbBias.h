#pragma once
#include "GnssStruct.h"
#include "FreqCombo.h"   // FreqCombo（codeBias/phaseBias/biases 按 IF 系数合并 OSB 偏差）
#include <map>
#include <string>
#include <utility>

class OsbBias {
public:
    bool read(const std::string &path);

    bool empty() const { return data.empty(); }

    double getBias(const SatID &sat, const std::string &obsCode) const;

    double getCodeBias(const SatID &sat, const std::string &obsCode) const {
        return getBias(sat, obsCode);
    }

    double codeBias(const FreqCombo &def, const SatID &sat) const {
        if (empty()) return 0.0;
        return def.c1 * getBias(sat, def.code1) + def.c2 * getBias(sat, def.code2);
    }
    double phaseBias(const FreqCombo &def, const SatID &sat) const {
        if (empty()) return 0.0;
        return def.c1 * getBias(sat, def.phase1) + def.c2 * getBias(sat, def.phase2);
    }
    void biases(const FreqCombo &def, const SatID &sat,
                double &bP1, double &bP2, double &bL1, double &bL2) const {
        bP1 = bP2 = bL1 = bL2 = 0.0;
        if (empty()) return;
        bP1 = getBias(sat, def.code1);
        bP2 = getBias(sat, def.code2);
        bL1 = getBias(sat, def.phase1);
        bL2 = getBias(sat, def.phase2);
    }

private:
    std::map<std::pair<SatID, std::string>, double> data;
};
