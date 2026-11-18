#pragma once
#include "core/SatID.h"
#include "core/obs/FreqCombo.h"
#include <map>
#include <set>
#include <string>
#include <tuple>

class OSBBias {
public:
    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    /// 某卫星/系统在指定测站（或全局）是否有码偏差标定。
    [[nodiscard]] bool covers(const SatID &sat, const std::string &station) const;
    [[nodiscard]] bool coversSystem(SatSystem sys, const std::string &station) const;

    [[nodiscard]] double getBias(const SatID &sat, const std::string &obsCode,
                                 const std::string &station = "") const;

    [[nodiscard]] double codeBias(const FreqCombo &def, const SatID &sat,
                                   const std::string &station = "") const;
    [[nodiscard]] double codeBiasAligned(const FreqCombo &def, const SatID &sat,
                                          const std::string &station = "") const;
    [[nodiscard]] double phaseBias(const FreqCombo &def, const SatID &sat,
                                    const std::string &station = "") const;
    void biases(const FreqCombo &def, const SatID &sat,
                double &bP1, double &bP2, double &bL1, double &bL2,
                const std::string &station = "") const;

private:
    struct Key {
        SatID sat;
        std::string obs;
        std::string station;

        bool operator<(const Key &o) const {
            return std::tie(sat, obs, station) < std::tie(o.sat, o.obs, o.station);
        }
    };

    std::map<Key, double> data;
    std::map<SatID, std::set<std::string> > codeStations;
    std::map<SatSystem, std::set<std::string> > codeSysStations;

    static std::string refCodeFor(const std::string &code, SatSystem sys);
};
