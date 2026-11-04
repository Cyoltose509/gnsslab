#pragma once

#include <set>
#include <string>

#include "SolverOptions.h"

namespace Solvers {
    enum class Mode { SPP, PPP, LEO, RTK };

    inline const char *modeLabel(const Mode m) {
        switch (m) {
            case Mode::SPP: return "SPP";
            case Mode::PPP: return "PPP";
            case Mode::LEO: return "LEO";
            case Mode::RTK: return "RTK";
        }
        return "SPP";
    }

    /// ini 键前缀 / 产品池归属，与 modeLabel 同序但小写
    inline const char *modeKey(const Mode m) {
        switch (m) {
            case Mode::SPP: return "spp";
            case Mode::PPP: return "ppp";
            case Mode::LEO: return "leo";
            case Mode::RTK: return "rtk";
        }
        return "spp";
    }

    inline std::string sysSetToString(const std::set<char> &s) {
        std::string out;
        for (char c: {'G', 'R', 'E', 'C'}) if (s.count(c)) out += c;
        return out;
    }

    inline std::set<char> stringToSysSet(const std::string &s) {
        std::set<char> out;
        for (char c: s) if (c == 'G' || c == 'R' || c == 'E' || c == 'C') out.insert(c);
        return out.empty() ? std::set{'G', 'R', 'E', 'C'} : out;
    }

    struct RealtimeEndpoint {
        std::string ip;
        std::string port;
    };

    inline SolverOptions defaultSolverOptions(const Mode mode) {
        SolverOptions o = {};
        switch (mode) {
            case Mode::SPP:
                o.spp.tropoMap = Troposphere::Map::Hopfield;
                break;
            case Mode::LEO:
                o.spp.sigmaPhase = 0.003;
                o.spp.firstEpochPosVar = 1.0;
                o.spp.ambNoise = 1.0E-7;
                o.spp.minSatsForUpdate = 4;
                o.spp.useCycleSlip = false;
                o.spp.applySolidTide = false;
                o.motionThresh = 100.0;
                break;
            default: break;
        }
        return o;
    }
}
