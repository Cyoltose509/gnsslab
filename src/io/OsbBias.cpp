#include "OsbBias.h"
#include <fstream>
#include <sstream>
#include <cctype>


bool OsbBias::read(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    bool inSol = false;
    while (std::getline(in, line)) {

        if (size_t p = line.find_first_not_of(" \t"); p != std::string::npos) line = line.substr(p);
        if (line.rfind("+BIAS/SOLUTION", 0) == 0) {
            inSol = true;
            continue;
        }
        if (line.rfind("-BIAS/SOLUTION", 0) == 0) {
            inSol = false;
            continue;
        }
        if (!inSol) continue;
        if (line.size() < 3) continue;
        if (line.rfind("OSB", 0) != 0) continue;

        std::istringstream ss(line);
        std::string tag, group, satStr, obs, t0, t1, unit;
        double val = 0.0, std = 0.0;
        ss >> tag >> group >> satStr >> obs >> t0 >> t1 >> unit >> val >> std;
        if (satStr.size() < 3) continue;
        SatID sat(satStr[0], std::stoi(satStr.substr(1, 2)));
        double meters = val * 1e-9 * C_MPS;
        data[{sat, obs}] = meters;
    }
    return !data.empty();
}

double OsbBias::getBias(const SatID &sat, const std::string &obsCode) const {
    if (const auto it = data.find({sat, obsCode}); it != data.end()) return it->second;
    if (obsCode.size() >= 3 && (obsCode[0] == 'C' || obsCode[0] == 'L') &&
        std::isdigit(static_cast<unsigned char>(obsCode[1]))) {
        const char ptype = obsCode[0];
        const char band = obsCode[1];
        static auto alts = "CWXPDSLAQZ";
        for (const char *p = alts; *p; ++p) {
            std::string alt = std::string(1, ptype) + band + *p;
            if (const auto ait = data.find({sat, alt}); ait != data.end()) return ait->second;
        }
    }
    return 0.0;
}

std::set<SatID> OsbBias::satelliteSet() const {
    std::set<SatID> s;
    for (const auto &[sat, _] : data) s.insert(sat.first);
    return s;
}

double OsbBias::codeBias(const FreqCombo &def, const SatID &sat) const {
    if (empty()) return 0.0;
    return def.c1 * getBias(sat, def.code1) + def.c2 * getBias(sat, def.code2);
}

double OsbBias::codeBiasAligned(const FreqCombo &def, const SatID &sat) const {
    if (empty()) return 0.0;
    const std::string ref1 = refCodeFor(def.code1);
    const std::string ref2 = refCodeFor(def.code2);
    const double r1 = getBias(sat, ref1);
    const double r2 = getBias(sat, ref2);
    return def.c1 * (getBias(sat, def.code1) - r1) + def.c2 * (getBias(sat, def.code2) - r2);
}

std::string OsbBias::refCodeFor(const std::string &code) {
    if (code.size() >= 3 && code[0] == 'C' && (code[1] == '1' || code[1] == '2'))
        return std::string("C") + code[1] + 'W';
    return code;
}

double OsbBias::phaseBias(const FreqCombo &def, const SatID &sat) const {
    if (empty()) return 0.0;
    return def.c1 * getBias(sat, def.phase1) + def.c2 * getBias(sat, def.phase2);
}

void OsbBias::biases(const FreqCombo &def, const SatID &sat,
                     double &bP1, double &bP2, double &bL1, double &bL2) const {
    bP1 = bP2 = bL1 = bL2 = 0.0;
    if (empty()) return;
    bP1 = getBias(sat, def.code1);
    bP2 = getBias(sat, def.code2);
    bL1 = getBias(sat, def.phase1);
    bL2 = getBias(sat, def.phase2);
}
