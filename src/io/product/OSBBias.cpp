#include "product/OSBBias.h"
#include <cctype>
#include <charconv>
#include <fstream>
#include <sstream>
#include <vector>

namespace {
    std::string trim(const std::string &s) {
        const auto a = s.find_first_not_of(" \t");
        if (a == std::string::npos) return {};
        const auto b = s.find_last_not_of(" \t");
        return s.substr(a, b - a + 1);
    }
}

bool OSBBias::read(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    bool inSol = false;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string head = trim(line);
        if (head.rfind("+BIAS/SOLUTION", 0) == 0) {
            inSol = true;
            continue;
        }
        if (head.rfind("-BIAS/SOLUTION", 0) == 0) {
            inSol = false;
            continue;
        }
        if (!inSol || line.size() < 34) continue;

        // BIAS-SINEX 是定宽格式：STATION / OBS1 / OBS2 都是可空列，用 token 顺读会串位。
        if (const std::string tag = trim(line.substr(1, 3)); tag != "OSB" && tag != "ABS") continue;
        const std::string prn = trim(line.substr(11, 3));
        if (prn.size() < 3) continue; // 系统级偏差（PRN 为空）不对应具体卫星
        int prnNo = 0;
        if (const auto [p, ec] = std::from_chars(prn.data() + 1, prn.data() + 3, prnNo);
            ec != std::errc() || p != prn.data() + 3)
            continue;
        const std::string station = trim(line.substr(15, 9));
        const std::string obs = trim(line.substr(25, 3));
        if (obs.empty() || !trim(line.substr(30, 3)).empty()) continue;

        std::istringstream ss(line.substr(34));
        std::string t0, t1, unit;
        double val = 0.0;
        if (double stddev = 0.0; !(ss >> t0 >> t1 >> unit >> val >> stddev)) continue;
        double meters = val;
        if (unit.empty() || unit[0] == 'n' || unit[0] == 'N') meters = val * 1e-9 * C_MPS;
        const SatID sat(prn[0], prnNo);
        data[{sat, obs, station}] = meters;
        if (obs[0] == 'C') {
            codeStations[sat].insert(station);
            codeSysStations[sat.system].insert(station);
        }
    }
    return !data.empty();
}

double OSBBias::getBias(const SatID &sat, const std::string &obsCode, const std::string &station) const {
    if (empty()) return 0.0;
    if (!station.empty())
        if (const auto it = data.find({sat, obsCode, station}); it != data.end()) return it->second;
    if (const auto it = data.find({sat, obsCode, ""}); it != data.end()) return it->second;
    if (obsCode.size() < 3 || (obsCode[0] != 'C' && obsCode[0] != 'L') ||
        !std::isdigit(static_cast<unsigned char>(obsCode[1])))
        return 0.0;

    std::vector<std::string> alts;
    for (auto p = "CWXPDSLAQZ"; *p; ++p)
        alts.emplace_back(std::string(1, obsCode[0]) + obsCode[1] + *p);
    if (!station.empty())
        for (const auto &alt: alts)
            if (const auto it = data.find({sat, alt, station}); it != data.end()) return it->second;
    for (const auto &alt: alts)
        if (const auto it = data.find({sat, alt, ""}); it != data.end()) return it->second;
    return 0.0;
}

bool OSBBias::covers(const SatID &sat, const std::string &station) const {
    const auto it = codeStations.find(sat);
    return it != codeStations.end() && (it->second.count("") > 0 || it->second.count(station) > 0);
}

bool OSBBias::coversSystem(const char sys, const std::string &station) const {
    const auto it = codeSysStations.find(sys);
    return it != codeSysStations.end() && (it->second.count("") > 0 || it->second.count(station) > 0);
}

double OSBBias::codeBias(const FreqCombo &def, const SatID &sat, const std::string &station) const {
    if (empty()) return 0.0;
    return def.c1 * getBias(sat, def.code1, station) + def.c2 * getBias(sat, def.code2, station);
}

std::string OSBBias::refCodeFor(const std::string &code, const char sys) {
    // GLONASS FDMA 的码偏差是逐测站给出的 IF 层偏差，没有 C1W/C2W 参考码；
    // 两频点同值，c1+c2=1，直接组合就是 IF 改正。
    if (sys == 'R') return {};
    if (code.size() >= 3 && code[0] == 'C' && (code[1] == '1' || code[1] == '2'))
        return std::string("C") + code[1] + 'W';
    return code;
}

double OSBBias::codeBiasAligned(const FreqCombo &def, const SatID &sat, const std::string &station) const {
    if (empty()) return 0.0;
    const std::string ref1 = refCodeFor(def.code1, sat.system);
    const std::string ref2 = refCodeFor(def.code2, sat.system);
    const double r1 = ref1.empty() ? 0.0 : getBias(sat, ref1, station);
    const double r2 = ref2.empty() ? 0.0 : getBias(sat, ref2, station);
    return def.c1 * (getBias(sat, def.code1, station) - r1)
           + def.c2 * (getBias(sat, def.code2, station) - r2);
}

double OSBBias::phaseBias(const FreqCombo &def, const SatID &sat, const std::string &station) const {
    if (empty()) return 0.0;
    return def.c1 * getBias(sat, def.phase1, station) + def.c2 * getBias(sat, def.phase2, station);
}

void OSBBias::biases(const FreqCombo &def, const SatID &sat,
                     double &bP1, double &bP2, double &bL1, double &bL2,
                     const std::string &station) const {
    bP1 = bP2 = bL1 = bL2 = 0.0;
    if (empty()) return;
    bP1 = getBias(sat, def.code1, station);
    bP2 = getBias(sat, def.code2, station);
    bL1 = getBias(sat, def.phase1, station);
    bL2 = getBias(sat, def.phase2, station);
}
