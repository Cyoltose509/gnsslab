#include "rinex/RINEXObsReader.h"

#include "StringUtils.h"
#include "TimeConvert.h"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace {
    // RINEX2 的两字符类型（C1/L1/P2…）→ RINEX3 的三字符（C1C/L1C/C2W…）
    std::string toObsType3(const char sys, const std::string &t2) {
        if (t2.size() != 2) return t2;
        if (sys == 'R') {
            // GLONASS：P1/P2 是 G1/G2 上的 P 码
            if (t2 == "C1") return "C1C";
            if (t2 == "P1") return "C1P";
            if (t2 == "C2") return "C2C";
            if (t2 == "P2") return "C2P";
            if (t2 == "L1") return "L1C";
            if (t2 == "L2") return "L2P";
        } else {
            if (t2 == "C1") return "C1C"; // L1 上的 C/A 码
            if (t2 == "P1") return "C1W"; // L1 上的 P 码
            if (t2 == "C2") return "C2C";
            if (t2 == "P2") return "C2W"; // L2 上的 P 码
            if (t2 == "L1") return "L1C";
            if (t2 == "L2") return "L2W";
        }
        std::string r = t2;
        if (r.size() == 2) r += r[1] == '1' ? 'C' : r[1] == '2' ? 'W' : 'X';
        return r;
    }

    std::string normalizeSat3(const std::string &s) {
        if (s.size() < 3) return {};
        char sys = s[0];
        if (sys == ' ') sys = 'G';
        if (std::isdigit(static_cast<unsigned char>(s[1])) == 0 && s[1] != ' ') return {};
        if (std::isdigit(static_cast<unsigned char>(s[2])) == 0 && s[2] != ' ') return {};
        const int prn = Str::toInt(s.substr(1, 2));
        if (prn <= 0) return {};
        char buf[8];
        std::snprintf(buf, sizeof buf, "%c%02d", sys, prn);
        return buf;
    }
}

CommonTime RINEX2ObsReader::parseTime(const std::string &line) {
    std::istringstream ss(line.substr(0, std::min<std::size_t>(26, line.size())));
    int yy = 0, mo = 0, d = 0, h = 0, mi = 0;
    double sec = 0.0;
    if (!(ss >> yy >> mo >> d >> h >> mi >> sec)) return CommonTime();
    const int year = yy < 80 ? 2000 + yy : 1900 + yy;
    return Time::CivilToCommon(CivilTime(year, mo, d, h, mi, sec));
}

bool RINEX2ObsReader::isEpochLine(const std::string &line) const {
    if (line.size() < 32) return false;
    std::istringstream ss(line.substr(0, 26));
    int yy = 0, mo = 0, d = 0, h = 0, mi = 0; //NOLINT
    double sec = 0.0; //NOLINT
    if (!(ss >> yy >> mo >> d >> h >> mi >> sec)) return false;
    if (mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59) return false;
    const char flag = line[28];
    return (flag >= '0' && flag <= '6') || flag == ' ';
}

CommonTime RINEX2ObsReader::epochTime(const std::string &line) const {
    return parseTime(line);
}

void RINEX2ObsReader::readEpochHead(std::string &line, int &flag, int &numSats) {
    flag = line.size() > 28 && line[28] >= '0' && line[28] <= '6' ? line[28] - '0' : 0;
    numSats = Str::toInt(Str::sub(line, 29, 3));

    mSatIds.clear();

    if (flag >= 2 && flag <= 5) {
        for (int i = 0; i < numSats; ++i) {
            if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        }
        return;
    }
    if (numSats <= 0) return;

    int left = numSats;
    for (int i = 0; i < 12 && left > 0; ++i, --left) {
        if (auto id = normalizeSat3(Str::sub(line, 32 + 3 * i, 3)); !id.empty())
            mSatIds.push_back(std::move(id));
    }
    while (left > 0) {
        if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        for (int i = 0; i < 12 && left > 0; ++i, --left) {
            if (auto id = normalizeSat3(Str::sub(line, 32 + 3 * i, 3)); !id.empty())
                mSatIds.push_back(std::move(id));
        }
    }
}

void RINEX2ObsReader::readSatObs(SatTypeValueMap &stvData, const int numSats,
                                SatTypeLLIMap &lliOut) {
    const auto nobs = static_cast<int>(mTypes2.size());
    if (nobs <= 0 || numSats <= 0) return;
    const int rowsPerSat = (nobs + 4) / 5;

    std::string line;

    bool withId = false;
    if (nextLine(line)) {
        const std::string id = normalizeSat3(Str::sub(line, 0, 3));
        withId = !id.empty() && std::find(mSatIds.begin(), mSatIds.end(), id) != mSatIds.end();
    } else {
        throw EndOfFile("EOF encountered!");
    }

    for (int k = 0; k < numSats; ++k) {
        if (k > 0 && !nextLine(line)) throw EndOfFile("EOF encountered!");

        std::string buf;
        std::string satId;
        if (withId) {
            satId = normalizeSat3(Str::sub(line, 0, 3));
            buf = line.size() > 3 ? line.substr(3) : std::string();
        } else {
            satId = k < static_cast<int>(mSatIds.size()) ? mSatIds[k] : std::string();
            buf = line;
        }
        buf.resize(80, ' ');

        for (int r = 1; r < rowsPerSat; ++r) {
            if (!nextLine(line)) throw EndOfFile("EOF encountered!");
            std::string seg = line;
            seg.resize(80, ' ');
            buf += seg;
        }

        if (satId.empty()) continue;
        SatID sat(satId[0], Str::toInt(satId.substr(1, 2)));
        if (sat.system == 'R') {
            if (const auto it = mGloChan.find(sat.toString()); it != mGloChan.end())
                sat.chan = it->second;
        }
        if (sat.system != 'G' && sat.system != 'C' && sat.system != 'R' && sat.system != 'E') {
            continue;
        }
        decodeObsBlock(sat, buf, stvData, lliOut);
    }
}

void RINEX2ObsReader::readHeaderLabel(const std::string &label, const std::string &line) {
    if (label != "# / TYPES OF OBSERV") return;
    if (mObsCount == 0) mObsCount = Str::toInt(Str::sub(line, 0, 6));
    constexpr int perLine = 9;
    for (int i = 0; i < perLine && static_cast<int>(mTypes2.size()) < mObsCount; ++i) {
        if (std::string t = Str::strip(Str::sub(line, 10 + 6 * i, 2)); !t.empty())
            mTypes2.push_back(t);
    }
}

void RINEX2ObsReader::afterHeader() {
    if (mTypes2.empty()) return;
    for (auto p = "GRECJIS"; *p; ++p) {
        std::vector<std::string> v;
        v.reserve(mTypes2.size());
        for (const auto &t: mTypes2) v.push_back(toObsType3(*p, t));
        rinexHeader.mapObsTypes[*p] = std::move(v);
    }
}
