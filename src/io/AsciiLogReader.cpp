#include "AsciiLogReader.h"
#include "Const.h"
#include <fstream>
#include <sstream>
#include <cstdlib>

#include "StringUtils.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace {
    bool startsWith(const std::string &s, const std::string &p) {
        return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
    }

    long long toCount(const std::string &f) {
        if (f.empty()) return 0;
        try {
            return std::stoll(f, nullptr, 10);
        } catch (...) {
            return 0;
        }
    }
}

bool AsciiLogReader::open(const std::string &path) {
    mLines.clear();
    mImuLog.clear();
    mImuType = -1;
    mLineIdx = 0;
    std::ifstream ifs(acpToWide(path), std::ios::in);
    if (!ifs) return false;
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        mLines.push_back(std::move(line));
    }
    return true;
}

bool AsciiLogReader::getNextImu(ImuSample &s) {
    while (mLineIdx < mLines.size()) {
        if (const std::string &line = mLines[mLineIdx++]; parseLine(line, s, mImuType)) {
            mImuLog.push_back(s);
            return true;
        }
    }
    return false;
}

bool AsciiLogReader::parseLine(const std::string &line, ImuSample &s, int &imuType) {
    const bool isX = startsWith(line, "#RAWIMUXA") || startsWith(line, "%RAWIMUSXA");
    if (const bool isPlain = startsWith(line, "#RAWIMUA") || startsWith(line, "%RAWIMUSA");
        !isX && !isPlain)
        return false;
    const auto sc = line.find(';');
    if (sc == std::string::npos) return false;
    std::string body = line.substr(sc + 1);
    if (const auto star = body.find('*'); star != std::string::npos) body = body.substr(0, star);

    std::vector<std::string> f;
    std::stringstream ss(body);
    std::string tok;
    while (std::getline(ss, tok, ',')) f.push_back(tok);
    if (f.empty()) return false;

    const int base = isX ? 5 : 3;
    if (static_cast<int>(f.size()) < base + 6) return false;

    if (isX && imuType < 0) {
        if (const long t = std::strtol(f[1].c_str(), nullptr, 16); t > 0 && t < 256)
            imuType = static_cast<int>(t);
    }

    const int week = static_cast<int>(std::strtol(f[base - 3].c_str(), nullptr, 10));
    const double seconds = std::strtod(f[base - 2].c_str(), nullptr);
    s.gnssWeek = week;
    s.tow = week * FULL_WEEK + seconds;

    const long long zacc = toCount(f[base + 0]);
    const long long nyacc = toCount(f[base + 1]);
    const long long xacc = toCount(f[base + 2]);
    const long long zgyro = toCount(f[base + 3]);
    const long long nygyro = toCount(f[base + 4]);
    const long long xgyro = toCount(f[base + 5]);

    s.ax = static_cast<int32_t>(xacc);
    s.ay = static_cast<int32_t>(-nyacc);
    s.az = static_cast<int32_t>(zacc);
    s.gx = static_cast<int32_t>(xgyro);
    s.gy = static_cast<int32_t>(-nygyro);
    s.gz = static_cast<int32_t>(zgyro);
    return true;
}
