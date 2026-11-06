#include "SinexReader.h"

#include <algorithm>
#include <cctype>
#include <vector>
#include <fstream>
#include <sstream>
#include <zlib.h>
#include <filesystem>

#include "core/Const.h"
#include "time/TimeConvert.h"
#include "util/StringUtils.h"

namespace {
    bool isGz(const std::string &path) {
        return path.size() >= 3 && Str::toLower(path.substr(path.size() - 3)) == ".gz";
    }
}

bool SinexReader::fail(const std::string &msg) {
    error = msg;
    return false;
}

std::string SinexReader::gunzipToString(const std::string &path) {
    gzFile in = gzopen(path.c_str(), "rb"); //NOLINT
    if (!in) return {};
    std::string out;
    char buf[65536];
    int n = 0;
    while ((n = gzread(in, buf, sizeof(buf))) > 0) out.append(buf, static_cast<std::size_t>(n));
    gzclose(in);
    return out;
}

std::string SinexReader::readFileToString(const std::string &path) {
    const std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void SinexReader::parseEstimate(const std::string &text) {
    std::istringstream ss(text);
    std::string line;
    bool inEstimate = false;
    int curRefYear = 0, curRefDoy = 0;

    while (std::getline(ss, line)) {
        if (line.find("+SOLUTION/ESTIMATE") != std::string::npos) {
            inEstimate = true;
            continue;
        }
        if (line.find("-SOLUTION/ESTIMATE") != std::string::npos) {
            inEstimate = false;
            continue;
        }
        if (!inEstimate) continue;
        if (line.empty()) continue;

        // 块级参考历元注释（部分 SINEX 有）：* ... ref epoch: YYYY:DDD:...
        if (const std::size_t rp = line.find("ref epoch:"); rp != std::string::npos) {
            double v[3]{0, 0, 0};
            if (Str::scanDoubles(line.c_str() + rp + 10, v, 3) >= 2) {
                curRefYear = static_cast<int>(v[0]);
                curRefDoy = static_cast<int>(v[1]);
            }
            continue;
        }

        std::istringstream ls(Str::strip(line));
        std::vector<std::string> tk;
        std::string w;
        while (ls >> w) tk.push_back(w);
        if (tk.size() < 9) continue;
        const std::string &type = tk[1];
        if (type != "STAX" && type != "STAY" && type != "STAZ" &&
            type != "VX" && type != "VY" && type != "VZ")
            continue;
        const std::string code = Str::toUpper(tk[2]);
        const double val = Str::toDouble(tk[8]);

        int lineRefYear = 0, lineRefDoy = 0;
        if (tk.size() > 5) {
            double v[3]{0, 0, 0};
            if (Str::scanDoubles(tk[5].c_str(), v, 3) >= 2) {
                lineRefYear = static_cast<int>(v[0]);
                lineRefDoy = static_cast<int>(v[1]);
                if (lineRefYear < 80) lineRefYear += 2000;
                else if (lineRefYear < 100) lineRefYear += 1900;
            }
        }

        auto &[x, y, z, vx, vy, vz, hasVel, velRefYear, velRefDoy] = stations[code];
        if (type == "STAX") x = val;
        else if (type == "STAY") y = val;
        else if (type == "STAZ") z = val;
        else if (type == "VX") {
            vx = val;
            hasVel = true;
            if (lineRefYear) {
                velRefYear = lineRefYear;
                velRefDoy = lineRefDoy;
            } else if (curRefYear) {
                velRefYear = curRefYear;
                velRefDoy = curRefDoy;
            }
        } else if (type == "VY") vy = val;
        else if (type == "VZ") vz = val;

        if (velRefYear == 0 && curRefYear > 0) {
            velRefYear = curRefYear;
            velRefDoy = curRefDoy;
        }
    }
}

bool SinexReader::readFile(const std::string &path, std::string &err) {
    const std::string text = isGz(path) ? gunzipToString(path) : readFileToString(path);
    if (text.empty()) return fail("无法读取 SINEX 文件或文件为空: " + path);
    parseEstimate(text);
    if (stations.empty()) return fail("SINEX 中未解析到任何站点坐标: " + path);
    err.clear();
    return true;
}

bool SinexReader::readString(const std::string &text, std::string &err) {
    if (text.empty()) return fail("SINEX 文本为空");
    parseEstimate(text);
    if (stations.empty()) return fail("SINEX 文本中未解析到任何站点坐标");
    err.clear();
    return true;
}

bool SinexReader::hasStation(const std::string &name) const {
    return stations.find(Str::toUpper(name)) != stations.end();
}

bool SinexReader::getStation(const std::string &name, const int targetYear, const int targetDoy,
                             Eigen::Vector3d &out) const {
    const auto it = stations.find(Str::toUpper(name));
    if (it == stations.end()) return false;
    const Station &s = it->second;
    double x = s.x, y = s.y, z = s.z;
    if (s.hasVel && s.velRefYear > 0) {
        const CommonTime velRef = Time::YDSToCommon(YDSTime(s.velRefYear, s.velRefDoy, 0.0, TimeSystem::GPS));
        const CommonTime obsCt = Time::YDSToCommon(YDSTime(targetYear, targetDoy, 0.0, TimeSystem::GPS));
        const double dtYears = (obsCt - velRef) / static_cast<double>(SEC_PER_DAY) / 365.25;
        x += s.vx * dtYears;
        y += s.vy * dtYears;
        z += s.vz * dtYears;
    }
    out = Eigen::Vector3d(x, y, z);
    return true;
}

bool SinexReader::obsStationAndDoy(const std::string &obsPath, std::string &station,
                                   int &year, int &doy) {
    station.clear();
    year = 0;
    doy = 0;
    if (obsPath.empty()) return false;
    const std::filesystem::path p(obsPath);
    const std::string base = p.stem().string();
    if (base.size() < 4) return false; // 文件名过短 → 无站点信息
    station = Str::toUpper(base.substr(0, 4));

    bool gotDate = false;
    Str::forEachLine(obsPath, [&](const std::string_view sv) {
        if (gotDate) return;
        const std::string L(sv);
        if (L.size() >= 60) {
            if (const std::string label = Str::strip(Str::sub(L, 60, 20)); label == "TIME OF FIRST OBS") {
                double v[6]{0, 0, 0, 0, 0, 0};
                if (const int cnt = Str::scanDoubles(L.c_str(), v, 6); cnt >= 3) {
                    int yr = static_cast<int>(v[0]);
                    const int mo = static_cast<int>(v[1]);
                    const int dy = static_cast<int>(v[2]);
                    if (yr >= 0 && yr < 80) yr += 2000;
                    else if (yr < 100) yr += 1900;
                    if (mo >= 1 && mo <= 12 && dy >= 1 && dy <= 31) {
                        doy = Time::YMDToDOY(YearMonthDay{yr, mo, dy});
                        year = yr;
                        gotDate = true;
                    }
                }
            }
        }
        if (L.find("END OF HEADER") != std::string::npos) gotDate = true; // 没读到就停
    });

    if (!gotDate) {
        // 回退：RINEX3 文件名里的 YYYYDDD 段
        for (const auto &t: Str::split(base, '_', false)) {
            if (t.size() >= 7 && std::all_of(t.begin(), t.end(),
                                             [](const unsigned char c) { return std::isdigit(c); })) {
                year = std::stoi(t.substr(0, 4));
                doy = std::stoi(t.substr(4, 3));
                gotDate = true;
                break;
            }
        }
        // 回退：RINEX2 的 ssssDDD0.yyO
        if (!gotDate && base.size() >= 7 &&
            std::isdigit(static_cast<unsigned char>(base[4])) != 0 &&
            std::isdigit(static_cast<unsigned char>(base[5])) != 0 &&
            std::isdigit(static_cast<unsigned char>(base[6])) != 0) {
            doy = std::stoi(base.substr(4, 3));
            std::string digits;
            for (const char c: p.extension().string())
                if (std::isdigit(static_cast<unsigned char>(c))) digits.push_back(c);
            if (digits.size() >= 2) {
                const int yy = std::stoi(digits.substr(0, 2));
                year = yy < 80 ? 2000 + yy : 1900 + yy;
                gotDate = true;
            }
        }
    }

    return !station.empty();
}
