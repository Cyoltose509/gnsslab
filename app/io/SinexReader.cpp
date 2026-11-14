#include "SinexReader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <vector>
#include <sstream>
#include <filesystem>

#include "download/local/ProductMeta.h"
#include "download/local/Unpack.h"
#include "core/Const.h"
#include "time/TimeStruct.h"
#include "StringUtils.h"

namespace {
    /// 日解/周解的站坐标只在该时段附近有效，跨期不能复用。
    constexpr double kMaxSpanDays = 8.0;

    int fullYear(const int y) {
        if (y < 80) return y + 2000;
        if (y < 100) return y + 1900;
        return y;
    }
}

bool SinexReader::fail(const std::string &msg, std::string &err) {
    err = msg;
    return false;
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
                curRefYear = fullYear(static_cast<int>(v[0]));
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

        int refYear = 0, refDoy = 0;
        if (tk.size() > 5) {
            double v[3]{0, 0, 0};
            if (Str::scanDoubles(tk[5].c_str(), v, 3) >= 2) {
                refYear = fullYear(static_cast<int>(v[0]));
                refDoy = static_cast<int>(v[1]);
                if (refDoy < 1 || refDoy > 366) refYear = refDoy = 0;
            }
        }
        if (refYear == 0 && curRefYear > 0) {
            refYear = curRefYear;
            refDoy = curRefDoy;
        }

        auto &group = stations[code];
        auto it = std::find_if(group.begin(), group.end(), [&](const Station &s) {
            return s.refYear == refYear && s.refDoy == refDoy;
        });
        if (it == group.end()) {
            group.push_back(Station{});
            it = group.end() - 1;
            it->refYear = refYear;
            it->refDoy = refDoy;
        }
        Station &s = *it;
        if (type == "STAX") s.x = val;
        else if (type == "STAY") s.y = val;
        else if (type == "STAZ") s.z = val;
        else if (type == "VX") {
            s.vx = val;
            s.hasVel = true;
            s.velRefYear = refYear;
            s.velRefDoy = refDoy;
        } else if (type == "VY") s.vy = val;
        else if (type == "VZ") s.vz = val;
    }
}

bool SinexReader::readFile(const std::string &path, std::string &err) {
    stations.clear();
    const std::string text = Download::Unpack::readPlain(path);
    if (text.empty()) return fail("无法读取 SINEX 文件或文件为空: " + path, err);
    parseEstimate(text);
    if (stations.empty()) return fail("SINEX 中未解析到任何站点坐标: " + path, err);
    err.clear();
    return true;
}

bool SinexReader::readString(const std::string &text, std::string &err) {
    stations.clear();
    if (text.empty()) return fail("SINEX 文本为空", err);
    parseEstimate(text);
    if (stations.empty()) return fail("SINEX 文本中未解析到任何站点坐标", err);
    err.clear();
    return true;
}

bool SinexReader::hasStation(const std::string &name) const {
    const auto it = stations.find(Str::toUpper(name));
    return it != stations.end() && !it->second.empty();
}

bool SinexReader::hasSolution(const std::string &name, const int refYear, const int refDoy) const {
    if (refYear <= 0 || refDoy <= 0) return false;
    const auto it = stations.find(Str::toUpper(name));
    if (it == stations.end()) return false;
    for (const Station &s: it->second)
        if (s.refYear == refYear && s.refDoy == refDoy) return true;
    return false;
}

bool SinexReader::getStation(const std::string &name, const int targetYear, const int targetDoy,
                             Eigen::Vector3d &out, int *usedYear, int *usedDoy) const {
    const auto it = stations.find(Str::toUpper(name));
    if (it == stations.end() || it->second.empty()) return false;

    const CommonTime target = (YDSTime(targetYear, targetDoy, 0.0, TimeSystem::GPS)).toCommonTime();
    const Station *best = nullptr;
    const Station *undated = nullptr;
    double bestDays = std::numeric_limits<double>::infinity();
    for (const Station &s: it->second) {
        if (s.refYear == 0) {
            if (!undated) undated = &s;
            continue;
        }
        const CommonTime ref = (YDSTime(s.refYear, s.refDoy, 0.0, TimeSystem::GPS)).toCommonTime();
        const double days = std::abs((target - ref) / static_cast<double>(SEC_PER_DAY));
        if (days < bestDays) {
            bestDays = days;
            best = &s;
        }
    }
    if (!best) best = undated;
    else if (bestDays > kMaxSpanDays) return false;
    if (!best) return false;

    double x = best->x, y = best->y, z = best->z;
    if (best->hasVel && best->velRefYear > 0) {
        const CommonTime velRef = (YDSTime(best->velRefYear, best->velRefDoy, 0.0, TimeSystem::GPS)).toCommonTime();
        const double dtYears = (target - velRef) / static_cast<double>(SEC_PER_DAY) / 365.25;
        x += best->vx * dtYears;
        y += best->vy * dtYears;
        z += best->vz * dtYears;
    }
    out = Eigen::Vector3d(x, y, z);
    if (usedYear) *usedYear = best->refYear;
    if (usedDoy) *usedDoy = best->refDoy;
    return true;
}

bool SinexReader::obsStationAndDOY(const std::string &obsPath, std::string &station,
                                   int &year, int &doy) {
    station.clear();
    year = 0;
    doy = 0;
    if (obsPath.empty()) return false;
    const std::string base = std::filesystem::path(obsPath).stem().string();
    if (base.size() < 4) return false; // 文件名过短 → 无站点信息
    station = Str::toUpper(base.substr(0, 4));

    // 日期只认 ProductMeta::parseObsDate（名字 → 路径 → 文件头 → 库目录四层回落，认 .gz）
    if (const auto d = ProductMeta::parseObsDate(obsPath)) {
        year = d->year;
        doy = d->doy;
    }
    return true;
}
