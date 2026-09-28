#include "TroReader.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace {
    // 解析 "2026:183:00000" (年:年积日:日秒) -> CommonTime
    bool parseEpoch(const std::string &s, CommonTime &out) {
        std::vector<std::string> parts;
        std::string tok;
        for (char c: s) {
            if (c == ':') { parts.push_back(tok); tok.clear(); }
            else tok.push_back(c);
        }
        parts.push_back(tok);
        if (parts.size() < 3) return false;
        try {
            const int year = std::stoi(parts[0]);
            const int doy = std::stoi(parts[1]);
            const double sod = std::stod(parts[2]);
            // TRO 产品头部声明 TIME SYSTEM G(=GPS)，与观测历元时系统一致
            out = YDSTime2CommonTime(YDSTime(year, doy, sod, TimeSystem::GPS));
        } catch (...) {
            return false;
        }
        return true;
    }
}

bool TroReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    data.clear();

    std::string line;
    bool inSolution = false;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line[0] == '+') {
            inSolution = (line.find("TROP/SOLUTION") != std::string::npos);
            continue;
        }
        if (!line.empty() && line[0] == '-') {
            if (line.find("TROP/SOLUTION") != std::string::npos) inSolution = false;
            continue;
        }
        if (!inSolution) continue;
        if (line.empty() || line[0] == '*') continue; // 注释行

        std::istringstream iss(line);
        std::string station, epoch;
        if (!(iss >> station >> epoch)) continue;
        double trotot = 0.0, troStd = 0.0;
        if (!(iss >> trotot >> troStd)) continue;            // 至少要有 TROTOT 与其 σ

        CommonTime t;
        if (!parseEpoch(epoch, t)) continue;

        Record r;
        r.t = t;
        MJD m;
        CommonTime2MJD(t, m);
        r.mjd = static_cast<double>(m.mjd);
        constexpr double MM = 1.0e-3; // 文件单位 mm -> m
        r.ztd = trotot * MM;
        r.ztdStd = troStd * MM;
        data[station].push_back(r);
    }

    // 丢弃空测站并按时间排序
    for (auto it = data.begin(); it != data.end();) {
        if (it->second.empty()) {
            it = data.erase(it);
        } else {
            std::sort(it->second.begin(), it->second.end(),
                      [](const Record &a, const Record &b) { return a.mjd < b.mjd; });
            ++it;
        }
    }
    return !data.empty();
}

bool TroReader::hasStation(const std::string &station) const {
    return findStation(station) != nullptr;
}

const std::vector<TroReader::Record> *TroReader::findStation(const std::string &station) const {
    auto it = data.find(station);
    if (it != data.end()) return &it->second;
    // 退化匹配：按 4 字符短名（HKWS00HKG ↔ HKWS）
    if (station.size() >= 4) {
        const std::string short4 = station.substr(0, 4);
        for (const auto &[name, recs]: data) {
            if (name.size() >= 4 && name.substr(0, 4) == short4) return &recs;
        }
    }
    return nullptr;
}

bool TroReader::getZtd(const std::string &station, const CommonTime &t,
                       double &ztd, double &sigma) const {
    const std::vector<Record> *recs = findStation(station);
    if (recs == nullptr || recs->empty()) return false;

    MJD m;
    CommonTime2MJD(t, m);
    const double mjd = static_cast<double>(m.mjd);

    const auto it = std::lower_bound(recs->begin(), recs->end(), mjd,
                                     [](const Record &r, const double v) { return r.mjd < v; });
    if (it == recs->begin()) {
        ztd = it->ztd;
        sigma = it->ztdStd;
        return true;
    }
    if (it == recs->end()) {
        // 超出表范围：按最近端点取常值（ZTD 缓变，外推 1h 误差远小于约束 σ）
        ztd = recs->back().ztd;
        sigma = recs->back().ztdStd;
        return true;
    }
    const Record &hi = *it;
    const Record &lo = *(it - 1);
    const double span = hi.mjd - lo.mjd;
    const double f = (span > 0.0) ? (mjd - lo.mjd) / span : 0.0;
    ztd = lo.ztd + (hi.ztd - lo.ztd) * f;
    sigma = lo.ztdStd + (hi.ztdStd - lo.ztdStd) * f;
    return true;
}
