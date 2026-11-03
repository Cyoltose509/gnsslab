#include "product/TROReader.h"

#include "MathUtils.h"
#include "StringUtils.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace {

    // 历元格式 "yyyy:doy:sod"，如 "2010:001:0"。
    bool parseEpoch(const std::string &s, CommonTime &out) {
        const std::vector<std::string> parts = Str::split(s, ':'); // 空段保留：缺字段应当判失败
        if (parts.size() < 3) return false;
        try {
            const int year = std::stoi(parts[0]);
            const int doy = std::stoi(parts[1]);
            const double sod = std::stod(parts[2]);

            out = Time::YDSToCommon(YDSTime(year, doy, sod, TimeSystem::GPS));
        } catch (...) {
            return false;
        }
        return true;
    }
}

bool TROReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    data.clear();

    std::string line;
    bool inSolution = false;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line[0] == '+') {
            inSolution = line.find("TROP/SOLUTION") != std::string::npos;
            continue;
        }
        if (!line.empty() && line[0] == '-') {
            if (line.find("TROP/SOLUTION") != std::string::npos) inSolution = false;
            continue;
        }
        if (!inSolution) continue;
        if (line.empty() || line[0] == '*') continue;

        std::istringstream iss(line);
        std::string station, epoch;
        if (!(iss >> station >> epoch)) continue;
        double trotot = 0.0, troStd = 0.0;
        if (!(iss >> trotot >> troStd)) continue;

        double gn = 0.0, gnStd = 0.0, ge = 0.0, geStd = 0.0;
        iss >> gn >> gnStd >> ge >> geStd;

        CommonTime t;
        if (!parseEpoch(epoch, t)) continue;

        Record r;
        r.t = t;
        const MJD m = Time::CommonToMJD(t);
        r.mjd = static_cast<double>(m.mjd);
        constexpr double MM = 1.0e-3;
        r.ztd = trotot * MM;
        r.ztdStd = troStd * MM;
        r.gradN = gn * MM;
        r.gradE = ge * MM;
        r.gradNStd = gnStd * MM;
        r.gradEStd = geStd * MM;
        data[station].push_back(r);
    }


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

bool TROReader::hasStation(const std::string &station) const {
    return findStation(station) != nullptr;
}

const std::vector<TROReader::Record> *TROReader::findStation(const std::string &station) const {
    const auto it = data.find(station);
    if (it != data.end()) return &it->second;

    if (station.size() >= 4) {
        const std::string short4 = station.substr(0, 4);
        for (const auto &[name, recs]: data) {
            if (name.size() >= 4 && name.substr(0, 4) == short4) return &recs;
        }
    }
    return nullptr;
}

bool TROReader::getZTD(const std::string &station, const CommonTime &t,
                       double &ztd, double &sigma) const {
    const std::vector<Record> *recs = findStation(station);
    if (recs == nullptr) return false;

    const auto b = Math::bracketSample(*recs, static_cast<double>(Time::CommonToMJD(t).mjd),
                                       [](const Record &r) { return r.mjd; });
    if (!b) return false;

    const Record &lo = (*recs)[b->lo], &hi = (*recs)[b->hi];
    ztd = Math::lerp(lo.ztd, hi.ztd, b->f);
    sigma = Math::lerp(lo.ztdStd, hi.ztdStd, b->f);
    return true;
}

bool TROReader::getTropoGrad(const std::string &station, const CommonTime &t,
                             double &gradN, double &gradE,
                             double &gradNStd, double &gradEStd) const {
    const std::vector<Record> *recs = findStation(station);
    if (recs == nullptr || recs->empty()) return false;

    bool anyGrad = false;
    for (const auto &r : *recs)
        if (r.gradN != 0.0 || r.gradE != 0.0) { anyGrad = true; break; }
    if (!anyGrad) return false;

    const auto b = Math::bracketSample(*recs, static_cast<double>(Time::CommonToMJD(t).mjd),
                                       [](const Record &r) { return r.mjd; });
    if (!b) return false;

    const Record &lo = (*recs)[b->lo], &hi = (*recs)[b->hi];
    gradN = Math::lerp(lo.gradN, hi.gradN, b->f);
    gradE = Math::lerp(lo.gradE, hi.gradE, b->f);
    gradNStd = Math::lerp(lo.gradNStd, hi.gradNStd, b->f);
    gradEStd = Math::lerp(lo.gradEStd, hi.gradEStd, b->f);
    return true;
}
