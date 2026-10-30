#include "TroReader.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace {

    bool parseEpoch(const std::string &s, CommonTime &out) {
        std::vector<std::string> parts;
        std::string tok;
        for (const char c: s) {
            if (c == ':') { parts.push_back(tok); tok.clear(); }
            else tok.push_back(c);
        }
        parts.push_back(tok);
        if (parts.size() < 3) return false;
        try {
            const int year = std::stoi(parts[0]);
            const int doy = std::stoi(parts[1]);
            const double sod = std::stod(parts[2]);

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
        MJD m;
        CommonTime2MJD(t, m);
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

bool TroReader::hasStation(const std::string &station) const {
    return findStation(station) != nullptr;
}

const std::vector<TroReader::Record> *TroReader::findStation(const std::string &station) const {
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

        ztd = recs->back().ztd;
        sigma = recs->back().ztdStd;
        return true;
    }
    const Record &hi = *it;
    const Record &lo = *(it - 1);
    const double span = hi.mjd - lo.mjd;
    const double f = span > 0.0 ? (mjd - lo.mjd) / span : 0.0;
    ztd = lo.ztd + (hi.ztd - lo.ztd) * f;
    sigma = lo.ztdStd + (hi.ztdStd - lo.ztdStd) * f;
    return true;
}

bool TroReader::getTropoGrad(const std::string &station, const CommonTime &t,
                             double &gradN, double &gradE,
                             double &gradNStd, double &gradEStd) const {
    const std::vector<Record> *recs = findStation(station);
    if (recs == nullptr || recs->empty()) return false;

    bool anyGrad = false;
    for (const auto &r : *recs)
        if (r.gradN != 0.0 || r.gradE != 0.0) { anyGrad = true; break; }
    if (!anyGrad) return false;

    MJD m;
    CommonTime2MJD(t, m);
    const double mjd = static_cast<double>(m.mjd);
    const auto it = std::lower_bound(recs->begin(), recs->end(), mjd,
                                     [](const Record &r, const double v) { return r.mjd < v; });
    if (it == recs->begin()) {
        gradN = it->gradN; gradE = it->gradE;
        gradNStd = it->gradNStd; gradEStd = it->gradEStd;
        return true;
    }
    if (it == recs->end()) {
        gradN = recs->back().gradN; gradE = recs->back().gradE;
        gradNStd = recs->back().gradNStd; gradEStd = recs->back().gradEStd;
        return true;
    }
    const Record &hi = *it, &lo = *(it - 1);
    const double span = hi.mjd - lo.mjd;
    const double f = span > 0.0 ? (mjd - lo.mjd) / span : 0.0;
    gradN = lo.gradN + (hi.gradN - lo.gradN) * f;
    gradE = lo.gradE + (hi.gradE - lo.gradE) * f;
    gradNStd = lo.gradNStd + (hi.gradNStd - lo.gradNStd) * f;
    gradEStd = lo.gradEStd + (hi.gradEStd - lo.gradEStd) * f;
    return true;
}
