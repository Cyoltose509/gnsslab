#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "ClkReader.h"
#include "MathUtils.h"
#include <algorithm>
#include <charconv>
#include <StringUtils.h>


bool ClkReader::read(const std::string &path) {
    return read(path, {});
}

bool ClkReader::read(const std::string &path, const std::set<char> &allowedSys) {
    bool headerEnd = false;
    std::map<SatID, std::vector<Record> > tmp;


    try {
        openFile(path);
    } catch (const FFStreamError &) {
        return false;
    }

    std::string line;
    while (nextLine(line)) {
        if (!headerEnd) {
            if (line.find("END OF HEADER") != std::string::npos) headerEnd = true;
            continue;
        }
        std::string_view sv(line);
        if (sv.size() < 4) continue;
        if (sv[0] != 'A' || sv[1] != 'S') continue;
        if (!allowedSys.empty() && !allowedSys.count(sv[3])) continue;
        const char *p = sv.data() + 2;
        const char *const qe = sv.data() + sv.size();

        while (p < qe && *p == ' ') ++p;
        if (p >= qe) continue;
        const char sys = *p++;
        int prn = 0;
        while (p < qe && *p >= '0' && *p <= '9') { prn = prn * 10 + (*p - '0'); ++p; }
        const SatID sat(sys, prn);

        auto nextInt = [&](int &out) -> bool {
            while (p < qe && *p == ' ') ++p;
            const auto [ptr, ec] = std::from_chars(p, qe, out);
            if (ec != std::errc()) return false;
            p = ptr; return true;
        };
        auto nextDbl = [&](double &out) -> bool {
            while (p < qe && *p == ' ') ++p;
            const auto [ptr, ec] = std::from_chars(p, qe, out);
            if (ec != std::errc()) return false;
            p = ptr; return true;
        };

        int y, mo, d, h, mi, nfields;
        double s, bias, drift = 0.0;
        if (!nextInt(y) || !nextInt(mo) || !nextInt(d) || !nextInt(h) ||
            !nextInt(mi) || !nextDbl(s) || !nextInt(nfields) || !nextDbl(bias))
            continue;
        nextDbl(drift);

        const CivilTime ct(y, mo, d, h, mi, s, TimeSystem::GPS);
        const Record rec{CivilTime2CommonTime(ct), bias, drift};
        tmp[sat].push_back(rec);
    }
    if (tmp.empty()) return false;

    relTimes.clear();
    for (const auto &[sat, recs]: tmp) {
        std::vector<double> &rt = relTimes[sat];
        rt.reserve(recs.size());
        if (!recs.empty()) {
            const CommonTime t0 = recs[0].t;
            for (const auto &r: recs) rt.push_back(r.t - t0);
        }
    }
    data = std::move(tmp);
    return true;
}

double ClkReader::getClockBias(const SatID &sat, const CommonTime &t) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty()) return 0.0;
    const auto &r = it->second;
    const int n = static_cast<int>(r.size());
    if (n == 1) return r[0].bias;

    if (t - r[0].t <= 0.0) return r[0].bias;
    if (t - r[n - 1].t >= 0.0) return r[n - 1].bias;
    const auto itT = relTimes.find(sat);
    const std::vector<double> &times = itT != relTimes.end() ? itT->second : std::vector<double>{};
    const double tRel = t - r[0].t;
    const int lb = Math::lowerBoundIndex(times, tRel);
    const double span = r[lb + 1].t - r[lb].t;
    const double frac = (t - r[lb].t) / span;
    return Math::lerp(r[lb].bias, r[lb + 1].bias, frac);
}

bool ClkReader::covers(const SatID &sat, const CommonTime &t, const double maxGap) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty()) return false;
    const auto &r = it->second;
    const auto itT = relTimes.find(sat);
    if (itT == relTimes.end()) return false;
    const std::vector<double> &times = itT->second;
    const double tRel = t - r.front().t;
    if (tRel < -1e-6) return false;
    const int n = static_cast<int>(times.size());
    const int lb = Math::lowerBoundIndex(times, tRel);
    if (lb >= n) return false;
    if (tRel - times[lb] > maxGap) return false;
    if (lb + 1 < n && times[lb + 1] - tRel > maxGap) return false;
    return true;
}
