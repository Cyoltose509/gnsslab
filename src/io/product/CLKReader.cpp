#include "product/CLKReader.h"
#include "MathUtils.h"
#include <algorithm>
#include <StringUtils.h>


bool CLKReader::read(const std::string &path) {
    return read(path, {});
}

bool CLKReader::read(const std::string &path, const std::set<char> &allowedSys) {
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
        while (p < qe && *p >= '0' && *p <= '9') {
            prn = prn * 10 + (*p - '0');
            ++p;
        }
        const SatID sat(sys, prn);

        int y, mo, d, h, mi, nfields;
        double s, bias, drift = 0.0;
        if (!Str::nextInt(p, qe, y) || !Str::nextInt(p, qe, mo) || !Str::nextInt(p, qe, d) ||
            !Str::nextInt(p, qe, h) || !Str::nextInt(p, qe, mi) || !Str::nextDouble(p, qe, s) ||
            !Str::nextInt(p, qe, nfields) || !Str::nextDouble(p, qe, bias))
            continue;
        (void) Str::nextDouble(p, qe, drift); // 漂移率可缺，缺了保持 0

        const CivilTime ct(y, mo, d, h, mi, s, TimeSystem::GPS);
        const Record rec{Time::CivilToCommon(ct), bias, drift};
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

double CLKReader::getClockBias(const SatID &sat, const CommonTime &t) const {
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
    // 重复历元会让 span 为 0，避免除零
    const double frac = span > 0.0 ? (t - r[lb].t) / span : 0.0;
    return Math::lerp(r[lb].bias, r[lb + 1].bias, frac);
}

bool CLKReader::covers(const SatID &sat, const CommonTime &t, const double maxGap) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty()) return false;
    const auto &r = it->second;
    const auto itT = relTimes.find(sat);
    if (itT == relTimes.end()) return false;
    const std::vector<double> &times = itT->second;
    const double tRel = t - r.front().t;
    // 文件首末样本外 maxGap 内仍算覆盖：getClockBias 夹到端点样本，钟漂移忽略不计
    if (tRel < -maxGap) return false;
    const int n = static_cast<int>(times.size());
    const int lb = Math::lowerBoundIndex(times, tRel);
    if (lb >= n) return tRel - times.back() <= maxGap;
    if (tRel - times[lb] > maxGap) return false;
    if (lb + 1 < n && times[lb + 1] - tRel > maxGap) return false;
    return true;
}
