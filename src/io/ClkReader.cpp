#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "ClkReader.h"
#include "MathUtils.h"
#include <fstream>
#include <algorithm>

bool ClkReader::read(const std::string &path) {
    return read(path, {}); // 空集 = 不按系统过滤
}

bool ClkReader::read(const std::string &path, const std::set<char> &allowedSys) {
    std::ifstream in(path);
    if (!in) return false;

    // 加大流缓冲，减少逐行读取的系统调用次数
    std::vector<char> rbuf(1 << 20); // 1 MB
    in.rdbuf()->pubsetbuf(rbuf.data(), static_cast<std::streamsize>(rbuf.size()));

    std::string line;
    bool headerEnd = false;
    std::map<SatID, std::vector<Record> > tmp;
    line.reserve(128);
    while (std::getline(in, line)) {
        if (!headerEnd) {
            if (line.find("END OF HEADER") != std::string::npos) headerEnd = true;
            continue;
        }
        if (line.size() < 3) continue;
        if (line[0] != 'A') continue;
        char type[3] = {}, satStr[4] = {};
        int y, mo, d, h, mi, nfields;
        double s, bias, drift;
        int n = sscanf(line.c_str(), "%2s %3s %d %d %d %d %d %lf %d %lf %lf", //NOLINT
                       type, satStr, &y, &mo, &d, &h, &mi, &s, &nfields, &bias, &drift);
        if (n < 11) continue;
        if (type[0] != 'A' || type[1] != 'S') continue; // 只要卫星钟差
        SatID sat(satStr[0], (satStr[1] - '0') * 10 + (satStr[2] - '0'));
        if (!allowedSys.empty() && !allowedSys.count(sat.system)) continue;
        CivilTime ct(y, mo, d, h, mi, s, TimeSystem::GPS);
        Record rec{CivilTime2CommonTime(ct), bias, drift};
        tmp[sat].push_back(rec);
    }

    if (tmp.empty()) return false;
    // 预计算每卫星相对首历元的秒数向量(查询时复用，避免每调用 O(n) 拷贝)
    relTimes.clear();
    for (const auto &[sat, recs] : tmp) {
        std::vector<double> &rt = relTimes[sat];
        rt.reserve(recs.size());
        if (!recs.empty()) {
            const CommonTime t0 = recs[0].t;
            for (const auto &r : recs) rt.push_back(r.t - t0);
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
    const std::vector<double> &times = (itT != relTimes.end()) ? itT->second : std::vector<double>{};
    const double tRel = t - r[0].t;
    const int lb = Math::lowerBoundIndex(times, tRel);      // times[lb] <= tRel < times[lb+1]
    const double span = r[lb + 1].t - r[lb].t;              // 两点间隔秒
    const double frac = (t - r[lb].t) / span;               // 相对左点比例 [0,1]
    return Math::lerp(r[lb].bias, r[lb + 1].bias, frac);
}
