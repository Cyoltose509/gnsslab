#include "IonexReader.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include "MathUtils.h"

namespace {
    constexpr int IONEX_FIELD_WIDTH = 5; // IONEX 数值定宽 5 字符

    // 从定宽 5 字符字段解析数值
    bool parseFixedField(const std::string &line, const int idx, double &out) {
        const size_t pos = static_cast<size_t>(idx) * IONEX_FIELD_WIDTH;
        if (pos >= line.size()) return false;
        const std::string fld = line.substr(pos, IONEX_FIELD_WIDTH);
        try {
            out = std::stod(fld);
        } catch (...) {
            return false;
        }
        return true;
    }
}

bool IonexReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    maps.clear();
    hgtKm = 450.0;
    exponent = -1.0;

    std::string line;
    bool inMap = false;
    TecMap cur;
    std::vector<double> row;
    int nLon = 0;

    while (std::getline(ifs, line)) {
        // —— 头段关键字 ——
        if (line.find("EXPONENT") != std::string::npos) {
            if (std::istringstream iss(line.substr(0, 20)); iss >> exponent) { /* ok */ }
            continue;
        }
        if (line.find("HGT1 / HGT2 / DHGT") != std::string::npos) {
            double h1 = 450.0, h2 = 450.0, dh = 0.0;
            if (std::istringstream iss(line.substr(0, 20)); iss >> h1 >> h2 >> dh) hgtKm = h1;
            continue;
        }
        if (line.find("START OF TEC MAP") != std::string::npos) {
            inMap = true;
            cur = TecMap();
            cur.hgt = hgtKm;
            row.clear();
            nLon = 0;
            continue;
        }
        if (line.find("END OF TEC MAP") != std::string::npos) {
            if (inMap && !cur.lats.empty()) {
                if (!row.empty()) { cur.tec.push_back(row); row.clear(); }
                if (cur.tec.size() == cur.lats.size()) maps.push_back(cur);
            }
            inMap = false;
            continue;
        }
        if (!inMap) continue;

        if (line.find("EPOCH OF CURRENT MAP") != std::string::npos) {
            double ss = 0.0;
            std::istringstream iss(line.substr(0, 40));
            if (int y = 0, mo = 0, d = 0, hh = 0, mm = 0;iss >> y >> mo >> d >> hh >> mm >> ss) {
                CommonTime t;
                const int doy = [] (const int yy, const int mm2, const int dd) {
                    static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
                    int d2 = cum[mm2 - 1] + dd;
                    if (const bool leap = (yy % 4 == 0 && yy % 100 != 0) || yy % 400 == 0; leap && mm2 > 2) d2 += 1;
                    return d2;
                }(y, mo, d);
                t = YDSTime2CommonTime(YDSTime(y, doy, hh * 3600.0 + mm * 60.0 + ss, TimeSystem::GPS));
                cur.t = t;
                MJD m;
                CommonTime2MJD(t, m);
                cur.mjd = static_cast<double>(m.mjd);
            }
            continue;
        }
        if (line.find("LAT/LON1/LON2/DLON/H") != std::string::npos) {
            // 新纬度行：先把上一行收尾
            if (!row.empty()) { cur.tec.push_back(row); row.clear(); }
            double lat = 0.0, l1 = -180.0, l2 = 180.0, dl = 5.0, h = hgtKm;
            if (std::istringstream iss(line.substr(0, 30)); !(iss >> lat >> l1 >> l2 >> dl >> h)) continue;
            cur.lats.push_back(lat);
            cur.lon1 = l1;
            cur.lon2 = l2;
            cur.dlon = dl;
            cur.hgt = h;
            nLon = dl > 0.0 ? static_cast<int>(std::floor((l2 - l1) / dl + 1.0e-6)) + 1 : 1;
            continue;
        }

        // —— 数据行：定宽 5 字符，每行最多 16 个 ——
        const int nFields = static_cast<int>(line.size()) / IONEX_FIELD_WIDTH;
        for (int i = 0; i < nFields && static_cast<int>(row.size()) < nLon; ++i) {
            double v = 0.0;
            if (!parseFixedField(line, i, v)) break;
            row.push_back(v * std::pow(10.0, exponent));
        }
    }

    if (maps.empty()) return false;
    std::sort(maps.begin(), maps.end(),
              [](const TecMap &a, const TecMap &b) { return a.mjd < b.mjd; });
    mjds.clear();
    mjds.reserve(maps.size());
    for (const auto &m : maps) mjds.push_back(m.mjd);
    return true;
}

double IonexReader::interpolateMap(const TecMap &mp, const double latDeg, const double lonDeg) {
    if (mp.tec.empty() || mp.lats.empty() || mp.tec[0].empty()) return 0.0;

    const int nLat = static_cast<int>(mp.lats.size());
    const int nLon = static_cast<int>(mp.tec[0].size());

    // 经度：单调递增（lon1 -> lon2）
    const double dlon = mp.dlon > 0.0 ? mp.dlon : 5.0;
    double fx = (lonDeg - mp.lon1) / dlon;
    fx = std::fmod(fx + static_cast<double>(nLon) * 2.0, static_cast<double>(nLon)); // 环绕
    const int i0 = static_cast<int>(std::floor(fx));
    const double tx = fx - i0;
    const int i1 = (i0 + 1) % nLon;

    // 纬度：文件通常由 +87.5 递减到 -87.5
    const bool descending = nLat > 1 && mp.lats[1] < mp.lats[0];
    double fy;
    if (descending) {
        const double dlat = mp.lats[0] - mp.lats[1];
        fy = dlat > 0.0 ? (mp.lats[0] - latDeg) / dlat : 0.0;
    } else {
        const double dlat = nLat > 1 ? mp.lats[1] - mp.lats[0] : 1.0;
        fy = dlat > 0.0 ? (latDeg - mp.lats[0]) / dlat : 0.0;
    }
    fy = std::max(0.0, std::min(static_cast<double>(nLat - 1), fy));
    const int j0 = static_cast<int>(std::floor(fy));
    const double ty = fy - j0;
    const int j1 = std::min(j0 + 1, nLat - 1);

    const double v00 = mp.tec[j0][i0], v01 = mp.tec[j0][i1];
    const double v10 = mp.tec[j1][i0], v11 = mp.tec[j1][i1];
    const double v0 = Math::lerp(v00, v01, tx);
    const double v1 = Math::lerp(v10, v11, tx);
    return Math::lerp(v0, v1, ty);
}

bool IonexReader::getTec(const CommonTime &t, const double latDeg, const double lonDeg,
                         double &tec) const {
    if (maps.empty()) return false;

    MJD m;
    CommonTime2MJD(t, m);
    const auto mjd = static_cast<double>(m.mjd);

    const int idx = Math::lowerBoundIndex(mjds, mjd); // 最后一个 mjd <= t 的索引(已边界 clamp)
    if (idx >= static_cast<int>(maps.size()) - 1) {
        tec = interpolateMap(maps.back(), latDeg, lonDeg);
        return true;
    }
    if (idx == 0 && mjd <= mjds.front()) { // t 早于首图：clamp 到首图(不时间插值)
        tec = interpolateMap(maps.front(), latDeg, lonDeg);
        return true;
    }
    const TecMap &hi = maps[idx + 1];
    const TecMap &lo = maps[idx];
    const double span = hi.mjd - lo.mjd;
    const double f = span > 0.0 ? (mjd - lo.mjd) / span : 0.0;
    const double vLo = interpolateMap(lo, latDeg, lonDeg);
    const double vHi = interpolateMap(hi, latDeg, lonDeg);
    tec = Math::lerp(vLo, vHi, f);
    return true;
}
