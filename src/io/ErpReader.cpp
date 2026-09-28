#include "ErpReader.h"
#include "Const.h"

#include <algorithm>
#include <fstream>
#include <sstream>

bool ErpReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    data.clear();

    std::string line;
    bool inData = false;
    while (std::getline(ifs, line)) {
        // 列头行同时含 "MJD" 与 "Xpole"，其后才是数据行
        if (!inData) {
            if (line.find("MJD") != std::string::npos &&
                line.find("Xpole") != std::string::npos) {
                inData = true;
            }
            continue;
        }
        // NRT 产品末行可能只有 MJD+Xpole+Ypole+UT1-UTC+LOD 五列，故只取前 5 个
        std::istringstream iss(line);
        double mjd = 0.0, xp = 0.0, yp = 0.0, ut1 = 0.0, lod = 0.0;
        if (!(iss >> mjd >> xp >> yp >> ut1 >> lod)) continue;
        if (mjd <= 0.0) continue;

        Record r;
        r.mjd = mjd;
        r.xp = xp * 1.0e-6 * ARCSEC_TO_RAD; // 10^-6 角秒 -> rad
        r.yp = yp * 1.0e-6 * ARCSEC_TO_RAD;
        r.ut1Utc = ut1 * 1.0e-7;            // 0.1 微秒 -> s
        r.lod = lod * 1.0e-7;
        data.push_back(r);
    }

    if (data.empty()) return false;

    std::sort(data.begin(), data.end(),
              [](const Record &a, const Record &b) { return a.mjd < b.mjd; });
    return true;
}

bool ErpReader::getErp(const CommonTime &t, double &xp, double &yp, double &ut1Utc) const {
    if (data.empty()) return false;

    MJD m;
    CommonTime2MJD(t, m);
    const double mjd = static_cast<double>(m.mjd);

    const auto it = std::lower_bound(
        data.begin(), data.end(), mjd,
        [](const Record &r, const double v) { return r.mjd < v; });

    if (it == data.begin()) {
        xp = it->xp;
        yp = it->yp;
        ut1Utc = it->ut1Utc;
        return true;
    }
    if (it == data.end()) {
        const Record &last = data.back();
        xp = last.xp;
        yp = last.yp;
        ut1Utc = last.ut1Utc;
        return true;
    }

    const Record &hi = *it;
    const Record &lo = *(it - 1);
    const double span = hi.mjd - lo.mjd;
    const double f = span > 0.0 ? (mjd - lo.mjd) / span : 0.0;

    xp = lo.xp + (hi.xp - lo.xp) * f;
    yp = lo.yp + (hi.yp - lo.yp) * f;
    ut1Utc = lo.ut1Utc + (hi.ut1Utc - lo.ut1Utc) * f;
    return true;
}
