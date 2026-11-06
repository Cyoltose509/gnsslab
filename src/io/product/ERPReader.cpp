#include "product/ERPReader.h"
#include "Const.h"
#include "MathUtils.h"

#include <algorithm>
#include <fstream>
#include <sstream>

bool ERPReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    data.clear();

    std::string line;
    bool inData = false;
    while (std::getline(ifs, line)) {

        if (!inData) {
            // IGS 的 ERP 表头写 Xpole/Ypole，IGS 组合产品（COD/GRG）写 X-P/Y-P
            if (line.find("MJD") != std::string::npos &&
                (line.find("Xpole") != std::string::npos || line.find("X-P") != std::string::npos)) {
                inData = true;
            }
            continue;
        }

        std::istringstream iss(line);
        double mjd = 0.0, xp = 0.0, yp = 0.0, ut1 = 0.0, lod = 0.0;
        if (!(iss >> mjd >> xp >> yp >> ut1 >> lod)) continue;
        if (mjd <= 0.0) continue;

        Record r;
        r.mjd = mjd;
        r.xp = xp * 1.0e-6 * ARCSEC_TO_RAD;
        r.yp = yp * 1.0e-6 * ARCSEC_TO_RAD;
        r.ut1Utc = ut1 * 1.0e-7;
        r.lod = lod * 1.0e-7;
        data.push_back(r);
    }

    if (data.empty()) return false;

    std::sort(data.begin(), data.end(),
              [](const Record &a, const Record &b) { return a.mjd < b.mjd; });
    return true;
}

bool ERPReader::getERP(const CommonTime &t, double &xp, double &yp, double &ut1Utc) const {
    const auto b = Math::bracketSample(data, static_cast<double>(Time::CommonToMJD(t).mjd),
                                       [](const Record &r) { return r.mjd; });
    if (!b) return false;

    const Record &lo = data[b->lo], &hi = data[b->hi];
    xp = Math::lerp(lo.xp, hi.xp, b->f);
    yp = Math::lerp(lo.yp, hi.yp, b->f);
    ut1Utc = Math::lerp(lo.ut1Utc, hi.ut1Utc, b->f);
    return true;
}
