#include "product/METReader.h"
#include "MathUtils.h"

#include <algorithm>
#include <sstream>

namespace {
    void splitObs(const std::string &line, const int want, std::vector<double> &out) {
        out.clear();
        if (want <= 0) return;
        for (int i = 0; i < want; ++i) {
            const std::size_t b = static_cast<std::size_t>(i) * 7;
            if (b >= line.size()) break;
            const std::string f = line.substr(b, 7);
            try {
                out.push_back(std::stod(f));
            } catch (...) { break; }
        }
        if (static_cast<int>(out.size()) != want) {
            out.clear();
            std::istringstream is(line);
            double v = 0.0;
            while (is >> v) out.push_back(v);
        }
    }
}

bool METReader::read(const std::string &path) {
    recs.clear();

    int nObs = 0;
    std::vector<std::string> codes;

    try {
        openFile(path);
        scanLabeledHeader([&](const std::string &label, const std::string &line) {
            if (label != "# / TYPES OF OBSERV") return;
            std::istringstream is(line);
            if (!(is >> nObs)) return;
            std::string c;
            while (static_cast<int>(codes.size()) < nObs && is >> c) codes.push_back(c);
        });
    } catch (const FFStreamError &) {
        return false;
    }

    bool haveEpoch = false;
    int y = 0, mo = 0, d = 0, hh = 0, mi = 0;
    double sec = 0.0;
    int remaining = 0;
    std::vector<double> vals;

    std::string line;
    while (nextLine(line)) {
        if (line.empty()) continue;
        if (line[0] == '>') {
            // > YYYY MM DD HH MM SS.fffffff  flag  nObs
            std::istringstream is(line.substr(1));
            int flag = 0, n = 0;
            if (!(is >> y >> mo >> d >> hh >> mi >> sec)) { haveEpoch = false; continue; }
            is >> flag >> n;
            if (flag != 0 || n <= 0) { haveEpoch = false; continue; } // 只收正常历元
            haveEpoch = true;
            remaining = n;
            vals.clear();
            continue;
        }

        if (!haveEpoch || remaining <= 0) continue;
        std::vector<double> chunk;
        splitObs(line, remaining, chunk);
        if (chunk.empty()) continue;
        for (const double v: chunk) vals.push_back(v);
        remaining -= static_cast<int>(chunk.size());

        if (remaining > 0) continue;

        Record r;
        const CommonTime t = Time::CivilToCommon(CivilTime(y, mo, d, hh, mi, sec, TimeSystem::GPS));
        const MJD mj = Time::CommonToMJD(t);
        r.mjd = static_cast<double>(mj.mjd);

        for (std::size_t i = 0; i < codes.size() && i < vals.size(); ++i) {
            if (const std::string &c = codes[i]; c == "PR") r.presHpa = vals[i];
            else if (c == "TD") r.tempC = vals[i];
            else if (c == "HR") r.humi = vals[i] / 100.0;
        }
        if (r.presHpa > 0.0) recs.push_back(r);
        haveEpoch = false;
        vals.clear();
    }

    std::sort(recs.begin(), recs.end(), [](const Record &a, const Record &b) { return a.mjd < b.mjd; });
    return !recs.empty();
}

bool METReader::get(const double mjd, double &presHpa, double &tempC, double &humi) const {
    presHpa = 0.0;
    tempC = 0.0;
    humi = 0.0;

    const auto b = Math::bracketSample(recs, mjd, [](const Record &r) { return r.mjd; });
    if (!b) return false;

    const Record &a = recs[b->lo], &c = recs[b->hi];
    presHpa = Math::lerp(a.presHpa, c.presHpa, b->f);
    tempC = Math::lerp(a.tempC, c.tempC, b->f);
    humi = Math::lerp(a.humi, c.humi, b->f);
    return presHpa > 0.0;
}
