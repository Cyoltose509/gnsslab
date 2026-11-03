#include "product/ATTReader.h"

#include "MathUtils.h"
#include "TimeConvert.h"

#include <algorithm>
#include <fstream>
#include <sstream>

bool ATTReader::read(const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    data.clear();

    double curMjd = 0.0;
    bool haveEpoch = false;

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.size() < 4) continue;

        if (line[0] == '#' && line[1] == '#') {
            // ## yyyy mm dd hh mm ss.ssssssssssss nnn
            std::istringstream is(line.substr(2));
            int y = 0, mo = 0, d = 0, hh = 0, mi = 0;
            double sec = 0.0;
            if (!(is >> y >> mo >> d >> hh >> mi >> sec)) { haveEpoch = false; continue; }
            const CommonTime t =
                    Time::CivilToCommon(CivilTime(y, mo, d, hh, mi, sec, TimeSystem::GPS));
            const MJD m = Time::CommonToMJD(t);
            curMjd = static_cast<double>(m.mjd);
            haveEpoch = true;
            continue;
        }

        if (const std::string rec = line.substr(1, 3); rec != "ATT") continue;
        if (!haveEpoch) continue;

        std::istringstream is(line.substr(4));
        std::string satTok;
        if (!(is >> satTok)) continue;
        if (satTok.size() != 3) continue;

        std::vector<double> nums;
        double v = 0.0;
        while (is >> v) nums.push_back(v);
        if (nums.size() == 5) nums.erase(nums.begin());
        if (nums.size() != 4) continue;

        const Eigen::Quaterniond raw(nums[0], nums[1], nums[2], nums[3]);
        if (const double n = raw.coeffs().norm(); !(n > 0.0) || !std::isfinite(n)) continue;
        const ATTQuat q = ATTQuat(raw).normalized();

        SatID sat;
        try {
            sat = SatID(satTok);
        } catch (...) { continue; }

        data[sat].push_back(Sample{curMjd, q});
    }

    for (auto &[sat, v]: data)
        std::sort(v.begin(), v.end(), [](const Sample &a, const Sample &b) { return a.mjd < b.mjd; });
    for (auto it = data.begin(); it != data.end();)
        it = it->second.empty() ? data.erase(it) : std::next(it);
    return !data.empty();
}

bool ATTReader::quaternion(const SatID &sat, const double mjd, ATTQuat &q) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty()) return false;
    const auto &v = it->second;
    constexpr double gapSec = kMaxGapSec / 86400.0;
    if (mjd <= v.front().mjd) {
        if (v.front().mjd - mjd > gapSec) return false;
        q = v.front().q;
        return true;
    }
    if (mjd >= v.back().mjd) {
        if (mjd - v.back().mjd > gapSec) return false;
        q = v.back().q;
        return true;
    }
    const auto b = Math::bracketSample(v, mjd, [](const Sample &s) { return s.mjd; });
    if (!b) return false;
    q = ATTQuat::slerp(v[b->lo].q, v[b->hi].q, b->f);
    return true;
}

bool ATTReader::bodyDCM(const SatID &sat, const double mjd, Eigen::Matrix3d &M) const {
    ATTQuat q;
    if (!quaternion(sat, mjd, q)) return false;
    M = q.toDCM();
    return true;
}
