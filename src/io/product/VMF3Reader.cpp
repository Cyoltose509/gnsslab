#include "product/VMF3Reader.h"

#include "Const.h"
#include "MathUtils.h" // Math::lerp（时次一维混合）
#include "TimeConvert.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace {
    /// "! Epoch:              2026 01 01 00 00  0.0" → MJD（含日内小数）
    bool parseEpochMjd(const std::string &line, double &mjd) {
        const auto p = line.find("Epoch:");
        if (p == std::string::npos) return false;
        int y = 0, mo = 0, d = 0, hh = 0, mi = 0;
        double sec = 0.0;
        if (std::istringstream is(line.substr(p + 6)); !(is >> y >> mo >> d >> hh >> mi >> sec)) return false;
        if (mo < 1 || mo > 12 || d < 1 || d > 31) return false;
        const double jd = Time::YMDToJD({y, mo, d});
        mjd = jd - MJD_TO_JD + (hh + mi / 60.0 + sec / 3600.0) / 24.0;
        return std::isfinite(mjd);
    }
}

void VMF3Reader::bilinear(const double latDeg, const double lonDeg, int idx[4], double w[4]) {
    double u = (87.5 - latDeg) / 5.0;
    u = std::min(std::max(u, 0.0), static_cast<double>(kLat - 1));
    const int i0 = std::min(static_cast<int>(std::floor(u)), kLat - 2);
    const int i1 = i0 + 1;
    const double fl = u - i0;

    double lon = lonDeg;
    while (lon < 0.0) lon += 360.0;
    while (lon >= 360.0) lon -= 360.0;
    double v = (lon - 2.5) / 5.0;
    if (v < 0.0) v += kLon;
    int j0 = static_cast<int>(std::floor(v));
    if (j0 > kLon - 1) j0 = kLon - 1;
    const int j1 = (j0 + 1) % kLon;
    const double fo = v - j0;

    idx[0] = i0 * kLon + j0;
    w[0] = (1.0 - fl) * (1.0 - fo);
    idx[1] = i0 * kLon + j1;
    w[1] = (1.0 - fl) * fo;
    idx[2] = i1 * kLon + j0;
    w[2] = fl * (1.0 - fo);
    idx[3] = i1 * kLon + j1;
    w[3] = fl * fo;
}

bool VMF3Reader::loadFile(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    Epoch ep;
    bool haveEpoch = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '!') {
            if (!haveEpoch && parseEpochMjd(line, ep.mjd)) haveEpoch = true;
            continue;
        }
        std::istringstream is(line);
        double lat = 0.0, lon = 0.0, ah = 0.0, aw = 0.0, zhd = 0.0, zwd = 0.0; //NOLINT
        if (!(is >> lat >> lon >> ah >> aw >> zhd >> zwd)) continue;
        ep.ah.push_back(ah);
        ep.aw.push_back(aw);
        ep.zhd.push_back(zhd);
        ep.zwd.push_back(zwd);
    }
    if (!haveEpoch || ep.ah.size() != static_cast<std::size_t>(kCell)) return false;

    // 同一天四个时次可能被重复加载（例如清单里手工加了重名文件）：按 mjd 去重后插入
    for (auto &e: epochs) {
        if (std::fabs(e.mjd - ep.mjd) < 1.0e-9) {
            e = std::move(ep);
            return true;
        }
    }
    epochs.push_back(std::move(ep));
    std::sort(epochs.begin(), epochs.end(),
              [](const Epoch &a, const Epoch &b) { return a.mjd < b.mjd; });
    return true;
}

bool VMF3Reader::loadFiles(const std::vector<std::string> &paths) {
    int ok = 0;
    for (const auto &p: paths)
        if (loadFile(p)) ++ok;
    return ok > 0;
}

bool VMF3Reader::coefficients(const double mjd, const double latRad, const double lonRad,
                              double &ah, double &aw) const {
    if (epochs.empty()) return false;

    int idx[4];
    double w[4];
    bilinear(latRad * RAD_TO_DEG, lonRad * RAD_TO_DEG, idx, w);

    const std::size_t n = epochs.size();
    std::size_t a = 0, b = 0;
    double tb = 0.0; // 后一个时次的权重
    if (mjd <= epochs.front().mjd) {
        a = b = 0;
    } else if (mjd >= epochs.back().mjd) {
        a = b = n - 1;
    } else {
        for (a = 0; a + 1 < n; ++a)
            if (mjd <= epochs[a + 1].mjd) break;
        b = a + 1;
        const double span = epochs[b].mjd - epochs[a].mjd;
        tb = span > 0.0 ? (mjd - epochs[a].mjd) / span : 0.0;
    }

    const Epoch &ea = epochs[a];
    const Epoch &eb = epochs[b];
    double ahA = 0.0, awA = 0.0, ahB = 0.0, awB = 0.0;
    for (int c = 0; c < 4; ++c) {
        ahA += w[c] * ea.ah[idx[c]];
        awA += w[c] * ea.aw[idx[c]];
        ahB += w[c] * eb.ah[idx[c]];
        awB += w[c] * eb.aw[idx[c]];
    }
    ah = Math::lerp(ahA, ahB, tb);
    aw = Math::lerp(awA, awB, tb);
    return std::isfinite(ah) && std::isfinite(aw) && ah > 0.0 && aw > 0.0;
}

bool VMF3Reader::zenithDelays(const double mjd, const double latRad, const double lonRad,
                              double &zhd, double &zwd) const {
    if (epochs.empty()) return false;

    int idx[4];
    double w[4];
    bilinear(latRad * RAD_TO_DEG, lonRad * RAD_TO_DEG, idx, w);

    const std::size_t n = epochs.size();
    std::size_t a = 0, b = 0;
    double tb = 0.0; // 后一个时次的权重
    if (mjd <= epochs.front().mjd) {
        a = b = 0;
    } else if (mjd >= epochs.back().mjd) {
        a = b = n - 1;
    } else {
        for (a = 0; a + 1 < n; ++a)
            if (mjd <= epochs[a + 1].mjd) break;
        b = a + 1;
        const double span = epochs[b].mjd - epochs[a].mjd;
        tb = span > 0.0 ? (mjd - epochs[a].mjd) / span : 0.0;
    }

    const Epoch &ea = epochs[a];
    const Epoch &eb = epochs[b];
    double zhdA = 0.0, zwdA = 0.0, zhdB = 0.0, zwdB = 0.0;
    for (int c = 0; c < 4; ++c) {
        zhdA += w[c] * ea.zhd[idx[c]];
        zwdA += w[c] * ea.zwd[idx[c]];
        zhdB += w[c] * eb.zhd[idx[c]];
        zwdB += w[c] * eb.zwd[idx[c]];
    }
    zhd = Math::lerp(zhdA, zhdB, tb);
    zwd = Math::lerp(zwdA, zwdB, tb);
    return std::isfinite(zhd) && std::isfinite(zwd) && zhd > 0.0 && zwd > 0.0;
}
