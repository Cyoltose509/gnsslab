#include "product/GPT3Reader.h"

#include "FileCache.h"

#include "Const.h"
#include "TimeConvert.h"
#include "StringUtils.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace {
    constexpr double kGm = 9.80665;       // 平均重力(m/s^2)
    constexpr double kDMtr = 28.965e-3;   // 干空气摩尔质量(kg/mol)
    constexpr double kRg = 8.3143;        // 通用气体常数(J/K/mol)

    double fold(const std::array<double, GPT3Reader::kCols> &r, const int base,
                const double cf, const double sf, const double chy, const double shy) {
        return r[base] + r[base + 1] * cf + r[base + 2] * sf + r[base + 3] * chy +
               r[base + 4] * shy;
    }
}

bool GPT3Reader::load(const std::string &path) {
    const IoCache::Key key = IoCache::keyOf(path);
    if (key.size <= 0) return false;
    if (const auto hit = IoCache::find<Table>(key)) {
        tab = hit;
        return true;
    }

    std::ifstream in(path);
    if (!in) return false;

    auto t = std::make_shared<Table>();
    std::vector<std::array<double, kCols> > &out = t->rows;
    out.reserve(kRows);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '%') continue; // 首行是列注释
        std::array<double, kCols> r{};
        std::istringstream is(line);
        bool ok = true;
        for (int c = 0; c < kCols; ++c)
            if (!(is >> r[c])) { ok = false; break; }
        if (ok) out.push_back(r);
    }
    if (out.size() != static_cast<std::size_t>(kRows)) return false;
    tab = t;
    IoCache::keep<Table>(key, std::move(t));
    return true;
}

void GPT3Reader::nodeWeights(const double latRad, const double lonRad, int idx[4], double w[4],
                             int &nPt) {
    const double latDeg = latRad * RAD_TO_DEG;
    double lonDeg = lonRad * RAD_TO_DEG;
    while (lonDeg < 0.0) lonDeg += 360.0;
    while (lonDeg >= 360.0) lonDeg -= 360.0;

    const double ppod = 90.0 - latDeg;
    const int ipod = std::min(std::max(static_cast<int>(std::floor((ppod + 5.0) / 5.0)), 1), 36);
    const int ilon = std::min(std::max(static_cast<int>(std::floor((lonDeg + 5.0) / 5.0)), 1), 72);
    const double diffpod = (ppod - (ipod * 5.0 - 2.5)) / 5.0;
    const double difflon = (lonDeg - (ilon * 5.0 - 2.5)) / 5.0;

    const bool doBilinear = ppod > 2.5 && ppod < 177.5;

    const auto rowOf = [](const int p, const int l) { return (p - 1) * 72 + (l - 1); };

    idx[0] = rowOf(ipod, ilon);
    idx[1] = idx[2] = idx[3] = idx[0];
    w[0] = 1.0;
    w[1] = w[2] = w[3] = 0.0;
    nPt = 1;
    if (!doBilinear) return;

    const int ipod1 = std::min(std::max(ipod + (diffpod >= 0.0 ? 1 : -1), 1), 36);
    int ilon1 = ilon + (difflon >= 0.0 ? 1 : -1);
    if (ilon1 > 72) ilon1 = 1;
    if (ilon1 < 1) ilon1 = 72;
    idx[1] = rowOf(ipod1, ilon);
    idx[2] = rowOf(ipod, ilon1);
    idx[3] = rowOf(ipod1, ilon1);
    const double dPod1 = std::fabs(diffpod), dPod2 = 1.0 - dPod1;
    const double dLon1 = std::fabs(difflon), dLon2 = 1.0 - dLon1;
    w[0] = dLon2 * dPod2;
    w[1] = dLon2 * dPod1;
    w[2] = dLon1 * dPod2;
    w[3] = dLon1 * dPod1;
    nPt = 4;
}

bool GPT3Reader::gridHeight(const double latRad, const double lonRad, double &hEllM) const {
    if (!tab || tab->rows.empty()) return false;

    int idx[4];
    double w[4];
    int nPt = 0;
    nodeWeights(latRad, lonRad, idx, w, nPt);

    double acc = 0.0;
    for (int k = 0; k < nPt; ++k) {
        const auto &r = tab->rows[static_cast<std::size_t>(idx[k])];
        acc += w[k] * (r[22] + r[23]); // undu + Hs = 格点椭球高
    }
    hEllM = acc;
    return std::isfinite(acc);
}

bool GPT3Reader::at(const double mjd, const double latRad, const double lonRad, const double hEllM,
                  GPT3Values &v) const {
    if (!tab || tab->rows.empty()) return false;
    const double doy = Time::MJDToDOY(mjd);

    const double fy = doy / 365.25 * 2.0 * PI;
    const double hy = doy / 365.25 * 4.0 * PI;
    const double cf = std::cos(fy), sf = std::sin(fy), chy = std::cos(hy), shy = std::sin(hy);

    int idx[4];
    double w[4];
    int nPt = 0;
    nodeWeights(latRad, lonRad, idx, w, nPt);

    double pAcc = 0.0, tAcc = 0.0, dTAcc = 0.0, tmAcc = 0.0, eAcc = 0.0;
    double ahAcc = 0.0, awAcc = 0.0, laAcc = 0.0, unduAcc = 0.0;
    double gnhAcc = 0.0, gehAcc = 0.0, gnwAcc = 0.0, gewAcc = 0.0;

    for (int k = 0; k < nPt; ++k) {
        const auto &r = tab->rows[static_cast<std::size_t>(idx[k])];
        const double T0 = fold(r, 7, cf, sf, chy, shy);      // K
        const double p0 = fold(r, 2, cf, sf, chy, shy);      // Pa
        const double Q = fold(r, 12, cf, sf, chy, shy) / 1000.0;
        const double dTm = fold(r, 17, cf, sf, chy, shy) / 1000.0; // K/m
        const double undu = r[22];
        const double Hs = r[23];                              // 格点正交高(m)
        const double ah = fold(r, 24, cf, sf, chy, shy) / 1000.0;
        const double aw = fold(r, 29, cf, sf, chy, shy) / 1000.0;
        const double la = fold(r, 34, cf, sf, chy, shy);
        const double Tm = fold(r, 39, cf, sf, chy, shy);
        const double gnh = fold(r, 44, cf, sf, chy, shy) / 1.0e5;
        const double geh = fold(r, 49, cf, sf, chy, shy) / 1.0e5;
        const double gnw = fold(r, 54, cf, sf, chy, shy) / 1.0e5;
        const double gew = fold(r, 59, cf, sf, chy, shy) / 1.0e5;

        // 测站正交高 − 格点正交高，作为 p/T/e 的归算高差
        const double redh = hEllM - undu - Hs;

        const double Tv = T0 * (1.0 + 0.6077 * Q);            // 虚温
        const double c = kGm * kDMtr / (kRg * Tv);
        const double p = p0 * std::exp(-c * redh) / 100.0;  // hPa
        const double tC = T0 + dTm * redh - 273.15;           // ℃
        const double e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0; // 格点处水汽压(hPa)
        // Askne & Nordius (1987) 式(14)：水汽压按 (p/p0)^(la+1) 递减
        const double e = p0 > 0.0 ? e0 * std::pow(100.0 * p / p0, la + 1.0) : 0.0;

        pAcc += w[k] * p;
        tAcc += w[k] * tC;
        dTAcc += w[k] * dTm * 1000.0; // K/km
        tmAcc += w[k] * Tm;
        eAcc += w[k] * e;
        ahAcc += w[k] * ah;
        awAcc += w[k] * aw;
        laAcc += w[k] * la;
        unduAcc += w[k] * undu;
        gnhAcc += w[k] * gnh;
        gehAcc += w[k] * geh;
        gnwAcc += w[k] * gnw;
        gewAcc += w[k] * gew;
    }

    v.pHpa = pAcc;
    v.tC = tAcc;
    v.dTKm = dTAcc;
    v.tmK = tmAcc;
    v.eHpa = eAcc;
    v.ah = ahAcc;
    v.aw = awAcc;
    v.la = laAcc;
    v.unduM = unduAcc;
    v.gnH = gnhAcc;
    v.geH = gehAcc;
    v.gnW = gnwAcc;
    v.geW = gewAcc;
    return std::isfinite(v.ah) && std::isfinite(v.aw) && v.ah > 0.0 && v.aw > 0.0;
}

bool GPT3Reader::validate(const std::string &path, std::string &detail) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        detail = "无法打开文件";
        return false;
    }
    std::string first, line;
    if (!std::getline(f, first)) {
        detail = "文件是空的";
        return false;
    }
    if (first.empty() || first[0] != '%') {
        detail = "不是 GPT3 格网文件（首行应为以 % 开头的列注释）";
        return false;
    }
    if (const std::string lo = Str::toLower(first); lo.find("lat") == std::string::npos || lo.find("a0") == std::string::npos) {
        detail = "GPT3 格网的列注释里找不到 lat / a0 列";
        return false;
    }

    int rows = 0;
    int cols = -1;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        int n = 0;
        std::istringstream is(line);
        std::string tok;
        while (is >> tok) ++n;
        if (cols < 0) cols = n;
        else if (n != cols) {
            detail = "格网行列数不一致（第 " + std::to_string(rows + 2) + " 行有 " +
                     std::to_string(n) + " 列，首行数据有 " + std::to_string(cols) + " 列）";
            return false;
        }
        ++rows;
    }
    if (rows < 100 || cols < 10) {
        detail = "格网点太少（" + std::to_string(rows) + " 行 × " + std::to_string(cols) +
                 " 列），文件可能被截断";
        return false;
    }
    detail = "GPT3 格网，" + std::to_string(rows) + " 个格点 × " + std::to_string(cols) + " 列";
    return true;
}
