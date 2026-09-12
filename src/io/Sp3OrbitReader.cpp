#include "Sp3OrbitReader.h"
#include "MathUtils.h"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace {
    SatID parseSatId(const std::string &s) {
        if (s.size() < 3) return {};
        return {s[0], std::stoi(s.substr(1, 2))};
    }

} // namespace

bool Sp3OrbitReader::read(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    // 跳过头部直到第一个 '*' 历元行
    bool inEpoch = false;
    CommonTime curT;
    std::map<SatID, std::vector<Record> > tmp;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '*') {
            // *  yyyy mm dd hh mm sss.ssssss
            std::istringstream ss(line);
            char star;
            int y, mo, d, h, mi;
            double s;
            ss >> star >> y >> mo >> d >> h >> mi >> s;
            CivilTime ct(y, mo, d, h, mi, s, TimeSystem::GPS);
            curT = CivilTime2CommonTime(ct);
            inEpoch = true;
            continue;
        }
        if (!inEpoch) continue;
        if (line[0] == 'P') {
            std::istringstream ss(line);
            std::string tag;
            double x, y, z, clk;
            ss >> tag >> x >> y >> z >> clk;
            if (tag.size() < 4) continue;
            SatID sat = parseSatId(tag.substr(1, 3));
            Record rec;
            rec.t = curT;
            rec.pos = Eigen::Vector3d(x, y, z) * 1000.0; // km -> m
            rec.clock = clk * 1e-6; // us -> s
            tmp[sat].push_back(rec);
        } else if (line[0] == 'V') {
            std::istringstream ss(line);
            std::string tag;
            double vx, vy, vz, vclk;
            ss >> tag >> vx >> vy >> vz >> vclk;
            if (tag.size() < 4) continue;
            if (SatID sat = parseSatId(tag.substr(1, 3)); tmp.count(sat) && !tmp[sat].empty()) {
                auto &rec = tmp[sat].back();
                rec.vel = Eigen::Vector3d(vx, vy, vz) * 0.1; // dm/s -> m/s
                rec.hasVel = true;
            }
        }
    }

    // 对每个卫星，若无 V 记录则用中心差分补速度
    for (auto &[sat, recs]: tmp) {
        int n = static_cast<int>(recs.size());
        if (n == 0) continue;
        if (!recs[0].hasVel) {
            for (int i = 0; i < n; ++i) {
                int a = std::max(0, i - 1), b = std::min(n - 1, i + 1);
                if (a == b) {
                    recs[i].vel.setZero();
                    continue;
                }
                double dt = recs[b].t - recs[a].t;
                if (dt == 0.0) {
                    recs[i].vel.setZero();
                    continue;
                }
                recs[i].vel = (recs[b].pos - recs[a].pos) / dt;
                recs[i].hasVel = true;
            }
        }
    }

    data = std::move(tmp);
    // 预计算每卫星相对首历元的秒数向量(查询时复用，避免每调用 O(n) 拷贝)
    relTimes.clear();
    for (const auto &[sat, recs] : data) {
        std::vector<double> &rt = relTimes[sat];
        rt.reserve(recs.size());
        if (!recs.empty()) {
            const CommonTime t0 = recs[0].t;
            for (const auto &r : recs) rt.push_back(r.t - t0);
        }
    }
    return !data.empty();
}

PVT Sp3OrbitReader::getPVT(const SatID &sat, const CommonTime &t) const {
    PVT pvt;
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty()) return pvt;
    const auto &r = it->second;
    const int n = static_cast<int>(r.size());

    // 记录按时间升序（SP3 历元有序）；二分定位最近的记录索引（复用 read() 时预计算的相对时间向量，O(log n)）
    const auto itT = relTimes.find(sat);
    const std::vector<double> &times = (itT != relTimes.end()) ? itT->second : std::vector<double>{};
    const double tRel = t - r[0].t;                          // 查询点同基准秒数
    const int lb = Math::lowerBoundIndex(times, tRel);       // 满足 times[lb] <= tRel < times[lb+1]
    int idx = lb;
    if (lb + 1 < n && t - r[lb].t > (r[lb + 1].t - t)) idx = lb + 1; // 取 lb 与 lb+1 中更近者
    const double dt = t - r[idx].t;

    const int half = std::min(5, n / 2);
    const int lo = std::max(0, idx - half);
    const int hi = std::min(n - 1, idx + half);
    const int m = hi - lo + 1;

    // 节点不足 2 个时直接取最近点，避免退化插值走到 Lagrange 的 0/1 节点分支。
    if (m < 2) {
        pvt.p = r[idx].pos;
        pvt.v = r[idx].hasVel ? r[idx].vel : Eigen::Vector3d::Zero();
        pvt.clockBias = 0.0;
        pvt.clockDrift = 0.0;
        pvt.relativityCorrection = 0.0;
        return pvt;
    }

    // 各节点相对 r[idx].t 的时间（秒），与查询点 dt 同基准 → 平移不变，可直接插值
    std::vector<double> tt(m);
    for (int k = 0; k < m; ++k) tt[k] = r[lo + k].t - r[idx].t;

    std::vector<Eigen::Vector3d> posVec(m);
    for (int k = 0; k < m; ++k) posVec[k] = r[lo + k].pos;
    pvt.p = Math::simpleLagrangeInterpolation(tt, posVec, dt);

    if (r[idx].hasVel) {
        std::vector<Eigen::Vector3d> velVec(m);
        for (int k = 0; k < m; ++k) velVec[k] = r[lo + k].vel;
        pvt.v = Math::simpleLagrangeInterpolation(tt, velVec, dt);
    } else {
        pvt.v.setZero();
    }
    pvt.clockBias = 0.0; // 钟差由调用方用 CLK 注入
    pvt.clockDrift = 0.0;
    pvt.relativityCorrection = 0.0;
    return pvt;
}

std::pair<CommonTime, CommonTime> Sp3OrbitReader::timeSpan(const SatID &sat) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty())
        return {CommonTime(), CommonTime()};
    return {it->second.front().t, it->second.back().t};
}
