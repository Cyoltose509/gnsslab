#include "Sp3OrbitReader.h"
#include "MathUtils.h"
#include "Const.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>


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
            if (line.size() < kSp3IdLen + 1) continue;
            const SatID sat = parseSatId(line.substr(1, 3));
            Record rec;
            rec.t = curT;

            if (double x = 0.0, y = 0.0, z = 0.0;
                getField(line, 0, x) && getField(line, 1, y) && getField(line, 2, z)
                && isValid(x) && isValid(y) && isValid(z)) {
                rec.pos = Eigen::Vector3d(x, y, z) * 1000.0; // km -> m
            } else {
                // 非标准列宽(与旧行为一致的后备)：按空白分隔读位置
                std::istringstream ss(line);
                double a = 0.0, b = 0.0, c = 0.0;
                if (std::string tag; !(ss >> tag >> a >> b >> c) || tag.size() < 4) continue;
                rec.pos = Eigen::Vector3d(a, b, c) * 1000.0;
            }
            if (double clk = 0.0; getField(line, 3, clk) &&isValid(clk)) {
                rec.clock = clk * 1e-6; // us -> s
                rec.hasClock = true;
                if (rec.clock != 0.0) anyClock = true;
            }
            tmp[sat].push_back(rec);
        } else if (line[0] == 'V') {
            if (line.size() < kSp3IdLen + 1) continue;
            if (SatID sat = parseSatId(line.substr(1, 3)); tmp.count(sat) && !tmp[sat].empty()) {
                auto &rec = tmp[sat].back();
                if (double vx = 0.0, vy = 0.0, vz = 0.0;
                    getField(line, 0, vx) && getField(line, 1, vy) && getField(line, 2, vz) &&
                    isValid(vx) && isValid(vy) && isValid(vz)) {
                    rec.vel = Eigen::Vector3d(vx, vy, vz) * 0.1; // dm/s -> m/s
                    rec.hasVel = true;
                } else {
                    std::istringstream ss(line);
                    std::string tag;
                    if (double a = 0.0, b = 0.0, c = 0.0; ss >> tag >> a >> b >> c && tag.size() >= 4) {
                        rec.vel = Eigen::Vector3d(a, b, c) * 0.1;
                        rec.hasVel = true;
                    }
                }
                if (double vclk = 0.0; getField(line, 3, vclk) && isValid(vclk)) {
                    rec.clockRate = vclk * 1e-6; // us/s -> s/s
                    rec.hasClockRate = true;
                }
            }
        }
    }

    for (auto &[sat, recs]: tmp) {
        int n = static_cast<int>(recs.size());
        if (n == 0) continue;
        if (recs[0].hasClockRate) continue;
        for (int i = 0; i < n; ++i) {
            if (!recs[i].hasClock) continue;
            int a = std::max(0, i - 1), b = std::min(n - 1, i + 1);
            if (a == b) continue;
            double dt = recs[b].t - recs[a].t;
            if (dt == 0.0) continue;
            recs[i].clockRate = (recs[b].clock - recs[a].clock) / dt;
            recs[i].hasClockRate = true;
        }
    }

    data = std::move(tmp);
    // 预计算每卫星相对首历元的秒数向量(查询时复用，避免每调用 O(n) 拷贝)
    relTimes.clear();
    for (const auto &[sat, recs]: data) {
        std::vector<double> &rt = relTimes[sat];
        rt.reserve(recs.size());
        if (!recs.empty()) {
            const CommonTime t0 = recs[0].t;
            for (const auto &r: recs) rt.push_back(r.t - t0);
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
    const std::vector<double> &times = itT != relTimes.end() ? itT->second : std::vector<double>{};
    const double tRel = t - r[0].t; // 查询点同基准秒数
    const int lb = Math::lowerBoundIndex(times, tRel); // 满足 times[lb] <= tRel < times[lb+1]
    int idx = lb;
    if (lb + 1 < n && t - r[lb].t > r[lb + 1].t - t) idx = lb + 1; // 取 lb 与 lb+1 中更近者
    const double dt = t - r[idx].t;

    const int half = std::min(5, n / 2);
    const int lo = std::max(0, idx - half);
    const int hi = std::min(n - 1, idx + half);
    const int m = hi - lo + 1;
    if (m < 2) {
        pvt.p = r[idx].pos;
        pvt.v = r[idx].hasVel ? r[idx].vel : Eigen::Vector3d::Zero();
        pvt.clockBias = r[idx].hasClock ? r[idx].clock : 0.0;
        pvt.clockDrift = r[idx].hasClockRate ? r[idx].clockRate : 0.0;
        pvt.relativityCorrection = satRelativityCorrection(pvt.p, pvt.v);
        return pvt;
    }
    std::vector<double> tt(m);
    for (int k = 0; k < m; ++k) tt[k] = r[lo + k].t - r[idx].t;

    std::vector<Eigen::Vector3d> posVec(m);
    for (int k = 0; k < m; ++k) {
        const Eigen::Vector3d &p = r[lo + k].pos;
        posVec[k] = Math::rotationZ(-OMEGA_EARTH * (tt[k] - dt)) * p;
    }
    pvt.p = Math::simpleLagrangeInterpolation(tt, posVec, dt);
    bool allVel = true;
    for (int k = 0; k < m; ++k) {
        if (!r[lo + k].hasVel) {
            allVel = false;
            break;
        }
    }
    if (allVel) {
        std::vector<Eigen::Vector3d> velVec(m);
        for (int k = 0; k < m; ++k) {
            const Eigen::Vector3d &v = r[lo + k].vel;
            velVec[k] = Math::rotationZ(-OMEGA_EARTH * (tt[k] - dt)) * v;
        }
        pvt.v = Math::simpleLagrangeInterpolation(tt, velVec, dt);
    } else {
        constexpr double kVelDt = 1.0e-3; //与 RTKLIB peph2pos 的 tt 一致
        std::vector<Eigen::Vector3d> posVecAhead(m);
        for (int k = 0; k < m; ++k) {
            const Eigen::Vector3d &p = r[lo + k].pos;
            posVecAhead[k] = Math::rotationZ(-OMEGA_EARTH * (tt[k] - (dt + kVelDt))) * p;
        }
        const Eigen::Vector3d pAhead = Math::simpleLagrangeInterpolation(tt, posVecAhead, dt + kVelDt);
        pvt.v = (pAhead - pvt.p) / kVelDt;
    }

    if (r[idx].hasClock) {
        std::vector<double> clkVec(m);
        for (int k = 0; k < m; ++k) clkVec[k] = r[lo + k].clock;
        pvt.clockBias = Math::simpleLagrangeInterpolation(tt, clkVec, dt);
        if (r[idx].hasClockRate) {
            std::vector<double> drtVec(m);
            for (int k = 0; k < m; ++k) drtVec[k] = r[lo + k].clockRate;
            pvt.clockDrift = Math::simpleLagrangeInterpolation(tt, drtVec, dt);
        } else {
            pvt.clockDrift = 0.0;
        }
    } else {
        pvt.clockBias = 0.0; // 无钟差列，钟差由调用方用 CLK 注入
        pvt.clockDrift = 0.0;
    }
    pvt.relativityCorrection = satRelativityCorrection(pvt.p, pvt.v);
    return pvt;
}

std::pair<CommonTime, CommonTime> Sp3OrbitReader::timeSpan(const SatID &sat) const {
    const auto it = data.find(sat);
    if (it == data.end() || it->second.empty())
        return {CommonTime(), CommonTime()};
    return {it->second.front().t, it->second.back().t};
}
