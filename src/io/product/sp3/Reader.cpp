#include "product/sp3/Reader.h"
#include "MathUtils.h"
#include "core/Const.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

namespace SP3 {
    namespace {
        bool sp3FrameName(const std::string &n, Frame::Ref &out) {
            if (Frame::refByName(n, out)) return true;
            if (n.size() == 5 && n[0] == 'I' && n[1] == 'G') {
                const std::string year = n.substr(3, 2);
                if (year == "14") {
                    out = Frame::Ref::IGS14;
                    return true;
                }
                if (year == "20") {
                    out = Frame::Ref::IGS20;
                    return true;
                }
            }
            return false;
        }
    }

    std::string Reader::probeAntexModel(const std::string &path) {
        std::ifstream in(path);
        if (!in) return {};
        std::string line;
        for (int n = 0; n < 80 && std::getline(in, line); ++n) {
            const std::size_t at = line.find("PCV:");
            if (at == std::string::npos) continue;
            std::istringstream ss(line.substr(at + 4));
            if (std::string tok; ss >> tok) return tok;
        }
        return {};
    }

    bool Reader::read(const std::string &path) {
        return read(path, false);
    }

    bool Reader::read(const std::string &path, bool append) {
        std::ifstream in(path);
        if (!in) return !data.empty();

        std::string line;
        frame = Frame::Ref::IGS14;
        hasFrame = false;

        bool inEpoch = false;
        CommonTime curT;
        std::map<SatID, std::vector<Record> > tmp;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            // 星历头：#cP（位置）/#cV（速度）/#dP（SP3-d），框架名就散在字段里。
            if (line.size() > 3 && line[0] == '#' && (line[1] == 'c' || line[1] == 'd')) {
                std::istringstream ss(line);
                std::string tok;
                while (ss >> tok)
                    if (sp3FrameName(tok, frame)) {
                        hasFrame = true;
                        break;
                    }
                if (!hasFrame) frame = Frame::Ref::IGS14;
                continue;
            }
            if (line[0] == '*') {
                std::istringstream ss(line);
                char star = 0;
                int y = 0, mo = 0, d = 0, h = 0, mi = 0;
                double s = 0.0;
                if (!(ss >> star >> y >> mo >> d >> h >> mi >> s)) {
                    inEpoch = false;
                    continue;
                }
                CivilTime ct(y, mo, d, h, mi, s, TimeSystem::GPS);
                curT = ct.toCommonTime();
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
                    // 位置三分量全 0 是 SP3 的「该历元无数据」哨兵，不是地心坐标。
                    if (x == 0.0 && y == 0.0 && z == 0.0) continue;
                    rec.pos = Eigen::Vector3d(x, y, z) * 1000.0;
                } else {

                    std::istringstream ss(line);
                    double a = 0.0, b = 0.0, c = 0.0;
                    if (std::string tag; !(ss >> tag >> a >> b >> c) || tag.size() < 4) continue;
                    rec.pos = Eigen::Vector3d(a, b, c) * 1000.0;
                }
                if (double clk = 0.0; getField(line, 3, clk) &&isValid(clk)) {
                    rec.clock = clk * 1e-6;
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
                        rec.vel = Eigen::Vector3d(vx, vy, vz) * 0.1;
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
                        rec.clockRate = vclk * 1e-6;
                        rec.hasClockRate = true;
                    }
                }
            }
        }

        if (tmp.empty()) return !data.empty();
        if (append) {
            for (auto &[sat, recs]: tmp) {
                auto &dst = data[sat];
                dst.insert(dst.end(), recs.begin(), recs.end());
            }
        } else {
            data = std::move(tmp);
        }
        finalize();
        return !data.empty();
    }

    void Reader::finalize() {
        relTimes.clear();
        for (auto &[sat, recs]: data) {
            if (recs.empty()) continue;
            std::sort(recs.begin(), recs.end(),
                      [](const Record &a, const Record &b) { return a.t < b.t; });
            const int n = static_cast<int>(recs.size());
            for (int i = 0; i < n; ++i) {
                if (!recs[i].hasClock) continue;
                const int a = std::max(0, i - 1);
                const int b = std::min(n - 1, i + 1);
                if (a == b) continue;
                const double dt = recs[b].t - recs[a].t;
                if (dt == 0.0) continue;
                recs[i].clockRate = (recs[b].clock - recs[a].clock) / dt;
                recs[i].hasClockRate = true;
            }
            std::vector<double> &rt = relTimes[sat];
            rt.reserve(recs.size());
            const CommonTime t0 = recs[0].t;
            for (const auto &r: recs) rt.push_back(r.t - t0);
        }
    }

    PVT Reader::getPVT(const SatID &sat, const CommonTime &t) const {
        PVT pvt;
        const auto it = data.find(sat);
        if (it == data.end() || it->second.empty()) return pvt;
        const auto &r = it->second;
        const int n = static_cast<int>(r.size());


        const auto itT = relTimes.find(sat);
        const std::vector<double> &times = itT != relTimes.end() ? itT->second : std::vector<double>{};
        const double tRel = t - r[0].t;
        const int lb = Math::lowerBoundIndex(times, tRel);
        int idx = lb;
        if (lb + 1 < n && t - r[lb].t > r[lb + 1].t - t) idx = lb + 1;
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
            constexpr double kVelDt = 1.0e-3;
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
            pvt.clockBias = 0.0;
            pvt.clockDrift = 0.0;
        }
        pvt.relativityCorrection = satRelativityCorrection(pvt.p, pvt.v);
        return pvt;
    }

    std::pair<CommonTime, CommonTime> Reader::timeSpan(const SatID &sat) const {
        const auto it = data.find(sat);
        if (it == data.end() || it->second.empty())
            return {CommonTime(), CommonTime()};
        return {it->second.front().t, it->second.back().t};
    }
}
