#include "ImuCalibrator.h"
#include "solver/SolverLSQ.h"

#include <algorithm>

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
    // 返回「轴(0/1/2) + 符号」合成的 0..5 键：正轴→2*ax，负轴→2*ax+1
    int axisKey(const double a, const double b, const double c) {
        int ax = 0;
        double m = std::fabs(a);
        if (std::fabs(b) > m) {
            m = std::fabs(b);
            ax = 1;
        }
        if (std::fabs(c) > m) {
            m = std::fabs(c); //NOLINT
            ax = 2;
        }
        const double v = ax == 0 ? a : ax == 1 ? b : c;
        return v >= 0.0 ? 2 * ax : 2 * ax + 1;
    }
}


Eigen::Vector3d ImuCalibrator::posForce(const Pos p, const double g) {
    switch (p) {
        case XUP: return {g, 0, 0};
        case XDN: return {-g, 0, 0};
        case YUP: return {0, g, 0};
        case YDN: return {0, -g, 0};
        case ZUP: return {0, 0, g};
        case ZDN: return {0, 0, -g};
    }
    return Eigen::Vector3d::Zero();
}

void ImuCalibrator::rawMean(const ImuLog &log, Eigen::Vector3d &acc, Eigen::Vector3d &gyr) {
    acc.setZero();
    gyr.setZero();
    if (log.empty()) return;
    for (const auto &s: log) {
        acc += Eigen::Vector3d(s.ax, s.ay, s.az);
        gyr += Eigen::Vector3d(s.gx, s.gy, s.gz);
    }
    const auto n = static_cast<double>(log.size());
    acc /= n;
    gyr /= n;
}

void ImuCalibrator::solveLinear(const std::vector<Eigen::Vector3d> &raw, const std::vector<Eigen::Vector3d> &f,
                                const std::vector<double> &w,
                                Eigen::Matrix3d &M, Eigen::Vector3d &b, double &rmsResid,
                                Eigen::Vector3d *scaleSigma) {
    M.setZero();
    b.setZero();
    rmsResid = 0.0;
    if (scaleSigma) scaleSigma->setZero();
    const size_t n = raw.size();
    if (n == 0 || f.size() != n || w.size() != n) return;

    static const std::string kStation = "imu_calib";
    const Variable vSx(kStation, Parameter::dX), vSy(kStation, Parameter::dY),
            vSz(kStation, Parameter::dZ), vB(kStation, Parameter::cdt);
    SolverLSQ lsq;
    for (int k = 0; k < 3; ++k) {
        EquSys equ;
        equ.station = kStation;
        equ.varSet = {vSx, vSy, vSz, vB};
        for (size_t i = 0; i < n; ++i) {
            if (w[i] <= 0.0) continue;
            EquData d;
            d.prefit = raw[i](k);
            d.weight = w[i];
            d.varCoeffData = {{vSx, f[i](0)}, {vSy, f[i](1)}, {vSz, f[i](2)}, {vB, 1.0}};
            equ.obsEquData[EquID(SatID('P', static_cast<int>(i)), "acc")] = d; // 'P'+段序号仅作 EquSys 内键
        }
        lsq.solve(equ);
        M(k, 0) = lsq.getSolution(Parameter::dX);
        M(k, 1) = lsq.getSolution(Parameter::dY);
        M(k, 2) = lsq.getSolution(Parameter::dZ);
        b(k) = lsq.getSolution(Parameter::cdt);
        // 比例因子 1σ
        if (scaleSigma && lsq.covMatrix.rows() == 4) {
            (*scaleSigma)(k) = std::sqrt(std::max(0.0, lsq.covMatrix(0, 0))) * lsq.sigma0;
        }
    }
    double ss = 0.0;
    double wtot = 0.0;
    for (size_t i = 0; i < n; ++i) wtot += w[i];
    for (int k = 0; k < 3; ++k) {
        for (size_t i = 0; i < n; ++i) {
            if (w[i] <= 0.0) continue;
            const double pred = f[i](0) * M(k, 0) + f[i](1) * M(k, 1) + f[i](2) * M(k, 2) + b(k);
            ss += w[i] * (raw[i](k) - pred) * (raw[i](k) - pred);
        }
    }
    rmsResid = std::sqrt(ss / (3.0 * wtot));
}


// 完整标定
CalibReport ImuCalibrator::calibrate(const CalibSegments &seg, double turnAngle, double g) {
    CalibReport r;
    r.ok = false;

    int nPos[6] = {0, 0, 0, 0, 0, 0};
    int nRot[6] = {0, 0, 0, 0, 0, 0};
    for (const auto &[fst, snd]: seg.pos) nPos[static_cast<int>(fst)]++;
    for (const auto &[fst, snd]: seg.rot) nRot[static_cast<int>(fst)]++;

    std::string miss;
    for (int p = 0; p < 6; ++p)
        if (nPos[p] == 0) miss += std::string(" 位置") + posLabel(static_cast<Pos>(p));
    for (int ax = 0; ax < 3; ++ax) {
        if (nRot[2 * ax] == 0) miss += std::string(" 旋转") + rotLabel(static_cast<Rot>(2 * ax));
        if (nRot[2 * ax + 1] == 0) miss += std::string(" 旋转") + rotLabel(static_cast<Rot>(2 * ax + 1));
    }
    if (!miss.empty()) {
        r.errorMsg = "覆盖不足，缺少：" + miss;
        return r;
    }

    // ---- 加速度计：堆叠所有静置段做最小二乘，行权重 = 段样本数 ----
    std::vector<Eigen::Vector3d> raw, f;
    std::vector<double> w;
    raw.reserve(seg.pos.size());
    f.reserve(seg.pos.size());
    w.reserve(seg.pos.size());
    for (const auto &[fst, samples]: seg.pos) {
        Eigen::Vector3d acc, gyr;
        rawMean(samples, acc, gyr);
        raw.push_back(acc);
        f.push_back(posForce(fst, g));
        w.push_back(static_cast<double>(samples.size()));
        r.nPos[static_cast<int>(fst)] += static_cast<int>(samples.size());
    }

    if (!seg.pos.empty()) r.dt = imuMedianDt(seg.pos.front().second);
    else if (!seg.rot.empty()) r.dt = imuMedianDt(seg.rot.front().second);

    solveLinear(raw, f, w, r.calib.accelM, r.calib.accelB, r.accelRms, &r.accelScaleSigma);
    r.accelScaleCounts = r.calib.accelM.diagonal();
    r.accelScaleG = r.accelScaleCounts * g;

    if (const double knorm = r.accelScaleCounts.mean(); knorm != 0.0) {
        r.accelCross = r.calib.accelM / knorm;
        r.accelCross.diagonal().setZero();
    }
    r.accelBiasPhys = r.calib.accelM.inverse() * r.calib.accelB;

    //陀螺零偏
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
    for (int ax = 0; ax < 3; ++ax) {
        double sumUp = 0.0, sumDn = 0.0, wUp = 0.0, wDn = 0.0;
        int nUp = 0, nDn = 0;
        for (const auto &[p, samples]: seg.pos) {
            Eigen::Vector3d acc, gyr;
            rawMean(samples, acc, gyr);
            const auto wt = static_cast<double>(samples.size());
            if (p == 2 * ax) {
                sumUp += gyr(ax) * wt;
                wUp += wt;
                ++nUp;
            } else if (p == 2 * ax + 1) {
                sumDn += gyr(ax) * wt;
                wDn += wt;
                ++nDn;
            }
        }
        if (wUp > 0.0 && wDn > 0.0)
            gyroBias(ax) = 0.5 * (sumUp / wUp + sumDn / wDn);
    }

    // 陀螺比例因子 + 交轴耦合
    Eigen::Matrix3d gyroM = Eigen::Matrix3d::Zero();
    for (int ax = 0; ax < 3; ++ax) {
        double s1[3] = {0.0, 0.0, 0.0}, s2[3] = {0.0, 0.0, 0.0};
        int nSeg = 0;
        for (const auto &[rr, samples]: seg.rot) {
            if (rr != 2 * ax && rr != 2 * ax + 1) continue;
            ++nSeg;
            for (const auto &s: samples) {
                const int32_t v[3] = {s.gx, s.gy, s.gz};
                for (int i = 0; i < 3; ++i) {
                    const double val = static_cast<double>(v[i]) - gyroBias(i);
                    if (rr == 2 * ax) s1[i] += val;
                    else s2[i] += val;
                }
            }
            r.nRot[rr] += static_cast<int>(samples.size());
        }
        for (int i = 0; i < 3; ++i)
            gyroM(i, ax) = nSeg > 0 ? (s1[i] - s2[i]) / (turnAngle * nSeg) : 0.0;
    }
    r.gyroM = gyroM;
    r.gyroKeff = gyroM.diagonal(); // 对角 = 各轴有效比例因子
    {
        // 交轴耦合归一化兜底（applyNominal 会用标称 K_eff,nom 覆写）
        if (const double gnorm = r.gyroKeff.mean(); gnorm != 0.0) {
            r.gyroCross = gyroM / gnorm;
            r.gyroCross.diagonal().setZero();
        }
    }
    r.calib.gyroBias = gyroBias;

    r.calib.gyroScale = r.gyroKeff * r.dt;

    // 陀螺零偏换算到物理单位：LSB/历元 -> rad/s
    for (int ax = 0; ax < 3; ++ax)
        r.gyroBiasPhys(ax) = r.calib.gyroScale(ax) != 0.0 ? gyroBias(ax) / r.calib.gyroScale(ax) : 0.0;

    r.ok = true;
    return r;
}

void ImuCalibrator::applyNominal(CalibReport &r, const ImuScale &scale) {
    if (!scale.known()) {
        r.ok = false;
        r.errorMsg = "型号当量未知（IMU Type 未识别），无法确定标称比例因子";
        return;
    }
    const double kA = r.dt / scale.accelMpsPerLsb; // LSB/(m/s²)
    const double kG = 1.0 / scale.gyroRadPerLsb;  // LSB/rad
    r.nominalAccelK = kA;
    r.nominalKeff = kG;
    r.accelDeltaS = r.accelScaleCounts / kA - Eigen::Vector3d::Ones();
    r.gyroDeltaS = r.gyroKeff / kG - Eigen::Vector3d::Ones();
    r.accelCross = r.calib.accelM / kA;
    r.accelCross.diagonal().setZero();
    r.gyroCross = r.gyroM / kG;
    r.gyroCross.diagonal().setZero();
}

std::vector<ImuCalibrator::CalibSegment> ImuCalibrator::detect(const ImuLog &log) {
    constexpr DetectParams p{};
    std::vector<CalibSegment> segs;
    if (log.empty()) return segs;
    const size_t N = log.size();

    std::vector<double> m(N), am(N);
    for (size_t i = 0; i < N; ++i) {
        const auto &s = log[i];
        m[i] = std::max({
            std::fabs(static_cast<double>(s.gx)),
            std::fabs(static_cast<double>(s.gy)),
            std::fabs(static_cast<double>(s.gz))
        });
        am[i] = std::max({
            std::fabs(static_cast<double>(s.ax)),
            std::fabs(static_cast<double>(s.ay)),
            std::fabs(static_cast<double>(s.az))
        });
    }
    // 百分位（排序副本）
    auto pctOf = [](const std::vector<double> &v, const double q) {
        std::vector<double> c = v;
        std::sort(c.begin(), c.end());
        const size_t idx = std::min<size_t>(c.size() - 1, static_cast<size_t>(q * (c.size() - 1)));
        return c[idx];
    };
    const double aMed = pctOf(am, 0.50);

    // 2) 旋转阈值
    const double thr = std::max(p.rotThrCounts, aMed * p.rotFrac);

    // 3) 每样本「段键」
    std::vector<int> key(N);
    for (size_t i = 0; i < N; ++i) {
        const auto &s = log[i];
        if (m[i] > thr)
            key[i] = 6 + axisKey(s.gx, s.gy, s.gz); // 6..11=旋转
        else
            key[i] = axisKey(s.ax, s.ay, s.az); // 0..5 =静置
    }

    // 4) 初分段（连续同键）
    struct Run {
        size_t i0, i1;
        int key;
    };
    std::vector<Run> runs;
    size_t i = 0;
    while (i < N) {
        const int k = key[i];
        size_t j = i;
        while (j < N && key[j] == k) ++j;
        runs.push_back({i, j, k});
        i = j;
    }

    // 4.5) 估计采样间隔
    auto estimateDt = [](const ImuLog &_log) -> double {
        if (_log.size() < 8) return 0.0;
        std::vector<double> d;
        d.reserve(_log.size() - 1);
        for (size_t j = 1; j < _log.size(); ++j) {
            if (const double v = _log[j].tow - _log[j - 1].tow; v > 0.0) d.push_back(v);
        }
        if (d.size() < _log.size() / 2) return 0.0;
        const size_t mid = d.size() / 2;
        std::nth_element(d.begin(), d.begin() + static_cast<long>(mid), d.end());
        return d[mid];
    };
    const double dtEst = estimateDt(log);
    const size_t minLen = dtEst > 0.0
                              ? std::max<size_t>(1, static_cast<size_t>(std::llround(p.minSegSeconds / dtEst)))
                              : static_cast<size_t>(p.minSegSamples);

    // 5) 合并过短段
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t k = 0; k < runs.size(); ++k) {
            if (runs[k].i1 - runs[k].i0 >= minLen) continue;
            const bool hasL = k > 0;
            const bool hasR = k + 1 < runs.size();
            if (!hasL && !hasR) break;
            const size_t lenL = hasL ? runs[k - 1].i1 - runs[k - 1].i0 : 0;
            const size_t lenR = hasR ? runs[k + 1].i1 - runs[k + 1].i0 : 0;
            const bool toLeft = !hasR || (hasL && lenL >= lenR);
            const size_t seg = toLeft ? k - 1 : k + 1; // 较长的邻段
            runs[seg] = {std::min(runs[k].i0, runs[seg].i0), std::max(runs[k].i1, runs[seg].i1), runs[seg].key};
            runs.erase(runs.begin() + static_cast<long>(k));
            changed = true;
            break;
        }
    }

    // 6) 逐段用段落均值重分类
    for (const auto &[i0, i1, _key]: runs) {
        const size_t n = i1 - i0;
        if (n == 0) continue;
        const double inv = 1.0 / static_cast<double>(n);
        double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        for (size_t k = i0; k < i1; ++k) {
            const auto &s = log[k];
            ax += s.ax;
            ay += s.ay;
            az += s.az;
            gx += s.gx;
            gy += s.gy;
            gz += s.gz;
        }
        CalibSegment seg;
        seg.i0 = i0;
        seg.i1 = i1;
        if (_key >= 6) {
            // 旋转
            seg.isStatic = false;
            seg.rot = static_cast<Rot>(axisKey(gx * inv, gy * inv, gz * inv));
        } else {
            // 静置
            seg.isStatic = true;
            seg.pos = static_cast<Pos>(axisKey(ax * inv, ay * inv, az * inv));
        }
        segs.push_back(seg);
    }
    return segs;
}

ImuCalibrator::CompMat ImuCalibrator::compMatrices(const CalibReport &r) {
    CompMat m;
    m.accelMinv = r.calib.accelM.inverse();
    m.gyroIgN = (Eigen::Matrix3d::Identity() + r.gyroCross).inverse();
    return m;
}

void ImuCalibrator::compensate(const CompMat &m, const CalibReport &r, const ImuSample &s,
                               Eigen::Vector3d &f, Eigen::Vector3d &w) {
    f = m.accelMinv * (Eigen::Vector3d(s.ax, s.ay, s.az) - r.calib.accelB);
    w = m.gyroIgN * (Eigen::Vector3d(s.gx, s.gy, s.gz) - r.calib.gyroBias);
}
