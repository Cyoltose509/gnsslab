#include "QCProcessor.h"
#include "Const.h"   // getFreq, C_MPS
#include "MathUtils.h"
#include <algorithm>
#include <tuple>
#include <Eigen/Dense>

namespace QC {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    static double getVal(const TypeValueMap &m, const char t, const char b) {
        for (auto &[k, v]: m) {
            if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return v;
        }
        return 0.0;
    }

    // 取某频点的信噪比；RINEX 中用 Sxx 表示（如 S1I, S2P, S6I）
    static double getSnr(const TypeValueMap &m, const char b) {
        for (auto &[k, v]: m) {
            if (k.size() >= 2 && k[0] == 'S' && k[1] == b && v > 0.0) return v;
        }
        return 0.0;
    }

    // 取某观测类型(码/相)的实际信号码（含属性字符，如 "C1I"/"C2X"）
    static std::string findObsType(const TypeValueMap &m, const char t, const char b) {
        for (auto &[k, v]: m)
            if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return k;
        return "";
    }

    // 在所有「同时有 C 与 L」的频点里，挑频率最高的两个做 IF 组合。
    struct BandsInfo {
        char hi = 0, lo = 0;
        double f1 = 0, f2 = 0;
    };

    BandsInfo pickBands(const char sys, const TypeValueMap &m) {
        std::vector<std::pair<char, std::string> > cand; // (band, 实际信号码)
        for (char b: {'1', '2', '5', '6', '7', '8'}) {
            std::string ct = findObsType(m, 'C', b);
            std::string lt = findObsType(m, 'L', b); //NOLINT
            if (!ct.empty() && !lt.empty()) cand.emplace_back(b, ct);
        }
        if (cand.size() < 2) return {};
        std::sort(cand.begin(), cand.end(),
                  [&](const std::pair<char, std::string> &a, const std::pair<char, std::string> &b) {
                      return getFreq(sys, a.second) > getFreq(sys, b.second);
                  });
        BandsInfo r;
        r.hi = cand[0].first;
        r.lo = cand[1].first;
        r.f1 = getFreq(sys, cand[0].second);
        r.f2 = getFreq(sys, cand[1].second);
        return r;
    }

    // ---------- 多项式最小二乘拟合 ----------
    static std::vector<double> polyfit(const std::vector<double> &x, const std::vector<double> &y, const int deg) {
        const int n = static_cast<int>(x.size());
        MatrixXd A(n, deg + 1);
        VectorXd Y(n);
        for (int i = 0; i < n; i++) {
            double p = 1;
            for (int j = 0; j <= deg; j++) {
                A(i, j) = p;
                p *= x[i];
            }
            Y(i) = y[i];
        }
        VectorXd c = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
        std::vector<double> r(deg + 1);
        for (int j = 0; j <= deg; j++) r[j] = c(j);
        return r;
    }

    static double polyVal(const std::vector<double> &c, const double x) {
        double r = 0, p = 1;
        for (const double cc: c) {
            r += cc * p;
            p *= x;
        }
        return r;
    }

    // ---------- 观测值噪声：分弧段三次差标准差 ----------
    static double tripleStd(const std::vector<double> &x, const std::vector<double> &t,
                            const double interval, const std::vector<char> &ok) {
        const auto n = static_cast<int>(x.size());
        if (n < 4) return 0;
        double s = 0;
        int cnt = 0;
        for (int i = 3; i < n; i++) {
            if (!ok[i] || !ok[i - 1] || !ok[i - 2] || !ok[i - 3]) continue;
            bool cont = true;
            for (int k = i - 2; k <= i; k++)
                if (std::fabs(t[k] - t[k - 1] - interval) > 0.5 * interval) {
                    cont = false;
                    break;
                }
            if (!cont) continue;
            const double d3 = x[i] - 3.0 * x[i - 1] + 3.0 * x[i - 2] - x[i - 3];
            s += d3 * d3;
            cnt++;
        }
        if (cnt < 2) return 0; // 需要 N-1 >= 1
        return std::sqrt(s / (8.0 * (cnt - 1.0)));
    }

    // ---------- 粗差 / 周跳探测 (MW 组合递推) ----------
    static void detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier) {
        const int n = static_cast<int>(MW.size());
        slip.assign(n, 0);
        outlier.assign(n, 0);
        if (n < 2) return;
        double mean = MW[0], var = 0.0;
        int cnt = 1;
        for (int k = 1; k < n; k++) {
            if (!ok[k]) {
                // 断档：新弧段，重置递推
                mean = MW[k];
                var = 0.0;
                cnt = 1;
                continue;
            }
            const double dev = MW[k] - mean;
            if (var > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(var) && k + 1 < n && ok[k + 1]) {
                // 递推更新到历元 k，用于预测 k+1 是否超限
                const double meanK = (cnt * mean + MW[k]) / (cnt + 1);
                const double varK = (cnt * var + dev * dev) / (cnt + 1);
                const double devNext = MW[k + 1] - meanK; //NOLINT
                if (!(varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)) || MW[k + 1] - MW[k] > 1.0) {
                    outlier[k] = 1; // ti+1 不超限，或两者均超限但 ΔMW>1m → 粗差
                } else {
                    slip[k] = 1; // 两者均超限且 ΔMW≤1m → 周跳，从 k 起新弧段
                    mean = MW[k];
                    var = 0.0;
                    cnt = 1;
                    continue;
                }
            }
            // 正常递推更新
            mean = (cnt * mean + MW[k]) / (cnt + 1);
            var = (cnt * var + dev * dev) / (cnt + 1);
            cnt++;
        }
    }

    // ---------- 周跳补充探测 (GF 组合) ----------
    static void detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, const double lam1, const double lam2,
                         std::vector<char> &slip) {
        const int n = static_cast<int>(LGF.size());
        if (n < 6) return;
        const double thr = 6.0 * (lam2 - lam1);
        int i = 0;
        while (i < n) {
            if (!ok[i]) {
                i++;
                continue;
            }
            int j = i;
            while (j < n && (j == i || ok[j])) j++; // [i, j) 为连续弧段
            // 弧段内进一步按已探测周跳切片，逐片拟合 PGF 并检测
            int a = i;
            while (a < j) {
                while (a < j && slip[a]) a++; // 跳过前导周跳边界，避免死循环
                if (a >= j) break;
                int b = a;
                while (b < j && !slip[b]) b++; // [a, b) 片内无周跳
                if (const int len = b - a; len >= 6) {
                    int q = len / 100 >= 6 ? 6 : len / 100 + 1;
                    if (q >= len) q = len - 1;
                    if (q >= 1) {
                        std::vector<double> xv(len), yv(len);
                        for (int k = a; k < b; k++) {
                            xv[k - a] = k - a;
                            yv[k - a] = PGF[k];
                        }
                        auto coef = polyfit(xv, yv, q);
                        std::vector<double> r(len);
                        for (int k = a; k < b; k++) r[k - a] = LGF[k] - polyVal(coef, k - a);
                        // 连续两跳均超阈值 → 周跳
                        for (int k = a + 1; k < b - 1; k++) {
                            if (slip[k]) continue;
                            const double jumpK = std::fabs(r[k - a] - r[k - 1 - a]);
                            if (const double jumpK1 = std::fabs(r[k + 1 - a] - r[k - a]); jumpK > thr && jumpK1 > thr) slip[k] = 1;
                        }
                    }
                }
                a = b;
            }
            i = j;
        }
    }

    // ---------- 多路径滑动窗口 RMS ----------

    static double slidingWindowMp(const std::vector<double> &mp, const std::vector<char> &ok, const int n) {
        double sum = 0;
        int cnt = 0;
        for (int i = 1; i < n; i++) {
            constexpr int Nsw = 50;
            if (!ok[i]) continue;
            int a = i, len = 1;
            while (a - 1 >= 0 && ok[a - 1] && len < Nsw) {
                a--;
                len++;
            }
            if (len < 3) continue; // 线性去趋势至少 3 点
            double Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
            for (int t = 0; t < len; t++) {
                const double y = mp[a + t];
                Sx += t;
                Sy += y;
                Sxx += t * t;
                Sxy += t * y;
            }
            const double denom = static_cast<double>(len) * Sxx - Sx * Sx;
            double p1 = 0, p0 = 0;
            if (std::fabs(denom) > 1e-12) {
                p1 = (static_cast<double>(len) * Sxy - Sx * Sy) / denom;
                p0 = (Sy - p1 * Sx) / static_cast<double>(len);
            }
            double s = 0;
            for (int t = 0; t < len; t++) {
                const double r = mp[a + t] - (p0 + p1 * static_cast<double>(t));
                s += r * r;
            }
            sum += std::sqrt(s / static_cast<double>(len)); // 去趋势残差 RMS = 多路径误差
            cnt++;
        }
        return cnt ? sum / cnt : 0.0;
    }

    // ---------- 综合评估：同趋势化 + 无量纲化 + 熵权 + TOPSIS ----------
    static void computeComprehensive(QualityReport &rep) {
        ComprehensiveEval ce;
        ce.indicators = {"完整率", "周跳比", "多路径", "电离层变化率", "伪距噪声", "载波噪声", "载噪比"};
        const std::vector<SatID> &sats = rep.satOrder;
        if (sats.empty()) return;
        const int J = static_cast<int>(sats.size());
        constexpr int I = 7;

        // 构造指标矩阵 y[l][m] (l=指标, m=卫星)，并做同趋势化(全转为极大型)
        std::vector y(I, std::vector<double>(J));
        for (int m = 0; m < J; m++) {
            const SatID &sat = sats[m];
            const SatQC &q = rep.sats.at(sat);
            const double mp = 0.5 * (q.mp1Rms + q.mp2Rms);
            const double rho = 0.5 * (q.sigRho1 + q.sigRho2);
            const double ph = 0.5 * (q.sigPh1 + q.sigPh2);
            y[0][m] = q.completeness; // 完整率：越大越好
            y[1][m] = q.slipRatio; // 周跳比：越大越好
            y[2][m] = 1.0 / (mp + 1e-6); // 多路径：取倒数(极小型→极大型)
            y[3][m] = 1.0 / (q.ionoRateStd + 1e-6); // 电离层变化率：取倒数
            y[4][m] = 1.0 / (rho + 1e-6); // 伪距噪声：取倒数
            y[5][m] = 1.0 / (ph + 1e-6); // 载波噪声：取倒数
            y[6][m] = q.snrMean; // 载噪比：越大越好
        }

        // 无量纲化
        std::vector z(I, std::vector<double>(J));
        for (int l = 0; l < I; l++) {
            double mn = 1e300, mx = -1e300;
            for (int m = 0; m < J; m++) {
                mn = std::min(mn, y[l][m]);
                mx = std::max(mx, y[l][m]);
            }
            for (int m = 0; m < J; m++) {
                constexpr double dpar = 0.9;
                constexpr double cpar = 0.1;
                const double zv = mx - mn < 1e-12
                                      ? cpar + 0.5 * dpar
                                      : cpar + (y[l][m] - mn) / (mx - mn) * dpar;
                z[l][m] = zv;
                ce.z[sats[m]].push_back(zv);
            }
        }

        // 熵权
        ce.weights.assign(I, 1.0 / I);
        if (J > 1) {
            std::vector w(I, 0.0);
            const double lnJ = std::log(static_cast<double>(J));
            for (int l = 0; l < I; l++) {
                double sumZ = 0;
                for (int m = 0; m < J; m++) sumZ += z[l][m];
                double psi = 0;
                for (int m = 0; m < J; m++) {
                    if (const double p = z[l][m] / sumZ; p > 0) psi += p * std::log(p);
                }
                psi = -psi / lnJ;
                w[l] = 1.0 - psi;
            }
            double sw = 0;
            for (int l = 0; l < I; l++) sw += w[l];
            if (sw > 0) for (int l = 0; l < I; l++) ce.weights[l] = w[l] / sw;
        }

        double sumE = 0;
        for (int m = 0; m < J; m++) {
            double Em = 0;
            for (int l = 0; l < I; l++) {
                double zstar = -1e300;
                for (int mm = 0; mm < J; mm++) zstar = std::max(zstar, z[l][mm]);
                const double diff = z[l][m] - zstar;
                Em += ce.weights[l] * diff * diff;
            }
            ce.E[sats[m]] = Em;
            sumE += Em;
        }
        ce.overallE = sumE / J;

        // 各系统 E：系统内卫星 E 的平均
        std::map<char, std::vector<double> > sysEs;
        for (const SatID &sat: sats) {
            sysEs[sat.system].push_back(ce.E[sat]);
        }
        for (auto &[s, vec]: sysEs) {
            double sE = 0;
            for (double v: vec) sE += v;
            ce.sysE[s] = sE / static_cast<double>(vec.size());
        }

        rep.comprehensive = std::move(ce);
    }

    // 单星工作缓存
    struct Work {
        int n = 0;
        std::vector<double> tt, c1s, c2s, l1m, l2m, l1cyc, l2cyc, MW, LGF, PGF, mp1s, mp2s, ionos, snrs;
        std::vector<int> epIdx;
        std::vector<char> gapOk;
        double lam1 = 0, lam2 = 0;
    };

    QualityReport compute(const std::vector<QCObsEpoch> &epochs) {
        QualityReport rep;
        rep.totalEpochs = static_cast<int>(epochs.size());
        rep.totalInputEpochs = rep.totalEpochs;
        if (rep.totalEpochs < 2) return rep;

        // 历元间隔 (取中位历元差)
        {
            std::vector<double> diffs;
            for (int i = 1; i < static_cast<int>(epochs.size()); i++)
                if (double d = epochs[i].sow - epochs[i - 1].sow; d > 0 && d < 1e4) diffs.push_back(d);
            if (!diffs.empty()) {
                std::sort(diffs.begin(), diffs.end());
                rep.interval = diffs[diffs.size() / 2];
            } else rep.interval = 1.0;
        }

        std::map<SatID, std::vector<std::tuple<int, double, double, double, double, double> > > recs; // (k,sow,c1,l1,c2,l2)
        std::map<SatID, BandsInfo> bands;
        std::map<SatID, int> freq1Valid, freq2Valid;

        for (int k = 0; k < static_cast<int>(epochs.size()); k++) {
            const auto &ep = epochs[k];
            for (size_t j = 0; j < ep.satIds.size(); j++) {
                const SatID &sat = ep.satIds[j];
                const TypeValueMap &m = ep.allObs[j];
                BandsInfo b;
                if (auto it = bands.find(sat); it == bands.end()) {
                    b = pickBands(sat.system, m);
                    bands[sat] = b;
                } else b = it->second;
                if (b.hi == 0) continue;
                const double c1 = getVal(m, 'C', b.hi), l1 = getVal(m, 'L', b.hi);
                const double c2 = getVal(m, 'C', b.lo), l2 = getVal(m, 'L', b.lo);
                // 频点级有效计数 (DI_f)
                if (c1 != 0.0 && l1 != 0.0) freq1Valid[sat]++;
                if (c2 != 0.0 && l2 != 0.0) freq2Valid[sat]++;
                if (c1 == 0 || l1 == 0 || c2 == 0 || l2 == 0) continue;
                recs[sat].emplace_back(k, ep.sow, c1, l1, c2, l2);
            }
        }

        // 第一遍：构造工作缓存 + MW/GF 周跳与粗差探测
        std::map<SatID, Work> works;
        for (auto &[sat, R]: recs) {
            if (R.size() < 2) continue;
            const BandsInfo &b = bands[sat]; //NOLINT
            const double f1 = b.f1, f2 = b.f2;
            if (f1 <= 0 || f2 <= 0) continue;
            const double lam1 = C_MPS / f1, lam2 = C_MPS / f2;
            const double alpha = f1 / f2 * (f1 / f2);
            const double a = (alpha + 1.0) / (alpha - 1.0); // MP1 中 L1 系数
            const double bb = 2.0 / (alpha - 1.0); // MP1 中 L2 系数
            const double c = 2.0 * alpha / (alpha - 1.0); // MP2 中 L1 系数
            const double d = (alpha + 1.0) / (alpha - 1.0); // MP2 中 L2 系数

            Work w;
            w.n = static_cast<int>(R.size());
            w.tt.resize(w.n);
            w.c1s.resize(w.n);
            w.c2s.resize(w.n);
            w.l1m.resize(w.n);
            w.l2m.resize(w.n);
            w.l1cyc.resize(w.n);
            w.l2cyc.resize(w.n);
            w.MW.resize(w.n);
            w.LGF.resize(w.n);
            w.PGF.resize(w.n);
            w.mp1s.resize(w.n);
            w.mp2s.resize(w.n);
            w.ionos.resize(w.n);
            w.snrs.resize(w.n);
            w.epIdx.resize(w.n);
            w.gapOk.assign(w.n, 1);

            for (int i = 0; i < w.n; i++) {
                const auto &[kk, sow, c1, l1, c2, l2] = R[i];
                w.tt[i] = sow;
                w.c1s[i] = c1;
                w.c2s[i] = c2;
                w.l1m[i] = l1;
                w.l2m[i] = l2;
                w.l1cyc[i] = l1 / lam1;
                w.l2cyc[i] = l2 / lam2;
                w.MW[i] = (f1 * l1 - f2 * l2) / (f1 - f2) - (f1 * c1 + f2 * c2) / (f1 + f2); // MW (m)
                w.LGF[i] = l2 - l1; // GF 组合 (m)
                w.PGF[i] = c2 - c1; // 伪距 GF 组合 (m)
                w.mp1s[i] = c1 - a * l1 + bb * l2; // Estey-Meertens MP1 (m)
                w.mp2s[i] = c2 - c * l1 + d * l2; // Estey-Meertens MP2 (m)
                w.ionos[i] = f2 * f2 / (f1 * f1 - f2 * f2) * (l1 - l2); // 电离层 GF 组合 (m)
                w.epIdx[i] = kk;
            }
            // 断档标记
            for (int i = 1; i < w.n; i++)
                if (std::fabs(w.tt[i] - w.tt[i - 1] - rep.interval) > 0.5 * rep.interval) w.gapOk[i] = 0;

            //周跳/粗差探测
            SatQC q;
            q.sat = sat;
            q.band1 = b.hi;
            q.band2 = b.lo;
            q.totalEpochs = rep.totalEpochs;
            q.validDual = w.n;
            detectMW(w.MW, w.gapOk, q.slipFlag, q.outlierFlag);
            detectGF(w.LGF, w.PGF, w.gapOk, lam1, lam2, q.slipFlag);
            q.clockJumpFlag.assign(w.n, 0); // 钟跳标记，初始化为 0
            q.slips = static_cast<int>(std::count(q.slipFlag.begin(), q.slipFlag.end(), 1));
            q.outliers = static_cast<int>(std::count(q.outlierFlag.begin(), q.outlierFlag.end(), 1));
            q.slipRatio = q.slips > 0
                              ? static_cast<double>(rep.totalEpochs) / q.slips
                              : static_cast<double>(rep.totalEpochs);
            q.di_f1 = 100.0 * freq1Valid[sat] / rep.totalEpochs;
            q.di_f2 = 100.0 * freq2Valid[sat] / rep.totalEpochs;
            q.completeness = 100.0 * w.n / rep.totalEpochs;
            w.lam1 = lam1;
            w.lam2 = lam2;
            works[sat] = std::move(w);
            rep.sats[sat] = std::move(q);
        }

        // 接收机钟跳探测 (全局跨星)
        {
            constexpr double c = C_MPS;
            constexpr double xi = 4.0; // 经验观测噪声 (m)
            constexpr double msLo = 1e-7 * c - 3.0 * xi, msHi = 1e-5 * c + 3.0 * xi; // 毫秒级
            constexpr double usThr = 1e-3 * c - 3.0 * xi; // 微秒级
            // 建立 卫星→全局历元 索引表
            std::map<SatID, std::vector<int> > locAt;
            for (auto &[sat, w]: works) {
                locAt[sat].assign(rep.totalEpochs, -1);
                for (int i = 0; i < w.n; i++) locAt[sat][w.epIdx[i]] = i;
            }
            for (int gi = 0; gi + 1 < rep.totalEpochs; gi++) {
                int nms = 0, nus = 0, totalValid = 0;
                for (auto &[sat, w]: works) {
                    const int li = locAt[sat][gi], lj = locAt[sat][gi + 1];
                    if (li < 0 || lj < 0) continue;
                    if (const double dL = w.c1s[lj] - w.c1s[li] - (w.l1m[lj] - w.l1m[li]); dL > msLo && dL < msHi) nms++;
                    else if (dL > usThr) nus++;
                    totalValid++;
                }
                if (const int ns = nms + nus; totalValid >= 4 && ns >= 4 && ns >= 0.6 * totalValid) {
                    for (auto &[sat, w]: works) {
                        const int lj = locAt[sat][gi + 1];
                        if (lj < 0) continue;
                        rep.sats[sat].slipFlag[lj] = 0; // 钟跳导致的"周跳"为误报，撤销
                        rep.sats[sat].outlierFlag[lj] = 0;
                        rep.sats[sat].clockJumpFlag[lj] = 1;
                        rep.sats[sat].clockJumps++;
                    }
                    rep.clockJumpEpochs++;
                }
            }
        }

        // 第二遍：最终弧段 + 各指标计算
        for (auto &[sat, w]: works) {
            SatQC &q = rep.sats[sat];
            const int n = w.n;
            // 断档 / 周跳 / 钟跳 处断开
            std::vector<char> ok(n, 1);
            for (int i = 0; i < n; i++)
                if (!w.gapOk[i] || q.slipFlag[i] || q.clockJumpFlag[i]) ok[i] = 0;

            // 观测值噪声 (三次差)
            q.sigRho1 = tripleStd(w.c1s, w.tt, rep.interval, ok);
            q.sigRho2 = tripleStd(w.c2s, w.tt, rep.interval, ok);
            q.sigPh1 = tripleStd(w.l1cyc, w.tt, rep.interval, ok);
            q.sigPh2 = tripleStd(w.l2cyc, w.tt, rep.interval, ok);

            // 多路径 (滑动窗口 RMS)
            q.mp1Rms = slidingWindowMp(w.mp1s, ok, n);
            q.mp2Rms = slidingWindowMp(w.mp2s, ok, n);
            q.mp1Mean = Math::mean(w.mp1s);
            q.mp2Mean = Math::mean(w.mp2s);

            // 电离层延迟变化率 IOD
            std::vector ionoRate(n, 0.0);
            for (int i = 1; i < n; i++) {
                if (!ok[i] || !ok[i - 1]) continue;
                const double dt = w.tt[i] - w.tt[i - 1];
                if (dt <= 0 || std::fabs(dt - rep.interval) > 0.5 * rep.interval) continue;
                ionoRate[i] = (w.ionos[i] - w.ionos[i - 1]) / dt;
            }
            double sr = 0;
            int nc = 0, ionoJumps = 0;
            for (int i = 0; i < n; i++) {
                if (ionoRate[i] != 0.0) {
                    sr += ionoRate[i] * ionoRate[i];
                    nc++;
                }
                if (std::fabs(ionoRate[i]) > 0.07) ionoJumps++;
            }
            q.ionoRateStd = nc > 0 ? std::sqrt(sr / nc) : 0.0;
            q.ionoJumps = ionoJumps;

            // 载噪比 (逐星均值)
            for (int i = 0; i < n; i++) {
                const auto &ep = epochs[w.epIdx[i]];
                int idx = -1;
                for (int j = 0; j < static_cast<int>(ep.satIds.size()); j++)
                    if (ep.satIds[j] == sat) {
                        idx = j;
                        break;
                    }
                if (idx >= 0) w.snrs[i] = getSnr(ep.allObs[idx], q.band1);
            }
            {
                double s = 0, mn = 1e300, mx = -1e300;
                int cnt = 0;
                for (double v: w.snrs)
                    if (v > 0) {
                        s += v;
                        cnt++;
                        mn = std::min(mn, v);
                        mx = std::max(mx, v);
                    }
                if (cnt == 0) {
                    q.snrMean = 0;
                    q.snrMin = 0;
                    q.snrMax = 0;
                } else {
                    q.snrMean = s / cnt;
                    q.snrMin = mn;
                    q.snrMax = mx;
                }
            }
            q.snr = w.snrs;


            // 绘图序列
            q.epIdx = w.epIdx;
            q.t = w.tt;
            // 保存原始 MP 序列
            q.mp1 = std::move(w.mp1s);
            q.mp2 = std::move(w.mp2s);
            q.ionoRate = std::move(ionoRate);

            rep.satOrder.push_back(sat);
        }

        // 系统 / 总体聚合
        std::map<char, double> sumComp, sumDI1, sumDI2, sumSlipRatio, sumMp1, sumMp2;
        std::map<char, double> sumSigRho, sumSigPh, sumSnr, sumIonoRate;
        std::map<char, int> cnt, sumSlips, sumIono, sumOut, sumClk, cntSnr, cntIono;
        for (auto &[sat, q]: rep.sats) {
            const char s = sat.system;
            sumComp[s] += q.completeness;
            sumDI1[s] += q.di_f1;
            sumDI2[s] += q.di_f2;
            sumSlipRatio[s] += q.slipRatio;
            sumMp1[s] += q.mp1Rms;
            sumMp2[s] += q.mp2Rms;
            sumSigRho[s] += 0.5 * (q.sigRho1 + q.sigRho2);
            sumSigPh[s] += 0.5 * (q.sigPh1 + q.sigPh2);
            sumSlips[s] += q.slips;
            sumIono[s] += q.ionoJumps;
            sumOut[s] += q.outliers;
            sumClk[s] += q.clockJumps;
            sumIonoRate[s] += q.ionoRateStd;
            cntIono[s]++;
            if (q.snrMean > 0) {
                sumSnr[s] += q.snrMean;
                cntSnr[s]++;
            }
            cnt[s]++;
        }
        for (auto &[s, c]: cnt) {
            rep.sysCompleteness[s] = sumComp[s] / c;
            rep.sysDI_f1[s] = sumDI1[s] / c;
            rep.sysDI_f2[s] = sumDI2[s] / c;
            rep.sysSlipRatio[s] = sumSlipRatio[s] / c;
            rep.sysMp1[s] = sumMp1[s] / c;
            rep.sysMp2[s] = sumMp2[s] / c;
            rep.sysSigRho[s] = sumSigRho[s] / c;
            rep.sysSigPhase[s] = sumSigPh[s] / c;
            rep.sysSlips[s] = sumSlips[s];
            rep.sysIonoJumps[s] = sumIono[s];
            rep.sysOutliers[s] = sumOut[s];
            rep.sysClockJumps[s] = sumClk[s];
            rep.sysIonoRate[s] = cntIono.count(s) ? sumIonoRate[s] / cntIono[s] : 0.0;
            if (cntSnr.count(s)) rep.sysSnr[s] = sumSnr[s] / cntSnr[s];
        }
        double allComp = 0, allSnr = 0;
        int allCnt = 0, allSnrCnt = 0;
        for (auto &[sat, q]: rep.sats) {
            allComp += q.completeness;
            allCnt++;
            if (q.snrMean > 0) {
                allSnr += q.snrMean;
                allSnrCnt++;
            }
        }
        rep.overallCompleteness = allCnt ? allComp / allCnt : 0;
        rep.overallSnr = allSnrCnt ? allSnr / allSnrCnt : 0;

        // 6综合评估
        computeComprehensive(rep);
        return rep;
    }
} // namespace QC
