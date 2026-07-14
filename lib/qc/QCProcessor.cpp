#include "QCProcessor.h"
#include "Const.h"   // getFreq, C_MPS
#include "../util/StatsUtils.h"   // Stats::mean / rms / median
#include <algorithm>
#include <tuple>

namespace QC {
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

    // 选择双频组合：返回 {高频 band, 低频 band, 及其真实信号频率}
    struct BandsInfo {
        char hi = 0, lo = 0;
        double f1 = 0, f2 = 0;
    };

    // 取 m 中某观测类型(码/相)的实际信号码（含属性字符，如 "C1I"/"C2X"），
    // 直接交给 getFreq 即可拿到真实频率——避免只存 band 单字符后还得回头重扫 m 判断 I/C。
    static std::string findObsType(const TypeValueMap &m, const char t, const char b) {
        for (auto &[k, v]: m)
            if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return k;
        return "";
    }

    // 在所有「同时有 C 与 L」的频点里，挑频率最高的两个做 IF 组合。
    // 频率由 getFreq(sys, 真实信号码) 直接得出（getFreq 已按第3字符区分 BDS 的 B1I/B1C），
    // 不再另写一份频率判定逻辑。
    static BandsInfo pickBands(const char sys, const TypeValueMap &m) {
        std::vector<std::pair<char, std::string>> cand; // (band, 实际信号码)
        for (char b: {'1', '2', '5', '6', '7', '8'}) {
            std::string ct = findObsType(m, 'C', b);
            std::string lt = findObsType(m, 'L', b);//NOLINT
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

    // 按连续弧段(避开 gap/周跳)去均值(= 该弧段 ambig 常数)，结果为围绕 0 的小波动，MP RMS 才有物理意义。
    static void centerArcs(std::vector<double> &y, const std::vector<char> &ok, int n) {
        int i = 0;
        while (i < n) {
            int j = i;
            while (j < n && (j == i || ok[j] == 1)) j++; // [i, j) 为一个弧段
            if (const int len = j - i; len >= 1) {
                double m = 0;
                for (int k = i; k < j; k++) m += y[k];
                m /= len;
                for (int k = i; k < j; k++) y[k] -= m;
            }
            i = j;
        }
    }

    // 对 >0 的观测值求 (均值, 最小, 最大)
    static std::tuple<double, double, double> positiveStats(const std::vector<double> &v) {
        double s = 0, mn = 1e300, mx = -1e300;
        int cnt = 0;
        for (double x: v)
            if (x > 0) { s += x; cnt++; mn = std::min(mn, x); mx = std::max(mx, x); }
        if (cnt == 0) return {0.0, 0.0, 0.0};
        return {s / cnt, mn, mx};
    }

    /// 观测值噪声：三次差标准差
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
        if (cnt < 1) return 0;
        return std::sqrt(s / cnt / 20.0);
    }


    struct Rec {
        int k;
        double sow, c1, l1, c2, l2;
    };

    QualityReport compute(const std::vector<QCObsEpoch> &epochs) {
        QualityReport rep;
        rep.totalEpochs = static_cast<int>(epochs.size());
        if (rep.totalEpochs < 2) {
            return rep;
        }

        // 历元间隔 (取中位历元差)
        {
            std::vector<double> diffs;
            for (int i = 1; i < static_cast<int>(epochs.size()); i++) {
                if (double d = epochs[i].sow - epochs[i - 1].sow; d > 0 && d < 1e4) diffs.push_back(d);
            }
            if (!diffs.empty()) {
                std::sort(diffs.begin(), diffs.end());
                rep.interval = diffs[diffs.size() / 2];
            } else rep.interval = 1.0;
        }

        std::map<SatID, std::vector<Rec> > recs;
        std::map<SatID, BandsInfo> bands;

        // 历元间隔已估算，开始按卫星聚合双频序列

        for (int k = 0; k < static_cast<int>(epochs.size()); k++) {
            const auto &ep = epochs[k];
            for (size_t j = 0; j < ep.satIds.size(); j++) {
                const SatID &sat = ep.satIds[j];
                const TypeValueMap &m = ep.allObs[j];
                auto it = bands.find(sat);
                BandsInfo b;
                if (it == bands.end()) {
                    b = pickBands(sat.system, m);
                    bands[sat] = b;
                } else b = it->second;
                if (b.hi == 0) continue;
                double c1 = getVal(m, 'C', b.hi), l1 = getVal(m, 'L', b.hi);
                double c2 = getVal(m, 'C', b.lo), l2 = getVal(m, 'L', b.lo);
                if (c1 == 0 || l1 == 0 || c2 == 0 || l2 == 0) continue;
                recs[sat].push_back({k, ep.sow, c1, l1, c2, l2});
            }
        }

        const size_t totalSats = recs.size();
        size_t satDone = 0;
        for (auto &[sat, R]: recs) {
            if (R.size() < 2) continue;
            char b1 = bands[sat].hi, b2 = bands[sat].lo;
            double f1 = bands[sat].f1, f2 = bands[sat].f2;
            if (f1 <= 0 || f2 <= 0) continue;
            double lam1 = C_MPS / f1, lam2 = C_MPS / f2;
            double lamWL = C_MPS / (f1 - f2);
            double alpha = f1 / f2 * (f1 / f2);
            double a = (alpha + 1.0) / (alpha - 1.0); // MP1 中 L1 系数
            double b = 2.0 / (alpha - 1.0); // MP1 中 L2 系数
            double c = 2.0 * alpha / (alpha - 1.0); // MP2 中 L1 系数
            double d = (alpha + 1.0) / (alpha - 1.0); // MP2 中 L2 系数

            SatQC q;
            q.sat = sat;
            q.band1 = b1;
            q.band2 = b2;
            q.totalEpochs = rep.totalEpochs;
            q.validDual = static_cast<int>(R.size());
            q.completeness = 100.0 * q.validDual / q.totalEpochs;

            int n = q.validDual;
            std::vector<double> tt(n);
            for (int i = 0; i < n; i++) tt[i] = R[i].sow;
            std::vector<double> MW(n), L4(n), mp1s(n), mp2s(n), ionos(n);
            std::vector<double> c1s(n), l1cyc(n), c2s(n), l2cyc(n);
            for (int i = 0; i < n; i++) {
                double c1 = R[i].c1, l1 = R[i].l1, c2 = R[i].c2, l2 = R[i].l2;
                MW[i] = (f1 * l1 - f2 * l2) / (f1 - f2) - (f1 * c1 + f2 * c2) / (f1 + f2);
                L4[i] = l1 / lam1 - f1 / f2 * (l2 / lam2); // 几何无关组合(周)，仅用于周跳探测，不对外展示
                mp1s[i] = c1 - a * l1 + b * l2; // Estey-Meertens MP1 (m)
                mp2s[i] = c2 - c * l1 + d * l2; // Estey-Meertens MP2 (m)
                ionos[i] = f2 * f2 / (f1 * f1 - f2 * f2) * (l1 - l2);
                c1s[i] = c1;
                c2s[i] = c2;
                l1cyc[i] = l1 / lam1;
                l2cyc[i] = l2 / lam2;
            }

            // ===== 周跳 / 失锁探测 (在去趋势之前，用原始 MW/L4 差分) =====
            std::vector<double> dL4(n, 0), dMW(n, 0);
            for (int i = 1; i < n; i++) {
                dL4[i] = L4[i] - L4[i - 1];
                dMW[i] = MW[i] - MW[i - 1];
            }
            std::vector<double> dL4g, dMWg;
            for (int i = 1; i < n; i++)
                if (std::fabs(tt[i] - tt[i - 1] - rep.interval) <= 0.5 * rep.interval) {
                    dL4g.push_back(dL4[i]);
                    dMWg.push_back(dMW[i]);
                }
            double medL = Stats::median(dL4g), medW = Stats::median(dMWg);
            std::vector<double> aL, aW;
            for (double v: dL4g) aL.push_back(std::fabs(v - medL));
            for (double v: dMWg) aW.push_back(std::fabs(v - medW));
            double madL = 1.4826 * Stats::median(aL), madW = 1.4826 * Stats::median(aW);
            double thrL = std::max(5.0 * madL, 20.0); // 周
            double thrW = std::max(5.0 * madW, 0.5 * lamWL); // m (宽巷)
            q.slipFlag.assign(n, 0);
            int nSlip = 0;
            for (int i = 1; i < n; i++) {
                if (bool gap = std::fabs(tt[i] - tt[i - 1] - rep.interval) > 0.5 * rep.interval;
                    !gap && (std::fabs(dL4[i] - medL) > thrL || std::fabs(dMW[i] - medW) > thrW)) {
                    q.slipFlag[i] = 1;
                    nSlip++;
                }
            }
            q.slips = nSlip;
            q.slipRatio = q.slips > 0 ? static_cast<double>(q.totalEpochs) / q.slips : static_cast<double>(q.totalEpochs);

            // ===== 弧段掩码：gap 或 周跳处断开，弧段内连续点 ok=1 =====
            std::vector<char> ok(n, 1);
            for (int i = 1; i < n; i++) {
                if (std::fabs(tt[i] - tt[i - 1] - rep.interval) > 0.5 * rep.interval || q.slipFlag[i]) ok[i] = 0;
            }

            // 观测值噪声 (三次差)：仅弧段内连续历元求差(避开 gap/周跳放大)
            q.sigRho1 = tripleStd(c1s, tt, rep.interval, ok);
            q.sigRho2 = tripleStd(c2s, tt, rep.interval, ok);
            q.sigPh1 = tripleStd(l1cyc, tt, rep.interval, ok);
            q.sigPh2 = tripleStd(l2cyc, tt, rep.interval, ok);

            // ===== MP 去均值：按连续弧段(避开 gap/周跳)移除该弧段的整周模糊度常数(均值) =====
            // 按连续弧段去均值(= 该弧段 ambig 常数)，结果为围绕 0 的小波动，MP RMS 才有物理意义。
            centerArcs(mp1s, ok, n);
            centerArcs(mp2s, ok, n);

            // 统计量在去均值序列上计算
            q.mp1Mean = Stats::mean(mp1s);
            q.mp1Rms = Stats::rms(mp1s);
            q.mp2Mean = Stats::mean(mp2s);
            q.mp2Rms = Stats::rms(mp2s);

            // ===== 电离层残差变化率 IOD (BD 420022—2019 §6.2.3) =====
            // IOD = ΔI/Δt, I = 几何无关相位组合 LGF = (f2²/(f1²−f2²))·(L1−L2) (m)。
            // 有限差分天然消去整周模糊度常数与线性漂移 → 无需去趋势；跨断档/周跳不差分。
            // 判定阈值 0.07 m/s (标准 §6.2.3)。
            std::vector ionoRate(n, 0.0);
            for (int i = 1; i < n; i++) {
                if (!ok[i] || !ok[i - 1]) continue; // 跨断档/周跳不差分
                double dt = tt[i] - tt[i - 1];
                if (dt <= 0 || std::fabs(dt - rep.interval) > 0.5 * rep.interval) continue;
                ionoRate[i] = (ionos[i] - ionos[i - 1]) / dt;
            }
            double sr = 0;
            int nc = 0;
            for (double v: ionoRate)
                if (v != 0.0) {
                    sr += v * v;
                    nc++;
                }
            double ionoRateStd = nc > 0 ? std::sqrt(sr / nc) : 0.0;
            int ionoJumps = 0;
            for (int i = 1; i < n; i++) if (std::fabs(ionoRate[i]) > 0.07) ionoJumps++;

            // 从原始历元中找回 SNR/EL/AZ (与 satIds 同序)
            std::vector<double> snrs(n, 0);
            for (int i = 0; i < n; i++) {
                const auto &ep = epochs[R[i].k];
                int idx = -1;
                for (int j = 0; j < static_cast<int>(ep.satIds.size()); j++) if (ep.satIds[j] == sat) {
                    idx = j;
                    break;
                }
                if (idx >= 0) {
                    snrs[i] = getSnr(ep.allObs[idx], b1);
                }
            }
            auto [snrMean, snrMin, snrMax] = positiveStats(snrs);
            q.snr = std::move(snrs);
            q.snrMean = snrMean;
            q.snrMin = snrMin;
            q.snrMax = snrMax;

            q.epIdx.reserve(n);
            q.t = tt;
            q.mp1 = std::move(mp1s);
            q.mp2 = std::move(mp2s);
            q.ionoRate = std::move(ionoRate);
            q.ionoRateStd = ionoRateStd;
            q.ionoJumps = ionoJumps;
            for (int i = 0; i < n; i++) q.epIdx.push_back(R[i].k);

            rep.sats[sat] = std::move(q);
            rep.satOrder.push_back(sat);

            ++satDone;
        }

        // 系统 / 总体聚合
        std::map<char, double> sumComp, sumSlipRatio, sumMp1, sumMp2, sumSigRho, sumSigPh, sumSnr;
        std::map<char, int> cnt, sumSlips, sumIono, cntSnr;
        for (auto &[sat, q]: rep.sats) {
            char s = sat.system;
            sumComp[s] += q.completeness;
            sumSlipRatio[s] += q.slipRatio;
            sumMp1[s] += q.mp1Rms;
            sumMp2[s] += q.mp2Rms;
            sumSigRho[s] += (q.sigRho1 + q.sigRho2) / 2.0;
            sumSigPh[s] += (q.sigPh1 + q.sigPh2) / 2.0;
            sumSlips[s] += q.slips;
            sumIono[s] += q.ionoJumps;
            if (q.snrMean > 0) {
                sumSnr[s] += q.snrMean;
                cntSnr[s]++;
            }
            cnt[s]++;
        }
        for (auto &[s, c]: cnt) {
            rep.sysCompleteness[s] = sumComp[s] / c;
            rep.sysSlipRatio[s] = sumSlipRatio[s] / c;
            rep.sysMp1[s] = sumMp1[s] / c;
            rep.sysMp2[s] = sumMp2[s] / c;
            rep.sysSigRho[s] = sumSigRho[s] / c;
            rep.sysSigPhase[s] = sumSigPh[s] / c;
            rep.sysSlips[s] = sumSlips[s];
            rep.sysIonoJumps[s] = sumIono[s];
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
        return rep;
    }
} // namespace QC
