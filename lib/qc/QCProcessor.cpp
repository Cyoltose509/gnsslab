#include "QCProcessor.h"
#include "Const.h"   // getFreq, C_MPS
#include <cmath>
#include <algorithm>
#include <numeric>

namespace QC {

static double getVal(const TypeValueMap &m, char t, char b) {
    for (auto &[k, v] : m) {
        if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return v;
    }
    return 0.0;
}

static bool hasObs(const TypeValueMap &m, char t, char b) { return getVal(m, t, b) != 0.0; }

// 取某频点的信噪比；RINEX 中用 Sxx 表示（如 S1I, S2P, S6I）
static double getSnr(const TypeValueMap &m, char b) {
    for (auto &[k, v] : m) {
        if (k.size() >= 2 && k[0] == 'S' && k[1] == b && v > 0.0) return v;
    }
    return 0.0;
}

// 选择双频组合：返回 {高频, 低频, 及其真实信号频率}
struct BandsInfo { char hi = 0, lo = 0; double f1 = 0, f2 = 0; };

// RINEX3 真实信号频率：BDS 的 "C1I"(B1I)=1561.098 在 getFreq 里被误标到 band '2'，
// 而 band '1' 返回的是 B1C(1575.42)。组合必须用实际信号频率，否则几何/电离层无法消去。
static double signalFreq(char sys, char band, const TypeValueMap &m) {
    if (sys == 'G') return getFreq(sys, std::string("C") + band);
    char attr = 'I';
    for (auto &[k, v] : m)
        if (k.size() >= 3 && k[0] == 'C' && k[1] == band) { attr = k[2]; break; }
    if (band == '1') return (attr == 'C') ? 1575.420e6 : 1561.098e6; // B1C / B1I
    if (band == '2') return 1561.098e6;                              // B1I(legacy)
    if (band == '5') return 1176.450e6;                              // B2a
    if (band == '6') return 1268.520e6;                              // B3I
    if (band == '7') return 1207.140e6;                              // B2b
    if (band == '8') return 1191.795e6;                              // B2
    return 0.0;
}

static BandsInfo pickBands(char sys, const TypeValueMap &m) {
    std::vector<char> cand;
    for (char b : {'1', '2', '5', '6', '7', '8'}) {
        if (hasObs(m, 'C', b) && hasObs(m, 'L', b)) cand.push_back(b);
    }
    if (cand.size() < 2) return {};
    std::sort(cand.begin(), cand.end(), [&](char a, char b) {
        return signalFreq(sys, a, m) > signalFreq(sys, b, m);
    });
    BandsInfo r;
    r.hi = cand[0]; r.lo = cand[1];
    r.f1 = signalFreq(sys, r.hi, m);
    r.f2 = signalFreq(sys, r.lo, m);
    return r;
}

static double mean(const std::vector<double> &x) {
    if (x.empty()) return 0;
    double s = 0; for (double v : x) s += v; return s / x.size();
}
static double rms(const std::vector<double> &x) {
    if (x.size() < 2) return 0;
    double m = mean(x); double s = 0; for (double v : x) s += (v - m) * (v - m);
    return std::sqrt(s / (x.size() - 1));
}
// 观测值噪声：三次差标准差 (BD 420022—2019 §6.5/6.6)
// 关键：只在「同一弧段内」的连续历元做差。卫星升落/失锁/周跳造成断档，跨断档求三次差
// 会把两侧数百米的几何漂移误当噪声(实测可达上千米)；只在 dt≈interval 且未跨越
// gap/周跳的相邻历元内计算才是真实观测噪声。ok[i]==1 表示该点可参与(同弧段连续)。
// 三次差白噪声方差 = 20σ² → σ = sqrt(mean(Δ³²)/20)。
static double tripleStd(const std::vector<double> &x, const std::vector<double> &t,
                        double interval, const std::vector<char> &ok) {
    int n = (int)x.size(); if (n < 4) return 0;
    double s = 0; int cnt = 0;
    for (int i = 3; i < n; i++) {
        if (!ok[i] || !ok[i - 1] || !ok[i - 2] || !ok[i - 3]) continue;
        bool cont = true;
        for (int k = i - 2; k <= i; k++)
            if (std::fabs((t[k] - t[k - 1]) - interval) > 0.5 * interval) { cont = false; break; }
        if (!cont) continue;
        double d3 = x[i] - 3.0 * x[i - 1] + 3.0 * x[i - 2] - x[i - 3];
        s += d3 * d3; cnt++;
    }
    if (cnt < 1) return 0;
    return std::sqrt(s / cnt / 20.0);
}

// 局部(单弧段)多项式最小二乘拟合并返回残差。x 归一化到 [0,1] 提升数值稳定性。
static std::vector<double> polyDetrendLocal(const std::vector<double> &t, const std::vector<double> &y, int deg) {
    int n = (int)t.size(); int m = deg + 1;
    if (n < m) return y;
    double t0 = t.front(), ts = (t.back() - t.front()); if (std::fabs(ts) < 1e-9) ts = 1;
    std::vector<double> xx(n); for (int i = 0; i < n; i++) xx[i] = (t[i] - t0) / ts;
    std::vector<std::vector<double>> A(m, std::vector<double>(m, 0));
    std::vector<double> B(m, 0);
    for (int i = 0; i < n; i++) {
        std::vector<double> pows(2 * m - 1); double acc = 1;
        for (int p = 0; p < 2 * m - 1; p++) { pows[p] = acc; acc *= xx[i]; }
        for (int r = 0; r < m; r++) { B[r] += pows[r] * y[i]; for (int cc = 0; cc < m; cc++) A[r][cc] += pows[r + cc]; }
    }
    for (int col = 0; col < m; col++) {
        int piv = col; for (int r = col + 1; r < m; r++) if (std::fabs(A[r][col]) > std::fabs(A[piv][col])) piv = r;
        std::swap(A[col], A[piv]); std::swap(B[col], B[piv]);
        if (std::fabs(A[col][col]) < 1e-15) continue;
        for (int r = 0; r < m; r++) { if (r == col) continue; double f = A[r][col] / A[col][col];
            for (int cc = col; cc < m; cc++) A[r][cc] -= f * A[col][cc]; B[r] -= f * B[col]; }
    }
    std::vector<double> c(m, 0); for (int r = 0; r < m; r++) c[r] = (std::fabs(A[r][r]) > 1e-15) ? B[r] / A[r][r] : 0;
    std::vector<double> res(n);
    for (int i = 0; i < n; i++) { double f = 0, acc = 1; for (int p = 0; p < m; p++) { f += c[p] * acc; acc *= xx[i]; } res[i] = y[i] - f; }
    return res;
}

struct Rec { int k; double sow, c1, l1, c2, l2; };

QualityReport compute(const std::vector<QCObsEpoch> &epochs) {
    QualityReport rep;
    rep.totalEpochs = (int)epochs.size();
    if (rep.totalEpochs < 2) return rep;

    // 历元间隔 (取中位历元差)
    {
        std::vector<double> diffs;
        for (int i = 1; i < (int)epochs.size(); i++) {
            double d = epochs[i].sow - epochs[i - 1].sow;
            if (d > 0 && d < 1e4) diffs.push_back(d);
        }
        if (!diffs.empty()) { std::sort(diffs.begin(), diffs.end()); rep.interval = diffs[diffs.size() / 2]; }
        else rep.interval = 1.0;
    }

    // 与 RTKLIB 对齐：DOP/Nsat 时序（解算器提供时填充）
    for (const auto &ep : epochs) {
        rep.t.push_back(ep.sow);
        rep.gdop.push_back(ep.gdop);
        rep.pdop.push_back(ep.pdop);
        rep.hdop.push_back(ep.hdop);
        rep.vdop.push_back(ep.vdop);
        rep.nsat.push_back(ep.nsat);
    }

    std::map<SatID, std::vector<Rec>> recs;
    std::map<SatID, BandsInfo> bands;

    for (int k = 0; k < (int)epochs.size(); k++) {
        const auto &ep = epochs[k];
        for (size_t j = 0; j < ep.satIds.size(); j++) {
            const SatID &sat = ep.satIds[j];
            const TypeValueMap &m = ep.allObs[j];
            auto it = bands.find(sat);
            BandsInfo b;
            if (it == bands.end()) { b = pickBands(sat.system, m); bands[sat] = b; }
            else b = it->second;
            if (b.hi == 0) continue;
            double c1 = getVal(m, 'C', b.hi), l1 = getVal(m, 'L', b.hi);
            double c2 = getVal(m, 'C', b.lo), l2 = getVal(m, 'L', b.lo);
            if (c1 == 0 || l1 == 0 || c2 == 0 || l2 == 0) continue;
            recs[sat].push_back({k, ep.sow, c1, l1, c2, l2});
        }
    }

    for (auto &[sat, R] : recs) {
        if (R.size() < 2) continue;
        char b1 = bands[sat].hi, b2 = bands[sat].lo;
        double f1 = bands[sat].f1, f2 = bands[sat].f2;
        if (f1 <= 0 || f2 <= 0) continue;
        double lam1 = C_MPS / f1, lam2 = C_MPS / f2;
        double lamWL = C_MPS / (f1 - f2);
        // 严格几何无关 + 电离层无关的 Estey & Meertens (1999) 多路径组合：
        // MP1 = P1 - a·L1 + b·L2, MP2 = P2 - c·L1 + d·L2, α=(f1/f2)^2
        double alpha = (f1 / f2) * (f1 / f2);
        double a = (alpha + 1.0) / (alpha - 1.0);   // MP1 中 L1 系数
        double b = 2.0 / (alpha - 1.0);             // MP1 中 L2 系数
        double c = 2.0 * alpha / (alpha - 1.0);     // MP2 中 L1 系数
        double d = (alpha + 1.0) / (alpha - 1.0);   // MP2 中 L2 系数

        SatQC q; q.sat = sat; q.band1 = b1; q.band2 = b2;
        q.totalEpochs = rep.totalEpochs;
        q.validDual = (int)R.size();
        q.completeness = 100.0 * q.validDual / q.totalEpochs;

        int n = (int)R.size();
        std::vector<double> tt(n);
        for (int i = 0; i < n; i++) tt[i] = R[i].sow;
        std::vector<double> MW(n), L4(n), mp1s(n), mp2s(n), ionos(n);
        std::vector<double> c1s(n), l1cyc(n), c2s(n), l2cyc(n);
        for (int i = 0; i < n; i++) {
            double c1 = R[i].c1, l1 = R[i].l1, c2 = R[i].c2, l2 = R[i].l2;
            MW[i] = (f1 * l1 - f2 * l2) / (f1 - f2) - (f1 * c1 + f2 * c2) / (f1 + f2);
            // 几何无关组合 (周)：L4 = φ1 - (f1/f2)·φ2，消去几何项，残余为电离层/模糊度常数
            L4[i] = l1 / lam1 - (f1 / f2) * (l2 / lam2);
            mp1s[i] = c1 - a * l1 + b * l2;   // Estey-Meertens MP1 (m)
            mp2s[i] = c2 - c * l1 + d * l2;   // Estey-Meertens MP2 (m)
            ionos[i] = (f2 * f2 / (f1 * f1 - f2 * f2)) * (l1 - l2);
            c1s[i] = c1; c2s[i] = c2; l1cyc[i] = l1 / lam1; l2cyc[i] = l2 / lam2;
        }

        // ===== 周跳 / 失锁探测 (在去趋势之前，用原始 MW/L4 差分) =====
        // 关键陷阱：BDS 相位含 ~1.79e6 周整周模糊度常数；组合本身还带有 ~10 cyc/epoch
        // 的线性漂移(电离层/时钟相关)。旧判据 |dL4|>2 会把「每个历元的正常漂移」都误判
        // 成周跳(实测 C07 每历元都触发)。真实周跳是偏离典型漂移的「巨大跳变」(如 C10 的
        // 178 万周台阶)。故改用「相对中位漂移的离群」判据，对漂移免疫、只对真跳变敏感。
        std::vector<double> dL4(n, 0), dMW(n, 0);
        for (int i = 1; i < n; i++) { dL4[i] = L4[i] - L4[i - 1]; dMW[i] = MW[i] - MW[i - 1]; }
        auto medianOf = [](std::vector<double> v) {
            if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[v.size() / 2];
        };
        std::vector<double> dL4g, dMWg;
        for (int i = 1; i < n; i++)
            if (std::fabs((tt[i] - tt[i - 1]) - rep.interval) <= 0.5 * rep.interval) { dL4g.push_back(dL4[i]); dMWg.push_back(dMW[i]); }
        double medL = medianOf(dL4g), medW = medianOf(dMWg);
        std::vector<double> aL, aW;
        for (double v : dL4g) aL.push_back(std::fabs(v - medL));
        for (double v : dMWg) aW.push_back(std::fabs(v - medW));
        double madL = 1.4826 * medianOf(aL), madW = 1.4826 * medianOf(aW);
        double thrL = std::max(5.0 * madL, 20.0);          // 周
        double thrW = std::max(5.0 * madW, 0.5 * lamWL);    // m (宽巷)
        q.slipFlag.assign(n, 0); int nSlip = 0;
        for (int i = 1; i < n; i++) {
            bool gap = std::fabs((tt[i] - tt[i - 1]) - rep.interval) > 0.5 * rep.interval;
            bool slip = (!gap) && (std::fabs(dL4[i] - medL) > thrL || std::fabs(dMW[i] - medW) > thrW);
            if (slip) { q.slipFlag[i] = 1; nSlip++; }
        }
        q.slips = nSlip;
        q.slipRatio = q.slips > 0 ? (double)q.totalEpochs / q.slips : (double)q.totalEpochs;

        // ===== 弧段掩码：gap 或 周跳处断开，弧段内连续点 ok=1 =====
        std::vector<char> ok(n, 1);
        for (int i = 1; i < n; i++) {
            bool gap = std::fabs((tt[i] - tt[i - 1]) - rep.interval) > 0.5 * rep.interval;
            if (gap || q.slipFlag[i]) ok[i] = 0;
        }

        // 观测值噪声 (三次差)：仅弧段内连续历元求差(避开 gap/周跳放大)
        q.sigRho1 = tripleStd(c1s, tt, rep.interval, ok); q.sigRho2 = tripleStd(c2s, tt, rep.interval, ok);
        q.sigPh1 = tripleStd(l1cyc, tt, rep.interval, ok); q.sigPh2 = tripleStd(l2cyc, tt, rep.interval, ok);

        // ===== 逐弧段处理：去该弧段模糊度常数(均值) + 二次去趋势 =====
        // 仅做「全局」去趋势会失败：C10 的 178 万周台阶(非 gap 的失锁重锁)压不平，
        // 二次拟合被迫弯曲 → 显示成「大抛物线」+ 巨量值。逐弧段处理可彻底消除台阶。
        // doDetrend=false 用于「原始趋势」版(仅去模糊度常数，保留漂移斜率供检视)。
        auto processArcs = [&](std::vector<double> &y, bool doDetrend) {
            int i = 0;
            while (i < n) {
                // i 为某弧段起点(0，或某断点历元——断点历元是有效数据，应作为新弧段起点被纳入，
                // 切勿跳过，否则其原始 ~1.79e6 周模糊度常数会残留、撑爆 RMS)。
                int j = i;
                while (j < n && (j == i || ok[j] == 1)) j++;   // [i, j) 为一个弧段
                int len = j - i;
                if (len >= 1) {
                    double m = 0; for (int k = i; k < j; k++) m += y[k]; m /= len;
                    for (int k = i; k < j; k++) y[k] -= m;          // 去该弧段模糊度常数
                    if (doDetrend && len >= 3) {
                        std::vector<double> ttA(len), yA(len);
                        for (int k = 0; k < len; k++) { ttA[k] = tt[i + k]; yA[k] = y[i + k]; }
                        auto r = polyDetrendLocal(ttA, yA, 2);
                        for (int k = 0; k < len; k++) y[i + k] = r[k];
                    }
                }
                i = j;
            }
        };
        std::vector<double> mp1c = mp1s, mp2c = mp2s, L4c = L4, ionoc = ionos;
        processArcs(mp1c, false); processArcs(mp2c, false); processArcs(L4c, false); processArcs(ionoc, false);
        processArcs(mp1s, true);  processArcs(mp2s, true);  processArcs(L4, true);    processArcs(ionos, true);

        // 统计量在去趋势序列上计算，反映真实波动(不被整体漂移/台阶放大)
        q.mp1Mean = mean(mp1s); q.mp1Rms = rms(mp1s);
        q.mp2Mean = mean(mp2s); q.mp2Rms = rms(mp2s);
        q.ionoStd = rms(ionos);
        for (int i = 1; i < n; i++) {
            double dt = tt[i] - tt[i - 1];
            if (dt > 0 && std::fabs(ionos[i] - ionos[i - 1]) / dt > 0.07) q.ionoJumps++;
        }

        // 从原始历元中找回 SNR/EL/AZ (与 satIds 同序)
        std::vector<double> snrs(n, 0), elevs(n, 0), azims(n, 0);
        for (int i = 0; i < n; i++) {
            const auto &ep = epochs[R[i].k];
            int idx = -1;
            for (int j = 0; j < (int)ep.satIds.size(); j++) if (ep.satIds[j] == sat) { idx = j; break; }
            if (idx >= 0) {
                snrs[i] = getSnr(ep.allObs[idx], b1);
                if (idx < (int)ep.elevations.size()) elevs[i] = ep.elevations[idx];
                if (idx < (int)ep.azimuths.size()) azims[i] = ep.azimuths[idx];
            }
        }
        auto validStats = [](const std::vector<double> &v) {
            double s = 0; int c = 0; double mn = 1e300, mx = -1e300;
            for (double x : v) if (x > 0) { s += x; c++; mn = std::min(mn, x); mx = std::max(mx, x); }
            return std::make_tuple(c ? s / c : 0.0, c ? mn : 0.0, c ? mx : 0.0);
        };
        auto [snrMean, snrMin, snrMax] = validStats(snrs);
        auto [elevMean, elevMin, elevMax] = validStats(elevs);
        q.snr = std::move(snrs); q.snrMean = snrMean; q.snrMin = snrMin; q.snrMax = snrMax;
        q.elev = std::move(elevs); q.elevMean = elevMean; q.elevMin = elevMin; q.elevMax = elevMax;
        q.azim = std::move(azims);

        q.epIdx.reserve(n); q.t = tt;
        q.mp1 = std::move(mp1s); q.mp2 = std::move(mp2s);
        q.l4 = std::move(L4); q.iono = std::move(ionos);
        q.mp1Raw = std::move(mp1c); q.mp2Raw = std::move(mp2c);
        q.l4Raw = std::move(L4c); q.ionoRaw = std::move(ionoc);
        for (int i = 0; i < n; i++) q.epIdx.push_back(R[i].k);

        rep.sats[sat] = std::move(q);
        rep.satOrder.push_back(sat);
    }

    // 系统 / 总体聚合
    std::map<char, double> sumComp, sumSlipRatio, sumMp1, sumMp2, sumSigRho, sumSigPh, sumSnr, sumElev;
    std::map<char, int> cnt, sumSlips, sumIono, cntSnr, cntElev;
    for (auto &[sat, q] : rep.sats) {
        char s = sat.system;
        sumComp[s] += q.completeness; sumSlipRatio[s] += q.slipRatio;
        sumMp1[s] += q.mp1Rms; sumMp2[s] += q.mp2Rms;
        sumSigRho[s] += (q.sigRho1 + q.sigRho2) / 2.0;
        sumSigPh[s] += (q.sigPh1 + q.sigPh2) / 2.0;
        sumSlips[s] += q.slips; sumIono[s] += q.ionoJumps;
        if (q.snrMean > 0) { sumSnr[s] += q.snrMean; cntSnr[s]++; }
        if (q.elevMean > 0) { sumElev[s] += q.elevMean; cntElev[s]++; }
        cnt[s]++;
    }
    for (auto &[s, c] : cnt) {
        rep.sysCompleteness[s] = sumComp[s] / c;
        rep.sysSlipRatio[s] = sumSlipRatio[s] / c;
        rep.sysMp1[s] = sumMp1[s] / c; rep.sysMp2[s] = sumMp2[s] / c;
        rep.sysSigRho[s] = sumSigRho[s] / c; rep.sysSigPhase[s] = sumSigPh[s] / c;
        rep.sysSlips[s] = sumSlips[s]; rep.sysIonoJumps[s] = sumIono[s];
        if (cntSnr.count(s)) rep.sysSnr[s] = sumSnr[s] / cntSnr[s];
        if (cntElev.count(s)) rep.sysElev[s] = sumElev[s] / cntElev[s];
    }
    double allComp = 0, allSnr = 0, allElev = 0; int allCnt = 0, allSnrCnt = 0, allElevCnt = 0;
    for (auto &[sat, q] : rep.sats) {
        allComp += q.completeness; allCnt++;
        if (q.snrMean > 0) { allSnr += q.snrMean; allSnrCnt++; }
        if (q.elevMean > 0) { allElev += q.elevMean; allElevCnt++; }
    }
    rep.overallCompleteness = allCnt ? allComp / allCnt : 0;
    rep.overallSnr = allSnrCnt ? allSnr / allSnrCnt : 0;
    rep.overallElev = allElevCnt ? allElev / allElevCnt : 0;
    if (!rep.pdop.empty()) {
        rep.overallPDOP = mean(rep.pdop); rep.overallHDOP = mean(rep.hdop);
        rep.overallVDOP = mean(rep.vdop); rep.overallGDOP = mean(rep.gdop);
    }
    if (!rep.nsat.empty()) {
        double s = 0; for (int v : rep.nsat) s += v;
        rep.overallNsat = (int)std::round(s / rep.nsat.size());
    }
    return rep;
}

} // namespace QC
