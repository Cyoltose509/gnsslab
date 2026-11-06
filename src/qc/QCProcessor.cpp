#include "QCProcessor.h"
#include "FreqCombo.h"
#include "Const.h"
#include <algorithm>
#include <deque>
#include <utility>

namespace QC {
    namespace {
        /// 多路径滑窗长度：向前最多回看多少个连续有效点。
        constexpr int kMpWin = 50;

        /// 结算滞后历元数：MW 周跳要看下一点（1），钟跳要看下一历元全星座（2）。
        constexpr int kSettleLag = 2;

        double getVal(const TypeValueMap &m, const char t, const char b) {
            for (auto &[k, v]: m) {
                if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return v;
            }
            return 0.0;
        }


        double getCnr(const TypeValueMap &m, const char b) {
            for (auto &[k, v]: m) {
                if (k.size() >= 2 && k[0] == 'S' && k[1] == b && v > 0.0) return v;
            }
            return 0.0;
        }


        std::string findObsType(const TypeValueMap &m, const char t, const char b) {
            for (auto &[k, v]: m)
                if (k.size() >= 2 && k[0] == t && k[1] == b && v != 0.0) return k;
            return "";
        }

        struct BandsInfo {
            char hi = 0, lo = 0;
            double f1 = 0, f2 = 0;
        };

        BandsInfo pickBands(const char sys, const TypeValueMap &m) {
            std::vector<std::pair<char, std::string> > cand;
            for (char b: {'1', '2', '5', '6', '7', '8'}) {
                std::string ct = findObsType(m, 'C', b);
                std::string lt = findObsType(m, 'L', b);
                if (!ct.empty() && !lt.empty()) cand.emplace_back(b, ct);
            }
            if (cand.size() < 2) return {};
            BandsInfo r;
            double bestDiff = -1.0;
            for (std::size_t i = 0; i < cand.size(); ++i) {
                for (std::size_t j = i + 1; j < cand.size(); ++j) {
                    const double fi = getFreq(sys, cand[i].second);
                    const double fj = getFreq(sys, cand[j].second);
                    if (fi <= 0.0 || fj <= 0.0) continue;
                    if (const double diff = std::fabs(fi - fj); diff > bestDiff) {
                        bestDiff = diff;
                        if (fi > fj) {
                            r.hi = cand[i].first;
                            r.lo = cand[j].first;
                            r.f1 = fi;
                            r.f2 = fj;
                        } else {
                            r.hi = cand[j].first;
                            r.lo = cand[i].first;
                            r.f1 = fj;
                            r.f2 = fi;
                        }
                    }
                }
            }
            return r;
        }


        void computeComprehensive(QualityReport &rep) {
            ComprehensiveEval ce;
            ce.indicators = {"完整率", "周跳比", "多路径", "电离层变化率", "伪距噪声", "载波噪声", "载噪比"};
            const std::vector<SatID> &sats = rep.satOrder;
            if (sats.empty()) return;
            const int J = static_cast<int>(sats.size());
            constexpr int I = 7;


            std::vector y(I, std::vector<double>(J));
            for (int m = 0; m < J; m++) {
                const SatID &sat = sats[m];
                const SatQC &q = rep.sats.at(sat);
                const double mp = 0.5 * (q.mp1Rms + q.mp2Rms);
                const double rho = 0.5 * (q.sigRho1 + q.sigRho2);
                const double ph = 0.5 * (q.sigPh1 + q.sigPh2);
                y[0][m] = q.completeness;
                y[1][m] = q.slipRatio;
                y[2][m] = 1.0 / (mp + 1e-6);
                y[3][m] = 1.0 / (q.ionoRateStd + 1e-6);
                y[4][m] = 1.0 / (rho + 1e-6);
                y[5][m] = 1.0 / (ph + 1e-6);
                y[6][m] = q.cnrMean;
            }


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


        /// 三重差分的 σ：sum(d3²) 与计数累积后一次换算。
        double tripleSigma(const double sum, const int cnt) {
            if (cnt < 2) return 0.0;
            return std::sqrt(sum / (8.0 * (cnt - 1.0)));
        }


        double meanOrZero(const double sum, const int cnt) {
            return cnt > 0 ? sum / static_cast<double>(cnt) : 0.0;
        }
    }


    /// 单颗卫星的累积器：原始序列全保留（绘图与结算都要按索引回看），
    /// 派生量随结算逐步累加，不在快照时重算。
    struct SatStream {
        BandsInfo band{};
        bool bandReady = false;
        double lam1 = 0, lam2 = 0;

        int n1 = 0, n2 = 0;

        std::vector<double> tt, c1s, c2s, l1m, l2m, l1cyc, l2cyc;
        std::vector<double> MW, LGF, PGF, mp1s, mp2s, ionos, ionos2;
        std::vector<double> cnrs, cnrs2;
        std::vector<double> mp1Resid, mp2Resid, ionoRate, ionoRate2;
        std::vector<int> epIdx;
        std::vector<char> gapOk, slipFlag, outlierFlag, clockJumpFlag;

        /// MW 检测状态：均值/方差在线累积，pending 是等待下一点才能定稿的索引。
        double mwMean = 0, mwVar = 0;
        int mwCnt = 0;
        int pending = -1;

        int slips = 0, outliers = 0, clockJumps = 0;
        bool slipEver = false;

        double mp1Sum = 0, mp2Sum = 0;
        double win1Sum = 0, win2Sum = 0;
        int win1Cnt = 0, win2Cnt = 0;

        double triSum[4] = {0, 0, 0, 0};
        int triCnt[4] = {0, 0, 0, 0};

        double ionoSq = 0, ionoSq2 = 0;
        int ionoCnt = 0, ionoCnt2 = 0;
        int ionoJump1 = 0, ionoJump2 = 0;

        double cnrSum = 0, cnrMin = 0, cnrMax = 0;
        int cnrCnt = 0;
        double cnrSum2 = 0, cnrMin2 = 0, cnrMax2 = 0;
        int cnrCnt2 = 0;

        /// 下一个待结算的序列下标。
        int settled = 0;

        [[nodiscard]] int size() const { return static_cast<int>(tt.size()); }

        /// 本点是否参与统计（断档、周跳、钟跳都排除）。
        [[nodiscard]] bool okAt(const int i) const {
            return gapOk[i] != 0 && slipFlag[i] == 0 && clockJumpFlag[i] == 0;
        }
    };


    struct Stream::Impl {
        double nominal = 0.0;
        double interval = 1.0;
        bool intervalSeeded = false;

        int totalEpochs = 0;
        double lastSow = 0;
        bool hasLast = false;

        std::map<SatID, SatStream> sats;
        /// 最近几个历元的「卫星 → 序列下标」，钟跳判定只在相邻两历元间做。
        std::deque<std::map<SatID, int> > ring;

        int clockJumpEpochs = 0;
        bool finished = false;

        /// 单程 MW 检测的一步：等价于 detectMW 的循环体，next 由调用方提供。
        static void applyMw(SatStream &s, const int k, const double nextMw, const bool nextOk, const bool hasNext) {
            const double mw = s.MW[k];
            if (!s.gapOk[k]) {
                s.mwMean = mw;
                s.mwVar = 0.0;
                s.mwCnt = 1;
                return;
            }
            const double dev = mw - s.mwMean;
            if (s.mwVar > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(s.mwVar) && hasNext && nextOk) {
                const double meanK = (s.mwCnt * s.mwMean + mw) / (s.mwCnt + 1);
                const double varK = (s.mwCnt * s.mwVar + dev * dev) / (s.mwCnt + 1);
                const double devNext = nextMw - meanK;
                if (!(varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK))
                    || std::fabs(nextMw - mw) > 1.0) {
                    s.outlierFlag[k] = 1;
                    s.outliers++;
                } else {
                    markSlip(s, k);
                    s.mwMean = mw;
                    s.mwVar = 0.0;
                    s.mwCnt = 1;
                    return;
                }
            }
            s.mwMean = (s.mwCnt * s.mwMean + mw) / (s.mwCnt + 1);
            s.mwVar = (s.mwCnt * s.mwVar + dev * dev) / (s.mwCnt + 1);
            s.mwCnt++;
        }


        /// MW 与 GF 共用一条 slipFlag：先到先标，不互相覆盖。
        static void markSlip(SatStream &s, const int k) {
            if (s.slipFlag[k]) return;
            s.slipFlag[k] = 1;
            s.slips++;
            s.slipEver = true;
        }


        /// 结算序列下标 i：三重差分、MP 滑窗、电离层变化率、载噪比统计。
        /// 只用序列自身与已定型的标志位，按索引向前回看，不依赖未来历元。
        void settle(SatStream &s, const int i) const {
            const bool okI = s.okAt(i);

            if (i >= 3 && okI && s.okAt(i - 1) && s.okAt(i - 2) && s.okAt(i - 3)) {
                const double d0 = s.c1s[i] - 3.0 * s.c1s[i - 1] + 3.0 * s.c1s[i - 2] - s.c1s[i - 3];
                const double d1 = s.c2s[i] - 3.0 * s.c2s[i - 1] + 3.0 * s.c2s[i - 2] - s.c2s[i - 3];
                const double d2 = s.l1cyc[i] - 3.0 * s.l1cyc[i - 1] + 3.0 * s.l1cyc[i - 2] - s.l1cyc[i - 3];
                const double d3 = s.l2cyc[i] - 3.0 * s.l2cyc[i - 1] + 3.0 * s.l2cyc[i - 2] - s.l2cyc[i - 3];
                s.triSum[0] += d0 * d0;
                s.triSum[1] += d1 * d1;
                s.triSum[2] += d2 * d2;
                s.triSum[3] += d3 * d3;
                for (int c = 0; c < 4; ++c) s.triCnt[c]++;
            }

            double mpMean1 = 0.0, mpMean2 = 0.0;
            bool haveWin = false;
            if (okI) {
                int a = i, len = 1;
                while (a - 1 >= 0 && s.okAt(a - 1) && len < kMpWin) {
                    --a;
                    ++len;
                }
                if (len >= 2) {
                    double m1 = 0.0, m2 = 0.0;
                    for (int t = a; t <= i; ++t) {
                        m1 += s.mp1s[t];
                        m2 += s.mp2s[t];
                    }
                    m1 /= static_cast<double>(len);
                    m2 /= static_cast<double>(len);
                    double v1 = 0.0, v2 = 0.0;
                    for (int t = a; t <= i; ++t) {
                        const double r1 = s.mp1s[t] - m1;
                        const double r2 = s.mp2s[t] - m2;
                        v1 += r1 * r1;
                        v2 += r2 * r2;
                    }
                    s.win1Sum += std::sqrt(v1 / static_cast<double>(len - 1));
                    s.win2Sum += std::sqrt(v2 / static_cast<double>(len - 1));
                    s.win1Cnt++;
                    s.win2Cnt++;
                    mpMean1 = m1;
                    mpMean2 = m2;
                    haveWin = true;
                }
            }
            s.mp1Resid[i] = haveWin ? s.mp1s[i] - mpMean1 : 0.0;
            s.mp2Resid[i] = haveWin ? s.mp2s[i] - mpMean2 : 0.0;

            double r1 = 0.0, r2 = 0.0;
            if (i >= 1 && okI && s.okAt(i - 1)) {
                const double dt = s.tt[i] - s.tt[i - 1];
                if (dt > 0 && std::fabs(dt - interval) <= 0.5 * interval) {
                    r1 = (s.ionos[i] - s.ionos[i - 1]) / dt;
                    r2 = (s.ionos2[i] - s.ionos2[i - 1]) / dt;
                }
            }
            s.ionoRate[i] = r1;
            s.ionoRate2[i] = r2;
            if (r1 != 0.0) {
                s.ionoSq += r1 * r1;
                s.ionoCnt++;
            }
            if (r2 != 0.0) {
                s.ionoSq2 += r2 * r2;
                s.ionoCnt2++;
            }
            if (std::fabs(r1) > 0.07) s.ionoJump1++;
            if (std::fabs(r2) > 0.07) s.ionoJump2++;

            if (const double v = s.cnrs[i]; v > 0.0) {
                s.cnrSum += v;
                if (s.cnrCnt == 0 || v < s.cnrMin) s.cnrMin = v;
                if (s.cnrCnt == 0 || v > s.cnrMax) s.cnrMax = v;
                s.cnrCnt++;
            }
            if (const double v = s.cnrs2[i]; v > 0.0) {
                s.cnrSum2 += v;
                if (s.cnrCnt2 == 0 || v < s.cnrMin2) s.cnrMin2 = v;
                if (s.cnrCnt2 == 0 || v > s.cnrMax2) s.cnrMax2 = v;
                s.cnrCnt2++;
            }
        }


        /// 结算某历元上所有卫星的记录；按索引保证只结算一次。
        void settleEpoch(const std::map<SatID, int> &loc) {
            for (const auto &[sat, idx]: loc) {
                const auto it = sats.find(sat);
                if (it == sats.end()) continue;
                SatStream &s = it->second;
                for (int i = s.settled; i <= idx; ++i) settle(s, i);
                if (idx + 1 > s.settled) s.settled = idx + 1;
            }
        }


        /// 钟跳判定：相邻两历元上「无周跳历史」的卫星码相跳变是否同向共模。
        /// 判为钟跳后把后一历元的周跳/粗差改判为钟跳，并从 slipFlag 里撤回。
        void judgeClockJump(const std::map<SatID, int> &a, const std::map<SatID, int> &b) {
            constexpr double c = C_MPS;
            constexpr double xi = 4.0;
            constexpr double msLo = 1e-7 * c - 3.0 * xi, msHi = 1e-5 * c + 3.0 * xi;
            constexpr double usThr = 1e-3 * c - 3.0 * xi;

            int nms = 0, nus = 0, totalValid = 0;
            for (const auto &[sat, ia]: a) {
                const auto it = sats.find(sat);
                if (it == sats.end() || it->second.slipEver) continue;
                const auto jt = b.find(sat);
                if (jt == b.end()) continue;
                const SatStream &s = it->second;
                const int ib = jt->second;
                const double dL = std::fabs(s.c1s[ib] - s.c1s[ia] - (s.l1m[ib] - s.l1m[ia]));
                if (dL > msLo && dL < msHi) nms++;
                else if (dL > usThr) nus++;
                totalValid++;
            }
            if (const int ns = nms + nus; totalValid < 4 || ns < 4 || ns < 0.6 * totalValid) return;

            for (const auto &[sat, ib]: b) {
                const auto it = sats.find(sat);
                if (it == sats.end()) continue;
                SatStream &s = it->second;
                if (s.slipFlag[ib]) {
                    s.slipFlag[ib] = 0;
                    if (s.slips > 0) s.slips--;
                }
                s.outlierFlag[ib] = 0;
                s.clockJumpFlag[ib] = 1;
                s.clockJumps++;
            }
            clockJumpEpochs++;
        }


        void trimRing() {
            while (ring.size() > static_cast<std::size_t>(kSettleLag + 2)) ring.pop_front();
        }
    };


    Stream::Stream(const double nominalInterval) : mImpl(std::make_unique<Impl>()) {
        mImpl->nominal = nominalInterval;
        if (nominalInterval > 0.0) {
            mImpl->interval = nominalInterval;
            mImpl->intervalSeeded = true;
        }
    }


    Stream::~Stream() = default;

    Stream::Stream(Stream &&) noexcept = default;

    Stream &Stream::operator=(Stream &&) noexcept = default;


    void Stream::reset(const double nominalInterval) {
        mImpl = std::make_unique<Impl>();
        mImpl->nominal = nominalInterval;
        if (nominalInterval > 0.0) {
            mImpl->interval = nominalInterval;
            mImpl->intervalSeeded = true;
        }
    }


    void Stream::feed(const QCObsEpoch &epoch) {
        Impl &d = *mImpl;
        d.finished = false;

        const int gi = d.totalEpochs++;

        if (d.hasLast && !d.intervalSeeded) {
            if (const double dt = epoch.sow - d.lastSow; dt > 0 && dt < 1e4) {
                d.interval = dt;
                d.intervalSeeded = true;
            }
        }
        d.lastSow = epoch.sow;
        d.hasLast = true;

        std::map<SatID, int> loc;

        for (std::size_t j = 0; j < epoch.satIds.size(); ++j) {
            const SatID &sat = epoch.satIds[j];
            const TypeValueMap &mv = epoch.allObs[j];

            SatStream &s = d.sats[sat];
            if (!s.bandReady) {
                const BandsInfo b = pickBands(sat.system, mv);
                if (b.hi == 0) continue;
                s.band = b;
                s.bandReady = true;
                s.lam1 = C_MPS / b.f1;
                s.lam2 = C_MPS / b.f2;
            }

            const double c1 = getVal(mv, 'C', s.band.hi);
            const double l1 = getVal(mv, 'L', s.band.hi);
            const double c2 = getVal(mv, 'C', s.band.lo);
            const double l2 = getVal(mv, 'L', s.band.lo);

            if (c1 != 0.0 && l1 != 0.0) s.n1++;
            if (c2 != 0.0 && l2 != 0.0) s.n2++;
            if (c1 == 0.0 || l1 == 0.0 || c2 == 0.0 || l2 == 0.0) continue;

            const int idx = s.size();
            const double f1 = s.band.f1, f2 = s.band.f2;
            const FreqCombo fc = FreqCombo::fromFreq(f1, f2);
            const double alpha = f1 / f2 * (f1 / f2);
            const double a = (alpha + 1.0) / (alpha - 1.0);
            const double b = 2.0 / (alpha - 1.0);
            const double c = 2.0 * alpha / (alpha - 1.0);
            const double e = (alpha + 1.0) / (alpha - 1.0);

            s.tt.push_back(epoch.sow);
            s.c1s.push_back(c1);
            s.c2s.push_back(c2);
            s.l1m.push_back(l1);
            s.l2m.push_back(l2);
            s.l1cyc.push_back(l1 / s.lam1);
            s.l2cyc.push_back(l2 / s.lam2);
            s.MW.push_back(fc.MW_meter(c1, c2, l1, l2));
            s.LGF.push_back(FreqCombo::GF_phase(l1, l2));
            s.PGF.push_back(FreqCombo::GF_code(c1, c2));
            s.mp1s.push_back(c1 - a * l1 + b * l2);
            s.mp2s.push_back(c2 - c * l1 + e * l2);
            s.ionos.push_back(f2 * f2 / (f1 * f1 - f2 * f2) * (l1 - l2));
            s.ionos2.push_back(f1 * f1 / (f1 * f1 - f2 * f2) * (l1 - l2));
            s.epIdx.push_back(gi);
            s.gapOk.push_back(1);
            s.slipFlag.push_back(0);
            s.outlierFlag.push_back(0);
            s.clockJumpFlag.push_back(0);
            s.mp1Resid.push_back(0.0);
            s.mp2Resid.push_back(0.0);
            s.ionoRate.push_back(0.0);
            s.ionoRate2.push_back(0.0);
            s.cnrs.push_back(getCnr(mv, s.band.hi));
            s.cnrs2.push_back(getCnr(mv, s.band.lo));
            s.mp1Sum += s.mp1s.back();
            s.mp2Sum += s.mp2s.back();

            if (idx >= 1 && std::fabs(s.tt[idx] - s.tt[idx - 1] - d.interval) > 0.5 * d.interval)
                s.gapOk[idx] = 0;

            if (idx >= 1 && s.gapOk[idx] && s.gapOk[idx - 1]) {
                constexpr double gfMult = 3.0;
                if (const double thr = gfMult * (s.lam2 - s.lam1);
                    std::fabs(s.LGF[idx] - s.LGF[idx - 1]) > thr
                    && std::fabs(s.PGF[idx] - s.PGF[idx - 1]) <= 0.10) {
                    Impl::markSlip(s, idx);
                }
            }

            // MW 检测：原算法以 MW[0] 初始化、从 k=1 起判定，所以 index 0 不参与判定。
            if (s.pending >= 1) {
                Impl::applyMw(s, s.pending, s.MW[idx], s.gapOk[idx] != 0, true);
            } else if (s.pending < 0) {
                s.mwMean = s.MW[idx];
                s.mwVar = 0.0;
                s.mwCnt = 1;
            }
            s.pending = idx;

            loc[sat] = idx;
        }

        d.ring.push_back(std::move(loc));
        d.trimRing();

        if (d.ring.size() >= 3) {
            d.judgeClockJump(d.ring[d.ring.size() - 3], d.ring[d.ring.size() - 2]);
            d.settleEpoch(d.ring[d.ring.size() - 3]);
        }
    }


    void Stream::finish() {
        Impl &d = *mImpl;
        if (d.finished) return;
        if (d.ring.size() >= 2)
            d.judgeClockJump(d.ring[d.ring.size() - 2], d.ring[d.ring.size() - 1]);
        for (const auto &loc: d.ring) d.settleEpoch(loc);
        d.finished = true;
    }


    int Stream::epochs() const { return mImpl->totalEpochs; }


    QualityReport Stream::report() const {
        const Impl &d = *mImpl;
        QualityReport rep;
        rep.totalEpochs = d.totalEpochs;
        rep.totalInputEpochs = d.totalEpochs;
        rep.interval = d.interval;
        rep.clockJumpEpochs = d.clockJumpEpochs;
        if (rep.totalEpochs < 2) return rep;

        for (const auto &[sat, s]: d.sats) {
            if (!s.bandReady || s.size() < 2) continue;
            const int n = s.size();

            SatQC q;
            q.sat = sat;
            q.band1 = s.band.hi;
            q.band2 = s.band.lo;
            q.totalEpochs = rep.totalEpochs;
            q.validDual = n;
            q.slips = s.slips;
            q.outliers = s.outliers;
            q.clockJumps = s.clockJumps;
            q.slipRatio = q.slips > 0
                              ? static_cast<double>(n) / static_cast<double>(q.slips)
                              : static_cast<double>(n);
            q.di_f1 = 100.0 * static_cast<double>(s.n1) / rep.totalEpochs;
            q.di_f2 = 100.0 * static_cast<double>(s.n2) / rep.totalEpochs;
            q.completeness = 100.0 * static_cast<double>(n) / rep.totalEpochs;

            q.mp1Mean = meanOrZero(s.mp1Sum, n);
            q.mp2Mean = meanOrZero(s.mp2Sum, n);
            q.mp1Rms = meanOrZero(s.win1Sum, s.win1Cnt);
            q.mp2Rms = meanOrZero(s.win2Sum, s.win2Cnt);

            q.sigRho1 = tripleSigma(s.triSum[0], s.triCnt[0]);
            q.sigRho2 = tripleSigma(s.triSum[1], s.triCnt[1]);
            q.sigPh1 = tripleSigma(s.triSum[2], s.triCnt[2]);
            q.sigPh2 = tripleSigma(s.triSum[3], s.triCnt[3]);

            q.ionoRateStd1 = s.ionoCnt > 0 ? std::sqrt(s.ionoSq / s.ionoCnt) : 0.0;
            q.ionoRateStd2 = s.ionoCnt2 > 0 ? std::sqrt(s.ionoSq2 / s.ionoCnt2) : 0.0;
            q.ionoRateStd = 0.5 * (q.ionoRateStd1 + q.ionoRateStd2);
            q.ionoJumps1 = s.ionoJump1;
            q.ionoJumps2 = s.ionoJump2;
            q.ionoJumps = s.ionoJump1 + s.ionoJump2;

            q.cnrMean = meanOrZero(s.cnrSum, s.cnrCnt);
            q.cnrMin = s.cnrCnt > 0 ? s.cnrMin : 0.0;
            q.cnrMax = s.cnrCnt > 0 ? s.cnrMax : 0.0;
            q.cnrMean2 = meanOrZero(s.cnrSum2, s.cnrCnt2);
            q.cnrMin2 = s.cnrCnt2 > 0 ? s.cnrMin2 : 0.0;
            q.cnrMax2 = s.cnrCnt2 > 0 ? s.cnrMax2 : 0.0;

            q.epIdx = s.epIdx;
            q.t = s.tt;
            q.mp1 = s.mp1s;
            q.mp2 = s.mp2s;
            q.mp1Resid = s.mp1Resid;
            q.mp2Resid = s.mp2Resid;
            q.ionoRate = s.ionoRate;
            q.ionoRate2 = s.ionoRate2;
            q.slipFlag = s.slipFlag;
            q.outlierFlag = s.outlierFlag;
            q.clockJumpFlag = s.clockJumpFlag;
            q.cnr = s.cnrs;
            q.cnr2 = s.cnrs2;

            rep.satOrder.push_back(sat);
            rep.sats[sat] = std::move(q);
        }

        std::map<char, double> sumComp, sumDI1, sumDI2, sumSlipRatio, sumMp1, sumMp2;
        std::map<char, double> sumSigRho, sumSigPh, sumCnr, sumCnr1, sumCnr2, sumIonoRate, sumIonoRate1,
                sumIonoRate2;
        std::map<char, int> cnt, sumSlips, sumIono, sumIono1, sumIono2, sumOut, sumClk, cntCnr, cntCnr1,
                cntCnr2, cntIono, cntIono1, cntIono2;
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
            sumIono1[s] += q.ionoJumps1;
            sumIono2[s] += q.ionoJumps2;
            sumOut[s] += q.outliers;
            sumClk[s] += q.clockJumps;
            sumIonoRate[s] += q.ionoRateStd;
            cntIono[s]++;
            sumIonoRate1[s] += q.ionoRateStd1;
            cntIono1[s]++;
            sumIonoRate2[s] += q.ionoRateStd2;
            cntIono2[s]++;
            if (q.cnrMean > 0) {
                sumCnr[s] += q.cnrMean;
                cntCnr[s]++;
                sumCnr1[s] += q.cnrMean;
                cntCnr1[s]++;
            }
            if (q.cnrMean2 > 0) {
                sumCnr[s] += q.cnrMean2;
                cntCnr[s]++;
                sumCnr2[s] += q.cnrMean2;
                cntCnr2[s]++;
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
            rep.sysIonoJumps1[s] = sumIono1[s];
            rep.sysIonoJumps2[s] = sumIono2[s];
            rep.sysOutliers[s] = sumOut[s];
            rep.sysClockJumps[s] = sumClk[s];
            rep.sysIonoRate[s] = cntIono.count(s) ? sumIonoRate[s] / cntIono[s] : 0.0;
            rep.sysIonoRate1[s] = cntIono1.count(s) ? sumIonoRate1[s] / cntIono1[s] : 0.0;
            rep.sysIonoRate2[s] = cntIono2.count(s) ? sumIonoRate2[s] / cntIono2[s] : 0.0;
            if (cntCnr.count(s)) rep.sysCnr[s] = sumCnr[s] / cntCnr[s];
            if (cntCnr1.count(s)) rep.sysCnr1[s] = sumCnr1[s] / cntCnr1[s];
            if (cntCnr2.count(s)) rep.sysCnr2[s] = sumCnr2[s] / cntCnr2[s];
        }

        double allComp = 0, allCnr = 0, allCnr1 = 0, allCnr2 = 0;
        int allCnt = 0, allCnrCnt = 0, allCnr1Cnt = 0, allCnr2Cnt = 0;
        for (auto &[sat, q]: rep.sats) {
            allComp += q.completeness;
            allCnt++;
            if (q.cnrMean > 0) {
                allCnr += q.cnrMean;
                allCnrCnt++;
                allCnr1 += q.cnrMean;
                allCnr1Cnt++;
            }
            if (q.cnrMean2 > 0) {
                allCnr += q.cnrMean2;
                allCnrCnt++;
                allCnr2 += q.cnrMean2;
                allCnr2Cnt++;
            }
        }
        rep.overallCompleteness = allCnt ? allComp / allCnt : 0;
        rep.overallCnr = allCnrCnt ? allCnr / allCnrCnt : 0;
        rep.overallCnr1 = allCnr1Cnt ? allCnr1 / allCnr1Cnt : 0;
        rep.overallCnr2 = allCnr2Cnt ? allCnr2 / allCnr2Cnt : 0;

        computeComprehensive(rep);
        return rep;
    }


    QualityReport compute(const std::vector<QCObsEpoch> &epochs, const double nominalInterval) {
        Stream s(nominalInterval);
        for (const auto &e: epochs) s.feed(e);
        s.finish();
        return s.report();
    }
}
