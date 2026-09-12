#include "CycleSlip.h"
#include "Const.h"
#include "MathUtils.h"

void CycleSlip::configure(const FreqCombo &fcIn) {
    fc = fcIn;
    lam1 = fc.f1 > 0 ? C_MPS / fc.f1 : fc.lambda1;
    lam2 = fc.f2 > 0 ? C_MPS / fc.f2 : fc.lambda2;
}

void CycleSlip::reset() {
    mwMean = 0; mwVar = 0; mwCnt = 0;
    pendActive = false;
}

CycleSlip::Verdict CycleSlip::feed(const double L1, const double L2, const double C1, const double C2, const bool valid,
                                  const  double osbP1, const double osbP2, const double osbL1, const double osbL2) {
    if (!valid) { reset(); return Verdict::Gap; }
    if (lam1 <= 0 || lam2 <= 0) return Verdict::None;

    const double mw  = fc.MW_meter(C1, C2, L1, L2, osbP1, osbP2, osbL1, osbL2); // MW (米)

    auto v = Verdict::None;

    if (pendActive) {
        const double meanK = (mwCntB * mwMeanB + pendMw) / (mwCntB + 1.0);
        const double varK  = (mwCntB * mwVarB + pendDev * pendDev) / (mwCntB + 1.0);
        const double devNext = mw - meanK;
        const bool slip = varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)
                         && std::fabs(mw - pendMw) <= 1.0;
        if (slip) {
            mwMean = mw; mwVar = 0; mwCnt = 0;
            pendActive = false;
            v = Verdict::Slip;
        } else {
            mwMean = meanK; mwVar = varK; mwCnt++;
            pendActive = false;
        }
    }

    {
        if (const double dev = mw - mwMean; mwCnt >= 5 && mwVar > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(mwVar)) {
            pendActive = true;
            pendMw = mw;
            pendDev = dev;
            mwMeanB = mwMean;
            mwVarB  = mwVar;
            mwCntB  = mwCnt;
        } else {
            mwMean = (mwCnt * mwMean + mw) / (mwCnt + 1.0);
            mwVar  = (mwCnt * mwVar + dev * dev) / (mwCnt + 1.0);
            mwCnt++;
        }
    }

    return v;
}

void CycleSlip::detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier) {
    const int n = static_cast<int>(MW.size());
    slip.assign(n, 0);
    outlier.assign(n, 0);
    if (n < 2) return;
    double mean = MW[0], var = 0.0;
    int cnt = 1;
    for (int k = 1; k < n; k++) {
        if (!ok[k]) {
            mean = MW[k];
            var = 0.0;
            cnt = 1;
            continue;
        }
        const double dev = MW[k] - mean;
        if (var > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(var) && k + 1 < n && ok[k + 1]) {
            const double meanK = (cnt * mean + MW[k]) / (cnt + 1);
            const double varK = (cnt * var + dev * dev) / (cnt + 1);
            if (const double devNext = MW[k + 1] - meanK; !(varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)) || std::fabs(MW[k + 1] - MW[k]) > 1.0) {
                outlier[k] = 1; // ti+1 不超限，或两者均超限但 |ΔMW|>1m → 粗差
            } else {
                slip[k] = 1; // 两者均超限且 ΔMW≤1m → 周跳，从 k 起新弧段
                mean = MW[k];
                var = 0.0;
                cnt = 1;
                continue;
            }
        }
        mean = (cnt * mean + MW[k]) / (cnt + 1);
        var = (cnt * var + dev * dev) / (cnt + 1);
        cnt++;
    }
}

void CycleSlip::detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, const double lam1, const double lam2,
                         std::vector<char> &slip) {
    const int n = static_cast<int>(LGF.size());
    if (n < 6) return;
    const double gfThrLarge = 6.0 * (lam2 - lam1);
    const double gfThrSmall = std::fabs(lam2 - lam1);
    int i = 0;
    while (i < n) {
        if (!ok[i]) { i++; continue; }
        int j = i;
        while (j < n && (j == i || ok[j])) j++; // [i, j) 为连续弧段
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
                    auto coef = Math::polyFit(xv, yv, q);
                    std::vector<double> r(len);
                    for (int k = a; k < b; k++) r[k - a] = LGF[k] - Math::polyVal(coef, k - a);
                    for (int k = a + 1; k < b - 1; k++) {
                        if (slip[k]) continue;
                        const double jumpK = std::fabs(r[k - a] - r[k - 1 - a]);
                        if (const double jumpK1 = std::fabs(r[k + 1 - a] - r[k - a]); jumpK > gfThrLarge && jumpK1 > gfThrSmall) slip[k] = 1;
                    }
                }
            }
            a = b;
        }
        i = j;
    }
}
