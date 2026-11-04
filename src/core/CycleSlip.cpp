#include "CycleSlip.h"
#include "Const.h"
#include "MathUtils.h"

void CycleSlip::configure(const FreqCombo &fcIn, const double gfThresh, const double gfMult) {
    fc = fcIn;
    lam1 = fc.f1 > 0 ? C_MPS / fc.f1 : fc.lambda1;
    lam2 = fc.f2 > 0 ? C_MPS / fc.f2 : fc.lambda2;
    gfThres = gfThresh;
    gfSlipMult = gfMult > 0.0 ? gfMult : 3.0;
}

void CycleSlip::reset() {
    mwMean = 0;
    mwVar = 0;
    mwCnt = 0;
    pendActive = false;
    gfPrev = 0;
    gfHasPrev = false;
    mGfWlPrev = 0;
    mGfWlHasPrev = false;
}

void CycleSlip::flushGf() {
    if (gfThres > 0.0 && gfLbuf.size() >= 6) {
        std::vector<char> slip(gfLbuf.size(), 0), artifact(gfLbuf.size(), 0);
        detectGF(gfLbuf, gfPbuf, gfObuf, lam1, lam2, slip, artifact, gfSlipMult);
        for (std::size_t k = 0; k < slip.size(); ++k) {
            if (slip[k]) gfSlipSet.insert(static_cast<int>(k));
            if (artifact[k]) gfArtifactSet.insert(static_cast<int>(k));
        }
    }
    gfLbuf.clear();
    gfPbuf.clear();
    gfObuf.clear();
    gfSlipSet.clear();
    gfArtifactSet.clear();
    reset();
}

CycleSlip::Result CycleSlip::feedGap() {
    flushGf();
    Result r;
    r.gap = true;
    return r;
}

CycleSlip::Result CycleSlip::feed(const SatID &sat, const TypeValueMap &tv, const ObsData *obs,
                                  const double osbP1, const double osbP2,
                                  const double osbL1, const double osbL2) {
    const auto itL1 = tv.find(fc.phase1), itL2 = tv.find(fc.phase2);
    const auto itC1 = tv.find(fc.code1), itC2 = tv.find(fc.code2);
    if (itL1 == tv.end() || itL2 == tv.end() || itC1 == tv.end() || itC2 == tv.end())
        return feedGap();

    const double L1 = itL1->second, L2 = itL2->second;
    const double C1 = itC1->second, C2 = itC2->second;

    Result r;
    if (lam1 <= 0 || lam2 <= 0) return r;

    // RINEX里的LLI
    int lli1 = 0, lli2 = 0;
    if (obs) {
        if (const auto it = obs->satTypeLLI.find(sat); it != obs->satTypeLLI.end()) {
            if (const auto j = it->second.find(fc.phase1); j != it->second.end()) lli1 = j->second;
            if (const auto j = it->second.find(fc.phase2); j != it->second.end()) lli2 = j->second;
        }
    }
    if (((lli1 | lli2) & 0x3) != 0) {
        flushGf();
        r.slip = true;
        return r;
    }


    const double mw = fc.MW_meter(C1, C2, L1, L2, osbP1, osbP2, osbL1, osbL2); // MW (米)
    // 物理上不可能的 MW 偏差阈值(米)
    static constexpr double kMwOutlierM = 1000.0;
    if (std::fabs(mw) > kMwOutlierM) {
        mwMean = 0;
        mwVar = 0;
        mwCnt = 0;
        pendActive = false;
        return r;
    }

    const double gfL = FreqCombo::GF_phase(L1, L2); // = L2 - L1 (米)
    const double gfP = FreqCombo::GF_code(C1, C2); // = P2 - P1 (米)
    const int arcIdx = static_cast<int>(gfLbuf.size());
    gfLbuf.push_back(gfL);
    gfPbuf.push_back(gfP);
    gfObuf.push_back(1);

    if (gfThres > 0.0 && gfLbuf.size() >= 6) {
        std::vector<char> slip(gfLbuf.size(), 0), artifact(gfLbuf.size(), 0);
        detectGF(gfLbuf, gfPbuf, gfObuf, lam1, lam2, slip, artifact, gfSlipMult);
        for (std::size_t k = 0; k < slip.size(); ++k) {
            if (slip[k]) gfSlipSet.insert(static_cast<int>(k));
            if (artifact[k]) gfArtifactSet.insert(static_cast<int>(k));
        }
    }

    bool mwSlip = false;
    bool gfSlip = false;
    bool gfArtifact = false;

    if (pendActive) {
        const double meanK = (mwCntB * mwMeanB + pendMw) / (mwCntB + 1.0);
        const double varK = (mwCntB * mwVarB + pendDev * pendDev) / (mwCntB + 1.0);
        const double devNext = mw - meanK;
        if (std::fabs(devNext) > kMwOutlierM) {
            pendActive = false;
            return r;
        }
        const bool slip = varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)
                          && std::fabs(mw - pendMw) <= 1.0;
        if (slip) {
            mwMean = mw;
            mwVar = 0;
            mwCnt = 0;
            pendActive = false;
            mwSlip = true;
        } else {
            mwMean = meanK;
            mwVar = varK;
            mwCnt++;
            pendActive = false;
        }
    }

    {
        if (const double dev = mw - mwMean; mwCnt >= 5 && mwVar > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(mwVar)) {
            if (std::fabs(dev) > kMwOutlierM) {
                pendActive = false;
                return r;
            }
            pendActive = true;
            pendMw = mw;
            pendDev = dev;
            mwMeanB = mwMean;
            mwVarB = mwVar;
            mwCntB = mwCnt;
        } else {
            mwMean = (mwCnt * mwMean + mw) / (mwCnt + 1.0);
            mwVar = (mwCnt * mwVar + dev * dev) / (mwCnt + 1.0);
            mwCnt++;
        }
    }

    if (arcIdx >= 1 && gfThres > 0.0 && gfSlipSet.count(arcIdx - 1)) {
        mwMean = mw;
        mwVar = 0;
        mwCnt = 0;
        pendActive = false;
        gfSlip = true;
        gfArtifact = gfArtifactSet.count(arcIdx - 1) > 0;
    }

    r.slip = mwSlip;
    r.gf = gfSlip;
    r.artifact = gfArtifact;
    return r;
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
            if (const double devNext = MW[k + 1] - meanK;
                !(varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)) || std::fabs(MW[k + 1] - MW[k]) > 1.0) {
                outlier[k] = 1;
            } else {
                slip[k] = 1;
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
                         std::vector<char> &slip, std::vector<char> &artifact, const double mult) {
    const int n = static_cast<int>(LGF.size());
    slip.assign(n, 0);
    artifact.assign(n, 0);
    if (n < 2) return;
    const double step = lam2 - lam1;
    const double thr = mult * step;
    for (int k = 1; k < n; ++k) {
        if (!ok[k] || !ok[k - 1]) continue;
        if (std::fabs(LGF[k] - LGF[k - 1]) > thr) {
            if (std::fabs(PGF[k] - PGF[k - 1]) > 0.10)
                artifact[k] = 1;
            else
                slip[k] = 1;
        }
    }
}

bool CycleSlip::gfWideLaneSlip(const double L1, const double L2, const double thr) {
    const double gf = FreqCombo::GF_phase(L1, L2);
    bool slip = false;
    if (mGfWlHasPrev) slip = std::fabs(gf - mGfWlPrev) > thr;
    mGfWlPrev = gf;
    mGfWlHasPrev = true;
    return slip;
}
