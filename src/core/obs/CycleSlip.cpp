#include "core/obs/CycleSlip.h"
#include "core/Const.h"
#include "MathUtils.h"

void CycleSlip::configure(const FreqCombo &fcIn, const double GFThresh, const double GFMult) {
    fc = fcIn;
    lam1 = fc.f1 > 0 ? C_MPS / fc.f1 : fc.lambda1;
    lam2 = fc.f2 > 0 ? C_MPS / fc.f2 : fc.lambda2;
    this->GFThresh = GFThresh;
    GFSlipMult = GFMult > 0.0 ? GFMult : 3.0;
}

void CycleSlip::reset() {
    MWMean = 0;
    MWVar = 0;
    MWCnt = 0;
    pendActive = false;
}

void CycleSlip::flushGF() {
    GFLbuf.clear();
    GFPbuf.clear();
    GFObuf.clear();
    GFSlipSet.clear();
    GFArtifactSet.clear();
    reset();
}

CycleSlip::Result CycleSlip::feedGap() {
    flushGF();
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
        flushGF();
        r.slip = true;
        return r;
    }


    const double MW = fc.MW_meter(C1, C2, L1, L2, osbP1, osbP2, osbL1, osbL2); // MW (米)
    // 物理上不可能的 MW 偏差阈值(米)
    static constexpr double kMWOutlierM = 1000.0;
    if (std::fabs(MW) > kMWOutlierM) {
        MWMean = 0;
        MWVar = 0;
        MWCnt = 0;
        pendActive = false;
        return r;
    }

    const double GFL = FreqCombo::GF_phase(L1, L2); // = L2 - L1 (米)
    const double GFP = FreqCombo::GF_code(C1, C2); // = P2 - P1 (米)
    const int arcIdx = static_cast<int>(GFLbuf.size());
    GFLbuf.push_back(GFL);
    GFPbuf.push_back(GFP);
    GFObuf.push_back(1);

    if (GFThresh > 0.0 && GFLbuf.size() >= 6) {
        std::vector<char> slip(GFLbuf.size(), 0), artifact(GFLbuf.size(), 0);
        detectGF(GFLbuf, GFPbuf, GFObuf, lam1, lam2, slip, artifact, GFSlipMult);
        for (std::size_t k = 0; k < slip.size(); ++k) {
            if (slip[k]) GFSlipSet.insert(static_cast<int>(k));
            if (artifact[k]) GFArtifactSet.insert(static_cast<int>(k));
        }
    }

    bool mwSlip = false;
    bool GFSlip = false;
    bool GFArtifact = false;

    if (pendActive) {
        const double meanK = (MWCntB * MWMeanB + pendMW) / (MWCntB + 1.0);
        const double varK = (MWCntB * MWVarB + pendDev * pendDev) / (MWCntB + 1.0);
        const double devNext = MW - meanK;
        if (std::fabs(devNext) > kMWOutlierM) {
            pendActive = false;
            return r;
        }
        const bool slip = varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)
                          && std::fabs(MW - pendMW) <= 1.0;
        if (slip) {
            MWMean = MW;
            MWVar = 0;
            MWCnt = 0;
            pendActive = false;
            mwSlip = true;
        } else {
            MWMean = meanK;
            MWVar = varK;
            MWCnt++;
            pendActive = false;
        }
    }

    {
        if (const double dev = MW - MWMean; MWCnt >= 5 && MWVar > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(MWVar)) {
            if (std::fabs(dev) > kMWOutlierM) {
                pendActive = false;
                return r;
            }
            pendActive = true;
            pendMW = MW;
            pendDev = dev;
            MWMeanB = MWMean;
            MWVarB = MWVar;
            MWCntB = MWCnt;
        } else {
            MWMean = (MWCnt * MWMean + MW) / (MWCnt + 1.0);
            MWVar = (MWCnt * MWVar + dev * dev) / (MWCnt + 1.0);
            MWCnt++;
        }
    }

    // 当前历元的索引就是 arcIdx（push 前取的 size），detectGF 的 slip[k] 判的是 k 与 k-1 之间。
    if (arcIdx >= 1 && GFThresh > 0.0 && GFSlipSet.count(arcIdx)) {
        MWMean = MW;
        MWVar = 0;
        MWCnt = 0;
        pendActive = false;
        GFSlip = true;
        GFArtifact = GFArtifactSet.count(arcIdx) > 0;
    }

    r.slip = mwSlip;
    r.GF = GFSlip;
    r.artifact = GFArtifact;
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
        const double dL = LGF[k] - LGF[k - 1];
        if (std::fabs(dL) <= thr) continue;
        const double dP = PGF[k] - PGF[k - 1];
        // 一阶电离层在 GF 相位/码上等幅反号，dL+dP 抵消它；剩下的才是周跳/伪影。
        if (std::fabs(dL + dP) <= thr) continue;
        if (std::fabs(dP) > 0.10)
            artifact[k] = 1;
        else
            slip[k] = 1;
    }
}
