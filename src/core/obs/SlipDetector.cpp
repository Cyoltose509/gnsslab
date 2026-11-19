#include "core/obs/SlipDetector.h"
#include "core/Const.h"
#include "MathUtils.h"

void SlipDetector::configure(const FreqCombo &freqCombo, const bool useGF, const double GFMult) {
    mFreqCombo = freqCombo;
    lam1 = mFreqCombo.f1 > 0 ? C_MPS / mFreqCombo.f1 : mFreqCombo.lambda1;
    lam2 = mFreqCombo.f2 > 0 ? C_MPS / mFreqCombo.f2 : mFreqCombo.lambda2;
    mUseGF = useGF;
    mGFSlipMulti = GFMult > 0.0 ? GFMult : 3.0;
}

void SlipDetector::reset() {
    mMeanMW = 0;
    mVarMW = 0;
    mCountMW = 0;
    mPendActive = false;
}

void SlipDetector::flushGF() {
    mLBufGF.clear();
    mPBufGF.clear();
    reset();
}

SlipDetector::Result SlipDetector::setCycleGap() {
    flushGF();
    Result r;
    r.gap = true;
    return r;
}

SlipDetector::Result SlipDetector::phaseCycle(const SatID &sat, const TypeValueMap &tv, const ObsData *obs,
                                  const double osbP1, const double osbP2,
                                  const double osbL1, const double osbL2) {
    const auto itL1 = tv.find(mFreqCombo.phase1), itL2 = tv.find(mFreqCombo.phase2);
    const auto itC1 = tv.find(mFreqCombo.code1), itC2 = tv.find(mFreqCombo.code2);
    if (itL1 == tv.end() || itL2 == tv.end() || itC1 == tv.end() || itC2 == tv.end())
        return setCycleGap();

    const double L1 = itL1->second, L2 = itL2->second;
    const double C1 = itC1->second, C2 = itC2->second;

    Result r;
    if (lam1 <= 0 || lam2 <= 0) return r;

    // RINEX里的LLI
    int lli1 = 0, lli2 = 0;
    if (obs) {
        if (const auto it = obs->satTypeLLI.find(sat); it != obs->satTypeLLI.end()) {
            if (const auto j = it->second.find(mFreqCombo.phase1); j != it->second.end()) lli1 = j->second;
            if (const auto j = it->second.find(mFreqCombo.phase2); j != it->second.end()) lli2 = j->second;
        }
    }
    if (((lli1 | lli2) & 0x3) != 0) {
        flushGF();
        r.slip = true;
        return r;
    }


    const double MW = mFreqCombo.meterOfMW(C1, C2, L1, L2, osbP1, osbP2, osbL1, osbL2); // MW (米)
    // 物理上不可能的 MW 偏差阈值(米)
    static constexpr double kMWOutlierM = 1000.0;
    if (std::fabs(MW) > kMWOutlierM) {
        mMeanMW = 0;
        mVarMW = 0;
        mCountMW = 0;
        mPendActive = false;
        return r;
    }

    const double GFL = FreqCombo::phaseGF(L1, L2); // = L2 - L1 (米)
    const double GFP = FreqCombo::codeGF(C1, C2); // = P2 - P1 (米)
    const int arcIdx = static_cast<int>(mLBufGF.size());
    mLBufGF.push_back(GFL);
    mPBufGF.push_back(GFP);

    bool mwSlip = false;
    bool GFSlip = false;
    bool GFArtifact = false;

    if (mPendActive) {
        const double meanK = (mPrevCountMW * mPrevMeanMW + mPendMW) / (mPrevCountMW + 1.0);
        const double varK = (mPrevCountMW * mPrevVarMW + mPendDeviation * mPendDeviation) / (mPrevCountMW + 1.0);
        const double devNext = MW - meanK;
        if (std::fabs(devNext) > kMWOutlierM) {
            mPendActive = false;
            return r;
        }
        const bool slip = varK > 0.0 && std::fabs(devNext) >= 4.0 * std::sqrt(varK)
                          && std::fabs(MW - mPendMW) <= 1.0;
        if (slip) {
            mMeanMW = MW;
            mVarMW = 0;
            mCountMW = 0;
            mPendActive = false;
            mwSlip = true;
        } else {
            mMeanMW = meanK;
            mVarMW = varK;
            mCountMW++;
            mPendActive = false;
        }
    }

    {
        if (const double dev = MW - mMeanMW; mCountMW >= 5 && mVarMW > 0.0 && std::fabs(dev) >= 4.0 * std::sqrt(mVarMW)) {
            if (std::fabs(dev) > kMWOutlierM) {
                mPendActive = false;
                return r;
            }
            mPendActive = true;
            mPendMW = MW;
            mPendDeviation = dev;
            mPrevMeanMW = mMeanMW;
            mPrevVarMW = mVarMW;
            mPrevCountMW = mCountMW;
        } else {
            mMeanMW = (mCountMW * mMeanMW + MW) / (mCountMW + 1.0);
            mVarMW = (mCountMW * mVarMW + dev * dev) / (mCountMW + 1.0);
            mCountMW++;
        }
    }

    // GF 判相邻历元：dL=LGF[k]-LGF[k-1]，dP 同理；当前是 arcIdx，前一个是 arcIdx-1。
    // 原批量版要求缓冲区至少 6 个才判，等价于此处的 arcIdx >= 5。
    if (mUseGF && arcIdx >= 5) {
        const double dL = mLBufGF[arcIdx] - mLBufGF[arcIdx - 1];
        const double thr = mGFSlipMulti * (lam2 - lam1);
        if (std::fabs(dL) > thr) {
            // 一阶电离层在 GF 相位/码上等幅反号，dL+dP 抵消它；剩下的才是周跳/伪影。
            const double dP = mPBufGF[arcIdx] - mPBufGF[arcIdx - 1];
            if (std::fabs(dL + dP) > thr) {
                mMeanMW = MW;
                mVarMW = 0;
                mCountMW = 0;
                mPendActive = false;
                GFSlip = true;
                GFArtifact = std::fabs(dP) > 0.10;
            }
        }
    }

    r.slip = mwSlip;
    r.GF = GFSlip;
    r.artifact = GFArtifact;
    return r;
}

bool SlipDetector::clockJump(const std::vector<double> &codePhaseDiffs) {
    constexpr double c = C_MPS;
    constexpr double xi = 4.0;
    constexpr double msLo = 1e-7 * c - 3.0 * xi, msHi = 1e-5 * c + 3.0 * xi;
    constexpr double usThr = 1e-3 * c - 3.0 * xi;

    int nms = 0, nus = 0;
    for (const double dL: codePhaseDiffs) {
        if (dL > msLo && dL < msHi) nms++;
        else if (dL > usThr) nus++;
    }
    const int ns = nms + nus;
    const int totalValid = static_cast<int>(codePhaseDiffs.size());
    return totalValid >= 4 && ns >= 4 && ns >= 0.6 * totalValid;
}
