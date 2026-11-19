#pragma once

#include <vector>
#include <set>
#include "core/SatID.h"
#include "core/obs/FreqCombo.h"

class SlipDetector {
public:
    struct Result {
        bool gap = false;      // 本历元无有效观测，弧段中断
        bool slip = false;     // 重设模糊度 / 剔除该星
        bool GF = false;       // GF 判跳，交 SlipEpoch 裁决
        bool artifact = false; // 上述 GF 跳判为伪影（码相跳不一致）
    };

    void configure(const FreqCombo &freqCombo, bool useGF = false, double GFMult = 3.0);

    void reset();

    Result phaseCycle(const SatID &sat, const TypeValueMap &tv, const ObsData *obs,
                      double osbP1 = 0, double osbP2 = 0, double osbL1 = 0, double osbL2 = 0);

    Result setCycleGap();

    /// 相邻历元上无周跳历史卫星的码相跳变是否同向共模。
    static bool clockJump(const std::vector<double> &codePhaseDiffs);

private:
    void flushGF();

    FreqCombo mFreqCombo;
    double lam1 = 0, lam2 = 0;


    double mMeanMW = 0, mVarMW = 0;
    int mCountMW = 0;

    bool mUseGF = false;
    double mGFSlipMulti = 3.0;

    std::vector<double> mLBufGF;
    std::vector<double> mPBufGF;

    bool mPendActive = false;
    double mPendMW = 0, mPendDeviation = 0;
    double mPrevMeanMW = 0, mPrevVarMW = 0;
    int mPrevCountMW = 0;
};


class SlipEpoch {
public:

    void record(const SatID &sat, const bool isArtifact = false) {
        (isArtifact ? mGFArtifact : mGFSats).insert(sat);
    }

    void record(const SatID &sat, const SlipDetector::Result &r) {
        if (r.GF) record(sat, r.artifact);
    }

    std::set<SatID> decide(const int commonModeMinSats, std::set<SatID> &excludeOut) {
        excludeOut.clear();
        if (const int total = static_cast<int>(mGFSats.size() + mGFArtifact.size());
            commonModeMinSats > 0 && total >= commonModeMinSats) {
            excludeOut = mGFSats;
            excludeOut.insert(mGFArtifact.begin(), mGFArtifact.end());
            return {};
        }

        excludeOut = mGFArtifact;
        return mGFSats;
    }

    void clear() {
        mGFSats.clear();
        mGFArtifact.clear();
    }

private:
    std::set<SatID> mGFSats;
    std::set<SatID> mGFArtifact;
};
