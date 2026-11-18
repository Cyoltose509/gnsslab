#pragma once

#include <vector>
#include <set>
#include "core/obs/FreqCombo.h"

class CycleSlip {
public:
    struct Result {
        bool gap = false;      // 本历元无有效观测，弧段中断
        bool slip = false;     // 重设模糊度 / 剔除该星
        bool GF = false;       // GF 判跳，交 CycleSlipEpoch 裁决
        bool artifact = false; // 上述 GF 跳判为伪影（码相跳不一致）
    };

    void configure(const FreqCombo &fcIn, double GFThresh = 0.0, double GFMult = 3.0);

    void reset();

    Result feed(const SatID &sat, const TypeValueMap &tv, const ObsData *obs,
                double osbP1 = 0, double osbP2 = 0, double osbL1 = 0, double osbL2 = 0);

    Result feedGap();

    static void detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier);

    static void detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, double lam1, double lam2,
                         std::vector<char> &slip, std::vector<char> &artifact,
                         double mult = 3.0);

private:
    void flushGF();

    FreqCombo fc;
    double lam1 = 0, lam2 = 0;


    double MWMean = 0, MWVar = 0;
    int MWCnt = 0;

    double GFThresh = 0.0;
    double GFSlipMult = 3.0;

    std::vector<double> GFLbuf;
    std::vector<double> GFPbuf;
    std::vector<char> GFObuf;
    std::set<int> GFSlipSet;
    std::set<int> GFArtifactSet;

    bool pendActive = false;
    double pendMW = 0, pendDev = 0;
    double MWMeanB = 0, MWVarB = 0;
    int MWCntB = 0;
};


class CycleSlipEpoch {
public:

    void record(const SatID &sat, const bool isArtifact = false) {
        (isArtifact ? GFArtifact : GFSats).insert(sat);
    }

    void record(const SatID &sat, const CycleSlip::Result &r) {
        if (r.GF) record(sat, r.artifact);
    }

    SatIDSet decide(const int commonModeMinSats, SatIDSet &excludeOut) {
        excludeOut.clear();
        if (const int total = static_cast<int>(GFSats.size() + GFArtifact.size()); commonModeMinSats > 0 && total >= commonModeMinSats) {
            excludeOut = GFSats;
            excludeOut.insert(GFArtifact.begin(), GFArtifact.end());
            return {};
        }

        excludeOut = GFArtifact;
        return GFSats;
    }

    void clear() {
        GFSats.clear();
        GFArtifact.clear();
    }

private:
    SatIDSet GFSats;
    SatIDSet GFArtifact;
};
