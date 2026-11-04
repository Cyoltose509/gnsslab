#pragma once

#include <vector>
#include <set>
#include "FreqCombo.h"

class CycleSlip {
public:
    struct Result {
        bool gap = false;      // 本历元无有效观测，弧段中断
        bool slip = false;     // 重设模糊度 / 剔除该星
        bool gf = false;       // GF 判跳，交 CycleSlipEpoch 裁决
        bool artifact = false; // 上述 GF 跳判为伪影（码相跳不一致）
    };

    void configure(const FreqCombo &fcIn, double gfThresh = 0.0, double gfMult = 3.0);

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

    bool gfWideLaneSlip(double L1, double L2, double thr = 0.12);

private:
    void flushGf();

    FreqCombo fc;
    double lam1 = 0, lam2 = 0;


    double mwMean = 0, mwVar = 0;
    int mwCnt = 0;

    double gfPrev = 0;
    bool gfHasPrev = false;
    double gfThres = 0.0;
    double gfSlipMult = 3.0;

    double mGfWlPrev = 0;
    bool mGfWlHasPrev = false;

    std::vector<double> gfLbuf;
    std::vector<double> gfPbuf;
    std::vector<char> gfObuf;
    std::set<int> gfSlipSet;
    std::set<int> gfArtifactSet;

    bool pendActive = false;
    double pendMw = 0, pendDev = 0;
    double mwMeanB = 0, mwVarB = 0;
    int mwCntB = 0;
};


class CycleSlipEpoch {
public:

    void record(const SatID &sat, const bool isArtifact = false) {
        (isArtifact ? gfArtifact : gfSats).insert(sat);
    }

    void record(const SatID &sat, const CycleSlip::Result &r) {
        if (r.gf) record(sat, r.artifact);
    }

    SatIDSet decide(const int kCommonMode, SatIDSet &excludeOut) {
        excludeOut.clear();
        if (const int total = static_cast<int>(gfSats.size() + gfArtifact.size()); kCommonMode > 0 && total >= kCommonMode) {
            excludeOut = gfSats;
            excludeOut.insert(gfArtifact.begin(), gfArtifact.end());
            return {};
        }

        excludeOut = gfArtifact;
        return gfSats;
    }

    void clear() {
        gfSats.clear();
        gfArtifact.clear();
    }

private:
    SatIDSet gfSats;
    SatIDSet gfArtifact;
};
