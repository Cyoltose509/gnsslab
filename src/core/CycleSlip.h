#pragma once

#include <vector>
#include <set>
#include "FreqCombo.h"

class CycleSlip {
public:
    enum class Verdict { None, Slip, Outlier, Gap };

    void configure(const FreqCombo &fcIn);

    void reset();

    Verdict feed(double L1, double L2, double C1, double C2, bool valid,
                 double osbP1 = 0, double osbP2 = 0, double osbL1 = 0, double osbL2 = 0);

    static void detectMW(const std::vector<double> &MW, const std::vector<char> &ok,
                         std::vector<char> &slip, std::vector<char> &outlier);

    static void detectGF(const std::vector<double> &LGF, const std::vector<double> &PGF,
                         const std::vector<char> &ok, double lam1, double lam2,
                         std::vector<char> &slip, std::vector<char> &artifact,
                         const double mult = 3.0);

    void setGfThreshold(const double thr) { gfThres = thr; }



    void setGfSlipMult(const double m) { gfSlipMult = m > 0.0 ? m : 3.0; }

    bool gfWideLaneSlip(double L1, double L2, double thr = 0.12);

    bool mLastMwSlip = false;
    bool mLastGfSlip = false;
    bool mLastGfArtifact = false;
private:
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


    SatIDSet decide(const int kCommonMode, SatIDSet &excludeOut) {
        excludeOut.clear();
        if (const int total = static_cast<int>(gfSats.size() + gfArtifact.size()); kCommonMode > 0 && total >= kCommonMode) {
            excludeOut = gfSats;
            excludeOut.insert(gfArtifact.begin(), gfArtifact.end());
            return SatIDSet();
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
