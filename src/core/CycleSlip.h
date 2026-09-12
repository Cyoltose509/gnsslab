#pragma once

#include <vector>
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
                         std::vector<char> &slip);

private:
    FreqCombo fc;
    double lam1 = 0, lam2 = 0;

    double mwMean = 0, mwVar = 0;
    int mwCnt = 0;

    bool pendActive = false;
    double pendMw = 0, pendDev = 0;
    double mwMeanB = 0, mwVarB = 0;
    int mwCntB = 0;
};
