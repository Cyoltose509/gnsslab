#pragma once

#include "Const.h"
#include "Weight.h"
#include <set>

struct RtkConfig {
    double cutoffElevRad = 10.0 * DEG_TO_RAD;
    std::set<char> enabledSystems = {'G', 'R', 'E', 'C'};
    double arThreshold = 3.0;
    double convEps = 1e-5;
    int maxIterFloat = 8;
    int maxIterFix = 6;
    int minSats = 4;
    bool useAR = true;
    bool useCycleSlip = true;
    bool useGfCycleSlip = true;
    double gfSlipMult = 3.0;

    double codeOverPhaseVar = 10000.0;
    bool fullCovariance = true;
    bool excludeGeoRef = true;
    Weight phaseWeight{0.004, 0.003, 1e-3};
};
