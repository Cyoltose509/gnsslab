#pragma once

#include "core/Const.h"
#include "core/obs/Weight.h"
#include <set>

struct RTKConfig {
    double cutoffElevRad = 10.0 * DEG_TO_RAD;
    std::set<SatSystem> enabledSystems = {SatSystem::G, SatSystem::R, SatSystem::E, SatSystem::C};
    double arThreshold = 3.0;
    double convEps = 1e-5;
    int maxIterFloat = 8;
    int maxIterFix = 6;
    int minSats = 4;
    bool useAR = true;
    bool useHealth = true;
    bool useCycleSlip = true;
    bool useGFCycleSlip = true;
    double GFSlipMult = 3.0;

    double codeOverPhaseSigma = 100.0;
    bool fullCovariance = true;
    bool excludeGeoRef = true;
    Weight phaseWeight{0.004, 0.003, 1e-3};

    double ambProcSigma = 1.0e-2;
    double ambInitSigma = 30.0;
};
