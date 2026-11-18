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

    /// 码 σ : 相位 σ 的比值（原为方差比 1e4，σ 口径下为 100）
    double codeOverPhaseSigma = 100.0;
    bool fullCovariance = true;
    bool excludeGeoRef = true;
    Weight phaseWeight{0.004, 0.003, 1e-3};

    /// 滤波档开关不在这里：与 SPP 家族共用 `SolverOptions::filter`（none = 每历元重置滤波，forward = 跨历元 EKF）。
    /// 位置/速度 σ 也共用 SolverOptions 的 posProcSigma/accProcSigma/velInitSigma。
    double ambProcSigma = 1.0e-2;
    double ambInitSigma = 30.0;   // 新弧段/参考星切换后的模糊度初值 σ (周)
};
