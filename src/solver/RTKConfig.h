#pragma once

#include "core/Const.h"
#include "core/obs/Weight.h"
#include <set>

struct RTKConfig {
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

    /// 码 σ : 相位 σ 的比值（原为方差比 1e4，σ 口径下为 100）
    double codeOverPhaseSigma = 100.0;
    bool fullCovariance = true;
    bool excludeGeoRef = true;
    Weight phaseWeight{0.004, 0.003, 1e-3};

    /// 滤波档开关不在这里：与 SPP 家族共用 `SolverOptions::filter`（none = 逐历元 LSQ，forward = EKF）。
    /// 位置/速度 σ 也共用 SolverOptions 的 posProcSigma/accProcSigma/velInitSigma。
    /// 双差模糊度随机游走 σ (周/历元)。它给的是「未建模误差在弧段上的积累速度」（电离层/多路径/天线），
    /// 不是纯数学量：给 1e-4（RTKLIB 的静态口径）会把模糊度锁在弧段起点的值上，位置变好但浮点解与
    /// 协方差脱节，ratio 掉到 3 以下固定率反而降；实测动态数据 1e-2 最平衡。
    double ambProcSigma = 1.0e-2;
    double ambInitSigma = 30.0;   // 新弧段/参考星切换后的模糊度初值 σ (周)
};
