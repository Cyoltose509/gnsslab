#pragma once

#include "core/Const.h"
#include "core/atmo/Troposphere.h"
#include <algorithm>
#include <set>


struct SPPConfig {
    double cutoffElevRad = 15.0 * DEG_TO_RAD;
    std::set<char> enabledSystems{};
    double codeSigma = 1.273;
    double minSinElev = 0.0;

    [[nodiscard]] double sinElevFloor() const {
        return minSinElev > 0.0 ? minSinElev : (std::max)(std::sin(cutoffElevRad), 1e-3);
    }

    double convEps = 1e-4;
    int maxIter = 10;
    int minCheckIter = 2;

    bool useAR = true;
    bool useCycleSlip = true;
    double gfSlipMult = 3.0;
    bool applyWindup = true;
    bool applySolidTide = true;
    bool applyAtx = true;
    bool useTroGrad = true;

    bool useHealth = true;

    // σ 口径：下面所有 *Sigma 均为真 σ（进 P/Q 阵的使用点各自平方）；2026-11 前部分默认值是方差口径。
    double isbProcSigma = 3.16227766E-2; // = √1e-3，旧方差口径默认值的精确开方
    double isbInitSigma = 1.0;
    double gloIfcbProcSigma = 1.0E-4;
    double gloIfcbInitSigma = 30.0;

    Troposphere::Map tropoMap = Troposphere::Map::GMF;
    double troZtdSigma = 0.05;

    double phaseSigma = 0.01273;
    double arRatioThreshold = 3.0;
    double arGateSigma = 5.0;
    double posProcSigma = 0.0;

    /// 常速度预测：accProcSigma>0 时滤波状态含速度三态，位置按 x+v·Δt 预测、多普勒解作速度量测。
    /// 这两个成员给的是「关」的基线，SPP 的默认值在 app/solvers/spp/Task.h 的 defaultOptions 里打开；
    /// PPP/LEO 不接常速度模型，速度由多普勒单历元解算，不进滤波状态。
    double accProcSigma = 0.0;  // 加速度过程 σ (m/s²)
    double velInitSigma = 10.0; // 速度状态初始 σ (m/s)

    double posFloorSigma = 1.0E-3;
    double ambProcSigma = 1.0E-4;

    double ambInitSigma = 60.0;
    double clockProcSigma = 0.0;
    double ztdInitValue = 0.0;
    double ztdInitSigma = 0.6;
    double ztdProcSigma = 1.0E-4;
    double ztdPriorSigma = 0.30;

    double firstEpochPosSigma = 60.0;
    int minSatsForUpdate = 0;

    double keepAmbMaxGap = 0.0;
    double codeBiasProcSigma = 1.0E-4;
    double codeBiasInitSigma = 30.0;
    int maxReanchorIter = 8;
    double sppResidualThr = 10.0;
};
