#pragma once

#include "Const.h"
#include "Troposphere.h"
#include <algorithm>
#include <cmath>
#include <set>


struct SppConfig {
    
    double cutoffElevRad = 15.0 * DEG_TO_RAD;
    std::set<char> enabledSystems{};
    double sigIFCode = 1.273;
    double minSinElev = 0.0;
    [[nodiscard]] double sinElevFloor() const {
        return minSinElev > 0.0 ? minSinElev : (std::max)(std::sin(cutoffElevRad), 1e-3);
    }

    double convEps = 1e-4; 
    int maxIter = 10; 
    int minCheckIter = 2; 

    // —— 处理开关（PPPStatic 族；SPP / RTK 不读本结构体中的这几项）——
    bool useAR = true;           // 是否尝试 AR 固定
    bool useCycleSlip = true;    // 是否做相位周跳探测
    double gfSlipMult = 3.0;     // GF 周跳阈值倍数（×单周跳阶跃）
    bool applyWindup = true;     // 相位缠绕改正
    bool applySolidTide = true;  // 固体潮改正
    bool useTroGrad = true;      // TRO 水平梯度固定改正（关=退化为仅 ZTD）

    double isbProcNoise = 1.0e-3; 
    double isbInitVar = 1.0;   
    double gloIfcbProcNoise = 1.0E-8;   
    double gloIfcbInitVar = 900.0;      

    TropoMap tropoMap = TropoMap::Gmf;
    double troZtdSigma = 0.05; 
    
    double sigmaPhase = 0.01273; 
    double arRatioThreshold = 3.0; 
    double arSigGate = 5.0; 
    double posNoise = 0.0; 

    double posVarFloor = 1.0E-6;
    double ambNoise = 1.0E-8; 

    double ambInitVar = 3600.0;
    double clockNoise = 0.0; 
    double ztdInitValue = 0.0; 
    double ztdInitVar = 0.36;
    double ztdProcNoise = 1.0E-8; 
    double ztdPriorSigma = 0.30; 

    double firstEpochPosVar = 3600.0;
    int minSatsForUpdate = 0; 

    double keepAmbMaxGap = 0.0;
    double codeBiasProcNoise = 1.0E-8;   
    double codeBiasInitVar = 900.0;
    int maxReanchorIter = 8; 
    double sppResidualThr = 10.0; 
};
