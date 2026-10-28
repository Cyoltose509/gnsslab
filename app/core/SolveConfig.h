#pragma once
#include <set>
#include <string>

namespace AppConfig { struct IniFile; }

struct SolveConfig {
    std::string source = "file";
    std::string obsPath;
    std::string navPath;
    std::string basePath, roverPath;
    std::string ip;
    int port = 0;
    std::string baseIp, roverIp;
    int basePort = 0, roverPort = 0;

    // PPP 精密产品
    std::string sp3, clk, atx, osb, erp, tro, ionex;
    std::string gnssSp3, gnssClk, gnssBrdc, refSp3, aux;
    int refPrn = 49;

    int realtimeSec = 0;
    int maxEpochs = 0;


    double cutoffDeg = 15.0;
    std::set<char> systems{'G', 'C'};
    double sigIFCode = 1.273, sigPhase = 0.01273;
    int useAR = 1, useCycleSlip = 1, useGfCycleSlip = 1, fullCovariance = 1;
    int solidTide = 1;
    int kinematic = 0;
    double posNoise = 100.0;
    double staticPosNoise = 0.0;
    int applyWindup = 1;
    double ztdQ = 1.0E-8;
    double ambQ = 1.0E-8;
    double troZtdSigma = 0.05;
    int useTroGrad = 1;
    std::string tropoMap = "gmf";
    double ztdPrior = 0.30;
    double ztdInitValue = 0.0, ztdInitVar = 0.36;

    double ambInitVar = 3600.0;
    double motionThresh = 0.0, sppVsPredThresh = 300.0;

    double firstEpochPosVar = 3600.0;

    double keepAmbMaxGap = 0.0;

    double codeBiasProcNoise = 1.0E-8;
    double codeBiasInitVar = 900.0;

    double gloIfcbProcNoise = 1.0E-8;
    double gloIfcbInitVar = 900.0;

    double isbProcNoise = 1.0e-3;
    double isbInitVar = 1.0;
    double posVarFloor = 1.0E-6;


    std::string csvPath;
    double refX = 0, refY = 0, refZ = 0;
    bool hasRef = false;

    double baseCoordX = 0, baseCoordY = 0, baseCoordZ = 0;
};

SolveConfig loadConfig(const AppConfig::IniFile &ini);
