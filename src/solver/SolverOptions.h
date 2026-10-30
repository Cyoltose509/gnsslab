#pragma once

#include "SppConfig.h"
#include "RtkConfig.h"
#include <set>
#include <string>

struct SolverOptions {
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
    double sampleInterval = 0.0;
    int kinematic = 0;

    SppConfig spp;
    RtkConfig rtk;

    double posNoise = 100.0;
    double staticPosNoise = 0.0;

    double motionThresh = 0.0, sppVsPredThresh = 300.0;

    std::string csvPath;
    double refX = 0, refY = 0, refZ = 0;
    bool hasRef = false;

    double baseCoordX = 0, baseCoordY = 0, baseCoordZ = 0;
};
