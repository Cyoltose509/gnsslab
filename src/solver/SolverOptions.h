#pragma once

#include "SPPConfig.h"
#include "RTKConfig.h"
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

    std::string sp3, clk, atx, osb, erp, tro, ionex, vmf, gpt3, att;
    std::string gnssSp3, gnssClk, gnssBrdc, refSp3, aux;
    int refPrn = 49;

    int realtimeSec = 0;
    int maxEpochs = 0;

    double cutoffDeg = 15.0;
    std::set<char> systems{'G', 'C'};
    double sampleInterval = 0.0;

    SPPConfig spp;
    RTKConfig rtk;

    /// 位置过程噪声（m^2/epoch）。PPP 静态为 0（位置固定），PPP 动态与 LEO 为 100。
    double posNoise = 100.0;

    double motionThresh = 0.0, sppVsPredThresh = 300.0;

    std::string csvPath;
    double refX = 0, refY = 0, refZ = 0;
    bool hasRef = false;

    double baseCoordX = 0, baseCoordY = 0, baseCoordZ = 0;
};
