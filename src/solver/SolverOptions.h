#pragma once

#include "SPPConfig.h"
#include "RTKConfig.h"
#include "solver/ins/INSConfig.h"
#include <set>
#include <string>

enum class RefForm { None, Fixed, File };

struct SolverOptions {

    enum class FilterMode { LSQ, Forward, Backward, Both };
    FilterMode filter = FilterMode::Forward;

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

    std::string leoAnt, leoPco;
    RefForm refForm = RefForm::None;

    int realtimeSec = 0;
    int maxEpochs = 0;

    double cutoffDeg = 15.0;
    std::set<char> systems{'G', 'C'};
    double sampleInterval = 0.0;

    SPPConfig spp;
    RTKConfig rtk;
    INSConfig ins;  // 纯惯导模式专用；其它模式不读

    /// 位置过程 σ：PPP 静态为 0（位置固定），PPP 动态与 LEO 为 10。
    double posProcSigma = 10.0;

    /// 常速度预测（SPP 滤波档）：加速度过程 σ (m/s/epoch) 与速度状态初始 σ (m/s)。
    double accProcSigma = 0.0;
    double velInitSigma = 10.0;

    double motionThresh = 0.0, sppVsPredThresh = 300.0;

    std::string csvPath;
    double refX = 0, refY = 0, refZ = 0;
    bool hasRef = false;

    double baseCoordX = 0, baseCoordY = 0, baseCoordZ = 0;
};

/// 滤波方向语义的唯一真源：`Filter` 与不套 `Filter::Session` 的双站重放都按这两条判。
/// 纯后向档正序只扫不解（留空槽位给倒序趟填）；backward/both 才要跑倒序趟。
constexpr bool filterSolvesForward(const SolverOptions::FilterMode m) {
    return m != SolverOptions::FilterMode::Backward;
}

constexpr bool filterHasBackward(const SolverOptions::FilterMode m) {
    return m == SolverOptions::FilterMode::Backward || m == SolverOptions::FilterMode::Both;
}

constexpr SolverOptions::FilterMode filterForRealtime(const SolverOptions::FilterMode m) {
    return filterHasBackward(m) ? SolverOptions::FilterMode::Forward : m;
}
