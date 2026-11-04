#pragma once

#include <set>
#include <string>

#include "core/PathMemory.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"

namespace Solvers::Helpers {
    void renderAdvancedParams(SolverOptions &opts, Mode mode);

    void saveSolverParams(Mode mode, const SolverOptions &opts);

    void loadSolverParams(Mode mode, SolverOptions &opts);

    // 截止角/采样间隔/系统/解算参数四项的统一读档（键名 = prefix_cutoff 等）
    inline void loadCommonCfg(const char *prefix, const Mode mode, SolverOptions *opts,
                              double *cutoffDeg, double *sampleInterval, std::set<char> *systems) {
        const auto &pm = AppConfig::PathMemory::instance();
        *cutoffDeg = pm.getNum(std::string(prefix) + "_cutoff", *cutoffDeg, 0.0, 90.0);
        *sampleInterval = pm.getNum(std::string(prefix) + "_interval", *sampleInterval, 0.0, 3600.0);
        if (const std::string sysStr = pm.get(std::string(prefix) + "_systems"); !sysStr.empty())
            *systems = stringToSysSet(sysStr);
        loadSolverParams(mode, *opts);
    }

    // 与 loadCommonCfg 对应的统一存档
    inline void saveCommonCfg(const char *prefix, const Mode mode, const SolverOptions &opts,
                              const double cutoffDeg, const double sampleInterval, const std::set<char> &systems) {
        const auto &pm = AppConfig::PathMemory::instance();
        pm.setNum(std::string(prefix) + "_cutoff", cutoffDeg);
        pm.setNum(std::string(prefix) + "_interval", sampleInterval);
        pm.set(std::string(prefix) + "_systems", sysSetToString(systems));
        saveSolverParams(mode, opts);
    }
}
