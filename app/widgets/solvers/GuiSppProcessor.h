#pragma once

#include <memory>
#include <string>

#include "SolverTask.h"
#include "core/PathMemory.h"

namespace Solvers::SppProcessor {
    struct SppTask : GnssTask::SolverTaskBase {
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override;

        bool initSolver(void *solver) override;

        void beforeSolveEpoch(void *solver, GnssTask::ObsItem *item) override;
        // afterSolveEpoch 不 override：走基类默认实现（首历元参考点初始化）

        std::shared_ptr<GnssTask::SolveTask> core;

        SppTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &pm = AppConfig::PathMemory::instance();
            core->obsPathBuf = pm.get("spp_obs");
            core->brdcPathBuf = pm.get("spp_brdc");
            core->rtEndpoints.resize(1);
            core->rtEndpoints[0].ip = pm.get("rt_ip", "47.114.134.129");
            core->rtEndpoints[0].port = pm.get("rt_port", "7190");
            core->cutoffDeg = pm.getNum("spp_cutoff", core->cutoffDeg, 0.0, 90.0);
            if (const std::string sysStr = pm.get("spp_systems"); !sysStr.empty()) core->enabledSystems = GnssTask::stringToSysSet(sysStr);
        }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
}
