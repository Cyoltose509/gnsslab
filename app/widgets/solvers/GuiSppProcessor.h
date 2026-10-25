#pragma once

#include <memory>
#include <string>

#include "GnssStruct.h"
#include "SolverTask.h"
#include "core/PathMemory.h"

namespace Solvers::SppProcessor {
    struct SppTask : GnssTask::SolverTaskBase {
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override; // 定义见 .cpp

        // ---- 解算钩子（骨架见 RunSingleStationSolve）----
        bool initSolver(void *solver) override;
        void beforeSolveEpoch(void *solver, GnssTask::ObsItem *item) override;
        bool afterSolveEpoch(void *solver, GnssTask::EpochData *data, GnssTask::ObsItem *item, int index) override;

        std::shared_ptr<GnssTask::SolveTask> core;
        std::shared_ptr<EphemerisTable> lastGoodEph;
        SppTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &pm = AppConfig::PathMemory::instance();
            core->obsPathBuf = pm.get("spp_obs");
            core->brdcPathBuf = pm.get("spp_brdc");
            // 实时流端点统一进 SolveTask::rtEndpoints（SPP 用 [0]）。默认与 ConnectionConfig / Application 一致。
            core->rtEndpoints.resize(1);
            core->rtEndpoints[0].ip = pm.get("rt_ip", "47.114.134.129");
            core->rtEndpoints[0].port = pm.get("rt_port", "7190");
            // 截止高度角 / 星座筛选：记忆进 ini（默认 15° / 全系统，见 SolverTask.h）
            core->cutoffDeg = pm.getNum("spp_cutoff", core->cutoffDeg, 0.0, 90.0);
            const std::string sysStr = pm.get("spp_systems");
            if (!sysStr.empty()) core->enabledSystems = GnssTask::stringToSysSet(sysStr);
        }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
}
