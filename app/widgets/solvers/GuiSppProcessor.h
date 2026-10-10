#pragma once

#include <memory>
#include <string>

#include "GnssStruct.h"
#include "SolverTask.h"
#include "core/PathMemory.h"

namespace GuiSppProcessor {
    struct SppTask : GnssTask::SolverTaskBase {
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override; // 定义见 .cpp

        // ---- 解算钩子（骨架见 RunSingleStationSolve）----
        bool initSolver(void *solver) override;
        void beforeSolveEpoch(void *solver, GnssTask::ObsItem *item) override;
        bool afterSolveEpoch(void *solver, GnssTask::EpochData *data, GnssTask::ObsItem *item, int index) override;

        std::shared_ptr<GnssTask::SolveTask> core;
        // 持有最近一份有效星历：OEM7/实时逐历元给的快照可能整表为空，
        // 直接用会让该历元无星历可用，故留一份上次的（成员而非局部，因钩子要跨调用访问）。
        std::shared_ptr<EphemerisTable> lastGoodEph;
        // 回填上次填写（观测路径与实时 IP/端口此前都只写不读）
        SppTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &pm = GnssCore::PathMemory::instance();
            core->obsPathBuf = pm.get("spp_obs");
            core->rtIpBuf = pm.get("rt_ip");
            core->rtPortBuf = pm.get("rt_port");
        }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
}
