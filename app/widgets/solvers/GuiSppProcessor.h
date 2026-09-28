#pragma once

#include <memory>
#include <string>

#include "GnssStruct.h"
#include "SolverTask.h"

namespace GuiSppProcessor {
    struct SppTask : GnssTask::SolverTaskBase {
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override; // 定义见 .cpp

        std::shared_ptr<GnssTask::SolveTask> core;
        SppTask() { core = std::make_shared<GnssTask::SolveTask>(); }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
}
