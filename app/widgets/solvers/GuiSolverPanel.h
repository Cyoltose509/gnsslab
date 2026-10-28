#pragma once


#include <memory>


#include "SolverTask.h"

namespace Solvers::SolverPanel {
    void RenderTask(const std::shared_ptr<GnssTask::SolverTaskBase> &job);
    void LaunchQC(const std::shared_ptr<GnssTask::SolveTask> &task);
    void ExportCsv(const std::shared_ptr<GnssTask::SolveTask> &task);
}
