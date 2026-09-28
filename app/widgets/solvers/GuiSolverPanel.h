#pragma once

// ---------------------------------------------------------------------------
// 解算结果渲染分发层（SPP / PPP / LEO / RTK / 实时五种解算器共用）
//
// 这里不含任何解算器专属实现：RenderTask 只依赖 SolverTaskBase 的两个多态点
// （solveResult() / renderConfigPanel()），两个入口分派到各解算器自己的 Processor。
// 数据骨架（SolveTask / EpochData / SolverMode …）在 SolverTask.h 的 GnssTask 里。
// ---------------------------------------------------------------------------

#include <memory>


#include "SolverTask.h"

namespace GuiSolverPanel {
    void RenderTask(const std::shared_ptr<GnssTask::SolverTaskBase> &job);
    void LaunchQC(const std::shared_ptr<GnssTask::SolveTask> &task);
    void ExportCsv(const std::shared_ptr<GnssTask::SolveTask> &task);
}
