#pragma once

#include <memory>

namespace GnssTask { struct SolverTaskBase; }

namespace Tools::InsPanel {
    // 新建"六位置法标定"任务
    std::shared_ptr<GnssTask::SolverTaskBase> MakeSixPosCalibTask();
    // 新建"静态解析粗对准"任务
    std::shared_ptr<GnssTask::SolverTaskBase> MakeStaticAlignTask();

    // 结果渲染分派：job 是 INS 工具任务则返回 true 且已渲染；否则返回 false（交给 GNSS 路径）。
    bool RenderInsResult(const std::shared_ptr<GnssTask::SolverTaskBase> &job);
}
