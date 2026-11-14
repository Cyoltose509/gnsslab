#pragma once

#include <memory>
#include <string>

#include "solvers/export/Core.h"

namespace Solvers {
    struct Task;
}

namespace Solvers::ExportUI {
    /// 打开导出配置（结果页「导出…」按钮调用）。
    void open(const std::shared_ptr<Task> &task);

    /// 每帧渲染一次模态；未打开时什么都不做。
    void render();

    /// 默认文件名：观测名_模式.扩展名
    std::string defaultName(const Task &task, Export::Format fmt);
}
