#pragma once

#ifndef GNSSLAB_APPLICATION_H
#define GNSSLAB_APPLICATION_H

#include "ui/Gui.h"

#include <vector>
#include <memory>
#include <string>

namespace GuiOem7Processor {
    struct SppTask;
}

class Application {
public:
    Application();
    void Run();

private:
    void Initialize();
    void Shutdown();
    void Update();
    void Render();
    void RenderMenuBar();
    void RenderDockSpace();
    void RenderTasks();

    void OpenOem7File();

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;

    // OEM7 文件任务
    std::vector<std::shared_ptr<GuiOem7Processor::SppTask>> m_tasks;
    int m_activeTask = -1;  // 当前激活的标签页索引
};

#endif
