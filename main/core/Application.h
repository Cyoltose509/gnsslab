#pragma once


#include "ui/Gui.h"

#include <vector>
#include <memory>

namespace GuiFileProcessor {
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
    void RenderTasks();

    void OpenOem7File();
    void ConnectToRealtime();

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showAbout          = false;
    bool  m_showConnectDialog  = false;

    char  m_ipBuffer[64] = "47.114.134.129";
    int   m_portBuffer = 7190;

    // OEM7 文件任务
    std::vector<std::shared_ptr<GuiFileProcessor::SppTask>> m_tasks;
    std::vector<std::shared_ptr<GuiFileProcessor::SppTask>> m_closingTasks;
    int m_activeTask = -1;  // 当前激活的标签页索引
    int m_taskToFocus = -1; // 待强制选中的标签页索引
};
