#pragma once


#include "ui/Gui.h"

#include <vector>
#include <memory>

namespace Solvers {
    struct Core;
    struct Task;
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
    void RenderTaskTabList();

    void OpenSppSolve();
    void OpenPppSolve();
    void OpenLeoSolve();
    void OpenRtkSolve();
    void OpenInsCalib();
    void OpenInsAlign();

    /// 新任务入列并聚焦：六个 Open* 共用的收尾。
    void attachTask(const std::shared_ptr<Solvers::Task> &task);

    Gui   m_ui;
    bool  m_quitRequested      = false;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showAbout          = false;

    std::vector<std::shared_ptr<Solvers::Task>> m_tasks;
    std::vector<std::shared_ptr<Solvers::Task>> m_closingTasks;
    int m_activeTask   = -1;
    int m_taskToFocus  = -1;
};
