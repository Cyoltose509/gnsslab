#pragma once


#include "ui/Gui.h"

#include <vector>
#include <memory>

namespace GnssTask {
    struct SolveTask;
    struct SolverTaskBase;
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

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showAbout          = false;

    std::vector<std::shared_ptr<GnssTask::SolverTaskBase>> m_tasks;
    std::vector<std::shared_ptr<GnssTask::SolverTaskBase>> m_closingTasks;
    int m_activeTask   = -1;
    int m_taskToFocus  = -1;
};
