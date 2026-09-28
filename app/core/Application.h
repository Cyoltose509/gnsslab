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

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showAbout          = false;

    // 五种解算器（SPP / PPP / LEO / RTK / 实时）的任务混在同一张列表里：它们都实现
    // SolverTaskBase（生命周期状态机 + 渲染层的两个多态点），外壳逻辑因此完全同构。
    std::vector<std::shared_ptr<GnssTask::SolverTaskBase>> m_tasks;
    std::vector<std::shared_ptr<GnssTask::SolverTaskBase>> m_closingTasks;
    int m_activeTask   = -1;
    int m_taskToFocus  = -1;
};
