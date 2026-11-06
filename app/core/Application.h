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

    void makeSppTask();
    void makePppStaticTask();
    void makePppKinematicTask();
    void makeLeoTask();
    void makeRtkTask();
    void makeInsCalibTask();
    void makeInsAlignTask();

    /// 新任务入列并聚焦：六个 Open* 共用的收尾。
    void attachTask(const std::shared_ptr<Solvers::Task> &task);

    /// 「工具」菜单里的独立面板：菜单文字、开关、画面入口。菜单项与渲染都遍历同一张表。
    struct ToolPanel {
        const char *menu;
        bool Application::*open;
        void (*render)(bool *);
    };
    static const ToolPanel kToolPanels[3];

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
