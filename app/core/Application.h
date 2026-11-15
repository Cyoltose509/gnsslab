#pragma once


#include "ui/Gui.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

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

    /// 关掉的标签页由后台回收线程做线程收尾与析构，界面线程只搬指针。
    void startReaper();
    void stopReaper();

    void makeSppTask();
    void makePppStaticTask();
    void makePppKinematicTask();
    void makeLeoTask();
    void makeRtkTask();
    void makeInsTask();
    void makeInsCalibTask();
    void makeInsAlignTask();
    void makeInsVerifyTask();

    /// 新任务入列并聚焦：六个 Open* 共用的收尾。
    void attachTask(const std::shared_ptr<Solvers::Task> &task);

    /// 「工具」菜单里的独立面板：菜单文案的 key、开关、画面入口。菜单项与渲染都遍历同一张表，
    /// 存 key 而不是译文，切语言后下一帧就是新文案。
    struct ToolPanel {
        const char *menuKey;
        bool Application::*open;
        void (*render)(bool *);
    };
    static const ToolPanel kToolPanels[3];

    /// 切语言 + 记住选择 + 记一行日志，三件事绑在一起，免得漏存或漏记。
    static void switchLanguage(std::string_view code);

    Gui   m_ui;
    bool  m_quitRequested      = false;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showEarthdata      = false;
    bool  m_showAbout          = false;

    std::vector<std::shared_ptr<Solvers::Task>> m_tasks;
    std::vector<std::shared_ptr<Solvers::Task>> m_closingTasks;
    int m_taskToFocus  = -1;

    std::mutex m_reapMutex;
    std::condition_variable m_reapCv;
    std::vector<std::shared_ptr<Solvers::Task>> m_reapQueue;
    std::thread m_reaper;
    bool m_reapQuit = false;
};
