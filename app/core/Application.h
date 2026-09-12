#pragma once


#include "ui/Gui.h"

#include <vector>
#include <memory>

namespace GuiFileProcessor {
    struct SppTask;
}

namespace GuiPppProcessor {
    struct PppTask;
}

namespace GuiLeoProcessor {
    struct LeoTask;
}

// 一组任务标签页相关的全部状态（任务列表 / 待清理列表 / 激活与聚焦索引 / tab 标识前缀），
// SPP / PPP / LEO 三套结构完全一致，打包后避免在调用点重复传 4~5 个耦合参数。
template <typename Task>
struct TaskTabGroup {
    std::vector<std::shared_ptr<Task>> tasks;
    std::vector<std::shared_ptr<Task>> closingTasks;
    int activeTask   = -1;
    int taskToFocus   = -1;
    const char *idPrefix = "";

    TaskTabGroup() = default;
    explicit TaskTabGroup(const char *prefix) : idPrefix(prefix) {}
};

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

    void OpenSppSolve();
    void OpenPppSolve();
    void OpenLeoSolve();

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
    bool  m_showLsqSolver      = false;
    bool  m_showAbout          = false;

    // 三组任务（SPP / PPP / LEO），每组打包任务列表、待清理列表、激活/聚焦索引与 tab 前缀
    TaskTabGroup<GuiFileProcessor::SppTask> m_spp{"spp"};
    TaskTabGroup<GuiPppProcessor::PppTask>  m_ppp{"ppp"};
    TaskTabGroup<GuiLeoProcessor::LeoTask>  m_leo{"leo"};
};
