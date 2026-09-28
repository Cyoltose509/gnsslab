#include "core/Application.h"
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <cstdint>
#include "widgets/tools/GuiTimeConverter.h"
#include "widgets/tools/GuiCoordConverter.h"
#include "widgets/tools/GuiLsqSolver.h"
#include "widgets/solvers/GuiSolverPanel.h"
#include "widgets/solvers/GuiSppProcessor.h"
#include "widgets/solvers/GuiPppProcessor.h"
#include "widgets/solvers/GuiLeoProcessor.h"
#include "widgets/solvers/GuiRtkProcessor.h"
#include "widgets/solvers/GuiRealtimeProcessor.h"
#include "widgets/solvers/GuiHelpers.h"   // isSp3Path（旧单栏星历键迁移用）
#include "core/AppConfig.h"
#include "version.h"
#include "Log.h"

#include "imgui.h"

namespace {
    using TaskList = std::vector<std::shared_ptr<GnssTask::SolverTaskBase> >;

    void stopAndJoinTasks(TaskList &tasks) {
        for (auto &task: tasks) {
            task->stop = true;
            if (auto c = task->solveResult()) c->stop = true;
        }
        for (auto &task: tasks) {
            if (task->worker.joinable()) task->worker.join();
        }
        tasks.clear();
    }

    void gcClosingTasks(TaskList &closing) {
        for (auto it = closing.begin(); it != closing.end();) {
            if (auto &task = *it; task->done.load() || task->errorFlag() || !task->worker.joinable()) {
                if (task->worker.joinable()) task->worker.join();
                it = closing.erase(it);
            } else {
                ++it;
            }
        }
    }
} // namespace

Application::Application() = default;

void Application::Initialize() {
    Log::init("gnsslab.log");
    LOG_INFO << "GnssLab v" PROJECT_VERSION " 启动";

    std::string title = "GnssLab v" PROJECT_VERSION;
    m_ui.Initialize(title.c_str(), 1280, 720);
}

void Application::Shutdown() {
    stopAndJoinTasks(m_tasks);
    stopAndJoinTasks(m_closingTasks);
    m_ui.Shutdown();
}

void Application::Update() {
    gcClosingTasks(m_closingTasks);
}

void Application::RenderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("任务")) {
            if (ImGui::MenuItem("SPP解算..."))
                OpenSppSolve();
            if (ImGui::MenuItem("PPP解算..."))
                OpenPppSolve();
            if (ImGui::MenuItem("LEO定轨..."))
                OpenLeoSolve();
            if (ImGui::MenuItem("RTK解算..."))
                OpenRtkSolve();
            ImGui::Separator();
            if (ImGui::MenuItem("退出", "Alt+F4"))
                m_ui.Shutdown();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("工具")) {
            ImGui::MenuItem("时间转换", nullptr, &m_showTimeConverter);
            ImGui::MenuItem("坐标转换", nullptr, &m_showCoordConverter);
            ImGui::MenuItem("最小二乘工具", nullptr, &m_showLsqSolver);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("帮助")) {
            if (ImGui::MenuItem("关于", nullptr))
                m_showAbout = true;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (m_showAbout) {
        ImGui::OpenPopup("关于 GnssLab");
        m_showAbout = false;
    }

    if (ImGui::BeginPopupModal("关于 GnssLab", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("GnssLab - GNSS 数据处理实验室");
        ImGui::Separator();
        ImGui::Text("版本: %s", PROJECT_VERSION);
        ImGui::Text("开发者: %s", COMPANY_NAME);
        ImGui::Spacing();
        if (ImGui::Button("确定", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }
}

void Application::RenderTaskTabList() {
    auto &tasks = m_tasks;
    auto &closingTasks = m_closingTasks;
    int &activeTask = m_activeTask;
    int &taskToFocus = m_taskToFocus;

    for (int i = 0; i < static_cast<int>(tasks.size()); ++i) {
        auto &task = tasks[i];
        using State = GnssTask::SolverTaskBase::State;
        const auto res = task->solveResult();
        const char *tag = GnssTask::modeLabel(res->mode);
        const bool isRealtime = task->isRealtime;
        const std::string defaultName = std::string(tag) + "解算";

        std::string label;
        if (task->state == State::Config)
            label = std::string("[配置][") + tag + "] " + (res->fileName.empty() ? defaultName : res->fileName);
        else if (isRealtime)
            label = std::string("[实时][") + tag + "] " + res->fileName;
        else
            label = std::string("[") + tag + "] " + res->fileName;

        if (task->state != State::Config && !isRealtime) {
            if (task->loading.load())
                label += " [加载中]";
            else if (task->errorFlag())
                label += " [错误]";
            else if (task->done.load())
                label += " [已停止]";
        }

        label += "###";
        label += std::to_string(i);

        ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
        if (i == taskToFocus) tabFlags |= ImGuiTabItemFlags_SetSelected;

        bool open = true;
        if (ImGui::BeginTabItem(label.c_str(), &open, tabFlags)) {
            activeTask = i;
            if (taskToFocus == i) taskToFocus = -1;
            GuiSolverPanel::RenderTask(task);
            ImGui::EndTabItem();
        }

        if (!open) {
            task->stop = true;
            if (const auto c = task->solveResult()) c->stop = true;
            closingTasks.push_back(task);
            tasks.erase(tasks.begin() + i);

            if (activeTask >= static_cast<int>(tasks.size()))
                activeTask = static_cast<int>(tasks.size()) - 1;
            --i;
        }
    }
}

void Application::RenderTasks() {
    if (m_tasks.empty()) {
        // 空状态：居中提示
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        const auto center = ImVec2(
            viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
            viewport->WorkPos.y + viewport->WorkSize.y * 0.5f
        );

        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        constexpr ImGuiWindowFlags flags =
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove;

        if (ImGui::Begin("##empty_hint", nullptr, flags)) {
            ImGui::Text("GnssLab");
            ImGui::Spacing();
            ImGui::TextDisabled("任务 -> 开始处理");
        }
        ImGui::End();
        return;
    }

    // 渲染每个任务的标签页
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    constexpr ImGuiWindowFlags windowFlags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove;

    // 用固定 ID 的窗口做全视口容器
    if (ImGui::Begin("##task_host", nullptr, windowFlags)) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));

        if (ImGui::BeginTabBar("##tasks", ImGuiTabBarFlags_None)) {
            RenderTaskTabList();

            ImGui::EndTabBar();
        }

        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void Application::OpenSppSolve() {
    const auto task = std::make_shared<GuiSppProcessor::SppTask>();
    auto &core = *task->core;

    core.obsPathBuf = AppConfig::instance().get("spp_obs");
    if (!core.obsPathBuf.empty())
        core.navFiles = GnssTask::ScanNavFiles(core.obsPathBuf);

    core.rtIpBuf = AppConfig::instance().get("rt_ip", "47.114.134.129");
    core.rtPortBuf = AppConfig::instance().get("rt_port", "7190");

    task->state = GnssTask::SolverTaskBase::State::Config;
    task->loading = false;
    task->done = false;

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::OpenPppSolve() {
    const auto task = std::make_shared<GuiPppProcessor::PppTask>();

    const auto &cfg = AppConfig::instance();
    task->obsPathBuf = cfg.get("ppp_obs");
    task->sp3PathBuf = cfg.get("ppp_sp3");
    task->brdcPathBuf = cfg.get("ppp_brdc");
    if (const std::string legacy = cfg.get("ppp_orb"); !legacy.empty()) {
        // 旧单栏键名按扩展名迁移
        if (GuiHelpers::isSp3Path(legacy)) { if (task->sp3PathBuf.empty()) task->sp3PathBuf = legacy; } else {
            if (task->brdcPathBuf.empty()) task->brdcPathBuf = legacy;
        }
    }
    task->clkPathBuf = cfg.get("ppp_clk");
    task->osbPathBuf = cfg.get("ppp_osb");
    task->atxPathBuf = cfg.get("ppp_atx");
    task->kinematic = cfg.get("ppp_kinematic") == "1";

    task->core->mode = GnssTask::SolverMode::Ppp;

    task->state = GnssTask::SolverTaskBase::State::Config;
    task->loading = false;
    task->done = false;

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::OpenLeoSolve() {
    const auto task = std::make_shared<GuiLeoProcessor::LeoTask>();

    // 载入用户上次选择（项目外 ini）；无记录则留空，由用户填写。不再预填默认路径。
    const auto &cfg = AppConfig::instance();
    task->leoObsPathBuf = cfg.get("leo_obs");
    task->gnssSp3PathBuf = cfg.get("leo_gnss_sp3");
    task->gnssBrdcPathBuf = cfg.get("leo_gnss_brdc");
    if (const std::string legacy = cfg.get("leo_gnss_orb"); !legacy.empty()) {
        // 旧单栏键名按扩展名迁移
        if (GuiHelpers::isSp3Path(legacy)) { if (task->gnssSp3PathBuf.empty()) task->gnssSp3PathBuf = legacy; } else {
            if (task->gnssBrdcPathBuf.empty()) task->gnssBrdcPathBuf = legacy;
        }
    }
    task->gnssClkPathBuf = cfg.get("leo_gnss_clk");
    task->osbPathBuf = cfg.get("leo_osb");
    task->atxPathBuf = cfg.get("leo_atx");
    task->refSp3PathBuf = cfg.get("leo_ref_sp3");
    task->core->mode = GnssTask::SolverMode::Leo;

    task->state = GnssTask::SolverTaskBase::State::Config;
    task->loading = false;
    task->done = false;

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::OpenRtkSolve() {
    const auto task = std::make_shared<GuiRtkProcessor::RtkTask>();

    // 载入用户上次选择（项目外 ini）；无记录则留空，由用户填写。
    const auto &cfg = AppConfig::instance();
    task->baseObsPathBuf = cfg.get("rtk_base");
    task->roverObsPathBuf = cfg.get("rtk_rover");
    task->navPathBuf = cfg.get("rtk_nav");
    task->core->mode = GnssTask::SolverMode::Rtk;

    task->state = GnssTask::SolverTaskBase::State::Config;
    task->loading = false;
    task->done = false;

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::Render() {
    RenderMenuBar();

    // ---- 工具子窗口（浮动） ----
    if (m_showTimeConverter)
        GuiTimeConverter::Render(&m_showTimeConverter);
    if (m_showCoordConverter)
        GuiCoordConverter::Render(&m_showCoordConverter);
    if (m_showLsqSolver)
        GuiLsqSolver::Render(&m_showLsqSolver);

    // ---- 任务内容（全视口标签页） ----
    RenderTasks();
}

void Application::Run() {
    Initialize();

    bool ready = true;
    while (m_ui.BeginFrame(&ready)) {
        if (!ready) continue;
        Update();
        Render();
        m_ui.EndFrame();
    }

    Shutdown();
}
