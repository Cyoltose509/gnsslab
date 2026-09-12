#include "core/Application.h"
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <cstdint>
#include "widgets/GuiTimeConverter.h"
#include "widgets/GuiCoordConverter.h"
#include "widgets/GuiLsqSolver.h"
#include "widgets/GuiFileProcessor.h"
#include "widgets/GuiPppProcessor.h"
#include "widgets/GuiLeoProcessor.h"
#include "widgets/GuiRealtimeProcessor.h"
#include "widgets/GuiHelpers.h"   // isSp3Path（旧单栏星历键迁移用）
#include "core/AppConfig.h"
#include "version.h"
#include "Log.h"

#include "imgui.h"

namespace {
    template<typename Task>
    bool taskIsRealtime(const Task &) { return false; }//NOLINT

    bool taskIsRealtime(const GuiFileProcessor::SppTask &t) { return t.isRealtime; }

    template<typename Task, typename RenderFn>
    void renderTaskTabList(TaskTabGroup<Task> &group,const char *tag,const char *defaultName,RenderFn &&renderFn) {
        auto &tasks = group.tasks;
        auto &closingTasks = group.closingTasks;
        int &activeTask = group.activeTask;
        int &taskToFocus = group.taskToFocus;
        const char *idPrefix = group.idPrefix;

        for (int i = 0; i < static_cast<int>(tasks.size()); ++i) {
            auto &task = tasks[i];
            using State = typename Task::State;
            std::string label;
            if (task->state == State::Config)
                label = std::string("[配置][") + tag + "] " + (task->fileName.empty() ? defaultName : task->fileName);
            else if (taskIsRealtime(*task))
                label = std::string("[实时][") + tag + "] " + task->fileName;
            else
                label = std::string("[") + tag + "] " + task->fileName;

            if (task->state != State::Config && !taskIsRealtime(*task)) {
                if (task->loading.load())
                    label += " [加载中]";
                else if (task->hasError)
                    label += " [错误]";
                else if (task->done.load())
                    label += " [已停止]";
            }

            label += "###";
            label += idPrefix;
            label += std::to_string(i);

            ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
            if (i == taskToFocus) tabFlags |= ImGuiTabItemFlags_SetSelected;

            bool open = true;
            if (ImGui::BeginTabItem(label.c_str(), &open, tabFlags)) {
                activeTask = i;
                if (taskToFocus == i) taskToFocus = -1;
                renderFn(task);
                ImGui::EndTabItem();
            }

            if (!open) {
                // 关闭任务：先发送停止信号，然后移入待清理列表
                task->stop = true;
                closingTasks.push_back(task);
                tasks.erase(tasks.begin() + i);

                if (activeTask >= static_cast<int>(tasks.size()))
                    activeTask = static_cast<int>(tasks.size()) - 1;
                --i;
            }
        }
    }

    template<typename Task>
    void stopAndJoinTasks(std::vector<std::shared_ptr<Task> > &tasks) {
        for (auto &task: tasks) task->stop = true;
        for (auto &task: tasks) {
            if (task->worker.joinable()) task->worker.join();
        }
        tasks.clear();
    }

    template<typename Task>
    void gcClosingTasks(std::vector<std::shared_ptr<Task> > &closing) {
        for (auto it = closing.begin(); it != closing.end();) {
            if (auto &task = *it; task->done.load() || task->hasError || !task->worker.joinable()) {
                if (task->worker.joinable()) task->worker.join();
                it = closing.erase(it);
            } else {
                ++it;
            }
        }
    }
    template<typename Task>
    void stopAndJoinTasks(TaskTabGroup<Task> &g) {
        stopAndJoinTasks(g.tasks);
        stopAndJoinTasks(g.closingTasks);
    }

    template<typename Task>
    void gcClosingTasks(TaskTabGroup<Task> &g) {
        gcClosingTasks(g.closingTasks);
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
    stopAndJoinTasks(m_spp);
    stopAndJoinTasks(m_ppp);
    stopAndJoinTasks(m_leo);
    m_ui.Shutdown();
}

void Application::Update() {
    gcClosingTasks(m_spp);
    gcClosingTasks(m_ppp);
    gcClosingTasks(m_leo);
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

void Application::RenderTasks() {
    if (m_spp.tasks.empty() && m_ppp.tasks.empty() && m_leo.tasks.empty()) {
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
            renderTaskTabList(m_spp, "SPP", "SPP解算",
                              [](const auto &task) { GuiFileProcessor::RenderTask(task, task->isRealtime); });
            renderTaskTabList(m_ppp, "PPP", "PPP解算",
                              [](const auto &task) { GuiPppProcessor::RenderTask(task); });
            renderTaskTabList(m_leo, "LEO", "LEO定轨",
                              [](const auto &task) { GuiLeoProcessor::RenderTask(task); });

            ImGui::EndTabBar();
        }

        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void Application::OpenSppSolve() {
    const auto task = std::make_shared<GuiFileProcessor::SppTask>();

    // 载入用户上次选择（项目外 ini）；无记录则留空，由用户填写。实时流 IP/端口保留默认配置。
    task->obsPathBuf = AppConfig::instance().get("obs_path");
    // 与 PPP 同样恢复上次伴随文件：obs 路径若存在，自动重新扫描同目录伴生星历(.??N/.??G/.??C)。
    if (!task->obsPathBuf.empty())
        task->navFiles = GuiFileProcessor::ScanNavFiles(task->obsPathBuf);

    task->rtIpBuf = AppConfig::instance().get("rt_ip", "47.114.134.129");
    task->rtPortBuf = AppConfig::instance().get("rt_port", "7190");

    // 进入配置面板（文件 / 实时 双标签页），等用户点击"开始解算"后才启动线程
    task->state = GuiFileProcessor::SppTask::State::Config;
    task->loading = false;
    task->done = false;

    m_spp.tasks.push_back(task);
    m_spp.activeTask = static_cast<int>(m_spp.tasks.size()) - 1;
    m_spp.taskToFocus = m_spp.activeTask;
}

void Application::OpenPppSolve() {
    const auto task = std::make_shared<GuiPppProcessor::PppTask>();

    // 载入用户上次选择（项目外 ini）；无记录则留空，由用户填写。不再预填默认路径。
    const auto &cfg = AppConfig::instance();
    task->obsPathBuf = cfg.get("ppp_obs");
    task->sp3PathBuf = cfg.get("ppp_sp3");
    task->brdcPathBuf = cfg.get("ppp_brdc");
    if (const std::string legacy = cfg.get("ppp_orb"); !legacy.empty()) { // 旧单栏键名按扩展名迁移
        if (GuiHelpers::isSp3Path(legacy)) { if (task->sp3PathBuf.empty()) task->sp3PathBuf = legacy; }
        else { if (task->brdcPathBuf.empty()) task->brdcPathBuf = legacy; }
    }
    task->clkPathBuf = cfg.get("ppp_clk");
    task->osbPathBuf = cfg.get("ppp_osb");
    task->atxPathBuf = cfg.get("ppp_atx");

    task->state = GuiPppProcessor::PppTask::State::Config;
    task->loading = false;
    task->done = false;

    m_ppp.tasks.push_back(task);
    m_ppp.activeTask = static_cast<int>(m_ppp.tasks.size()) - 1;
    m_ppp.taskToFocus = m_ppp.activeTask;
}

void Application::OpenLeoSolve() {
    const auto task = std::make_shared<GuiLeoProcessor::LeoTask>();

    // 载入用户上次选择（项目外 ini）；无记录则留空，由用户填写。不再预填默认路径。
    const auto &cfg = AppConfig::instance();
    task->leoObsPathBuf = cfg.get("leo_obs");
    task->gnssSp3PathBuf = cfg.get("leo_gnss_sp3");
    task->gnssBrdcPathBuf = cfg.get("leo_gnss_brdc");
    if (const std::string legacy = cfg.get("leo_gnss_orb"); !legacy.empty()) { // 旧单栏键名按扩展名迁移
        if (GuiHelpers::isSp3Path(legacy)) { if (task->gnssSp3PathBuf.empty()) task->gnssSp3PathBuf = legacy; }
        else { if (task->gnssBrdcPathBuf.empty()) task->gnssBrdcPathBuf = legacy; }
    }
    task->gnssClkPathBuf = cfg.get("leo_gnss_clk");
    task->osbPathBuf = cfg.get("leo_osb");
    task->atxPathBuf = cfg.get("leo_atx");
    task->refSp3PathBuf = cfg.get("leo_ref_sp3");

    task->state = GuiLeoProcessor::LeoTask::State::Config;
    task->loading = false;
    task->done = false;

    m_leo.tasks.push_back(task);
    m_leo.activeTask = static_cast<int>(m_leo.tasks.size()) - 1;
    m_leo.taskToFocus = m_leo.activeTask;
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
