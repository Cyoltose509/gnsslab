#include "core/Application.h"
#include <windows.h>
#include "widgets/tools/GuiTimeConverter.h"
#include "widgets/tools/GuiCoordConverter.h"
#include "widgets/tools/GuiLsqSolver.h"
#include "widgets/tools/GuiInsPanel.h"
#include "widgets/tools/GuiFtpDownload.h"
#include "solvers/view/GuiSolverPanel.h"
#include "solvers/spp/Task.h"
#include "solvers/ppp/Task.h"
#include "solvers/leo/Task.h"
#include "solvers/rtk/Task.h"
#include "solvers/view/GuiHelpers.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "download/ProductStore.h"
#include "download/ProductCatalog.h"
#include "io/ProductLoad.h"
#include "download/AutoSync.h"
#include "version.h"
#include "Log.h"

#include "imgui.h"

namespace {
    using TaskList = std::vector<std::shared_ptr<Solvers::Task> >;

    void stopAndJoinTasks(TaskList &tasks) {
        for (const auto &task: tasks) {
            task->stop = true;
            if (const auto c = task->solveResult()) {
                c->stop = true;
                if (c->view.qcWorker.joinable()) c->view.qcWorker.join();
            }
        }
        for (const auto &task: tasks) {
            if (task->worker.joinable()) task->worker.join();
        }
        tasks.clear();
    }

    void gcClosingTasks(TaskList &closing) {
        for (auto it = closing.begin(); it != closing.end();) {
            if (const auto &task = *it; task->done.load() || task->errorFlag() || !task->worker.joinable()) {
                if (task->worker.joinable()) task->worker.join();
                it = closing.erase(it);
            } else {
                ++it;
            }
        }
    }
}

namespace {
    /// GUI 提供的产品默认路径：引擎解析缺失项时回落到这里（CLI 不注册，只认 ini）。
    void registerProductDefaults() {
        ProductLoad::defaultsProvider() = [] {
            const auto &p = AppConfig::Products::instance();
            ProductLoad::Defaults d;
            d.paths = {p.sp3, p.clk, p.brdc, p.osb, p.att, p.erp,
                       Download::Store::activeFile(Download::Catalog::atx())};
            d.tro = p.tro;
            d.ionex = p.ionex;
            d.met = p.met;
            d.vmf = p.vmf;
            d.gpt3 = Download::Store::activeFile(Download::Catalog::gpt3());
            return d;
        };
    }
}

Application::Application() {
    registerProductDefaults();
}

void Application::Initialize() {
    Log::init("gnsslab.log");
    LOG_INFO << "GnssLab v" PROJECT_VERSION " 启动";

    const std::string title = "GnssLab v" PROJECT_VERSION;

    // 窗口分辨率与最大化状态持久化到 ini：默认 1280×720 普通窗口，
    // 可在 gnsslab.ini 的 [paths] 改 win_w / win_h / win_max。
    const auto &pm0 = AppConfig::PathMemory::instance();
    const int winW = pm0.getInt("win_w", 1280, 320, 7680);
    const int winH = pm0.getInt("win_h", 720, 240, 4320);
    const bool winMax = pm0.getInt("win_max", 0, 0, 1) != 0;
    m_ui.Initialize(title.c_str(), static_cast<float>(winW), static_cast<float>(winH), winMax);

    Download::AutoSync::setEnabled(AppConfig::PathMemory::instance().get("prod_autosync") != "0");
    Download::AutoSync::start();
}

void Application::Shutdown() {
    Download::AutoSync::stop();

    {
        int w = 0, h = 0;
        m_ui.GetRestoredSize(w, h);
        const auto &pm = AppConfig::PathMemory::instance();
        if (w > 0 && h > 0) {
            pm.setInt("win_w", w);
            pm.setInt("win_h", h);
        }
        pm.setInt("win_max", m_ui.IsMaximized() ? 1 : 0);
    }

    AppConfig::PathMemory::instance().flush();
    LOG_INFO << "GnssLab 退出";
    stopAndJoinTasks(m_tasks);
    stopAndJoinTasks(m_closingTasks);
    Tools::FtpDownload::Shutdown();
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
                m_quitRequested = true;
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("工具")) {
            ImGui::MenuItem("时间转换", nullptr, &m_showTimeConverter);
            ImGui::MenuItem("坐标转换", nullptr, &m_showCoordConverter);
            ImGui::MenuItem("最小二乘工具", nullptr, &m_showLsqSolver);
            if (ImGui::BeginMenu("惯导标定与对准")) {
                if (ImGui::MenuItem("六位置法标定")) OpenInsCalib();
                if (ImGui::MenuItem("静态解析粗对准")) OpenInsAlign();
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
        ImGui::MenuItem("下载", nullptr, &Tools::FtpDownload::s_open);
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
        const auto res = task->solveResult();
        const char *tag = Solvers::modeLabel(res->mode);
        const bool isRealtime = task->isRealtime;
        const std::string defaultName = std::string(tag) + "解算";

        std::string label;
        if (task->state == Solvers::Task::State::Config)
            label = std::string("[配置][") + tag + "] " + (res->fileName.empty() ? defaultName : res->fileName);
        else if (isRealtime)
            label = std::string("[实时][") + tag + "] " + res->fileName;
        else
            label = std::string("[") + tag + "] " + res->fileName;

        if (task->state != Solvers::Task::State::Config && !isRealtime) {
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
            Solvers::SolverPanel::renderTask(task);
            ImGui::EndTabItem();
        }

        if (!open) {
            LOG_INFO << "[任务] 关闭标签页: " << res->fileName;
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
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        const auto center = ImVec2(
            viewport->WorkPos.x + viewport->WorkSize.x * 0.5f,
            viewport->WorkPos.y + viewport->WorkSize.y * 0.5f
        );

        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        constexpr ImGuiWindowFlags flags =
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        if (ImGui::Begin("##empty_hint", nullptr, flags)) {
            ImGui::Text("GnssLab");
            ImGui::Spacing();
            ImGui::TextDisabled("任务 -> 开始处理");
        }
        ImGui::End();
        return;
    }


    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    constexpr ImGuiWindowFlags windowFlags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;


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

void Application::attachTask(const std::shared_ptr<Solvers::Task> &task) {
    task->state = Solvers::Task::State::Config;
    task->loading = false;
    task->done = false;
    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::OpenSppSolve() {
    const auto task = std::make_shared<Solvers::SPP::Task>();
    task->loadFromMemory();
    LOG_INFO << "[任务] 新建 SPP 配置：观测=" << task->core->view.obsPathBuf;

    attachTask(task);
}

void Application::OpenPppSolve() {
    const auto task = std::make_shared<Solvers::PPP::Task>();
    task->loadFromMemory();
    LOG_INFO << "[任务] 新建 PPP 配置：观测=" << task->core->view.obsPathBuf
            << ", 动态=" << (task->kinematic ? "是" : "否");

    attachTask(task);
}

void Application::OpenLeoSolve() {
    const auto task = std::make_shared<Solvers::LEO::Task>();
    task->loadFromMemory();
    LOG_INFO << "[任务] 新建 LEO 配置：观测=" << task->leoObsPathBuf;

    attachTask(task);
}

void Application::OpenRtkSolve() {
    const auto task = std::make_shared<Solvers::RTK::Task>();
    task->loadFromMemory();
    LOG_INFO << "[任务] 新建 RTK 配置：基准=" << task->baseObsPathBuf
            << ", 流动站=" << task->roverObsPathBuf;

    attachTask(task);
}

void Application::OpenInsCalib() {
    const auto task = Tools::InsPanel::MakeSixPosCalibTask();
    LOG_INFO << "[任务] 新建 六位置法标定 配置";
    attachTask(task);
}

void Application::OpenInsAlign() {
    const auto task = Tools::InsPanel::MakeStaticAlignTask();
    LOG_INFO << "[任务] 新建 静态解析粗对准 配置";
    attachTask(task);
}

void Application::Render() {
    RenderMenuBar();


    if (m_showTimeConverter)
        Tools::TimeConverter::Render(&m_showTimeConverter);
    if (m_showCoordConverter)
        Tools::CoordConverter::Render(&m_showCoordConverter);
    if (m_showLsqSolver)
        Tools::LsqSolver::Render(&m_showLsqSolver);
    if (Tools::FtpDownload::s_open)
        Tools::FtpDownload::Render(&Tools::FtpDownload::s_open);
    RenderTasks();
}

void Application::Run() {
    Initialize();

    bool ready = true;
    while (!m_quitRequested && m_ui.BeginFrame(&ready)) {
        if (!ready) continue;
        Update();
        Render();
        AppConfig::PathMemory::instance().flush();
        m_ui.EndFrame();
    }

    Shutdown();
}
