#include "core/Application.h"
#include "widgets/GuiTimeConverter.h"
#include "widgets/GuiCoordConverter.h"
#include "widgets/GuiLsqSolver.h"
#include "widgets/GuiFileProcessor.h"
#include "widgets/GuiRealtimeProcessor.h"
#include "version.h"
#include "Log.h"

#include "imgui.h"

Application::Application() = default;

void Application::Initialize()
{
    Log::init("gnsslab.log");
    LOG_INFO("GnssLab v" PROJECT_VERSION " 启动");

    std::string title = "GnssLab v" PROJECT_VERSION;
    m_ui.Initialize(title.c_str(), 1280, 720);
}

void Application::Shutdown()
{
    // 1. 信号所有任务停止 (包括主任务和待清理任务)
    for (const auto &task : m_tasks) task->stop = true;
    for (const auto &task : m_closingTasks) task->stop = true;

    // 2. 等待所有线程汇合
    for (const auto &task : m_tasks) {
        if (task->worker.joinable()) task->worker.join();
    }
    for (const auto &task : m_closingTasks) {
        if (task->worker.joinable()) task->worker.join();
    }

    m_tasks.clear();
    m_closingTasks.clear();
    m_ui.Shutdown();
}

void Application::Update()
{
    // 垃圾回收：清理已结束的待关闭任务
    for (auto it = m_closingTasks.begin(); it != m_closingTasks.end(); ) {
        auto &task = *it;
        // 如果任务已完成、出错或线程本身已不可汇合，则进行清理
        if (task->done.load() || task->hasError || !task->worker.joinable()) {
            if (task->worker.joinable()) task->worker.join();
            it = m_closingTasks.erase(it);
        } else {
            ++it;
        }
    }
}

void Application::RenderMenuBar()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("任务"))
        {
            if (ImGui::MenuItem("打开文件..."))
                OpenOem7File();
            if (ImGui::MenuItem("连接到..."))
                m_showConnectDialog = true;
            ImGui::Separator();
            if (ImGui::MenuItem("退出", "Alt+F4"))
                m_ui.Shutdown();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("工具"))
        {
            ImGui::MenuItem("时间转换", nullptr, &m_showTimeConverter);
            ImGui::MenuItem("坐标转换", nullptr, &m_showCoordConverter);
            ImGui::MenuItem("最小二乘工具", nullptr, &m_showLsqSolver);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("帮助"))
        {
            if (ImGui::MenuItem("关于", nullptr))
                m_showAbout = true;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (m_showAbout)
    {
        ImGui::OpenPopup("关于 GnssLab");
        m_showAbout = false;
    }

    if (ImGui::BeginPopupModal("关于 GnssLab", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("GnssLab - GNSS 数据处理实验室");
        ImGui::Separator();
        ImGui::Text("版本: %s", PROJECT_VERSION);
        ImGui::Text("开发者: %s", COMPANY_NAME);
        ImGui::Spacing();
        if (ImGui::Button("确定", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }

    // ---- 连接对话框 ----
    if (m_showConnectDialog)
    {
        ImGui::OpenPopup("连接到 Oem7 实时流");
        m_showConnectDialog = false;
    }

    if (ImGui::BeginPopupModal("连接到 Oem7 实时流", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::InputText("IP 地址", m_ipBuffer, sizeof(m_ipBuffer));
        ImGui::InputInt("端口", &m_portBuffer);
        ImGui::Separator();
        if (ImGui::Button("连接", ImVec2(120, 0))) {
            ConnectToRealtime();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("取消", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }
}

void Application::RenderTasks()
{
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
            ImGui::TextDisabled("任务 -> 打开文件 开始处理");
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
            for (int i = 0; i < static_cast<int>(m_tasks.size()); i++) {
                auto &task = m_tasks[i];

                std::string label;
                if (task->isRealtime)
                    label = "[实时] " + task->fileName;
                else
                    label = task->fileName;

                if (task->loading.load())
                    label += " [加载中]";
                else if (task->hasError)
                    label += " [错误]";
                else if (task->done.load())
                    label += " [已停止]";
                
                label += "###" + std::to_string(i);

                ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
                if (i == m_taskToFocus) {
                    tabFlags |= ImGuiTabItemFlags_SetSelected;
                }

                bool open = true;
                if (ImGui::BeginTabItem(label.c_str(), &open, tabFlags)) {
                    m_activeTask = i;
                    if (m_taskToFocus == i) m_taskToFocus = -1; // 消费掉选中请求

                    if (task->isRealtime)
                        GuiRealtimeProcessor::RenderTask(task);
                    else
                        GuiFileProcessor::RenderTask(task);
                    
                    ImGui::EndTabItem();
                }

                if (!open) {
                    // 关闭任务：先发送停止信号，然后移入待清理列表
                    task->stop = true;
                    m_closingTasks.push_back(task);
                    m_tasks.erase(m_tasks.begin() + i);
                    
                    if (m_activeTask >= static_cast<int>(m_tasks.size()))
                        m_activeTask = static_cast<int>(m_tasks.size()) - 1;
                    i--; // 抵消循环中的 i++
                }
            }
            ImGui::EndTabBar();
        }

        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void Application::OpenOem7File()
{
    auto hwnd = static_cast<HWND>(ImGui::GetMainViewport()->PlatformHandleRaw);
    if (!hwnd) hwnd = GetActiveWindow();

    const std::string filePath = GuiFileProcessor::ShowOpenFileDialog(hwnd);
    if (filePath.empty()) return;

    auto sepPos = filePath.rfind('\\');
    if (sepPos == std::string::npos) sepPos = filePath.rfind('/');
    const std::string fileName = sepPos != std::string::npos
        ? filePath.substr(sepPos + 1) : filePath;

    auto task = std::make_shared<GuiFileProcessor::SppTask>();
    task->filePath = filePath;
    task->fileName = fileName;
    task->isRealtime = false;
    task->loading = true;

    task->worker = std::thread(GuiFileProcessor::SolveThread, task);

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask; // 设置选中标记
}

void Application::ConnectToRealtime()
{
    GuiRealtimeProcessor::ConnectionConfig config;
    config.ip = m_ipBuffer;
    config.port = m_portBuffer;

    auto task = std::make_shared<GuiRealtimeProcessor::SppTask>();
    task->fileName = config.ip + ":" + std::to_string(config.port);
    task->isRealtime = true;
    task->loading = true;

    task->worker = std::thread(GuiRealtimeProcessor::SolveRealtimeThread, task, config);

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask;
}

void Application::Render()
{
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

void Application::Run()
{
    Initialize();

    bool ready = true;
    while (m_ui.BeginFrame(&ready))
    {
        if (!ready) continue;
        Update();
        Render();
        m_ui.EndFrame();
    }

    Shutdown();
}
