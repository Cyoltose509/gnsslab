#include "core/Application.h"
#include "tools/GuiTimeConverter.h"
#include "tools/GuiCoordConverter.h"
#include "tools/GuiOem7Processor.h"

#include "imgui.h"

Application::Application() = default;

void Application::Initialize()
{
    m_ui.Initialize("gnssLab", 1280, 800);
}

void Application::Shutdown()
{
    for (const auto &task: m_tasks) {
        if (task->worker.joinable()) task->worker.join();
    }
    m_tasks.clear();
    m_ui.Shutdown();
}

void Application::Update()
{
    // Reserved for future business logic updates
}

void Application::RenderMenuBar()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("任务"))
        {
            if (ImGui::MenuItem("打开OEM7文件..."))
                OpenOem7File();
            ImGui::Separator();
            if (ImGui::MenuItem("退出", "Alt+F4"))
                m_ui.Shutdown();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("工具"))
        {
            ImGui::MenuItem("时间转换", nullptr, &m_showTimeConverter);
            ImGui::MenuItem("坐标转换", nullptr, &m_showCoordConverter);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("帮助"))
        {
            ImGui::MenuItem("关于", nullptr, nullptr);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
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
            ImGui::Text("gnssLab");
            ImGui::Spacing();
            ImGui::TextDisabled("任务 -> 打开OEM7文件 开始处理");
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
                if (task->loading.load())
                    label = task->fileName + " [加载中]";
                else if (task->hasError)
                    label = task->fileName + " [错误]";
                else if (task->done.load())
                    label = task->fileName + " [完成]";
                else
                    label = task->fileName;
                label += "###" + std::to_string(i);

                ImGuiTabItemFlags tabFlags = ImGuiTabItemFlags_None;
                if (i == m_taskToFocus) {
                    tabFlags |= ImGuiTabItemFlags_SetSelected;
                }

                bool open = true;
                if (ImGui::BeginTabItem(label.c_str(), &open, tabFlags)) {
                    m_activeTask = i;
                    if (m_taskToFocus == i) m_taskToFocus = -1; // 消费掉选中请求

                    GuiOem7Processor::RenderTask(task);
                    ImGui::EndTabItem();
                }

                if (!open) {
                    // 关闭任务：如果正在处理，worker 会在析构时 join
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

    const std::string filePath = GuiOem7Processor::ShowOpenFileDialog(hwnd);
    if (filePath.empty()) return;

    auto sepPos = filePath.rfind('\\');
    if (sepPos == std::string::npos) sepPos = filePath.rfind('/');
    const std::string fileName = sepPos != std::string::npos
        ? filePath.substr(sepPos + 1) : filePath;

    auto task = std::make_shared<GuiOem7Processor::SppTask>();
    task->filePath = filePath;
    task->fileName = fileName;
    task->loading = true;

    task->worker = std::thread(GuiOem7Processor::SolveThread, task);

    m_tasks.push_back(task);
    m_activeTask = static_cast<int>(m_tasks.size()) - 1;
    m_taskToFocus = m_activeTask; // 设置选中标记
}

void Application::Render()
{
    RenderMenuBar();

    // ---- 工具子窗口（浮动） ----
    if (m_showTimeConverter)
        GuiTimeConverter::Render(&m_showTimeConverter);
    if (m_showCoordConverter)
        GuiCoordConverter::Render(&m_showCoordConverter);

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
