#include "core/Application.h"
#include "tools/GuiTimeConverter.h"
#include "tools/GuiCoordConverter.h"

#include "imgui.h"

Application::Application() = default;

void Application::Initialize()
{
    m_ui.Initialize("gnssLab", 1280, 800);
}

void Application::Shutdown()
{
    m_ui.Shutdown();
}

void Application::Update()
{
    // Reserved for future business logic updates
}

void Application::Render()
{
    // ---- Main menu bar ----
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("文件"))
        {
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

    // ---- Sub-windows ----
    if (m_showTimeConverter)
        GuiTimeConverter::Render(&m_showTimeConverter);
    if (m_showCoordConverter)
        GuiCoordConverter::Render(&m_showCoordConverter);
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
