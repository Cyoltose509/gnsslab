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
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Exit", "Alt+F4"))
                m_ui.Shutdown();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools"))
        {
            ImGui::MenuItem("Time Converter", nullptr, &m_showTimeConverter);
            ImGui::MenuItem("Coord Converter", nullptr, &m_showCoordConverter);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help"))
        {
            ImGui::MenuItem("About", nullptr, nullptr);
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
