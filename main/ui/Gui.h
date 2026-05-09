#pragma once

#ifndef GNSSLAB_GUI_H
#define GNSSLAB_GUI_H

#include <windows.h>

class Gui {
public:
    void Initialize(const char* title, int width, int height);
    void Shutdown();
    /// Returns false on WM_QUIT.
    /// Sets *frameReady to false when minimized/occluded — caller should skip Update/Render/EndFrame.
    bool BeginFrame(bool* frameReady = nullptr);
    void EndFrame();
    [[nodiscard]] bool IsInitialized() const { return m_initialized; }

private:
    HWND        m_hwnd{};
    WNDCLASSEXW m_wndClass{};
    bool        m_initialized = false;
};

#endif
