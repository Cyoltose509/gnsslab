#pragma once


#include <windows.h>
#include <string>
#include <vector>


void RequestCapturePNG(const std::wstring &path);


void RequestCaptureRegionPNG(const std::wstring &path, int x, int y, int w, int h);


HWND GetMainWindowHwnd();


struct GuiFileFilter {
    const wchar_t *name;
    const wchar_t *spec;
};


bool ShowSaveFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters,
                        const wchar_t *defName, const wchar_t *defExt);


bool ShowOpenFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters);


bool ShowOpenFilesDialog(std::vector<std::wstring> &out, const GuiFileFilter *filters, int nFilters);


bool ShowFolderDialog(std::wstring &out);

class Gui {
public:
    void Initialize(const char *title, float width, float height, bool maximized);

    void Shutdown();

    bool BeginFrame(bool *frameReady = nullptr) const;

    void EndFrame() const;

    [[nodiscard]] bool IsInitialized() const { return m_initialized; }

    void GetRestoredSize(int &w, int &h) const;

    [[nodiscard]] bool IsMaximized() const;

private:
    void captureWindowState();

    HWND m_hwnd{};
    WNDCLASSEXW m_wndClass{};
    bool m_initialized = false;

    float m_mainScale = 1.0f;

    int m_restoredW = 0;
    int m_restoredH = 0;
    bool m_wasMaximized = false;
};
