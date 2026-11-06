#pragma once


#include <windows.h>
#include <string>
#include <vector>


void RequestCapturePNG(const std::wstring &path);


void RequestCaptureRegionPNG(const std::wstring &path, int x, int y, int w, int h);


HWND GetMainWindowHwnd();


// 文件对话框过滤器：只存语义数据（显示名 + 后缀数组），不含点。
// 形如 "RINEX 观测" + {"??O","rnx","obs"}，对话框内部自动生成
//   name  = L"RINEX 观测 (*.??O;*.rnx;*.obs)"
//   spec  = L"*.??O;*.rnx;*.obs"
struct GuiFileFilter {
    std::wstring name;                 // 显示名，如 L"RINEX 观测"（不含后缀括号）
    std::vector<std::wstring> exts;    // 后缀数组，不含点
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
