#pragma once


#include <windows.h>
#include <string>

/// 请求把当前帧渲染结果（整个客户区）保存为 PNG。
/// 真正的截屏在 Gui::EndFrame 中、ImGui 绘制完成后、Present 之前执行，
/// 因此调用本函数后需让当前帧正常走完 EndFrame 才会写文件。
void RequestCapturePNG(const std::wstring &path);

/// 请求只截取某个矩形区域（屏幕/后台缓冲像素坐标）保存为 PNG，用于「导出单张图」。
/// x,y 为区域左上角，w,h 为宽高；越界会自动裁剪到后台缓冲范围。
void RequestCaptureRegionPNG(const std::wstring &path, int x, int y, int w, int h);

/// 主窗口句柄（供文件对话框设置归属，保证 DPI 与模态正确）。
HWND GetMainWindowHwnd();

/// 文件过滤器条目：显示名 + 通配（如 {L"PNG 文件", L"*.png"}）。
struct GuiFileFilter { const wchar_t *name; const wchar_t *spec; };

/// 现代 Explorer 风格保存对话框（IFileSaveDialog，高 DPI 清晰，替代旧版 comdlg32）。
/// 成功返回 true，out 为所选完整路径（宽字符）。
bool ShowSaveFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters,
                        const wchar_t *defName, const wchar_t *defExt);

/// 现代 Explorer 风格打开对话框（IFileOpenDialog，高 DPI 清晰）。
bool ShowOpenFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters);

/// 现代文件夹选择对话框（IFileOpenDialog + FOS_PICKFOLDERS，高 DPI 清晰）。
/// 成功返回 true，out 为所选文件夹的完整路径。
bool ShowFolderDialog(std::wstring &out);

class Gui {
public:
    void Initialize(const char *title, float width, float height);
    void Shutdown();
    /// Returns false on WM_QUIT.
    /// Sets *frameReady to false when minimized/occluded — caller should skip Update/Render/EndFrame.
    bool BeginFrame(bool* frameReady = nullptr) const;
    void EndFrame() const;
    [[nodiscard]] bool IsInitialized() const { return m_initialized; }

private:
    HWND        m_hwnd{};
    WNDCLASSEXW m_wndClass{};
    bool        m_initialized = false;
};
