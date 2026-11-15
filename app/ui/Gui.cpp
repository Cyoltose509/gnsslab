#include "ui/Gui.h"

#include "StringUtils.h"
#include "util/Clock.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include "implot.h"
#include "implot3d.h"
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <d3d11.h>
#include <wincodec.h>
#include <shobjidl.h>
#include <tchar.h>


static ID3D11Device *g_pd3dDevice = nullptr;
static ID3D11DeviceContext *g_pd3dDeviceContext = nullptr;
static IDXGISwapChain *g_pSwapChain = nullptr;
static ID3D11RenderTargetView *g_mainRenderTargetView = nullptr;
static bool g_SwapChainOccluded = false;
static UINT g_ResizeWidth = 0;
static UINT g_ResizeHeight = 0;
static HWND g_mainHwnd = nullptr;


static bool g_captureReq = false;
static bool g_captureRegion = false;
static int g_capX = 0, g_capY = 0, g_capW = 0, g_capH = 0;
static std::wstring g_capturePath;
static bool SavePixelsToPNG(const wchar_t *path, int x0, int y0, int w, int h,
                            UINT srcPitch, const BYTE *srcBGRA);
static bool CaptureBackBufferToPNG(const wchar_t *path);

// ========== Helper functions ==========
static void CreateRenderTarget();

static bool CreateDeviceD3D(HWND hWnd) { //NOLINT
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    constexpr UINT createDeviceFlags = 0;
    D3D_FEATURE_LEVEL featureLevel;
    constexpr D3D_FEATURE_LEVEL featureLevelArray[2] = {
        D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0
    };

    HRESULT res = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
        createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION,
        &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);

    if (res == DXGI_ERROR_UNSUPPORTED) {
        res = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr,
            createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION,
            &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    }
    if (res != S_OK)
        return false;

    CreateRenderTarget();
    return true;
}

static void CreateRenderTarget() {
    ID3D11Texture2D *pBackBuffer = nullptr;
    if (FAILED(g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer))) || !pBackBuffer) return;
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
    pBackBuffer->Release();
}

static void CleanupRenderTarget() {
    if (g_mainRenderTargetView) {
        g_mainRenderTargetView->Release();
        g_mainRenderTargetView = nullptr;
    }
}

static void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) {
        g_pSwapChain->Release();
        g_pSwapChain = nullptr;
    }
    if (g_pd3dDeviceContext) {
        g_pd3dDeviceContext->Release();
        g_pd3dDeviceContext = nullptr;
    }
    if (g_pd3dDevice) {
        g_pd3dDevice->Release();
        g_pd3dDevice = nullptr;
    }
}

// ========== WndProc ==========
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) //NOLINT
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
        case WM_SIZE:
            if (wParam == SIZE_MINIMIZED)
                return 0;
            g_ResizeWidth = static_cast<UINT>(LOWORD(lParam));
            g_ResizeHeight = static_cast<UINT>(HIWORD(lParam));
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU)
                return 0;
            break;
        case WM_CLOSE:
            // 只申请退出：窗口留到 Gui::Shutdown() 销毁，退出前那次读尺寸才拿得到真值。
            PostQuitMessage(0);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default: ;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

// ========== Gui implementation ==========

void Gui::Initialize(const char *title, const float width, const float height, const bool maximized) {
    if (m_initialized) return;

    const HRESULT comHr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    m_comInit = SUCCEEDED(comHr); // RPC_E_CHANGED_MODE 时不能配对 CoUninitialize

    ImGui_ImplWin32_EnableDpiAwareness();
    const float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(
        MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));
    m_mainScale = main_scale;

    // Window class
    m_wndClass = {
        sizeof(m_wndClass), CS_CLASSDC, WndProc, 0L, 0L,
        GetModuleHandle(nullptr), 
        LoadIcon(GetModuleHandle(nullptr), MAKEINTRESOURCE(101)), // Large icon
        nullptr, nullptr, nullptr,
        L"GnssLabClass", 
        LoadIcon(GetModuleHandle(nullptr), MAKEINTRESOURCE(101))  // Small icon
    };
    RegisterClassExW(&m_wndClass);
    const int w = static_cast<int>(width * main_scale);
    const int h = static_cast<int>(height * main_scale);

    const std::wstring wTitle = Str::UTF8ToWide(title);

    m_hwnd = ::CreateWindowW(
        m_wndClass.lpszClassName,
        wTitle.c_str(),
        WS_OVERLAPPEDWINDOW, 100, 100, w, h,
        nullptr, nullptr, m_wndClass.hInstance, nullptr);
    g_mainHwnd = m_hwnd;   // 供文件对话框归属使用

    // DX11
    if (!CreateDeviceD3D(m_hwnd)) {
        CleanupDeviceD3D();
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
        g_mainHwnd = nullptr;
        UnregisterClassW(m_wndClass.lpszClassName, m_wndClass.hInstance);
        return;
    }

    // 上次退出时是最大化就直接最大化显示；否则按 ini 里记的尺寸普通显示。
    ShowWindow(m_hwnd, maximized ? SW_SHOWMAXIMIZED : SW_SHOWDEFAULT);
    UpdateWindow(m_hwnd);

    // ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void) io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);
    style.FontScaleDpi = main_scale;

    // 中日韩字体：1.92 的图集按需加载字形，不必预设 GlyphRanges。主字体出中英，
    // 日文假名与韩文谚文由合并进来的系统字体补；按需查找按合并顺序取第一个有该字形的源。
    const float fontSize = 18.0f * main_scale;
    ImFont *font = nullptr;
    for (const char *path: {R"(C:\Windows\Fonts\msyh.ttc)", R"(C:\Windows\Fonts\simhei.ttf)",
                            R"(C:\Windows\Fonts\simsun.ttc)"}) {
        if (!std::filesystem::exists(path)) continue;
        font = io.Fonts->AddFontFromFileTTF(path, fontSize);
        if (font) break;
    }
    if (!font) {
        io.Fonts->AddFontDefault(); // 一个中文字体都没有时至少还能出英文
    } else {
        ImFontConfig merge;
        merge.MergeMode = true;
        for (const char *path: {R"(C:\Windows\Fonts\YuGothM.ttc)", R"(C:\Windows\Fonts\meiryo.ttc)",
                                R"(C:\Windows\Fonts\msgothic.ttc)",
                                R"(C:\Windows\Fonts\malgun.ttf)"}) {
            if (std::filesystem::exists(path)) io.Fonts->AddFontFromFileTTF(path, fontSize, &merge);
        }
    }

    // Backends
    ImGui_ImplWin32_Init(m_hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    m_initialized = true;
}

/// 读「还原态」尺寸并折算回逻辑像素；窗口已销毁或还没建好时返回 false。
static bool queryRestoredSize(const HWND hwnd, const float scale, int &w, int &h) {
    w = 0;
    h = 0;
    if (!hwnd || !IsWindow(hwnd)) return false;
    WINDOWPLACEMENT wp{};
    wp.length = sizeof(wp);
    if (!GetWindowPlacement(hwnd, &wp)) return false;
    const RECT &rc = wp.rcNormalPosition;
    w = static_cast<int>(static_cast<double>(rc.right - rc.left) / scale + 0.5);
    h = static_cast<int>(static_cast<double>(rc.bottom - rc.top) / scale + 0.5);
    return w > 0 && h > 0;
}

/// 是否最大化。最小化的窗口 IsZoomed 恒为 false，此时要看 WINDOWPLACEMENT：
/// 「从最大化状态最小化」的窗口靠 showCmd / WPF_RESTORETOMAXIMIZED 记住还原意图，
/// 否则最小化后退出会把最大化状态存丢。
static bool queryMaximized(const HWND hwnd, bool &out) {
    if (!hwnd || !IsWindow(hwnd)) return false;
    if (IsZoomed(hwnd)) {
        out = true;
        return true;
    }
    WINDOWPLACEMENT wp{};
    wp.length = sizeof(wp);
    if (!GetWindowPlacement(hwnd, &wp)) return false;
    out = wp.showCmd == SW_SHOWMAXIMIZED || (wp.flags & WPF_RESTORETOMAXIMIZED) != 0;
    return true;
}

void Gui::captureWindowState() {
    int w = 0, h = 0;
    if (queryRestoredSize(m_hwnd, m_mainScale, w, h)) {
        m_restoredW = w;
        m_restoredH = h;
    }
    if (bool maximized = false; queryMaximized(m_hwnd, maximized)) m_wasMaximized = maximized;
}

void Gui::GetRestoredSize(int &w, int &h) const {
    if (queryRestoredSize(m_hwnd, m_mainScale, w, h)) return;
    w = m_restoredW;
    h = m_restoredH;
}

bool Gui::IsMaximized() const {
    bool maximized = false;
    if (queryMaximized(m_hwnd, maximized)) return maximized;
    return m_wasMaximized;
}

void Gui::Shutdown() {
    if (!m_initialized) return;

    // 销毁之前先留一份：窗口一旦没了就再也读不到还原尺寸与最大化状态。
    captureWindowState();

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    DestroyWindow(m_hwnd);
    UnregisterClassW(m_wndClass.lpszClassName, m_wndClass.hInstance);

    // 不置空的话，退出后再问尺寸会拿一个已销毁的句柄去查。
    m_hwnd = nullptr;
    g_mainHwnd = nullptr;

    if (m_comInit) {
        CoUninitialize();
        m_comInit = false;
    }

    m_initialized = false;
}

bool Gui::BeginFrame(bool *frameReady) const {
    if (frameReady) *frameReady = true;
    if (!m_initialized) return false;

    MSG msg;
    while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
        TranslateMessage(&msg);
        ::DispatchMessage(&msg);
        if (msg.message == WM_QUIT)
            return false;
    }

    // Skip frame when minimized (DisplaySize would be 0,0 — causes assertions)
    if (IsIconic(m_hwnd)) {
        if (frameReady) *frameReady = false;
        Sleep(10);
        return true;
    }

    // Occlusion
    if (g_SwapChainOccluded &&
        g_pSwapChain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) {
        if (frameReady) *frameReady = false;
        Sleep(10);
        return true;
    }
    g_SwapChainOccluded = false;

    // Resize
    if (g_ResizeWidth != 0 && g_ResizeHeight != 0) {
        CleanupRenderTarget();
        g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);

        ImGuiIO &io = ImGui::GetIO();
        io.DisplaySize = ImVec2(static_cast<float>(g_ResizeWidth), static_cast<float>(g_ResizeHeight));

        g_ResizeWidth = g_ResizeHeight = 0;
        CreateRenderTarget();
    }

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    return true;
}

void Gui::EndFrame() const {
    if (!m_initialized || !g_mainRenderTargetView) return; // 尺寸重建失败时这一帧没得画

    ImGui::Render();

    constexpr float clear_color[4] = {0.10f, 0.10f, 0.10f, 1.00f};
    g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
    g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    // 在当前帧绘制完成后、Present 之前截屏（此时后台缓冲含本帧完整内容）
    if (g_captureReq) {
        CaptureBackBufferToPNG(g_capturePath.c_str());
        g_captureReq = false;
        g_captureRegion = false;
    }

    const HRESULT hr = g_pSwapChain->Present(1, 0);
    g_SwapChainOccluded = hr == DXGI_STATUS_OCCLUDED;
}

static bool SavePixelsToPNG(const wchar_t *path, int x0, int y0, int w, int h,
                            UINT srcPitch, const BYTE *srcBGRA) {
    if (w <= 0 || h <= 0) return false;
    IWICImagingFactory *factory = nullptr;
    IWICStream *stream = nullptr;
    IWICBitmapEncoder *encoder = nullptr;
    IWICBitmapFrameEncode *frame = nullptr;
    const auto bail = [&] {
        if (frame) frame->Release();
        if (encoder) encoder->Release();
        if (stream) stream->Release();
        if (factory) factory->Release();
        return false;
    };

    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                IID_PPV_ARGS(&factory))))
        return bail();
    if (FAILED(factory->CreateStream(&stream)) ||
        FAILED(stream->InitializeFromFilename(path, GENERIC_WRITE)) ||
        FAILED(factory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &encoder)) ||
        FAILED(encoder->Initialize(stream, WICBitmapEncoderNoCache)) ||
        FAILED(encoder->CreateNewFrame(&frame, nullptr)))
        return bail();
    frame->Initialize(nullptr);
    if (FAILED(frame->SetSize(static_cast<UINT>(w), static_cast<UINT>(h)))) return bail();
    WICPixelFormatGUID fmt = GUID_WICPixelFormat32bppRGBA;
    if (FAILED(frame->SetPixelFormat(&fmt))) return bail();

    std::vector<BYTE> rgba(static_cast<std::size_t>(w) * h * 4);
    for (int y = 0; y < h; y++) {
        const BYTE *s = srcBGRA + static_cast<std::size_t>(y0 + y) * srcPitch + static_cast<std::size_t>(x0) * 4;
        BYTE *d = rgba.data() + static_cast<std::size_t>(y) * (w * 4);
        for (int x = 0; x < w; x++) {
            d[x * 4 + 0] = s[x * 4 + 2];
            d[x * 4 + 1] = s[x * 4 + 1];
            d[x * 4 + 2] = s[x * 4 + 0];
            d[x * 4 + 3] = s[x * 4 + 3];
        }
    }
    if (FAILED(frame->WritePixels(static_cast<UINT>(h), static_cast<UINT>(w) * 4,
                                  static_cast<UINT>(rgba.size()), rgba.data())))
        return bail();
    frame->Commit();
    encoder->Commit();
    bail(); // 只做释放，写成功与否已由上面的 HRESULT 判过
    return true;
}

static bool CaptureBackBufferToPNG(const wchar_t *path) {
    if (!g_pSwapChain || !g_pd3dDevice || !g_pd3dDeviceContext) return false;
    ID3D11Texture2D *pBackBuffer = nullptr;
    HRESULT hr = g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    if (FAILED(hr)) return false;
    D3D11_TEXTURE2D_DESC desc;
    pBackBuffer->GetDesc(&desc);
    D3D11_TEXTURE2D_DESC sdesc = desc;
    sdesc.Usage = D3D11_USAGE_STAGING;
    sdesc.BindFlags = 0;
    sdesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    sdesc.MiscFlags = 0;
    ID3D11Texture2D *staging = nullptr;
    hr = g_pd3dDevice->CreateTexture2D(&sdesc, nullptr, &staging);
    if (FAILED(hr)) { pBackBuffer->Release(); return false; }
    g_pd3dDeviceContext->CopyResource(staging, pBackBuffer);
    // 等待 GPU 完成所有渲染，避免截到半帧/空图；设备丢失时不能无限自旋
    {   ID3D11Query *q = nullptr;
        D3D11_QUERY_DESC qd{D3D11_QUERY_EVENT, 0};
        if (SUCCEEDED(g_pd3dDevice->CreateQuery(&qd, &q))) {
            g_pd3dDeviceContext->End(q);
            const auto deadline = Clock::after(Clock::seconds(2));
            while (g_pd3dDeviceContext->GetData(q, nullptr, 0, 0) == S_FALSE) {
                if (Clock::due(deadline)) break;
                Clock::sleep(Clock::microseconds(200));
            }
            q->Release();
        }
    }
    D3D11_MAPPED_SUBRESOURCE mapped;
    hr = g_pd3dDeviceContext->Map(staging, 0, D3D11_MAP_READ, 0, &mapped);
    bool ok = false;
    if (SUCCEEDED(hr)) {
        int x0 = 0, y0 = 0, W = (int)desc.Width, H = (int)desc.Height;
        if (g_captureRegion) {
            x0 = g_capX; y0 = g_capY; W = g_capW; H = g_capH;
            // 裁剪到后台缓冲范围内，越界自动收缩
            if (x0 < 0) { W += x0; x0 = 0; }
            if (y0 < 0) { H += y0; y0 = 0; }
            if (x0 + W > (int)desc.Width)  W = (int)desc.Width  - x0;
            if (y0 + H > (int)desc.Height) H = (int)desc.Height - y0;
            if (W <= 0 || H <= 0) { x0 = 0; y0 = 0; W = (int)desc.Width; H = (int)desc.Height; }
        }
        ok = SavePixelsToPNG(path, x0, y0, W, H, mapped.RowPitch,
                             static_cast<const BYTE *>(mapped.pData));
        g_pd3dDeviceContext->Unmap(staging, 0);
    }
    staging->Release();
    pBackBuffer->Release();
    return ok;
}

void RequestCaptureRegionPNG(const std::wstring &path, int x, int y, int w, int h) {
    g_capturePath = path;
    g_capX = x; g_capY = y; g_capW = w; g_capH = h;
    g_captureRegion = true;
    g_captureReq = true;
}

HWND GetMainWindowHwnd() { return g_mainHwnd; }

// 把 GuiFileFilter[] 渲染成 COMDLG_FILTERSPEC[]。names/pats 由调用方持有，
// 其 .c_str() 在 SetFileTypes 调用期间必须保持有效。
static void fillFilterSpecs(std::vector<COMDLG_FILTERSPEC> &specs,
                            std::vector<std::wstring> &names,
                            std::vector<std::wstring> &pats,
                            const GuiFileFilter *filters, const int nFilters) {
    specs.reserve(nFilters);
    names.reserve(nFilters);
    pats.reserve(nFilters);
    for (int i = 0; i < nFilters; ++i) {
        const GuiFileFilter &f = filters[i];
        std::wstring pat;
        for (size_t k = 0; k < f.exts.size(); ++k) {
            if (k) pat += L';';
            pat += L"*.";
            pat += f.exts[k];
        }
        names.push_back(f.name + L" (" + pat + L")");
        pats.push_back(std::move(pat));
        specs.push_back({names.back().c_str(), pats.back().c_str()});
    }
}

static bool ShowFileDialogImpl(const bool save, std::wstring &out,
                               const GuiFileFilter *filters, const int nFilters,
                               const wchar_t *defName, const wchar_t *defExt) {
    IFileDialog *pfd = nullptr;
    const CLSID clsid = save ? CLSID_FileSaveDialog : CLSID_FileOpenDialog;
    const IID   iid   = save ? IID_IFileSaveDialog  : IID_IFileOpenDialog;
    HRESULT hr = CoCreateInstance(clsid, nullptr, CLSCTX_INPROC_SERVER, iid,
                                  reinterpret_cast<void **>(&pfd));
    if (FAILED(hr) || !pfd) return false;

    std::vector<COMDLG_FILTERSPEC> specs;
    std::vector<std::wstring> names, pats;
    fillFilterSpecs(specs, names, pats, filters, nFilters);
    if (!specs.empty())
        pfd->SetFileTypes(static_cast<UINT>(specs.size()), specs.data());
    if (defExt) pfd->SetDefaultExtension(defExt);
    if (defName && *defName) pfd->SetFileName(defName);

    hr = pfd->Show(g_mainHwnd);
    if (FAILED(hr)) { pfd->Release(); return false; }   // 用户取消或出错

    IShellItem *item = nullptr;
    hr = pfd->GetResult(&item);
    bool ok = false;
    if (SUCCEEDED(hr) && item) {
        PWSTR psz = nullptr;
        if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz)) && psz) {
            out.assign(psz);
            CoTaskMemFree(psz);
            ok = true;
        }
        item->Release();
    }
    pfd->Release();
    return ok;
}

bool ShowSaveFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters,
                        const wchar_t *defName, const wchar_t *defExt) {
    return ShowFileDialogImpl(true, out, filters, nFilters, defName, defExt);
}

bool ShowOpenFileDialog(std::wstring &out, const GuiFileFilter *filters, const int nFilters) {
    return ShowFileDialogImpl(false, out, filters, nFilters, nullptr, nullptr);
}

bool ShowOpenFilesDialog(std::vector<std::wstring> &out, const GuiFileFilter *filters, const int nFilters) {
    IFileOpenDialog *pfd = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER,
                                 IID_IFileOpenDialog, reinterpret_cast<void **>(&pfd));
    if (FAILED(hr) || !pfd) return false;

    std::vector<COMDLG_FILTERSPEC> specs;
    std::vector<std::wstring> names, pats;
    fillFilterSpecs(specs, names, pats, filters, nFilters);
    if (!specs.empty())
        pfd->SetFileTypes((UINT)specs.size(), specs.data());

    FILEOPENDIALOGOPTIONS opts = 0;
    if (SUCCEEDED(pfd->GetOptions(&opts)))
        pfd->SetOptions(opts | FOS_ALLOWMULTISELECT | FOS_FORCEFILESYSTEM);

    hr = pfd->Show(g_mainHwnd);
    if (FAILED(hr)) { pfd->Release(); return false; }   // 用户取消或出错

    IShellItemArray *items = nullptr;
    if (SUCCEEDED(pfd->GetResults(&items)) && items) {
        DWORD count = 0;
        items->GetCount(&count);
        for (DWORD i = 0; i < count; i++) {
            IShellItem *item = nullptr;
            if (SUCCEEDED(items->GetItemAt(i, &item)) && item) {
                PWSTR psz = nullptr;
                if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz)) && psz) {
                    out.emplace_back(psz);
                    CoTaskMemFree(psz);
                }
                item->Release();
            }
        }
        items->Release();
    }
    pfd->Release();
    return !out.empty();
}

bool ShowFolderDialog(std::wstring &out) {
    IFileDialog *pfd = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_IFileOpenDialog, reinterpret_cast<void **>(&pfd));
    if (FAILED(hr) || !pfd) return false;
    FILEOPENDIALOGOPTIONS opts = 0;
    if (SUCCEEDED(pfd->GetOptions(&opts)))
        pfd->SetOptions(opts | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
    hr = pfd->Show(g_mainHwnd);
    bool ok = false;
    if (SUCCEEDED(hr)) {
        IShellItem *item = nullptr;
        if (SUCCEEDED(pfd->GetResult(&item)) && item) {
            PWSTR psz = nullptr;
            if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz)) && psz) {
                out.assign(psz); CoTaskMemFree(psz); ok = true;
            }
            item->Release();
        }
    }
    pfd->Release();
    return ok;
}
