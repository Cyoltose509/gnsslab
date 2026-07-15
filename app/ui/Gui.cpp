#include "ui/Gui.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include "implot.h"
#include "implot3d.h"
#include <vector>
#include <string>
#include <d3d11.h>
#include <wincodec.h>
#include <shobjidl.h>   // IFileOpenDialog / IFileSaveDialog（现代 DPI 清晰对话框）
#include <tchar.h>

// ========== DX11 file-scope globals ==========
static ID3D11Device *g_pd3dDevice = nullptr;
static ID3D11DeviceContext *g_pd3dDeviceContext = nullptr;
static IDXGISwapChain *g_pSwapChain = nullptr;
static ID3D11RenderTargetView *g_mainRenderTargetView = nullptr;
static bool g_SwapChainOccluded = false;
static UINT g_ResizeWidth = 0;
static UINT g_ResizeHeight = 0;
static HWND g_mainHwnd = nullptr;   // 主窗口句柄（对话框归属 + DPI 上下文）

// ========== 帧截屏（PNG 导出）==========
static bool g_captureReq = false;
static bool g_captureRegion = false;          // true=只截子区域
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
    ID3D11Texture2D *pBackBuffer;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
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
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default: ;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

// ========== Gui implementation ==========

void Gui::Initialize(const char *title, const float width, const float height) {
    if (m_initialized) return;

    // COM：GUI 主线程用单线程套间(STA) —— IFileDialog(现代文件对话框) 要求 STA；
    // WIC(PNG 编码) 在 STA/MTA 均可。
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

    ImGui_ImplWin32_EnableDpiAwareness();
    const float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(
        MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));

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

    // Convert char* title to WCHAR* for CreateWindowW
    int len = MultiByteToWideChar(CP_UTF8, 0, title, -1, nullptr, 0);
    std::vector<wchar_t> wTitle(len);
    MultiByteToWideChar(CP_UTF8, 0, title, -1, wTitle.data(), len);

    m_hwnd = ::CreateWindowW(
        m_wndClass.lpszClassName,
        wTitle.data(),
        WS_OVERLAPPEDWINDOW, 100, 100, w, h,
        nullptr, nullptr, m_wndClass.hInstance, nullptr);
    g_mainHwnd = m_hwnd;   // 供文件对话框归属使用

    // DX11
    if (!CreateDeviceD3D(m_hwnd)) {
        CleanupDeviceD3D();
        UnregisterClassW(m_wndClass.lpszClassName, m_wndClass.hInstance);
        return;
    }

    ShowWindow(m_hwnd, SW_SHOWDEFAULT);
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

    // Chinese font
    const float fontSize = 18.0f * main_scale;
    const ImFont *font = io.Fonts->AddFontFromFileTTF(
        R"(C:\Windows\Fonts\msyh.ttc)", fontSize, nullptr,
        io.Fonts->GetGlyphRangesChineseFull());
    IM_ASSERT(font != nullptr);

    // Backends
    ImGui_ImplWin32_Init(m_hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    m_initialized = true;
}

void Gui::Shutdown() {
    if (!m_initialized) return;

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    DestroyWindow(m_hwnd);
    UnregisterClassW(m_wndClass.lpszClassName, m_wndClass.hInstance);

    CoUninitialize();

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
    if (!m_initialized) return;

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

// ========== PNG 导出：DX11 后台缓冲读回 + WIC 编码 ==========
// 从 srcBGRA(整帧) 中裁剪 [x0,y0,w,h] 区域，转 RGBA 后写 PNG。
static bool SavePixelsToPNG(const wchar_t *path, int x0, int y0, int w, int h,
                            UINT srcPitch, const BYTE *srcBGRA) {
    if (w <= 0 || h <= 0) return false;
    IWICImagingFactory *factory = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory));
    if (FAILED(hr)) return false;
    IWICStream *stream = nullptr;
    factory->CreateStream(&stream);
    stream->InitializeFromFilename(path, GENERIC_WRITE);
    IWICBitmapEncoder *encoder = nullptr;
    factory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &encoder);
    encoder->Initialize(stream, WICBitmapEncoderNoCache);
    IWICBitmapFrameEncode *frame = nullptr;
    encoder->CreateNewFrame(&frame, nullptr);
    frame->Initialize(nullptr);
    frame->SetSize((UINT)w, (UINT)h);
    WICPixelFormatGUID fmt = GUID_WICPixelFormat32bppRGBA;
    frame->SetPixelFormat(&fmt);
    // 后台缓冲为 BGRA(内存序 B,G,R,A)，逐行转 RGBA 写入（D3D11 纹理行 0 = 屏幕顶部，无需翻转）
    std::vector<BYTE> rgba((size_t)w * h * 4);
    for (int y = 0; y < h; y++) {
        const BYTE *s = srcBGRA + (size_t)(y0 + y) * srcPitch + (size_t)x0 * 4;
        BYTE *d = rgba.data() + (size_t)y * (w * 4);
        for (int x = 0; x < w; x++) {
            d[x * 4 + 0] = s[x * 4 + 2];
            d[x * 4 + 1] = s[x * 4 + 1];
            d[x * 4 + 2] = s[x * 4 + 0];
            d[x * 4 + 3] = s[x * 4 + 3];
        }
    }
    frame->WritePixels((UINT)h, (UINT)w * 4, (UINT)rgba.size(), rgba.data());
    frame->Commit();
    encoder->Commit();
    frame->Release(); encoder->Release(); stream->Release(); factory->Release();
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
    // 等待 GPU 完成所有渲染，避免截到半帧/空图
    {   ID3D11Query *q = nullptr;
        D3D11_QUERY_DESC qd{D3D11_QUERY_EVENT, 0};
        if (SUCCEEDED(g_pd3dDevice->CreateQuery(&qd, &q))) {
            g_pd3dDeviceContext->End(q);
            while (g_pd3dDeviceContext->GetData(q, nullptr, 0, 0) == S_FALSE) {}
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

void RequestCapturePNG(const std::wstring &path) {
    g_capturePath = path;
    g_captureRegion = false;
    g_captureReq = true;
}

void RequestCaptureRegionPNG(const std::wstring &path, int x, int y, int w, int h) {
    g_capturePath = path;
    g_capX = x; g_capY = y; g_capW = w; g_capH = h;
    g_captureRegion = true;
    g_captureReq = true;
}

HWND GetMainWindowHwnd() { return g_mainHwnd; }

// ========== 现代 Explorer 风格文件对话框（IFileDialog，高 DPI 清晰）==========
// 旧版 comdlg32 的 GetOpenFileName/GetSaveFileName 在进程 DPI 感知开启后会被系统
// 位图拉伸而发虚；IFileDialog(Common Item Dialog) 为 Vista+ 原生 DPI 清晰对话框。
static bool ShowFileDialogImpl(bool save, std::wstring &out,
                               const GuiFileFilter *filters, int nFilters,
                               const wchar_t *defName, const wchar_t *defExt) {
    IFileDialog *pfd = nullptr;
    const CLSID clsid = save ? CLSID_FileSaveDialog : CLSID_FileOpenDialog;
    const IID   iid   = save ? IID_IFileSaveDialog  : IID_IFileOpenDialog;
    HRESULT hr = CoCreateInstance(clsid, nullptr, CLSCTX_INPROC_SERVER, iid,
                                  reinterpret_cast<void **>(&pfd));
    if (FAILED(hr) || !pfd) return false;

    std::vector<COMDLG_FILTERSPEC> specs;
    specs.reserve(nFilters);
    for (int i = 0; i < nFilters; i++)
        specs.push_back({filters[i].name, filters[i].spec});
    if (!specs.empty())
        pfd->SetFileTypes((UINT)specs.size(), specs.data());
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

bool ShowOpenFileDialog(std::wstring &out, const GuiFileFilter *filters, int nFilters) {
    return ShowFileDialogImpl(false, out, filters, nFilters, nullptr, nullptr);
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
