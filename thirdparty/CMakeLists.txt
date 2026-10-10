# thirdparty/CMakeLists.txt
# 管理所有第三方库

# ---- Eigen（header-only）----
add_library(thirdparty_eigen INTERFACE)
target_include_directories(
        thirdparty_eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0
)

# ---- ImGui（header-only + backends）----
# ImGui 主库（imgui.cpp 需要参与编译）
# 用 GLOB 自动收集 imgui 主库源文件；第三方库更新新增 .cpp 后需重新 configure (cmake --fresh) 才会纳入编译
file(GLOB IMGUI_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp)
add_library(thirdparty_imgui STATIC ${IMGUI_SRCS})
target_include_directories(thirdparty_imgui
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)

# ImGui Win32 后端
add_library(thirdparty_imgui_impl_win32 STATIC
        imgui/backends/imgui_impl_win32.cpp
)
target_include_directories(thirdparty_imgui_impl_win32
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_win32 PRIVATE thirdparty_imgui)

# ImGui OpenGL3 后端
add_library(thirdparty_imgui_impl_opengl3 STATIC
        imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(thirdparty_imgui_impl_opengl3
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_opengl3 PRIVATE thirdparty_imgui)

# ImGui DirectX11 后端
add_library(thirdparty_imgui_impl_dx11 STATIC
        imgui/backends/imgui_impl_dx11.cpp
)
target_include_directories(thirdparty_imgui_impl_dx11
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_dx11 PRIVATE thirdparty_imgui d3d11 dxgi)

# ---- ImPlot ----
# 用 GLOB 自动收集 implot 主库源文件；更新新增 .cpp 后需 cmake --fresh
file(GLOB IMPLOT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/implot/*.cpp)
add_library(thirdparty_implot STATIC ${IMPLOT_SRCS})
target_include_directories(thirdparty_implot
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_implot PRIVATE thirdparty_imgui)

# ---- ImPlot3D ----
# 用 GLOB 自动收集 implot3d 主库源文件；更新新增 .cpp 后需 cmake --fresh
file(GLOB IMPLOT3D_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/implot3d/*.cpp)
add_library(thirdparty_implot3d STATIC ${IMPLOT3D_SRCS})
target_include_directories(thirdparty_implot3d
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot3d
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui ${CMAKE_CURRENT_SOURCE_DIR}/implot
)
target_link_libraries(thirdparty_implot3d PRIVATE thirdparty_imgui thirdparty_implot)

# ---- zlib（git submodule，静态库）----
# 镜像产物是 .gz，下载后由 DownloadStore 解压，故需要 zlib（只链 zlibstatic）。
# zlib 1.3.1 的 CMake 无"只编静态"开关：它会同时建出共享库 zlib，这里 EXCLUDE_FROM_ALL
# 免得 bin/ 里多一个 zlib.dll。
set(BUILD_SHARED_LIBS   OFF CACHE BOOL "" FORCE)
set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(zlib)
set_target_properties(zlib PROPERTIES EXCLUDE_FROM_ALL TRUE)

# ---- libcurl（git submodule，静态库）----
# 只留静态库 + FTP，关掉全部可选外部依赖（SSH/HTTP2/IDN/PSL/压缩编码）。
# 原因：这些 option 多数默认 ON，且 curl 内部用的是 find_package(... REQUIRED)：
#   · CURL_USE_LIBPSL 默认 ON → find_package(Libpsl MODULE REQUIRED)，机器没装就直接配置失败
#     （实测 CLion 报 "Could NOT find Libpsl"，配置中断）。
#   · CURL_ZSTD=AUTO 会经 CMake Config 命中机器上的第三方（实测命中 D:/Anaconda 的 zstd），
#     静态库里混进外部依赖，不可接受。
# 注：curl_dependency_option 建的是 AUTO/ON/OFF 的 **STRING** 缓存项，覆盖须用 CACHE STRING + FORCE。
# TLS：本 curl 版本在 Windows 上 CURL_USE_SCHANNEL 默认 OFF ⇒ 加密后端为空（只有明文 ftp://）。
# 实测 igs.gnsswhu.cn 没有 HTTPS 监听（TLS 握手被拒），当前用不到；若将来要 ftps/https，
# 加 set(CURL_USE_SCHANNEL ON CACHE BOOL "" FORCE) 即可（走系统库，无需第三方依赖）。
set(BUILD_CURL_EXE    OFF CACHE BOOL   "" FORCE)
set(BUILD_TESTING     OFF CACHE BOOL   "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL   "" FORCE)
set(CURL_USE_LIBPSL   OFF CACHE BOOL   "" FORCE)   # 默认 ON 且 REQUIRED ← 配置拦路虎
set(CURL_USE_LIBSSH2  OFF CACHE BOOL   "" FORCE)
set(USE_NGHTTP2       OFF CACHE BOOL   "" FORCE)
set(USE_LIBIDN2       OFF CACHE BOOL   "" FORCE)
set(CURL_ZLIB         OFF CACHE STRING "" FORCE)   # 不借系统的 zlib：.gz 由我们直接链 zlibstatic 解
set(CURL_BROTLI       OFF CACHE STRING "" FORCE)
set(CURL_ZSTD         OFF CACHE STRING "" FORCE)
add_subdirectory(curl)

