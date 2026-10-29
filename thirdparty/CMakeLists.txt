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
file(GLOB IMPLOT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/implot/*.cpp)
add_library(thirdparty_implot STATIC ${IMPLOT_SRCS})
target_include_directories(thirdparty_implot
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_implot PRIVATE thirdparty_imgui)

# ---- ImPlot3D ----
file(GLOB IMPLOT3D_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/implot3d/*.cpp)
add_library(thirdparty_implot3d STATIC ${IMPLOT3D_SRCS})
target_include_directories(thirdparty_implot3d
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot3d
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui ${CMAKE_CURRENT_SOURCE_DIR}/implot
)
target_link_libraries(thirdparty_implot3d PRIVATE thirdparty_imgui thirdparty_implot)

set(BUILD_SHARED_LIBS   OFF CACHE BOOL "" FORCE)
set(ZLIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(zlib)
set_target_properties(zlib PROPERTIES EXCLUDE_FROM_ALL TRUE)

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

