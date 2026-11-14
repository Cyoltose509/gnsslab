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
# 必须显式选 TLS 后端：Windows 上 curl 的 CMake 不会自己挑，三者全 OFF 会编出一个
# 没有 SSL 的 libcurl，任何 https:// 都直接报「Protocol "https" not supported」。
# Schannel 用系统证书库，零外部依赖（crypt32/secur32 由 curl 目标自带）。
set(CURL_USE_SCHANNEL ON  CACHE BOOL   "" FORCE)
set(CURL_ZLIB         OFF CACHE STRING "" FORCE)   # gzip 由 libarchive 解，curl 只搬原始字节
set(CURL_BROTLI       OFF CACHE STRING "" FORCE)
set(CURL_ZSTD         OFF CACHE STRING "" FORCE)
add_subdirectory(curl)

# ---- libarchive（解包：tar / zip / gzip / compress）----
# 只打开实际用得到的格式：IGS 是 .gz，UCAR 是 .tar.gz，COSMIC 是 .zip，IDS 的 DORIS 轨道是
# .Z（LZW）。bz2/lzma/zstd/lz4 一个都不碰，省得再为它们拉两三套压缩库进来。
set(ENABLE_ZLIB         ON  CACHE BOOL "" FORCE)
set(ENABLE_BZip2        OFF CACHE BOOL "" FORCE)
set(ENABLE_LZMA         OFF CACHE BOOL "" FORCE)
set(ENABLE_ZSTD         OFF CACHE BOOL "" FORCE)
set(ENABLE_LZ4          OFF CACHE BOOL "" FORCE)
set(ENABLE_LZO          OFF CACHE BOOL "" FORCE)
set(ENABLE_LIBB2        OFF CACHE BOOL "" FORCE)
set(ENABLE_MD           OFF CACHE BOOL "" FORCE)
set(ENABLE_OPENSSL      OFF CACHE BOOL "" FORCE)
set(ENABLE_MBEDTLS      OFF CACHE BOOL "" FORCE)
set(ENABLE_NETTLE       OFF CACHE BOOL "" FORCE)
set(ENABLE_CNG          OFF CACHE BOOL "" FORCE)
set(ENABLE_LIBXML2      OFF CACHE BOOL "" FORCE)
set(ENABLE_BSDXML       OFF CACHE BOOL "" FORCE)
set(ENABLE_EXPAT        OFF CACHE BOOL "" FORCE)
set(ENABLE_WIN32_XMLLITE OFF CACHE BOOL "" FORCE)
set(ENABLE_PCREPOSIX    OFF CACHE BOOL "" FORCE)
set(ENABLE_PCRE2POSIX   OFF CACHE BOOL "" FORCE)
set(ENABLE_XATTR        OFF CACHE BOOL "" FORCE)
set(ENABLE_ACL          OFF CACHE BOOL "" FORCE)
set(ENABLE_ICONV        OFF CACHE BOOL "" FORCE)
set(ENABLE_TAR          OFF CACHE BOOL "" FORCE)
set(ENABLE_CPIO         OFF CACHE BOOL "" FORCE)
set(ENABLE_CAT          OFF CACHE BOOL "" FORCE)
set(ENABLE_UNZIP        OFF CACHE BOOL "" FORCE)
set(ENABLE_TEST         OFF CACHE BOOL "" FORCE)
set(ENABLE_INSTALL      OFF CACHE BOOL "" FORCE)
set(ENABLE_COVERAGE     OFF CACHE BOOL "" FORCE)
# Debug 默认开 /WX，libarchive 在 MSVC 下一堆 C4xxx，开了必挂；它的警告与项目无关。
set(ENABLE_WERROR       OFF CACHE BOOL "" FORCE)
# libarchive 自己 FIND_PACKAGE(ZLIB)，系统里没有第二份，直接让它命中上面编的 zlibstatic。
# 只给 zlib.h 所在的源码目录不够：zconf.h 是 configure 生成到二进制目录的，编译期也要能找到。
set(ZLIB_INCLUDE_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/zlib" CACHE PATH "" FORCE)
set(ZLIB_LIBRARY_RELEASE "zlibstatic" CACHE STRING "" FORCE)
set(ZLIB_LIBRARY_DEBUG   "zlibstatic" CACHE STRING "" FORCE)
add_subdirectory(libarchive)
# BUILD_SHARED_LIBS=OFF 时目标名是 archive_static（共享构建才叫 archive），名字太通用，
# 按项目习惯包成 thirdparty_archive。
set_target_properties(archive_static PROPERTIES EXCLUDE_FROM_ALL TRUE)
target_include_directories(archive_static PRIVATE ${PROJECT_BINARY_DIR}/thirdparty/zlib)
# archive.h 在 Windows 上默认按 DLL 导入声明（__declspec(dllimport)），静态库必须由使用方
# 自己关掉，否则链接时全是 __imp_archive_* 未解析。
target_compile_definitions(archive_static INTERFACE LIBARCHIVE_STATIC)
add_library(thirdparty_archive ALIAS archive_static)

