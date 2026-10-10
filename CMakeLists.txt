cmake_minimum_required(VERSION 3.18)
project(GnssLab VERSION 4.0)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
# 本项目用 MSVC(cl.exe)。Release 必须用 MSVC 认得的 /O2；原先写的 "-O3 -Wall -g" 是 GCC 语法，
# cl.exe 会报 D9002 忽略 -O3、且覆盖掉 CMake 默认带的 /O2，导致"Release"实际零优化 → RTK 仍慢。
# 守卫 MSVC：换 GCC/Clang 时退回 -O3。
if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /Ob2 /DNDEBUG")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# MSVC：启用异步结构化异常处理 (/EHa)，使 __try/__except 能捕获 ImPlot/ImPlot3D 内部断言
# (0x80000003)，并允许在 SEH 块内执行 C++ 析构。CMake 默认 /EHsc 会与本项目冲突。
if(MSVC)
    string(REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

# 输出目录：仅 Release 落到 lib/ bin/，Debug 留在 cmake-build-debug（CLion 默认）
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(app)
