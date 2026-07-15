cmake_minimum_required(VERSION 3.10.2)
project(GnssLab VERSION 1.4)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 统一 CRT：Debug=/MTd  Release=/MT（所有 target 一致，避免 LNK2038）
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# 输出目录：仅 Release 落到 lib/ bin/，Debug 留在 cmake-build-debug（CLion 默认）
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(app)
