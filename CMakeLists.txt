cmake_minimum_required(VERSION 3.10.2)
project(gnssLab VERSION 1.2)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# DLL 和 EXE 输出到同一目录，避免运行时找不到 DLL
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(thirdparty)
add_subdirectory(lib)
add_subdirectory(main)

add_executable(spp_file test/spp_file.cpp)
target_link_libraries(spp_file gnss)

add_executable(spp_realtime test/spp_realtime.cpp)
target_link_libraries(spp_realtime gnss)
