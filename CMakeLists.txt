cmake_minimum_required(VERSION 3.10.2)
project(GnssLab VERSION 1.1)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


add_subdirectory(thirdparty)
add_subdirectory(lib)
add_subdirectory(main)

add_executable(spp_file test/spp_file.cpp)
target_link_libraries(spp_file gnss)

add_executable(spp_realtime test/spp_realtime.cpp)
target_link_libraries(spp_realtime gnss)
