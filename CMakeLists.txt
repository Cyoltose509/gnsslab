cmake_minimum_required(VERSION 3.10.2)
project(GnssLab VERSION 1.3)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


add_subdirectory(thirdparty)
add_subdirectory(lib)
add_subdirectory(main)
