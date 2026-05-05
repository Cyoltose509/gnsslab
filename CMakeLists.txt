
# 编译该软件时使用的cmake版本
cmake_minimum_required(VERSION 3.10.2)

# 项目名称
project(gnssLab VERSION 1.2)

# 编译选项
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Eigen矩阵库的位置
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0/)

# 添加头文件的路径
include_directories(${PROJECT_SOURCE_DIR}/lib/)

# 添加源文件名到变量中
#aux_source_directory( ${PROJECT_SOURCE_DIR}/lib/ DIR_LIB_SRCS )
set(DIR_LIB_SRCS
        lib/Const.h
        lib/CoordConvert.h
        lib/CoordStruct.h
        lib/Exception.h
        lib/TimeConvert.cpp
        lib/TimeConvert.h
        lib/TimeStruct.cpp
        lib/TimeStruct.h
        lib/OEM7Reader.cpp
        lib/OEM7Reader.h
        lib/GnssStruct.h
        lib/GnssStruct.cpp
        lib/SolverLSQ.h
        lib/SolverLSQ.cpp
        lib/Ephemeris.cpp
        lib/Ephemeris.h
        lib/SPPIFCode.h
        lib/SPPIFCode.cpp
)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
# 根据源文件创建库文件
add_library(gnss SHARED ${DIR_LIB_SRCS})

# 创建可执行程序
add_executable(ome7_read main/ome7_read.cpp)
target_link_libraries(ome7_read gnss)

add_executable(lsq main/lsq.cpp)
target_link_libraries(lsq gnss)

add_executable(lsq2 main/lsq2.cpp)
target_link_libraries(lsq2 gnss)

add_executable(spp_spv main/spp_spv.cpp)
target_link_libraries(spp_spv gnss)
