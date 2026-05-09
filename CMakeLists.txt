
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
        lib/SocketClient.h
        lib/SocketClient.cpp
        lib/OEM7SocketReader.h
        lib/OEM7SocketReader.cpp
)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
# 根据源文件创建库文件
add_library(gnss SHARED ${DIR_LIB_SRCS})

# 创建可执行程序

add_executable(lsq main/lsq.cpp)
target_link_libraries(lsq gnss)


# 实时网络定位程序
add_executable(spp_realtime main/spp_realtime.cpp)
target_link_libraries(spp_realtime gnss)
