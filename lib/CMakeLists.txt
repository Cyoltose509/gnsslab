# lib/CMakeLists.txt
# 构建 gnss 静态库，链接进 exe 无需额外 DLL

set(LIB_SOURCES
        Const.h
        CoordConvert.h
        CoordStruct.h
        EphemerisTable.h
        Exception.h
        TimeConvert.cpp
        TimeConvert.h
        TimeStruct.cpp
        TimeStruct.h
        OEM7Reader.cpp
        OEM7Reader.h
        GnssStruct.h
        GnssStruct.cpp
        SolverLSQ.h
        SolverLSQ.cpp
        Ephemeris.cpp
        Ephemeris.h
        SPPIFCode.h
        SPPIFCode.cpp
        SocketClient.h
        SocketClient.cpp
        OEM7SocketReader.h
        OEM7SocketReader.cpp
)

add_library(gnss STATIC ${LIB_SOURCES})

# 公共头文件路径（lib 的头文件暴露给下游）
target_include_directories(gnss PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 链接第三方库（由主 CMakeLists.txt 保证 thirdparty 先被 add_subdirectory）
target_link_libraries(gnss PUBLIC thirdparty_eigen)
