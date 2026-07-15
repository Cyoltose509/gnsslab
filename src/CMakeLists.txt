# lib/CMakeLists.txt
# 构建 gnss 静态库，链接进 exe 无需额外 DLL

set(LIB_SOURCES
        core/Const.h
        core/CoordStruct.h
        core/Ephemeris.h
        core/Ephemeris.cpp
        core/EphemerisTable.h
        core/GnssStruct.h
        core/GnssStruct.cpp
        time/TimeConvert.h
        time/TimeConvert.cpp
        time/TimeStruct.h
        time/TimeStruct.cpp
        coord/CoordConvert.h
        io/OEM7Reader.h
        io/OEM7Reader.cpp
        io/OEM7SocketReader.h
        io/OEM7SocketReader.cpp
        io/SocketClient.h
        io/SocketClient.cpp
        io/RinexObsReader.h
        io/RinexObsReader.cpp
        io/RinexNavStore.h
        io/RinexNavStore.cpp
        solver/SPPIFCode.h
        solver/SPPIFCode.cpp
        solver/SPPCodePhase.h
        solver/SPPCodePhase.cpp
        solver/SolverLSQ.h
        solver/SolverLSQ.cpp
        solver/KalmanFilter.h
        solver/KalmanFilter.cpp
        solver/SolverKalman.h
        solver/SolverKalman.cpp
        solver/ARLambda.h
        solver/ARLambda.cpp
        qc/QCProcessor.h
        qc/QCProcessor.cpp
        util/StringUtils.h
        util/Exception.h
        util/MathUtils.h
        util/Log.h
        util/Log.cpp
)

add_library(gnss STATIC ${LIB_SOURCES})

# 公共头文件路径：根目录 + 所有子目录，保持 #include 路径不变
target_include_directories(gnss PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/time
        ${CMAKE_CURRENT_SOURCE_DIR}/coord
        ${CMAKE_CURRENT_SOURCE_DIR}/io
        ${CMAKE_CURRENT_SOURCE_DIR}/solver
        ${CMAKE_CURRENT_SOURCE_DIR}/qc
        ${CMAKE_CURRENT_SOURCE_DIR}/util
)

# 链接第三方库（由主 CMakeLists.txt 保证 thirdparty 先被 add_subdirectory）
target_link_libraries(gnss PUBLIC thirdparty_eigen)
