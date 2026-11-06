

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
        core/CoordConvert.h
        core/CoordConvert.cpp
        core/Troposphere.h
        core/Troposphere.cpp
        core/Geodesy.h
        core/Ionosphere.h
        core/Weight.h
        io/Observer.h
        io/FileCache.h
        io/Decimator.h
        io/novatel/LogReader.h
        io/novatel/LogReader.cpp
        io/novatel/BinaryReader.h
        io/novatel/BinaryReader.cpp
        io/novatel/AsciiReader.h
        io/novatel/AsciiReader.cpp
        core/InsStruct.h
        core/InsMath.h
        ins/Aligner.h
        ins/Aligner.cpp
        ins/ImuCalibrator.h
        ins/ImuCalibrator.cpp
        io/novatel/SocketReader.h
        io/novatel/SocketReader.cpp
        client/SocketClient.h
        client/SocketClient.cpp
        io/rinex/RINEXFileBase.h
        io/rinex/RINEXObsReader.h
        io/rinex/RINEXObsReader.cpp
        io/rinex/RINEX2ObsReader.cpp
        io/rinex/RINEX3ObsReader.cpp
        io/rinex/RINEXNavStore.h
        io/rinex/RINEXNavStore.cpp
        io/product/SP3OrbitReader.h
        io/product/SP3OrbitReader.cpp
        io/product/CLKReader.h
        io/product/CLKReader.cpp
        io/product/ERPReader.h
        io/product/ERPReader.cpp
        io/product/TROReader.h
        io/product/TROReader.cpp
        io/product/ATTReader.h
        io/product/ATTReader.cpp
        io/product/VMF3Reader.h
        io/product/VMF3Reader.cpp
        io/product/GPT3Reader.h
        io/product/GPT3Reader.cpp
        io/product/IONEXReader.h
        io/product/IONEXReader.cpp
        io/product/OSBBias.h
        io/product/OSBBias.cpp
        io/product/ATXReader.h
        io/product/ATXReader.cpp
        core/FreqCombo.h
        core/FreqCombo.cpp
        core/CycleSlip.h
        core/CycleSlip.cpp
        solver/SolverOptions.h
        solver/DualPair.h
        solver/DualPair.cpp
        solver/TropoGrid.h
        solver/SPPConfig.h
        solver/SPP.h
        solver/SPP.cpp
        solver/TropoSite.h
        solver/TropoSite.cpp
        solver/PPPStatic.h
        solver/PPPStatic.cpp
        solver/PPPKinematic.h
        solver/PPPKinematic.cpp
        solver/LEO.h
        solver/LEO.cpp
        solver/SolverLSQ.h
        solver/SolverLSQ.cpp
        solver/SolverKalman.h
        solver/SolverKalman.cpp
        solver/GnssEKF.h
        solver/GnssEKF.cpp
        solver/ARLambda.h
        solver/ARLambda.cpp
        solver/RTKConfig.h
        solver/RTK.h
        solver/RTK.cpp
        qc/QCProcessor.h
        qc/QCProcessor.cpp
        util/StringUtils.h
        util/StringUtils.cpp
        util/Exception.h
        util/MathUtils.h
        util/MathUtils.cpp
        util/Log.h
        util/Log.cpp
        util/CSV.h
        util/CSV.cpp
)

add_library(gnss STATIC ${LIB_SOURCES})

target_compile_definitions(gnss PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

set_source_files_properties(qc/QCProcessor.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(core/CycleSlip.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(solver/PPPStatic.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(util/MathUtils.cpp PROPERTIES COMPILE_FLAGS /bigobj)

target_include_directories(gnss PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/time
        ${CMAKE_CURRENT_SOURCE_DIR}/io
        ${CMAKE_CURRENT_SOURCE_DIR}/io/novatel
        ${CMAKE_CURRENT_SOURCE_DIR}/io/rinex
        ${CMAKE_CURRENT_SOURCE_DIR}/io/product
        ${CMAKE_CURRENT_SOURCE_DIR}/client
        ${CMAKE_CURRENT_SOURCE_DIR}/solver
        ${CMAKE_CURRENT_SOURCE_DIR}/qc
        ${CMAKE_CURRENT_SOURCE_DIR}/util
)

target_link_libraries(gnss PUBLIC thirdparty_eigen)
