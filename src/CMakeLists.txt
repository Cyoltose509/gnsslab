

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
        core/CoordConvert.h
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
        io/Sp3OrbitReader.h
        io/Sp3OrbitReader.cpp
        io/ClkReader.h
        io/ClkReader.cpp
        io/OsbBias.h
        io/OsbBias.cpp
        io/AntxReader.h
        io/AntxReader.cpp
        core/FreqCombo.h
        core/FreqCombo.cpp
        core/CycleSlip.h
        core/CycleSlip.cpp
        solver/SPP.h
        solver/SPP.cpp
        solver/PPP.h
        solver/PPP.cpp
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
        qc/QCProcessor.h
        qc/QCProcessor.cpp
        util/StringUtils.h
        util/Exception.h
        util/MathUtils.h
        util/Log.h
        util/Log.cpp
)

add_library(gnss STATIC ${LIB_SOURCES})

set_source_files_properties(qc/QCProcessor.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(core/CycleSlip.cpp PROPERTIES COMPILE_FLAGS /bigobj)

target_include_directories(gnss PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/time
        ${CMAKE_CURRENT_SOURCE_DIR}/io
        ${CMAKE_CURRENT_SOURCE_DIR}/solver
        ${CMAKE_CURRENT_SOURCE_DIR}/qc
        ${CMAKE_CURRENT_SOURCE_DIR}/util
)

target_link_libraries(gnss PUBLIC thirdparty_eigen)
