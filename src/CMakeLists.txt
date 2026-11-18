

add_subdirectory(i18n)

set(LIB_SOURCES

        i18n/I18n.h
        i18n/I18n.cpp

        core/Const.h
        core/SatSystem.h
        core/SatID.h

        core/obs/ObsStruct.h
        core/obs/FreqCombo.h
        core/obs/FreqCombo.cpp
        core/obs/CycleSlip.h
        core/obs/CycleSlip.cpp
        core/obs/Weight.h

        core/geom/CoordStruct.h
        core/geom/Attitude.h
        core/geom/Attitude.cpp
        core/geom/CoordConvert.h
        core/geom/CoordConvert.cpp
        core/geom/Geodesy.h
        core/geom/Ephemeris.h
        core/geom/Ephemeris.cpp
        core/geom/EphemerisTable.h

        core/equ/EquStruct.h
        core/equ/EquStruct.cpp
        core/equ/Result.h
        core/equ/Precision.h

        core/atmo/Troposphere.h
        core/atmo/Troposphere.cpp
        core/atmo/Ionosphere.h

        core/ins/InsStruct.h
        core/ins/InsMath.h
        core/TimeStruct.h
        core/TimeStruct.cpp
        io/Observer.h
        io/Observer.cpp
        io/FileCache.h
        io/Decimator.h
        io/ins/BinReader.h
        io/ins/BinReader.cpp
        io/novatel/LogReader.h
        io/novatel/LogReader.cpp
        io/novatel/BinaryReader.h
        io/novatel/BinaryReader.cpp
        io/novatel/AsciiReader.h
        io/novatel/AsciiReader.cpp
        solver/ins/Aligner.h
        solver/ins/Aligner.cpp
        solver/ins/Calibrator.h
        solver/ins/Calibrator.cpp
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
        io/rinex/RINEXObsMerge.h
        io/rinex/RINEXObsMerge.cpp
        io/product/sp3/Reader.h
        io/product/sp3/Reader.cpp
        io/product/sp3/Writer.h
        io/product/sp3/Writer.cpp
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
        solver/SolverOptions.h
        solver/TropoGrid.h
        solver/Products.h
        solver/SPPConfig.h
        solver/Filter.h
        solver/Filter.cpp
        solver/GnssSolver.h
        solver/GnssSolver.cpp
        solver/SPP.h
        solver/SPP.cpp
        solver/TropoSite.h
        solver/TropoSite.cpp
        solver/ppp/Static.h
        solver/ppp/Static.cpp
        solver/ppp/Equation.cpp
        solver/ppp/Epoch.cpp
        solver/ppp/AR.cpp
        solver/ppp/Kinematic.h
        solver/ppp/Kinematic.cpp
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
        solver/rtk/DualPair.h
        solver/rtk/DualPair.cpp
        solver/rtk/RTKConfig.h
        solver/rtk/RTK.h
        solver/rtk/RTK.cpp
        solver/ins/INS.h
        solver/ins/INS.cpp
        solver/ins/InsVerify.h
        solver/ins/InsVerify.cpp
        solver/ins/INSConfig.h
        qc/QCProcessor.h
        qc/QCProcessor.cpp
        util/StringUtils.h
        util/StringUtils.cpp
        util/Exception.h
        util/FileUtils.h
        util/FileUtils.cpp
        util/MathUtils.h
        util/MathUtils.cpp
        util/Clock.h
        util/Clock.cpp
        util/Log.h
        util/Log.cpp
        util/CSV.h
        util/CSV.cpp
)

add_library(gnss STATIC ${LIB_SOURCES})


target_compile_options(gnss PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)


set(LANG_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lang ${PROJECT_SOURCE_DIR}/app/lang)
set(LANG_MT_ROOT ${PROJECT_SOURCE_DIR}/build/lang-mt)

file(MAKE_DIRECTORY ${LANG_MT_ROOT}/app ${LANG_MT_ROOT}/src)
file(GLOB_RECURSE LANG_CSV_FILES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/lang/*.csv ${PROJECT_SOURCE_DIR}/app/lang/*.csv
        ${LANG_MT_ROOT}/*.csv)
set(GEN_LANG_CPP ${CMAKE_CURRENT_BINARY_DIR}/gen/LangData.cpp)

set(LANG_STAMP ${CMAKE_CURRENT_BINARY_DIR}/gen/lang_inputs.stamp)
set(LANG_CSV_KEY "")
foreach(_f ${LANG_CSV_FILES})
    string(APPEND LANG_CSV_KEY "${_f}\n")
endforeach()
set(LANG_CSV_KEY "${LANG_CSV_KEY}")
set(_stamp_old "")
if(EXISTS ${LANG_STAMP})
    file(READ ${LANG_STAMP} _stamp_old)
endif()
if(NOT _stamp_old STREQUAL "${LANG_CSV_KEY}")
    file(WRITE ${LANG_STAMP} "${LANG_CSV_KEY}")
endif()

add_custom_command(
        OUTPUT ${GEN_LANG_CPP}
        COMMAND $<TARGET_FILE:gen_lang> ${GEN_LANG_CPP} ${LANG_DIRS}
                --mt ${LANG_MT_ROOT}/app --mt ${LANG_MT_ROOT}/src
        DEPENDS gen_lang ${LANG_CSV_FILES} ${LANG_STAMP}
        COMMENT "生成多语言表"
        VERBATIM)
target_sources(gnss PRIVATE ${GEN_LANG_CPP})
add_dependencies(gnss gen_lang)

target_compile_definitions(gnss PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

set_source_files_properties(qc/QCProcessor.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(core/obs/CycleSlip.cpp PROPERTIES COMPILE_FLAGS /bigobj)
set_source_files_properties(util/MathUtils.cpp PROPERTIES COMPILE_FLAGS /bigobj)

target_include_directories(gnss PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
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
