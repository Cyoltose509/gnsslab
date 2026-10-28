# main/CMakeLists.txt

# --- Application Metadata ---
set(COMPANY_NAME "Cyoltose")
set(FILE_DESCRIPTION "GnssLab Application")
string(TIMESTAMP CURRENT_YEAR "%Y")

# Handle version components if not fully specified in project()
if(NOT PROJECT_VERSION_PATCH)
    set(PROJECT_VERSION_PATCH 0)
endif()

# Configure the version header
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.h
    @ONLY
)

# Configure the Windows Resource file
if(WIN32)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/GnssLab.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/GnssLab.rc
        @ONLY
    )
    set(GNSSLAB_RESOURCES ${CMAKE_CURRENT_BINARY_DIR}/GnssLab.rc)
endif()

add_executable(GnssLab WIN32
    main.cpp
    core/Application.cpp
    core/PathMemory.cpp
    core/IniFile.cpp
    core/Csv.cpp
    core/DualObsPairing.cpp
    ui/Gui.cpp
        widgets/tools/GuiTimeConverter.cpp
        widgets/tools/GuiCoordConverter.cpp
        widgets/tools/GuiLsqSolver.cpp
        widgets/tools/GuiInsPanel.cpp
        widgets/tools/GuiInsCalib.cpp
        widgets/tools/GuiInsAlign.cpp
        widgets/solvers/GuiSolverPanel.cpp
        widgets/solvers/GuiSppProcessor.cpp
        widgets/solvers/SolverTask.cpp
        widgets/solvers/GuiPppProcessor.cpp
        widgets/solvers/GuiLeoProcessor.cpp
        widgets/solvers/GuiRtkProcessor.cpp
        widgets/solvers/GuiCharts.cpp
        widgets/solvers/GuiHelpers.cpp
        widgets/solvers/GuiRealtimeProcessor.cpp
        widgets/solvers/QualityControl.cpp
        widgets/tools/GuiFtpDownload.cpp
        download/ProductMeta.cpp
        download/DownloadStore.cpp
        download/FtpClient.cpp
    ${GNSSLAB_RESOURCES}
)

target_include_directories(GnssLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core   # SolveDriver.h（CLI 与 GUI 共享编排层）所在地
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/solvers
    ${PROJECT_SOURCE_DIR}/thirdparty/curl/include
    ${PROJECT_SOURCE_DIR}/thirdparty/zlib
    ${PROJECT_BINARY_DIR}/thirdparty/zlib
)

target_compile_definitions(GnssLab PRIVATE CURL_STATICLIB)

target_link_libraries(GnssLab PRIVATE
    thirdparty_imgui
    thirdparty_imgui_impl_win32
    thirdparty_imgui_impl_dx11
    thirdparty_implot
    thirdparty_implot3d
    gnss
    libcurl_static
    zlibstatic
    ws2_32
    crypt32
    normaliz
    bcrypt
    advapi32
    d3d11
    dxgi
    comdlg32
    windowscodecs
    ole32
)


set(CLI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cli)
set(CLI_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/SolveConfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/SolveConfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Csv.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/Csv.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/DualObsPairing.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/DualObsPairing.cpp
    ${CLI_DIR}/CliCommon.h
    ${CLI_DIR}/CliCommon.cpp
)
set(CLI_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CLI_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/widgets/solvers
    # CLI 头文件链会经 widgets/solvers 摸到 QualityControl.h（含 imgui.h），
    # 只需 include 路径可解析头文件即可；模板不实例化则不引入任何 imgui 符号。
    ${PROJECT_SOURCE_DIR}/thirdparty/imgui
    ${PROJECT_SOURCE_DIR}/thirdparty/imgui/backends
    ${PROJECT_SOURCE_DIR}/thirdparty/implot
    ${PROJECT_SOURCE_DIR}/thirdparty/implot3d
)

set(CLI_INIS spp ppp rtk leo)

foreach(_cli spp ppp rtk leo)
    add_executable(${_cli}_cli ${CLI_DIR}/${_cli}_cli.cpp ${CLI_COMMON})
    target_include_directories(${_cli}_cli PRIVATE ${CLI_INCLUDES})
    target_link_libraries(${_cli}_cli PRIVATE gnss ws2_32)
    target_compile_options(${_cli}_cli PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
    foreach(_ini ${CLI_INIS})
        add_custom_command(TARGET ${_cli}_cli POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${CLI_DIR}/${_ini}.ini
                    $<TARGET_FILE_DIR:${_cli}_cli>/${_ini}.ini
            COMMENT "Copy ${_ini}.ini -> $<TARGET_FILE_DIR:${_cli}_cli>")
    endforeach()
endforeach()
