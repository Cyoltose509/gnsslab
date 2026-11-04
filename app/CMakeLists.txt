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
    core/Products.cpp
    io/ProductPicker.cpp
    io/ObsSource.cpp
    core/IniFile.cpp
    ui/Gui.cpp
        tools/GuiTimeConverter.cpp
        tools/GuiCoordConverter.cpp
        tools/GuiLsqSolver.cpp
        tools/ins/GuiInsPanel.cpp
        tools/ins/GuiInsCommon.cpp
        tools/ins/GuiInsCalib.cpp
        tools/ins/GuiInsAlign.cpp
        solvers/view/GuiSolverPanel.cpp
        solvers/common/Task.cpp
        solvers/common/Core.cpp
        solvers/common/Epoch.cpp
        solvers/common/Session.cpp
        solvers/spp/Task.cpp
        solvers/spp/Panel.cpp
        solvers/ppp/Task.cpp
        solvers/ppp/Panel.cpp
        solvers/leo/Task.cpp
        solvers/leo/Panel.cpp
        solvers/rtk/Task.cpp
        solvers/rtk/Panel.cpp
        solvers/view/GuiCharts.cpp
        solvers/view/GuiPosTab.cpp
        solvers/view/GuiHelpers.cpp
        solvers/view/GuiProductSection.cpp
        solvers/view/GuiParamPage.cpp
        solvers/spp/Realtime.cpp
        solvers/rtk/Realtime.cpp
        solvers/rtk/Pipeline.cpp
        solvers/view/QualityControl.cpp
        download/GuiFtpDownload.cpp
        client/RemoteClient.cpp
        download/ProductMeta.cpp
        download/DownloadStore.cpp
        download/ProductCatalog.cpp
        download/ProductStore.cpp
        download/AutoSync.cpp
        ${GNSSLAB_RESOURCES}
)

target_compile_definitions(GnssLab PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

target_include_directories(GnssLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/client
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
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
    shell32
    windowscodecs
    ole32
)


set(CLI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cli)
set(CLI_COMMON
    ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.cpp
    ${CLI_DIR}/CliCommon.h
    ${CLI_DIR}/CliCommon.cpp
)
set(CLI_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CLI_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers
)

set(CLI_INIS spp ppp rtk leo)

# 引擎侧源文件：与 GUI 共用同一份任务骨架与逐历元求解，不含任何 imgui/view 依赖
set(CLI_ENGINE
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Epoch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Session.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ppp/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/leo/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Pipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsSource.cpp
)

foreach(_cli spp ppp rtk leo)
    add_executable(${_cli}_cli ${CLI_DIR}/${_cli}_cli.cpp ${CLI_COMMON} ${CLI_ENGINE})
    target_compile_definitions(${_cli}_cli PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
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
