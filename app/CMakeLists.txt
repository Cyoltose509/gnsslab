
set(COMPANY_NAME "Cyoltose")
set(FILE_DESCRIPTION "GnssLab Application")
string(TIMESTAMP CURRENT_YEAR "%Y")

if(NOT PROJECT_VERSION_PATCH)
    set(PROJECT_VERSION_PATCH 0)
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.h
    @ONLY
)

if(WIN32)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/GnssLab.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/GnssLab.rc
        @ONLY
    )
    set(GNSSLAB_RESOURCES ${CMAKE_CURRENT_BINARY_DIR}/GnssLab.rc)
endif()

set(GNSSLAB_README_HEADER ${CMAKE_CURRENT_BINARY_DIR}/EmbeddedReadme.h)
add_custom_command(
    OUTPUT ${GNSSLAB_README_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DINPUT_ZH=${PROJECT_SOURCE_DIR}/README_ZH.md
        -DINPUT_EN=${PROJECT_SOURCE_DIR}/README.md
        -DOUTPUT=${GNSSLAB_README_HEADER}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/EmbedReadme.cmake
    DEPENDS ${PROJECT_SOURCE_DIR}/README_ZH.md ${PROJECT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/EmbedReadme.cmake
    COMMENT "Embedding README_ZH.md / README.md into GnssLab.exe"
    VERBATIM
)

add_executable(GnssLab WIN32
    main.cpp
    core/Application.cpp
    core/SiteTruth.cpp
    core/PathMemory.cpp
    io/CRXDecoder.cpp
    io/SinexReader.cpp
    io/ProductPaths.cpp
    io/ProductReader.cpp
    io/ObsDate.cpp
    io/ObsSource.cpp
    solvers/ins/ImuScaleProbe.cpp
    core/IniFile.cpp
    ui/Gui.cpp
    ui/Markdown.h
    ui/Markdown.cpp
    ui/DatePicker.cpp
    ui/Splitter.cpp
    ui/ParamPage.cpp
    ui/ImGuiLabel.h
    tools/GuiTimeConverter.cpp
    tools/GuiCoordConverter.cpp
    tools/GuiLsqSolver.cpp
    tools/ins/GuiInsPanel.cpp
    tools/ins/GuiInsCommon.cpp
    tools/ins/GuiInsCalib.cpp
    tools/ins/GuiInsAlign.cpp
    tools/ins/GuiInsVerify.cpp
    solvers/view/GuiSolverPanel.cpp
    solvers/common/Task.cpp
    solvers/common/Core.cpp
    solvers/common/Realtime.cpp
    solvers/export/Core.cpp
    solvers/export/CSV.cpp
    solvers/export/POS.cpp
    solvers/export/SP3.cpp
    solvers/export/ORB.cpp
    solvers/export/TRAJ.cpp
    solvers/export/KML.cpp
    solvers/export/SBET.cpp
    solvers/spp/Task.cpp
    solvers/spp/Panel.cpp
    solvers/ppp_static/Task.cpp
    solvers/ppp_static/Panel.cpp
    solvers/ppp_kinematic/Task.cpp
    solvers/ppp_kinematic/Panel.cpp
    solvers/leo/Antenna.cpp
    solvers/leo/AntennaTable.cpp
    solvers/leo/Task.cpp
    solvers/leo/Panel.cpp
    solvers/rtk/Task.cpp
    solvers/rtk/Panel.cpp
    solvers/ins/Task.cpp
    solvers/ins/Panel.cpp
    solvers/ins/Result.cpp
    solvers/view/GuiCharts.cpp
    solvers/view/GuiPosTab.cpp
    solvers/view/GuiExport.cpp
    solvers/view/GuiHelpers.cpp
    solvers/view/TaskGui.cpp
    solvers/view/GuiProductSection.cpp
    solvers/view/GuiParamPage.cpp
    solvers/view/GuiRefTruth.cpp
    solvers/spp/Realtime.cpp
    solvers/rtk/Realtime.cpp
    solvers/rtk/Pipeline.cpp
    solvers/view/QualityControl.cpp
    download/remote/Fetcher.cpp
    download/remote/http/Core.cpp
    download/remote/http/List.cpp
    download/remote/http/Download.cpp
    download/remote/SourceTable.cpp
    download/remote/Catalog.cpp
    download/local/FileMeta.cpp
    download/local/ProductType.cpp
    download/local/Store.cpp
    download/local/Unpack.cpp
    download/ui/Panel.cpp
    download/ui/Acquire.cpp
    download/ui/Widgets.cpp
    ${GNSSLAB_README_HEADER}
    ${GNSSLAB_RESOURCES}
)

target_compile_definitions(GnssLab PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

target_include_directories(GnssLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/thirdparty/curl/include
    ${PROJECT_SOURCE_DIR}/thirdparty/imgui_markdown
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
    thirdparty_archive
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

set(CLI_ENGINE
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/CSV.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/POS.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/SP3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/ORB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/TRAJ.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/KML.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/export/SBET.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ppp_static/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ppp_kinematic/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/leo/Antenna.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/leo/AntennaTable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/leo/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ins/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Pipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsDate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ins/ImuScaleProbe.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ProductReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ProductPaths.cpp
    ${CLI_DIR}/TaskViews.cpp
)

set(CLI_DOWNLOAD
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/List.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/Download.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/Catalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/SourceTable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/Fetcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/Store.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/FileMeta.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/ProductType.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/Unpack.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/PathMemory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/CRXDecoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/SinexReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsDate.cpp
)

add_executable(gnss_cli ${CLI_DIR}/gnss_cli.cpp ${CLI_COMMON} ${CLI_ENGINE} ${CLI_DOWNLOAD})
target_compile_definitions(gnss_cli PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX CURL_STATICLIB)
target_include_directories(gnss_cli PRIVATE
    ${CLI_INCLUDES}
    ${PROJECT_SOURCE_DIR}/thirdparty/curl/include
)
target_link_libraries(gnss_cli PRIVATE
    gnss ws2_32 libcurl_static thirdparty_archive crypt32 normaliz bcrypt)
target_compile_options(gnss_cli PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

add_executable(fetch_cli ${CLI_DIR}/fetch_cli.cpp ${CLI_DIR}/CliCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.h ${CMAKE_CURRENT_SOURCE_DIR}/core/IniFile.cpp
    ${CLI_DOWNLOAD})
target_compile_definitions(fetch_cli PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX CURL_STATICLIB)
target_include_directories(fetch_cli PRIVATE
    ${CLI_INCLUDES}
    ${PROJECT_SOURCE_DIR}/thirdparty/curl/include
)
target_link_libraries(fetch_cli PRIVATE
    gnss ws2_32 libcurl_static thirdparty_archive crypt32 normaliz bcrypt)

add_custom_target(ini_template ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/cli/gnss.ini
            ${PROJECT_SOURCE_DIR}/bin/gnss.ini
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/cli/fetch.ini
            ${PROJECT_SOURCE_DIR}/bin/fetch.ini
    COMMENT "同步 bin/gnss.ini 与 bin/fetch.ini（CLI 配置模板）")

foreach(_t GnssLab gnss_cli fetch_cli)
    target_compile_options(${_t} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
endforeach()
