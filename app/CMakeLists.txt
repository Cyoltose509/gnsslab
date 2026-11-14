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

# 把 README.md 编成字节数组头文件，随 exe 一起链接；改动 README 会触发重新生成与重编
set(GNSSLAB_README_HEADER ${CMAKE_CURRENT_BINARY_DIR}/EmbeddedReadme.h)
add_custom_command(
    OUTPUT ${GNSSLAB_README_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DINPUT=${PROJECT_SOURCE_DIR}/README.md
        -DOUTPUT=${GNSSLAB_README_HEADER}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/EmbedReadme.cmake
    DEPENDS ${PROJECT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_SOURCE_DIR}/EmbedReadme.cmake
    COMMENT "Embedding README.md into GnssLab.exe"
    VERBATIM
)

add_executable(GnssLab WIN32
    main.cpp
    core/Application.cpp
    core/PathMemory.cpp
    io/CRXDecoder.cpp
    io/SinexReader.cpp
    io/Products.cpp
    io/ProductPicker.cpp
    io/ProductLoad.cpp
    io/ObsSource.cpp
    core/IniFile.cpp
    ui/Gui.cpp
    ui/Markdown.h
    ui/Markdown.cpp
    ui/DatePicker.cpp
    ui/DragBar.cpp
    ui/ParamPage.cpp
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
    download/local/ProductMeta.cpp
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

# 引擎侧源文件：与 GUI 共用同一份任务骨架与逐历元求解，不含任何 imgui/view 依赖
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
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ProductLoad.cpp
    ${CLI_DIR}/TaskViews.cpp
)

# 解算前自动维护天线表所需的「网络 + 本地」下载模块（不含任何 imgui/UI 面板），
# 让 CLI 也能在解算前检测产品天线版本并自动下载对齐，与 GUI 行为一致。
set(CLI_DOWNLOAD
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/List.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/http/Download.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/Catalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/SourceTable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/remote/Fetcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/Store.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/ProductMeta.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/download/local/Unpack.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/PathMemory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/CRXDecoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/SinexReader.cpp
)

# 一个 CLI 解所有模式：ini 里的 mode 键决定解算模式
add_executable(gnss_cli ${CLI_DIR}/gnss_cli.cpp ${CLI_COMMON} ${CLI_ENGINE} ${CLI_DOWNLOAD})
target_compile_definitions(gnss_cli PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX CURL_STATICLIB)
target_include_directories(gnss_cli PRIVATE
    ${CLI_INCLUDES}
    ${PROJECT_SOURCE_DIR}/thirdparty/curl/include
)
target_link_libraries(gnss_cli PRIVATE
    gnss ws2_32 libcurl_static thirdparty_archive crypt32 normaliz bcrypt)
target_compile_options(gnss_cli PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

# 产品下载 CLI：fetch.ini 或 --参数，日期/类型/数据中心/匹配字段批量取数，落盘与 GUI 同一个 downloads/
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
