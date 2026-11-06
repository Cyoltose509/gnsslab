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
    core/CRXDecoder.cpp
    core/SinexReader.cpp
    io/Products.cpp
    io/ProductPicker.cpp
    io/ObsSource.cpp
    core/IniFile.cpp
    ui/Gui.cpp
    ui/DatePicker.cpp
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
        solvers/common/Realtime.cpp
        solvers/spp/Task.cpp
        solvers/spp/Panel.cpp
        solvers/ppp_static/Task.cpp
        solvers/ppp_static/Panel.cpp
        solvers/ppp_kinematic/Task.cpp
        solvers/ppp_kinematic/Panel.cpp
        solvers/leo/Task.cpp
        solvers/leo/Panel.cpp
        solvers/rtk/Task.cpp
        solvers/rtk/Panel.cpp
        solvers/view/GuiCharts.cpp
        solvers/view/GuiPosTab.cpp
        solvers/view/GuiHelpers.cpp
        solvers/view/TaskGui.cpp
        solvers/view/GuiProductSection.cpp
        solvers/view/GuiParamPage.cpp
        solvers/view/GuiRefTruth.cpp
        solvers/spp/Realtime.cpp
        solvers/rtk/Realtime.cpp
        solvers/rtk/Pipeline.cpp
        solvers/view/QualityControl.cpp
        download/Remote.cpp
        download/Panel.cpp
        download/Acquire.cpp
        download/Widgets.cpp
        download/Transfer.cpp
        download/ProductMeta.cpp
        download/Store.cpp
        download/Catalog.cpp
        ${GNSSLAB_RESOURCES}
)

target_compile_definitions(GnssLab PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

target_include_directories(GnssLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/core
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

# 引擎侧源文件：与 GUI 共用同一份任务骨架与逐历元求解，不含任何 imgui/view 依赖
set(CLI_ENGINE
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/common/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/spp/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ppp_static/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/ppp_kinematic/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/leo/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Realtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/solvers/rtk/Pipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io/ObsSource.cpp
    ${CLI_DIR}/TaskViews.cpp
)

# 一个 CLI 解所有模式：ini 里的 mode 键决定解算模式
add_executable(gnss_cli ${CLI_DIR}/gnss_cli.cpp ${CLI_COMMON} ${CLI_ENGINE})
target_compile_definitions(gnss_cli PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(gnss_cli PRIVATE ${CLI_INCLUDES})
target_link_libraries(gnss_cli PRIVATE gnss ws2_32)
target_compile_options(gnss_cli PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
