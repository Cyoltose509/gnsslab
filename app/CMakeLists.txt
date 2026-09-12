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
    core/AppConfig.cpp
    ui/Gui.cpp
        widgets/GuiTimeConverter.cpp
        widgets/GuiCoordConverter.cpp
        widgets/GuiLsqSolver.cpp
        widgets/GuiFileProcessor.cpp
        widgets/GuiPppProcessor.cpp
        widgets/GuiLeoProcessor.cpp
        widgets/GuiCharts.cpp
        widgets/GuiHelpers.cpp
        widgets/GuiRealtimeProcessor.cpp
        widgets/QualityControl.cpp
    ${GNSSLAB_RESOURCES}
)

# 结构化异常处理：启用 /EHa，使 __try/__except 能捕获 ImPlot/ImPlot3D 内部断言(0x80000003)
# 并允许在 SEH 块内正确调用含 C++ 析构的代码。
target_include_directories(GnssLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(GnssLab PRIVATE
    thirdparty_imgui
    thirdparty_imgui_impl_win32
    thirdparty_imgui_impl_dx11
    thirdparty_implot
    thirdparty_implot3d
    gnss
    d3d11
    dxgi
    comdlg32
    windowscodecs
    ole32
)
