# main/CMakeLists.txt
# gnssLab GUI application (ImGui + Win32 + DirectX11)

add_executable(gnsslab_gui WIN32
    main.cpp
    core/Application.cpp
    ui/Gui.cpp
    tools/GuiTimeConverter.cpp
    tools/GuiCoordConverter.cpp
    tools/GuiOem7Processor.cpp
)

target_include_directories(gnsslab_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/lib
)

target_link_libraries(gnsslab_gui PRIVATE
    thirdparty_imgui
    thirdparty_imgui_impl_win32
    thirdparty_imgui_impl_dx11
    gnss
    d3d11
    dxgi
    comdlg32
)
