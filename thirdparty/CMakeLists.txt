# thirdparty/CMakeLists.txt
# 管理所有第三方库

# ---- Eigen（header-only）----
add_library(thirdparty_eigen INTERFACE)
target_include_directories(
        thirdparty_eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0
)

# ---- ImGui（header-only + backends）----
# ImGui 主库（imgui.cpp 需要参与编译）
add_library(thirdparty_imgui STATIC
        imgui/imgui.cpp
        imgui/imgui_demo.cpp
        imgui/imgui_draw.cpp
        imgui/imgui_tables.cpp
        imgui/imgui_widgets.cpp
)
target_include_directories(thirdparty_imgui
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)

# ImGui Win32 后端
add_library(thirdparty_imgui_impl_win32 STATIC
        imgui/backends/imgui_impl_win32.cpp
)
target_include_directories(thirdparty_imgui_impl_win32
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_win32 PRIVATE thirdparty_imgui)

# ImGui OpenGL3 后端
add_library(thirdparty_imgui_impl_opengl3 STATIC
        imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(thirdparty_imgui_impl_opengl3
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_opengl3 PRIVATE thirdparty_imgui)

# ImGui DirectX11 后端
add_library(thirdparty_imgui_impl_dx11 STATIC
        imgui/backends/imgui_impl_dx11.cpp
)
target_include_directories(thirdparty_imgui_impl_dx11
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_imgui_impl_dx11 PRIVATE thirdparty_imgui d3d11 dxgi)

# ---- ImPlot ----
add_library(thirdparty_implot STATIC
        implot/implot.cpp
        implot/implot_items.cpp
        implot/implot_demo.cpp
)
target_include_directories(thirdparty_implot
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)
target_link_libraries(thirdparty_implot PRIVATE thirdparty_imgui)

# ---- ImPlot3D ----
add_library(thirdparty_implot3d STATIC
        implot3d/implot3d.cpp
        implot3d/implot3d_items.cpp
        implot3d/implot3d_meshes.cpp
        implot3d/implot3d_demo.cpp
)
target_include_directories(thirdparty_implot3d
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot3d
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui ${CMAKE_CURRENT_SOURCE_DIR}/implot
)
target_link_libraries(thirdparty_implot3d PRIVATE thirdparty_imgui thirdparty_implot)
