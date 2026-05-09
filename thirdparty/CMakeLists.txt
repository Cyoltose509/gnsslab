# thirdparty/CMakeLists.txt
# 管理所有第三方库
# 当前只有 Eigen（header-only），直接暴露其 include 路径

add_library(thirdparty_eigen INTERFACE)
target_include_directories(
        thirdparty_eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0
)
