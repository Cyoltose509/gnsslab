cmake_minimum_required(VERSION 3.18)
project(GnssLab VERSION 0.7.0)


set(CMAKE_CXX_STANDARD 17)

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /Ob2 /DNDEBUG")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(MSVC)
    string(REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # 增量链接会留一个几百 MB 的 .ilk，而且每次重链都要先把它删掉重写；ninja 每次都是整链，
    # 增量链接没有收益，只多一份被各种"文件保护"工具盯上的大文件。直接关掉。
    add_link_options($<$<CONFIG:Debug>:/INCREMENTAL:NO>)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(app)
