
add_executable(gen_lang gen_lang.cpp)


target_compile_options(gen_lang PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

foreach(_cfg DEBUG RELEASE MINSIZEREL RELWITHDEBINFO)
    set_target_properties(gen_lang PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
            RUNTIME_OUTPUT_DIRECTORY_${_cfg} ${CMAKE_CURRENT_BINARY_DIR})
endforeach()
