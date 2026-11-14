cmake_minimum_required(VERSION 3.18)

# 把 README.md 按原始字节编进头文件，避免 MSVC 源码编码影响中文。
file(READ "${INPUT}" _hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytes "${_hex}")

file(WRITE "${OUTPUT}"
"#pragma once\n"
"// Generated from README.md at build time; do not edit.\n"
"#include <cstddef>\n"
"namespace UI {\n"
"inline constexpr unsigned char kEmbeddedReadme[] = {${_bytes}0};\n"
"inline constexpr std::size_t kEmbeddedReadmeLen = sizeof(kEmbeddedReadme) - 1;\n"
"}\n")
