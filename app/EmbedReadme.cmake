cmake_minimum_required(VERSION 3.18)

# 把中英两份 README 按原始字节编进头文件，避免 MSVC 源码编码影响中文。
file(READ "${INPUT_ZH}" _hexZh HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytesZh "${_hexZh}")
file(READ "${INPUT_EN}" _hexEn HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytesEn "${_hexEn}")

file(WRITE "${OUTPUT}"
"#pragma once\n"
"// Generated from README_ZH.md / README.md at build time; do not edit.\n"
"#include <cstddef>\n"
"namespace UI {\n"
"inline constexpr unsigned char kEmbeddedReadmeZH[] = {${_bytesZh}0};\n"
"inline constexpr std::size_t kEmbeddedReadmeZHLen = sizeof(kEmbeddedReadmeZH) - 1;\n"
"inline constexpr unsigned char kEmbeddedReadmeEN[] = {${_bytesEn}0};\n"
"inline constexpr std::size_t kEmbeddedReadmeENLen = sizeof(kEmbeddedReadmeEN) - 1;\n"
"}\n")
