#pragma once

#include <filesystem>
#include <fstream>
#include <functional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace File {
    /// 逐行读文本文件（剥 CR，跨块残行拼接），每行回调一次；打不开返回 false。
    bool forEachLine(const std::string &path, const std::function<void(std::string_view)> &fn);

    bool safeWrite(const std::filesystem::path &path,
                   const std::function<void(std::ostream &)> &write,
                   std::string &error, bool binary = true);

    /// 路径最后一段（不含目录）。
    std::string baseName(const std::string &path);

    /// 去掉最后一个 '.' 及其后缀（'.' 在开头时不认为是扩展名）。
    std::string stripExt(const std::string &fn);

    /// 文件内容快速指纹（头尾各 256KB + 长度的 FNV-1a），打不开返回 "ERR:" + 路径。
    std::string contentHash(const std::string &path);
}
