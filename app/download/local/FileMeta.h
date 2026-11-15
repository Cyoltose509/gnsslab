#pragma once

#include <optional>
#include <string>

#include "download/local/ProductType.h"

/// 一个已下载/待下载文件的身份：从文件名解析出它是哪种产品、哪一天的、哪个分析中心的。
/// 只认名字，不读内容——内容里的日期是观测日期，那归 io 管。
struct FileMeta {
    ProductType type = ProductType::Unknown;
    std::string datacenter;
    int year = 0;
    int doy = 0;
    int spanDays = 1;
    bool stamped = false;
    std::string interval;
    std::string ext;
    std::string filename;

    std::string slot;


    static std::optional<FileMeta> parse(const std::string &filename);

    static std::optional<FileMeta> parseInDir(const std::string &filename, ProductType dirType,
                                              int dateYear, int dateDoy);


    static std::string productLineOf(const std::string &filename);

    [[nodiscard]] bool covers(int y, int d) const;
};
