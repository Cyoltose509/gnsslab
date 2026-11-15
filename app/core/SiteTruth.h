#pragma once

#include <functional>
#include <string>

#include "download/remote/http/Core.h"

/// 站点静态真值的取法：GUI 注册成走下载库缓存，CLI 不注册，只用本地已有文件。
namespace SiteTruth {
    using Fetcher = std::function<bool(const std::string &site, int year, int doy, double xyz[3],
                                       bool &fromCache, std::string &detail,
                                       const Download::Http::ProgressCb &progress)>;

    Fetcher &fetcher();
}
