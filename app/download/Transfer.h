#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

/// 远端取数的 curl 底层：列目录、取文本、断点续传下载。只服务 download 模块。
namespace Download::Transfer {
    using ProgressCb = std::function<void(double total, double now)>;
    using LogCb = std::function<void(const std::string &)>;

    // cancel 非空时，传输过程中若其被置 true 则立即中止（用于退出时快速结束线程）。
    bool listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err,
                 const std::atomic<bool> *cancel = nullptr);

    bool fetchText(const std::string &url, std::string &out, std::string &err,
                   const std::atomic<bool> *cancel = nullptr);

    bool downloadFile(const std::string &url, const std::string &localPath,
                      const ProgressCb &progress, const LogCb &log, int retries = 3,
                      int timeoutSec = 120, const std::atomic<bool> *cancel = nullptr);
}
