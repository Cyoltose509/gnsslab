#pragma once

#include <curl/curl.h>
#include <functional>
#include <string>
#include <vector>

class FtpClient {
public:
    using ProgressCb = std::function<void(double total, double now)>;
    using LogCb      = std::function<void(const std::string &)>;

    FtpClient();
    ~FtpClient();

    // 列目录（url 应为以 '/' 结尾的目录）。返回文件名列表（不含路径）；失败时 err 有描述。
    static bool listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err);

    // 下载单文件到 localPath。progress/log 在调用线程触发。retries 次重试后失败返回 false。
    static bool downloadFile(const std::string &url, const std::string &localPath,
                             const ProgressCb &progress, const LogCb &log, int retries = 3);
};
