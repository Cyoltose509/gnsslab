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


    static bool listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err);


    static bool downloadFile(const std::string &url, const std::string &localPath,
                             const ProgressCb &progress, const LogCb &log, int retries = 3);
};
