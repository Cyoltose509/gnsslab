#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace Download::Http {
    using ProgressCb = std::function<void(double total, double now)>;
    using LogCb = std::function<void(const std::string &)>;

    float fracOf(double total, double now);

    void setEarthdata(const std::string &user, const std::string &pass);

    bool hasEarthdata();

    bool checkEarthdata(const std::string &url, std::string &msg,
                        const std::atomic<bool> *cancel = nullptr);

    bool listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err,
                 const std::atomic<bool> *cancel = nullptr,
                 std::map<std::string, std::string> *fullUrls = nullptr);

    std::string nameOfUrl(const std::string &url);

    bool fetchText(const std::string &url, std::string &out, std::string &err,
                   const std::atomic<bool> *cancel = nullptr);

    bool probeFile(const std::string &url, bool &exists, std::string &err,
                   const std::atomic<bool> *cancel = nullptr);

    void forgetDownHosts();


    bool downloadFile(const std::string &url, const std::string &localPath,
                      const ProgressCb &progress, const LogCb &log, int stalls = 5,
                      int idleSec = 60, const std::atomic<bool> *cancel = nullptr);

    /// 只取远端的 maxBytes 字节
    bool downloadPartial(const std::string &url, const std::string &localPath, long long maxBytes,
                         const ProgressCb &progress = {}, int timeoutSec = 180,
                         const std::atomic<bool> *cancel = nullptr,
                         long long from = 0, bool append = false);
}
