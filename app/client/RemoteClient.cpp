#include "RemoteClient.h"

#include "Log.h"
#include "StringUtils.h"

#include <fstream>
#include <thread>
#include <chrono>
#include <filesystem>
#include <regex>
#include <vector>
#include <string>

namespace {
    void ensureGlobalInit() {
        static bool ok = curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK;
        (void) ok;
    }

    struct DlCtx {
        std::ostream *out = nullptr;
        curl_off_t base = 0;
        RemoteClient::ProgressCb progress;
        RemoteClient::LogCb log;
    };


    std::size_t writeCb(const char *ptr, const std::size_t size, const std::size_t nmemb, void *userdata) {
        const auto *c = static_cast<DlCtx *>(userdata);
        if (!c->out) return 0;
        const std::size_t n = size * nmemb;
        c->out->write(ptr, static_cast<std::streamsize>(n));
        return c->out->good() ? n : 0;
    }

    int xferCb(void *clientp, const curl_off_t dltotal, const curl_off_t dlnow,
               curl_off_t, curl_off_t) {
        const auto *c = static_cast<DlCtx *>(clientp);
        if (!c->progress) return 0;
        const double total = dltotal > 0 ? static_cast<double>(dltotal + c->base) : 0.0;
        c->progress(total, static_cast<double>(dlnow + c->base));
        return 0;
    }

    curl_off_t fileSize(const std::string &p) {
        std::error_code ec;
        const auto n = std::filesystem::file_size(p, ec);
        return ec ? 0 : static_cast<curl_off_t>(n);
    }

    bool remoteGone(const CURLcode r, const long httpCode) {
        return httpCode == 404 || httpCode == 410 ||
               r == CURLE_REMOTE_FILE_NOT_FOUND || r == CURLE_URL_MALFORMAT;
    }

    std::size_t listWriteCb(const char *ptr, const std::size_t size, const std::size_t nmemb, void *userdata) {
        auto *s = static_cast<std::string *>(userdata);
        s->append(ptr, size * nmemb);
        return size * nmemb;
    }


    void parseList(const std::string &listing, std::vector<std::string> &out) {
        if (listing.find("href=") != std::string::npos) {
            std::regex re(R"(href\s*=\s*["']([^"']+)["'])", std::regex::icase);
            for (std::sregex_iterator it(listing.begin(), listing.end(), re), end; it != end; ++it) {
                std::string h = (*it)[1].str();
                if (h.empty() || h.back() == '/') continue; // 子目录链接，不是文件
                if (const std::size_t sl = h.find_last_of('/'); sl != std::string::npos) h = h.substr(sl + 1);
                if (!h.empty() && h != "." && h != ".." && h != "index.html" &&
                    h.find('*') == std::string::npos)
                    out.push_back(h);
            }
            return;
        }


        for (const std::string &raw: Str::split(listing, '\n')) {
            const std::string line = Str::strip(raw); // 顺带吃掉 \r 与行尾空白
            if (line.empty()) continue;
            const std::size_t sp = line.find_last_of(" \t");
            const std::string name = sp != std::string::npos ? line.substr(sp + 1) : line;
            if (name.empty() || name == "." || name == ".." ||
                name.find('*') != std::string::npos)
                continue;
            out.push_back(name);
        }
    }
}

RemoteClient::RemoteClient() { ensureGlobalInit(); }

RemoteClient::~RemoteClient() = default;

bool RemoteClient::listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err) {
    ensureGlobalInit();
    CURL *c = curl_easy_init();
    if (!c) {
        err = "curl_easy_init 失败";
        return false;
    }

    std::string dir = url;
    if (!dir.empty() && dir.back() != '/') dir.push_back('/');

    std::string listing;
    curl_easy_setopt(c, CURLOPT_URL, dir.c_str());
    curl_easy_setopt(c, CURLOPT_USERNAME, "anonymous");
    curl_easy_setopt(c, CURLOPT_PASSWORD, "user@gnsslab");
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, listWriteCb);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &listing);
    curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 20L);

    curl_easy_setopt(c, CURLOPT_TIMEOUT, 45L);

    const CURLcode r = curl_easy_perform(c);
    curl_easy_cleanup(c);
    if (r != CURLE_OK) {
        err = curl_easy_strerror(r);
        return false;
    }
    parseList(listing, outNames);
    return true;
}

bool RemoteClient::fetchText(const std::string &url, std::string &out, std::string &err) {
    ensureGlobalInit();
    CURL *c = curl_easy_init();
    if (!c) {
        err = "curl_easy_init 失败";
        return false;
    }

    out.clear();
    curl_easy_setopt(c, CURLOPT_URL, url.c_str());
    curl_easy_setopt(c, CURLOPT_USERNAME, "anonymous");
    curl_easy_setopt(c, CURLOPT_PASSWORD, "user@gnsslab");
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, listWriteCb);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &out);
    curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 15L);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 30L);

    const CURLcode r = curl_easy_perform(c);
    curl_easy_cleanup(c);
    if (r != CURLE_OK) {
        err = curl_easy_strerror(r);
        return false;
    }
    return true;
}

bool RemoteClient::downloadFile(const std::string &url, const std::string &localPath,
                                const ProgressCb &progress, const LogCb &log, const int retries,
                                const int timeoutSec) {
    ensureGlobalInit();
    for (int attempt = 1; attempt <= retries; ++attempt) {
        const curl_off_t base = fileSize(localPath);

        CURL *c = curl_easy_init();
        if (!c) {
            if (log) log("curl_easy_init 失败");
            return false;
        }

        const auto mode = std::ios::binary | std::ios::out |
                          (base > 0 ? std::ios::app : std::ios::trunc);
        std::ofstream out;
        out.open(localPath.c_str(), mode);
        if (!out) {
            std::error_code ec;
            if (const auto parent = std::filesystem::path(localPath).parent_path(); !parent.empty())
                std::filesystem::create_directories(parent, ec);
            out.clear();
            out.open(localPath.c_str(), mode);
        }
        if (!out) {
            curl_easy_cleanup(c);
            LOG_ERROR << "[下载] 无法创建本地文件: " << localPath;
            if (log) log("无法创建本地文件: " + localPath);
            return false;
        }

        DlCtx ctx{&out, base, progress, log};
        curl_easy_setopt(c, CURLOPT_URL, url.c_str());
        curl_easy_setopt(c, CURLOPT_USERNAME, "anonymous");
        curl_easy_setopt(c, CURLOPT_PASSWORD, "user@gnsslab");
        curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, writeCb);
        curl_easy_setopt(c, CURLOPT_WRITEDATA, &ctx);
        curl_easy_setopt(c, CURLOPT_XFERINFOFUNCTION, xferCb);
        curl_easy_setopt(c, CURLOPT_XFERINFODATA, &ctx);
        curl_easy_setopt(c, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 20L);
        if (base > 0) curl_easy_setopt(c, CURLOPT_RESUME_FROM_LARGE, base);
        if (timeoutSec > 0) curl_easy_setopt(c, CURLOPT_TIMEOUT, static_cast<long>(timeoutSec));
        curl_easy_setopt(c, CURLOPT_LOW_SPEED_LIMIT, 64L);
        curl_easy_setopt(c, CURLOPT_LOW_SPEED_TIME, 180L);
        curl_easy_setopt(c, CURLOPT_FAILONERROR, 1L);

        if (base > 0) {
            LOG_INFO << "[下载] 续传：" << localPath << " 已有 " << base << " 字节";
            if (log) log("续传：已有 " + std::to_string(base) + " 字节");
        }

        const CURLcode r = curl_easy_perform(c);
        long httpCode = 0;
        curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &httpCode);
        out.close();
        const bool writeOk = !out.fail();
        curl_easy_cleanup(c);

        if (r == CURLE_OK && writeOk) {
            if (base > 0 && httpCode == 200) {
                std::error_code ec;
                std::filesystem::remove(localPath, ec);
                LOG_WARN << "[下载] 服务器不支持断点续传，从头重下: " << url;
                if (log) log("服务器不支持断点续传，从头重下");
                if (attempt < retries) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(300 * attempt));
                    continue;
                }
                return false;
            }
            if (log) log("完成: " + localPath);
            return true;
        }

        if (log)
            log("下载失败(尝试 " + std::to_string(attempt) + "/" +
                std::to_string(retries) + "): " + curl_easy_strerror(r));
        LOG_WARN << "[下载] 尝试 " << attempt << "/" << retries << " 失败: " << url
                << " — " << curl_easy_strerror(r);
        const bool gone = remoteGone(r, httpCode);
        if (const bool rangeBad = r == CURLE_RANGE_ERROR || httpCode == 416; gone || rangeBad) {
            std::error_code ec;
            std::filesystem::remove(localPath, ec);
        } else if (const curl_off_t now = fileSize(localPath); now > 0) {
            LOG_INFO << "[下载] 保留已下载的 " << now << " 字节，下次续传: " << localPath;
        }
        if (attempt == retries) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(300 * attempt));
    }
    return false;
}
