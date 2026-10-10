#define _CRT_SECURE_NO_WARNINGS   // fopen 走 C 运行库传统接口（与 src/io/ClkReader.cpp 同）

#include "FtpClient.h"

#include "Log.h"

#include <thread>
#include <chrono>
#include <filesystem>
#include <vector>
#include <string>

namespace {
    // libcurl 全局只需初始化一次；析构略过（GUI 退出时泄露无碍，且避免handle存活期间cleanup）。
    void ensureGlobalInit() {
        static bool ok = curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK;
        (void) ok;
    }

    struct DlCtx {
        FILE *fp = nullptr;
        FtpClient::ProgressCb progress;
        FtpClient::LogCb log;
    };

    // 回调首参按 curl 文档写 char*（curl_easy_setopt 是变参，编译器不校验函数指针类型）
    size_t writeCb(const char *ptr, const size_t size, const size_t nmemb, void *userdata) {
        const auto *c = static_cast<DlCtx *>(userdata);
        if (!c->fp) return 0;
        return fwrite(ptr, size, nmemb, c->fp);
    }

    int xferCb(void *clientp, const curl_off_t dltotal, const curl_off_t dlnow,
               curl_off_t, curl_off_t) {
        if (const auto *c = static_cast<DlCtx *>(clientp); c->progress) c->progress(static_cast<double>(dltotal), static_cast<double>(dlnow));
        return 0;
    }

    size_t listWriteCb(const char *ptr, const size_t size, const size_t nmemb, void *userdata) {
        auto *s = static_cast<std::string *>(userdata);
        s->append(ptr, size * nmemb);
        return size * nmemb;
    }

    // FTP LIST 行 → 文件名（取末段空白分隔 token，跳过 . 和 ..）
    void parseList(const std::string &listing, std::vector<std::string> &out) {
        std::string line;
        for (const char c: listing) {
            if (c == '\n' || c == '\r') {
                if (!line.empty()) {
                    // 末段 token
                    if (const size_t p = line.find_last_not_of(" \t"); p != std::string::npos) line = line.substr(0, p + 1);
                    const size_t sp = line.find_last_of(" \t");
                    if (std::string name = sp != std::string::npos ? line.substr(sp + 1) : line;
                        !name.empty() && name != "." && name != ".." &&
                        name.find('*') == std::string::npos)
                        out.push_back(name);
                }
                line.clear();
            } else {
                line.push_back(c);
            }
        }
    }
} // namespace

FtpClient::FtpClient() { ensureGlobalInit(); }

FtpClient::~FtpClient() = default;

bool FtpClient::listDir(const std::string &url, std::vector<std::string> &outNames, std::string &err) {
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
    // 列目录超时短于下载：不存在/未发布的目录若服务器不立即拒绝，也不该拖到 60s
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

bool FtpClient::downloadFile(const std::string &url, const std::string &localPath,
                             const ProgressCb &progress, const LogCb &log, const int retries) {
    for (int attempt = 1; attempt <= retries; ++attempt) {
        CURL *c = curl_easy_init();
        if (!c) {
            if (log) log("curl_easy_init 失败");
            return false;
        }

        FILE *fp = fopen(localPath.c_str(), "wb");
        if (!fp) {
            curl_easy_cleanup(c);
            if (log) log("无法创建本地文件: " + localPath);
            return false;
        }

        DlCtx ctx{fp, progress, log};
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
        curl_easy_setopt(c, CURLOPT_TIMEOUT, 120L);

        const CURLcode r = curl_easy_perform(c);
        fclose(fp);
        curl_easy_cleanup(c);

        if (r == CURLE_OK) {
            if (log) log("完成: " + localPath);
            return true;
        }
        if (log)
            log("下载失败(尝试 " + std::to_string(attempt) + "/" +
                std::to_string(retries) + "): " + curl_easy_strerror(r));
        LOG_WARN << "[下载] 尝试 " << attempt << "/" << retries << " 失败: " << url
                << " — " << curl_easy_strerror(r);
        std::error_code ec;
        std::filesystem::remove(localPath, ec); // 删掉不完整文件
        if (attempt == retries) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(300 * attempt));
    }
    return false;
}
