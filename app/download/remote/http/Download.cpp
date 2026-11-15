#include "download/remote/http/Core.h"
#include "download/remote/http/Internal.h"
#include "i18n/I18n.h"

#include "Log.h"
#include "util/Clock.h"

#include <curl/curl.h>

#include <atomic>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace {
    struct DlCtx {
        std::ostream *out = nullptr;
        curl_off_t base = 0;
        Download::Http::ProgressCb progress;
        const std::atomic<bool> *cancel = nullptr;
        std::exception_ptr error;
    };

    std::size_t writeCb(const char *ptr, const std::size_t size, const std::size_t nmemb, void *userdata) {
        auto *c = static_cast<DlCtx *>(userdata);
        if (!c->out) return 0;
        const std::size_t n = size * nmemb;
        try {
            c->out->write(ptr, static_cast<std::streamsize>(n));
        } catch (...) {
            c->error = std::current_exception();
            return 0;
        }
        return c->out->good() ? n : 0;
    }

    int xferCb(void *clientp, const curl_off_t dltotal, const curl_off_t dlnow, curl_off_t, curl_off_t) {
        auto *c = static_cast<DlCtx *>(clientp);
        if (c->cancel && c->cancel->load()) return 1;
        if (!c->progress) return 0;
        try {
            c->progress(dltotal > 0 ? static_cast<double>(dltotal + c->base) : 0.0,
                        static_cast<double>(dlnow + c->base));
        } catch (...) {
            c->error = std::current_exception();
            return 1;
        }
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

    struct PartialCtx {
        std::ostream *out = nullptr;
        curl_off_t got = 0;
        curl_off_t limit = 0;
        bool hitLimit = false;
        Download::Http::ProgressCb progress;
        const std::atomic<bool> *cancel = nullptr;
        std::exception_ptr error;
    };

    std::size_t partialWriteCb(const char *ptr, const std::size_t size, const std::size_t nmemb,
                               void *userdata) {
        auto *c = static_cast<PartialCtx *>(userdata);
        if (!c->out) return 0;
        std::size_t n = size * nmemb;
        if (c->limit > 0 && c->got + static_cast<curl_off_t>(n) > c->limit) {
            n = static_cast<std::size_t>(c->limit - c->got);
            c->hitLimit = true;
        }
        try {
            c->out->write(ptr, static_cast<std::streamsize>(n));
        } catch (...) {
            c->error = std::current_exception();
            return 0;
        }
        if (!c->out->good()) return 0;
        c->got += static_cast<curl_off_t>(n);
        return c->hitLimit ? 0 : size * nmemb;
    }

    int partialXferCb(void *clientp, curl_off_t, const curl_off_t dlnow, curl_off_t, curl_off_t) {
        auto *c = static_cast<PartialCtx *>(clientp);
        if (c->cancel && c->cancel->load()) return 1;
        if (c->progress) {
            try {
                c->progress(static_cast<double>(c->limit), static_cast<double>(dlnow));
            } catch (...) {
                c->error = std::current_exception();
                return 1;
            }
        }
        return 0;
    }
}

namespace Download::Http {
    bool downloadPartial(const std::string &url, const std::string &localPath,
                         const long long maxBytes, const ProgressCb &progress,
                         const int timeoutSec, const std::atomic<bool> *cancel,
                         const long long from, const bool append) {
        if (maxBytes <= 0) return false;
        std::error_code ec;
        if (const auto parent = std::filesystem::path(localPath).parent_path(); !parent.empty())
            std::filesystem::create_directories(parent, ec);

        const auto flags = std::ios::binary | std::ios::out | (append ? std::ios::app : std::ios::trunc);
        std::ofstream out(localPath.c_str(), flags);
        if (!out) {
            LOG_ERROR << "[DL] " << I18n::tr("dl.http.create_file_failed", localPath);
            return false;
        }

        const EasyHandle c;
        if (!c.handle) return false;
        const auto rollback = [&] {
            if (append) std::filesystem::resize_file(localPath, static_cast<std::uintmax_t>(from), ec);
            else std::filesystem::remove(localPath, ec);
        };

        PartialCtx ctx{&out, 0, maxBytes, false, progress, cancel, nullptr};
        const std::string range = std::to_string(from) + "-" + std::to_string(from + maxBytes - 1);
        curl_easy_setopt(c.handle, CURLOPT_URL, url.c_str());
        applyAuth(c.handle, url);
        curl_easy_setopt(c.handle, CURLOPT_RANGE, range.c_str());
        curl_easy_setopt(c.handle, CURLOPT_WRITEFUNCTION, partialWriteCb);
        curl_easy_setopt(c.handle, CURLOPT_WRITEDATA, &ctx);
        curl_easy_setopt(c.handle, CURLOPT_XFERINFOFUNCTION, partialXferCb);
        curl_easy_setopt(c.handle, CURLOPT_XFERINFODATA, &ctx);
        curl_easy_setopt(c.handle, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(c.handle, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(c.handle, CURLOPT_CONNECTTIMEOUT, 20L);
        curl_easy_setopt(c.handle, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(c.handle, CURLOPT_FAILONERROR, 1L);
        if (timeoutSec > 0) curl_easy_setopt(c.handle, CURLOPT_TIMEOUT, static_cast<long>(timeoutSec));
        curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_LIMIT, 512L);
        curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_TIME, 60L);

        const CURLcode r = curl_easy_perform(c.handle);
        out.close();
        if (cancel && cancel->load()) {
            rollback();
            return false;
        }
        if (!callbackError(ctx.error).empty()) {
            rollback();
            return false;
        }
        if (ctx.hitLimit && ctx.got == maxBytes) {
            LOG_INFO << "[DL] " << I18n::tr("dl.http.head_only", ctx.got, url);
            return true;
        }
        if (r == CURLE_OK && ctx.got > 0) return true;
        rollback();
        LOG_WARN << "[DL] " << I18n::tr("dl.http.head_failed", ctx.got, url, curl_easy_strerror(r));
        return false;
    }

    bool downloadFile(const std::string &url, const std::string &localPath,
                      const ProgressCb &progress,
                      const LogCb &log, const int stalls,
                      const int idleSec, const std::atomic<bool> *cancel) {
        int stalled = 0;
        for (int attempt = 1; stalled < stalls && attempt <= 1000; ++attempt) {
            if (cancel && cancel->load()) {
                if (log) log(I18n::tr("dl.http.cancelled"));
                return false;
            }
            const curl_off_t base = fileSize(localPath);

            EasyHandle c;
            if (!c.handle) {
                if (log) log(I18n::tr("dl.http.init_failed"));
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
                LOG_ERROR << "[DL] " << I18n::tr("dl.http.create_file_failed", localPath);
                if (log) log(I18n::tr("dl.http.create_file_failed", localPath));
                return false;
            }

            DlCtx ctx{&out, base, progress, cancel};
            curl_easy_setopt(c.handle, CURLOPT_URL, url.c_str());
            applyAuth(c.handle, url);
            curl_easy_setopt(c.handle, CURLOPT_WRITEFUNCTION, writeCb);
            curl_easy_setopt(c.handle, CURLOPT_WRITEDATA, &ctx);
            curl_easy_setopt(c.handle, CURLOPT_XFERINFOFUNCTION, xferCb);
            curl_easy_setopt(c.handle, CURLOPT_XFERINFODATA, &ctx);
            curl_easy_setopt(c.handle, CURLOPT_NOPROGRESS, 0L);
            curl_easy_setopt(c.handle, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(c.handle, CURLOPT_CONNECTTIMEOUT, 20L);
            curl_easy_setopt(c.handle, CURLOPT_NOSIGNAL, 1L);
            if (base > 0) curl_easy_setopt(c.handle, CURLOPT_RESUME_FROM_LARGE, base);
            // 只掐「完全不动」的连接：idleSec 秒内平均低于 64 B/s 就重连续传。
            curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_LIMIT, 64L);
            curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_TIME, static_cast<long>(idleSec > 0 ? idleSec : 60));
            curl_easy_setopt(c.handle, CURLOPT_FAILONERROR, 1L);

            if (base > 0) {
                const std::string resumed = I18n::tr("dl.http.resume", base);
                LOG_INFO << "[DL] " << localPath << " " << resumed;
                if (log) log(resumed);
            }

            const CURLcode r = curl_easy_perform(c.handle);
            long httpCode = 0;
            curl_easy_getinfo(c.handle, CURLINFO_RESPONSE_CODE, &httpCode);
            out.close();
            const bool writeOk = !out.fail();
            const std::string cbErr = callbackError(ctx.error);
            const std::string why = cbErr.empty() ? std::string(curl_easy_strerror(r)) : cbErr;

            if (cancel && cancel->load()) {
                if (log) log(I18n::tr("dl.http.cancelled"));
                return false;
            }
            if (r == CURLE_OK && writeOk && cbErr.empty()) {
                if (base > 0 && httpCode == 200) {
                    std::error_code ec;
                    std::filesystem::remove(localPath, ec);
                    LOG_WARN << "[DL] " << I18n::tr("dl.http.no_range", url);
                    if (log) log(I18n::tr("dl.http.no_range_short"));
                    if (++stalled >= stalls) return false;
                    Clock::sleep(Clock::milliseconds(300));
                    continue;
                }
                if (log) log(I18n::tr("dl.status.done", localPath));
                return true;
            }

            const bool gone = remoteGone(r, httpCode);
            if (const bool rangeBad = r == CURLE_RANGE_ERROR || httpCode == 416; gone || rangeBad) {
                std::error_code ec;
                std::filesystem::remove(localPath, ec);
            }
            const curl_off_t now = fileSize(localPath);
            const bool progressed = now > base;
            stalled = progressed ? 0 : stalled + 1;
            if (log)
                log(I18n::tr("dl.http.disconnected", attempt, now, why));
            LOG_WARN << "[DL] " << I18n::tr("dl.http.disconnected_log", attempt, now, url, why);
            if (!progressed && now > 0)
                LOG_INFO << "[DL] " << I18n::tr("dl.http.keep_partial", now, localPath);
            if (stalled >= stalls) return false;
            Clock::sleep(Clock::milliseconds(attempt < 10 ? 300 * attempt : 3000));
        }
        return false;
    }
}
