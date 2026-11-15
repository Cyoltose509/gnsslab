#include "download/remote/http/Core.h"
#include "download/remote/http/Internal.h"
#include "i18n/I18n.h"
#include "download/remote/Earthdata.h"

#include "core/PathMemory.h"

#include <curl/curl.h>

#include "util/Clock.h"

#include <atomic>
#include <exception>
#include <map>
#include <mutex>
#include <string>
#include <utility>

namespace {
    struct EarthdataAuth {
        std::string user;
        std::string pass;
    };

    std::mutex g_authMutex;
    EarthdataAuth g_auth;

    struct HostHealth {
        int fails = 0;
        Clock::Point until{};
    };

    std::mutex g_downMutex;
    std::map<std::string, HostHealth> g_downHosts;
    constexpr auto kHostDownTtl = Clock::seconds(60);
    constexpr int kHostDownFailures = 2;

    /// 键取 scheme://host，同一站点下的不同目录共用一条记录。
    std::string hostOf(const std::string &url) {
        const std::size_t sep = url.find("://");
        if (sep == std::string::npos) return url;
        const std::size_t begin = sep + 3;
        const std::size_t end = url.find('/', begin);
        return url.substr(0, end == std::string::npos ? url.size() : end);
    }

    bool hostDown(const std::string &url) {
        const std::string host = hostOf(url);
        std::lock_guard lk(g_downMutex);
        const auto it = g_downHosts.find(host);
        if (it == g_downHosts.end()) return false;
        if (!Clock::due(it->second.until)) return true;
        g_downHosts.erase(it); // 冷却到期，重新给一次机会
        return false;
    }

    bool neverConnected(CURL *h) {
        double ct = 0.0;
        curl_easy_getinfo(h, CURLINFO_CONNECT_TIME, &ct);
        return ct < 0.001;
    }

    void markHostDown(const std::string &url) {
        const std::string host = hostOf(url);
        std::lock_guard lk(g_downMutex);
        if (auto &[fails, until] = g_downHosts[host]; ++fails >= kHostDownFailures)
            until = Clock::after(kHostDownTtl);
    }

    void markHostUp(const std::string &url) {
        const std::string host = hostOf(url);
        std::lock_guard lk(g_downMutex);
        g_downHosts.erase(host);
    }

    struct TextSink {
        std::string text;
        std::exception_ptr error;
    };

    std::size_t listWriteCb(const char *ptr, const std::size_t size, const std::size_t nmemb, void *userdata) {
        auto *s = static_cast<TextSink *>(userdata);
        try {
            s->text.append(ptr, size * nmemb);
        } catch (...) {
            s->error = std::current_exception();
            return 0;
        }
        return size * nmemb;
    }

    // 列目录/取文本用的极简进度回调：仅用于响应取消标志而中止传输。
    int abortXferCb(const void *clientp, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
        if (auto *cancel = static_cast<const std::atomic<bool> *>(clientp); cancel && cancel->load()) return 1;
        return 0;
    }
}

namespace Download::Http {
    Response request(const std::string &url, const Fetch &opt, const std::atomic<bool> *cancel) {
        Response r;
        if (hostDown(url)) {
            r.err = I18n::tr("dl.http.host_down");
            return r;
        }
        const EasyHandle c;
        if (!c.handle) {
            r.err = I18n::tr("dl.http.init_failed");
            return r;
        }

        TextSink sink;
        curl_easy_setopt(c.handle, CURLOPT_URL, url.c_str());
        applyAuth(c.handle, url);
        if (opt.headOnly) curl_easy_setopt(c.handle, CURLOPT_NOBODY, 1L);
        curl_easy_setopt(c.handle, CURLOPT_WRITEFUNCTION, listWriteCb);
        curl_easy_setopt(c.handle, CURLOPT_WRITEDATA, &sink);
        curl_easy_setopt(c.handle, CURLOPT_XFERINFOFUNCTION, abortXferCb);
        curl_easy_setopt(c.handle, CURLOPT_XFERINFODATA, const_cast<std::atomic<bool> *>(cancel));
        curl_easy_setopt(c.handle, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(c.handle, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(c.handle, CURLOPT_CONNECTTIMEOUT, opt.connectTimeout);
        curl_easy_setopt(c.handle, CURLOPT_TIMEOUT, opt.timeout);
        curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_LIMIT, 64L);
        curl_easy_setopt(c.handle, CURLOPT_LOW_SPEED_TIME, 120L);
        curl_easy_setopt(c.handle, CURLOPT_NOSIGNAL, 1L);

        r.code = curl_easy_perform(c.handle);
        curl_easy_getinfo(c.handle, CURLINFO_RESPONSE_CODE, &r.http);
        char *finalUrl = nullptr;
        curl_easy_getinfo(c.handle, CURLINFO_EFFECTIVE_URL, &finalUrl);
        if (finalUrl) r.finalUrl = finalUrl;

        if (const std::string cbErr = callbackError(sink.error); !cbErr.empty()) {
            r.err = I18n::tr("dl.http.callback_error", cbErr);
            return r;
        }
        if (r.code != CURLE_OK) {
            if (neverConnected(c.handle)) markHostDown(url);
            r.err = curl_easy_strerror(r.code);
            return r;
        }
        markHostUp(url);
        r.text = std::move(sink.text);
        return r;
    }

    void applyAuth(CURL *h, const std::string &url) {
        const bool earthdata = url.find("cddis.nasa.gov") != std::string::npos ||
                               url.find("earthdata.nasa.gov") != std::string::npos ||
                               url.find("urs.earthdata.nasa.gov") != std::string::npos;
        std::string user, pass;
        if (earthdata) {
            std::lock_guard lk(g_authMutex);
            user = g_auth.user;
            pass = g_auth.pass;
        }
        if (earthdata && !user.empty() && !pass.empty()) {
            curl_easy_setopt(h, CURLOPT_USERNAME, user.c_str());
            curl_easy_setopt(h, CURLOPT_PASSWORD, pass.c_str());
            curl_easy_setopt(h, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
            curl_easy_setopt(h, CURLOPT_UNRESTRICTED_AUTH, 1L);
            curl_easy_setopt(h, CURLOPT_COOKIEFILE, "");
        } else {
            curl_easy_setopt(h, CURLOPT_USERNAME, "anonymous");
            curl_easy_setopt(h, CURLOPT_PASSWORD, "user@gnsslab");
        }
    }

    std::string callbackError(const std::exception_ptr &e) {
        if (!e) return {};
        try {
            std::rethrow_exception(e);
        } catch (const std::exception &x) {
            return x.what();
        } catch (...) {
            return I18n::tr("dl.http.callback_unknown");
        }
    }

    void setEarthdata(const std::string &user, const std::string &pass) {
        std::lock_guard lk(g_authMutex);
        g_auth.user = user;
        g_auth.pass = pass;
    }

    float fracOf(const double total, const double now) {
        if (total <= 0.0) return -1.0f;
        const double r = now / total;
        return static_cast<float>(r < 0.0 ? 0.0 : r > 1.0 ? 1.0 : r);
    }

    bool hasEarthdata() {
        std::lock_guard lk(g_authMutex);
        return !g_auth.user.empty() && !g_auth.pass.empty();
    }

    bool checkEarthdata(const std::string &url, std::string &msg,
                        const std::atomic<bool> *cancel) {
        {
            std::lock_guard lk(g_authMutex);
            if (g_auth.user.empty() || g_auth.pass.empty()) {
                msg = I18n::tr("dl.http.no_credentials");
                return false;
            }
        }

        const Response r = request(url, kAuthCheckFetch, cancel);
        if (!r.err.empty()) {
            msg = r.err;
            return false;
        }
        if (const std::string landed = r.finalUrl.empty() ? url : r.finalUrl;
            landed.find("urs.earthdata.nasa.gov") != std::string::npos ||
            landed.find("oauth/authorize") != std::string::npos) {
            msg = I18n::tr("dl.http.auth_failed");
            return false;
        }
        if (r.http >= 400) {
            msg = I18n::tr("dl.http.http_status", r.http);
            return false;
        }
        msg = I18n::tr("dl.http.auth_ok", r.http);
        return true;
    }

    bool fetchText(const std::string &url, std::string &out, std::string &err,
                   const std::atomic<bool> *cancel) {
        out.clear();
        const Response r = request(url, kTextFetch, cancel);
        if (!r.err.empty()) {
            err = r.err;
            return false;
        }
        if (r.http >= 400) {
            err = "HTTP " + std::to_string(r.http);
            return false;
        }
        out = r.text;
        return true;
    }

    bool probeFile(const std::string &url, bool &exists, std::string &err,
                   const std::atomic<bool> *cancel) {
        exists = false;
        const Response r = request(url, kProbeFetch, cancel);
        if (!r.err.empty()) {
            err = r.err;
            return false;
        }
        if (r.http == 404 || r.http == 410) return true;
        if (r.http == 200 || r.http == 206) {
            exists = true;
            return true;
        }
        err = "HTTP " + std::to_string(r.http);
        return false;
    }

    void forgetDownHosts() {
        std::lock_guard lk(g_downMutex);
        g_downHosts.clear();
    }
}

namespace Download::Earthdata {
    void load() {
        const auto &pm = AppConfig::PathMemory::instance();
        Http::setEarthdata(pm.get(kUserKey), pm.get(kPassKey));
    }
}
