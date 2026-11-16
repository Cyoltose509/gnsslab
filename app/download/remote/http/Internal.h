#pragma once

#include <curl/curl.h>

#include <atomic>
#include <exception>
#include <string>

namespace Download::Http {
    struct EasyHandle {
        CURL *handle;

        EasyHandle() {
            static const bool ok = curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK;
            (void) ok;
            handle = curl_easy_init();
#ifdef CURLSSLOPT_NO_REVOKE
            if (handle) curl_easy_setopt(handle, CURLOPT_SSL_OPTIONS, static_cast<long>(CURLSSLOPT_NO_REVOKE));
#endif
        }

        ~EasyHandle() {
            if (handle) curl_easy_cleanup(handle);
        }

        EasyHandle(const EasyHandle &) = delete;

        EasyHandle &operator=(const EasyHandle &) = delete;
    };

    /// 一次取数的参数
    struct Fetch {
        long connectTimeout = 12;
        long timeout = 45;
        bool headOnly = false;
    };

    constexpr Fetch kListFetch{12, 0, false};
    constexpr Fetch kTextFetch{15, 30, false};
    constexpr Fetch kProbeFetch{12, 20, true};
    constexpr Fetch kAuthCheckFetch{15, 40, false};

    /// 一次请求的全部结果
    struct Response {
        std::string text;
        std::string finalUrl;
        long http = 0;
        CURLcode code = CURLE_OK;
        long long modified = -1; // 远端的 Last-Modified（Unix 秒，UTC）；服务器没给则 -1
        std::string err;
    };

    Response request(const std::string &url, const Fetch &opt, const std::atomic<bool> *cancel);

    void applyAuth(CURL *h, const std::string &url);

    std::string callbackError(const std::exception_ptr &e);
}
